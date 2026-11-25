/**
 * @file    foc_core.c
 * @brief   FOC mathematical core - implementation
 *
 *  Pure math, no peripheral access.  Two reasons for keeping it clean:
 *    1) it can be verified on a PC / in a unit test
 *    2) the 20 kHz ISR stays short and readable
 */

#include "foc_core.h"
#include <math.h>

/* ================================================================== */
/* Transforms                                                          */
/* ================================================================== */

void foc_clarke(float ia, float ib, float ic, foc_ab_t *out)
{
    (void)ic;   /* ia + ib + ic == 0, so ic is redundant */

    out->alpha = ia;
    out->beta  = (ia + 2.0f * ib) * FOC_1_SQRT3;
}

void foc_park(const foc_ab_t *in, float theta, foc_dq_t *out)
{
    float s = sinf(theta);
    float c = cosf(theta);

    out->d =  in->alpha * c + in->beta * s;
    out->q = -in->alpha * s + in->beta * c;
}

void foc_inv_park(const foc_dq_t *in, float theta, foc_ab_t *out)
{
    float s = sinf(theta);
    float c = cosf(theta);

    out->alpha = in->d * c - in->q * s;
    out->beta  = in->d * s + in->q * c;
}

/* ================================================================== */
/* SVPWM                                                               */
/* ================================================================== */

void foc_svpwm(float alpha, float beta, float vdc, float duty[3])
{
    float va, vb, vc;
    float vmax, vmin, voff;
    float inv_vdc;

    if (vdc < 1.0f) {
        /* no bus voltage -> safe 50 % (zero average phase voltage) */
        duty[0] = duty[1] = duty[2] = 0.5f;
        return;
    }
    inv_vdc = 1.0f / vdc;

    /* inverse Clarke: alpha/beta -> three phase voltages */
    va = alpha;
    vb = -0.5f * alpha + FOC_SQRT3_2 * beta;
    vc = -0.5f * alpha - FOC_SQRT3_2 * beta;

    /* mid-point injection: this is the SVPWM zero-vector placement */
    vmax = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;

    vmin = va;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;

    voff = -0.5f * (vmax + vmin);

    /* scale into 0..1, centre at 50 % */
    duty[0] = (va + voff) * inv_vdc + 0.5f;
    duty[1] = (vb + voff) * inv_vdc + 0.5f;
    duty[2] = (vc + voff) * inv_vdc + 0.5f;

    /* hard clamp - a saturated duty would short the bridge */
    duty[0] = foc_clampf(duty[0], 0.0f, 1.0f);
    duty[1] = foc_clampf(duty[1], 0.0f, 1.0f);
    duty[2] = foc_clampf(duty[2], 0.0f, 1.0f);
}

/* ================================================================== */
/* Self-test                                                           */
/* ================================================================== */

static int near(float a, float b, float tol)
{
    float d = a - b;
    if (d < 0.0f) d = -d;
    return d <= tol;
}

int foc_core_selftest(void)
{
    int fail = 0;
    foc_ab_t ab;
    foc_dq_t dq;
    float duty[3];

    /* ---- 1) Clarke: balanced set must give |I| = amplitude ---- */
    /* ia = A*cos(t), ib = A*cos(t-120), ic = A*cos(t+120) */
    {
        const float A = 1.0f;
        const float t = 0.7f;
        float ia = A * cosf(t);
        float ib = A * cosf(t - 2.0943951f);
        float ic = A * cosf(t + 2.0943951f);

        foc_clarke(ia, ib, ic, &ab);
        /* magnitude of (alpha,beta) must equal A */
        if (!near(sqrtf(ab.alpha * ab.alpha + ab.beta * ab.beta), A, 1e-3f)) {
            fail++;
        }
        /* and ia + ib + ic must be ~0 */
        if (!near(ia + ib + ic, 0.0f, 1e-3f)) {
            fail++;
        }
    }

    /* ---- 2) Park / inverse Park round trip ---- */
    {
        foc_dq_t in, back;
        foc_ab_t ab2;
        float th = 1.234f;

        in.d = 0.37f;
        in.q = -0.91f;

        foc_inv_park(&in, th, &ab2);
        foc_park(&ab2, th, &back);

        if (!near(back.d, in.d, 1e-3f)) fail++;
        if (!near(back.q, in.q, 1e-3f)) fail++;
    }

    /* ---- 3) SVPWM: zero vector -> all 50 % ---- */
    foc_svpwm(0.0f, 0.0f, 12.0f, duty);
    if (!near(duty[0], 0.5f, 1e-4f)) fail++;
    if (!near(duty[1], 0.5f, 1e-4f)) fail++;
    if (!near(duty[2], 0.5f, 1e-4f)) fail++;

    /* ---- 4) SVPWM: max linear vector -> one phase ~0.933, one ~0.067 */
    {
        float vmax_lin = 12.0f / FOC_SQRT3;   /* 6.928 V */
        float hi = 0.0f, lo = 1.0f;

        foc_svpwm(vmax_lin, 0.0f, 12.0f, duty);
        for (int i = 0; i < 3; i++) {
            if (duty[i] > hi) hi = duty[i];
            if (duty[i] < lo) lo = duty[i];
        }
        /* expect ~0.9333 and ~0.0667 */
        if (!near(hi, 0.9333f, 2e-3f)) fail++;
        if (!near(lo, 0.0667f, 2e-3f)) fail++;
    }

    /* ---- 5) SVPWM: duty stays inside 0..1 for a rotating vector ---- */
    {
        float th;
        for (th = 0.0f; th < FOC_2PI; th += 0.31f) {
            float mag = 12.0f / FOC_SQRT3 * 0.99f;
            float a = mag * cosf(th);
            float b = mag * sinf(th);
            int i;

            foc_svpwm(a, b, 12.0f, duty);
            for (i = 0; i < 3; i++) {
                if (duty[i] < 0.0f || duty[i] > 1.0f) fail++;
            }
        }
    }

    /* ---- 6) Clarke+Park of a pure q-axis current stays on q ---- */
    {
        /* build ia/ib corresponding to Id=0, Iq=A at angle th */
        const float A = 0.5f;
        const float th = 0.9f;
        foc_dq_t src, got;
        foc_ab_t ab3;

        src.d = 0.0f;
        src.q = A;
        foc_inv_park(&src, th, &ab3);
        foc_park(&ab3, th, &got);

        if (!near(got.q, A, 1e-3f)) fail++;
        if (!near(got.d, 0.0f, 1e-3f)) fail++;
    }

    (void)dq;
    return fail;
}

/* ================================================================== */
/* Fast sine / cosine lookup table                                     */
/* ================================================================== */

#define SC_TABLE_BITS   8
#define SC_TABLE_SIZE   (1 << SC_TABLE_BITS)          /* 256 */
#define SC_STEP         (FOC_2PI / (float)SC_TABLE_SIZE)
#define SC_INV_STEP     ((float)SC_TABLE_SIZE / FOC_2PI)

static float s_sin_tab[SC_TABLE_SIZE + 1];
static int   s_tab_ready = 0;

static void sc_build_table(void)
{
    int i;
    for (i = 0; i <= SC_TABLE_SIZE; i++) {
        s_sin_tab[i] = sinf((float)i * SC_STEP);
    }
    s_tab_ready = 1;
}

void foc_sincos(float theta, float *s, float *c)
{
    float x, frac, s0, s1, cv, sv;
    int   i;

    if (!s_tab_ready) {
        sc_build_table();
    }

    while (theta >= FOC_2PI) theta -= FOC_2PI;
    while (theta < 0.0f)     theta += FOC_2PI;

    x    = theta * SC_INV_STEP;
    i    = (int)x;
    if (i >= SC_TABLE_SIZE) {
        i = SC_TABLE_SIZE - 1;
    }
    frac = x - (float)i;

    s0 = s_sin_tab[i];
    s1 = s_sin_tab[i + 1];
    sv = s0 + frac * (s1 - s0);

    /* cos(x) = sin(x + 90 deg) -> shift the index by a quarter table */
    i += SC_TABLE_SIZE / 4;
    if (i > SC_TABLE_SIZE) {
        i -= SC_TABLE_SIZE;
    }
    s0 = s_sin_tab[i];
    s1 = s_sin_tab[(i + 1 > SC_TABLE_SIZE) ? 0 : (i + 1)];
    cv = s0 + frac * (s1 - s0);

    *s = sv;
    *c = cv;
}

void foc_park_sc(const foc_ab_t *in, float s, float c, foc_dq_t *out)
{
    out->d =  in->alpha * c + in->beta * s;
    out->q = -in->alpha * s + in->beta * c;
}

void foc_inv_park_sc(const foc_dq_t *in, float s, float c, foc_ab_t *out)
{
    out->alpha = in->d * c - in->q * s;
    out->beta  = in->d * s + in->q * c;
}