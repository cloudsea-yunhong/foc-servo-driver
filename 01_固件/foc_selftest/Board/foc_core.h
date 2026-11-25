/**
 * @file    foc_core.h
 * @brief   FOC mathematical core: Clarke / Park / inverse Park / SVPWM
 *
 *  All angles in radians.  All currents in Amps, voltages in Volts.
 *  No hardware dependency - pure math, so it can be unit-tested on a PC.
 */

#ifndef __FOC_CORE_H
#define __FOC_CORE_H

#include <stdint.h>

#define FOC_PI          3.14159265358979f
#define FOC_2PI         6.28318530717959f
#define FOC_SQRT3       1.73205080756888f
#define FOC_1_SQRT3     0.57735026918963f
#define FOC_SQRT3_2     0.86602540378444f   /* sqrt(3)/2 */

/* ------------------------------------------------------------------ */
/* Coordinate frames                                                   */
/* ------------------------------------------------------------------ */
typedef struct {
    float alpha;
    float beta;
} foc_ab_t;

typedef struct {
    float d;
    float q;
} foc_dq_t;

/* ------------------------------------------------------------------ */
/* Transforms                                                          */
/* ------------------------------------------------------------------ */

/**
 * Clarke transform (amplitude invariant).
 *   Ialpha = Ia
 *   Ibeta  = (Ia + 2*Ib) / sqrt(3)
 * ic is accepted for symmetry / diagnostics only (ia+ib+ic should be 0).
 */
void foc_clarke(float ia, float ib, float ic, foc_ab_t *out);

/**
 * Park transform - rotate stationary frame into the rotor frame.
 *   Id =  Ialpha*cos(theta) + Ibeta*sin(theta)
 *   Iq = -Ialpha*sin(theta) + Ibeta*cos(theta)
 */
void foc_park(const foc_ab_t *in, float theta, foc_dq_t *out);

/**
 * Inverse Park transform - rotor frame back to stationary frame.
 *   Valpha = Vd*cos(theta) - Vq*sin(theta)
 *   Vbeta  = Vd*sin(theta) + Vq*cos(theta)
 */
void foc_inv_park(const foc_dq_t *in, float theta, foc_ab_t *out);

/* ------------------------------------------------------------------ */
/* Modulation                                                          */
/* ------------------------------------------------------------------ */

/**
 * Space Vector PWM via mid-point (min/max) injection.
 *
 * HOW THIS RELATES TO THE CLASSIC SECTOR-BASED SVPWM
 * --------------------------------------------------
 * The textbook SVPWM splits the circle into 6 sectors, picks the two
 * adjacent active vectors, computes their on-times T1/T2 from
 *   T1 = (sqrt(3)*Ts/Vdc) * |Vref| * sin(60deg - theta)
 *   T2 = (sqrt(3)*Ts/Vdc) * |Vref| * sin(theta)
 * and then distributes the zero vector symmetrically.
 *
 * Algebraically, the duty cycle that comes out of that procedure is
 * exactly the same as:
 *   1) convert Vref back to three phase voltages (inverse Clarke)
 *   2) subtract the mid-point of (max, min) of those three voltages
 *   3) scale to 0..1
 *
 * So this implementation IS SVPWM - it just does not need the sector
 * arithmetic, square roots or trig tables.  It is what most production
 * motor drives actually run.  Advantages:
 *   - no sector decision, no divide-by-zero at sector boundaries
 *   - linear range 0 .. Vdc/sqrt(3), i.e. ~15 % more than sine PWM
 *
 * @param alpha   Valpha [V]
 * @param beta    Vbeta  [V]
 * @param vdc     DC bus voltage [V]
 * @param duty    output duty in 0.0 .. 1.0 for phase U, V, W
 */
void foc_svpwm(float alpha, float beta, float vdc, float duty[3]);

/**
 * Math self-test - verifies the transforms and modulation with no
 * hardware attached.  Returns 0 when everything passes, otherwise the
 * number of failed checks.
 */
int foc_core_selftest(void);

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/** Wrap angle into [0, 2*pi) */
static inline float foc_wrap_2pi(float a)
{
    while (a >= FOC_2PI) a -= FOC_2PI;
    while (a < 0.0f)     a += FOC_2PI;
    return a;
}

/** Clamp a float into [lo, hi] */
static inline float foc_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


/* ------------------------------------------------------------------ */
/* Fast sine/cosine (lookup table + linear interpolation)              */
/*                                                                     */
/* Calling sinf()/cosf() inside a 20 kHz interrupt is too slow on this */
/* part - it was measured to lock the CPU up.  These are ~20 cycles.   */
/* ------------------------------------------------------------------ */

/** sin/cos of an angle in radians, table based */
void foc_sincos(float theta, float *s, float *c);

/** Park with pre-computed sin/cos (avoids a second trig call) */
void foc_park_sc(const foc_ab_t *in, float s, float c, foc_dq_t *out);

/** Inverse Park with pre-computed sin/cos */
void foc_inv_park_sc(const foc_dq_t *in, float s, float c, foc_ab_t *out);
#endif /* __FOC_CORE_H */
