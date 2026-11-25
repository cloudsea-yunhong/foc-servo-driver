/**

 * @file    foc_ctrl.c
 * @brief   Cascade control implementation
 *
 *  NOTE ON GAINS
 *  -------------
 *  The initial PI gains below come from the M创动工坊 reference firmware
 *  (current-sense manual, page 16).  That implementation ran at an
 *  unspecified rate with voltage-mode output, so treat these as a
 *  STARTING POINT ONLY.  Tuning order is:
 *      1) current loop  (inner, fastest)
 *      2) velocity loop
 *      3) position loop (outer, slowest)
 *  Rule of thumb: each outer loop should be 5..20x slower than the loop
 *  inside it, otherwise the cascade oscillates.
 */

#include "foc_ctrl.h"
#include "foc_core.h"
#include "hw_config.h"

#include <math.h>

/* ================================================================== */
/* PI controller                                                       */
/* ================================================================== */

void foc_pi_init(foc_pi_t *pi, float kp, float ki, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
    /* integral clamp defaults to the output range */
    pi->i_term_min = out_min;
    pi->i_term_max = out_max;
    pi->i_sep_thresh = 0.0f;      /* integral separation disabled by default */
}

void foc_pi_reset(foc_pi_t *pi)
{
    pi->integral = 0.0f;
}

float foc_pi_step(foc_pi_t *pi, float error, float dt)
{
    float p_term = pi->kp * error;
    float i_new;
    float out;

    /* ================================================================
     *  INTEGRAL SEPARATION      [EXPERIMENTAL - DISABLED BY DEFAULT]
     *
     *  Idea  : freeze the integrator while |error| is large, so the long
     *          approach phase does not wind it up into overshoot.
     *  Result: TRIED ON HARDWARE AND REJECTED.  On the position loop the
     *          frozen integrator removed the only mechanism able to clear
     *          the residual error, so the shaft stopped short of target.
     *  State : i_sep_thresh is initialised to 0 everywhere, so the branch
     *          below is dead code kept for documentation.
     *  Fix   : a trapezoidal velocity profile, not a patch here.
     * ================================================================ */
    if (pi->i_sep_thresh > 0.0f &&
        (error > pi->i_sep_thresh || error < -pi->i_sep_thresh)) {
        i_new = pi->integral;
    } else {
        i_new = pi->integral + pi->ki * error * dt;
    }

    /* clamp the integral term itself (anti-windup, part 1) */
    i_new = foc_clampf(i_new, pi->i_term_min, pi->i_term_max);

    out = p_term + i_new;

    /* conditional integration: freeze the integrator while saturated
     * in the same direction, so it can recover quickly (part 2) */
    if (out > pi->out_max) {
        out = pi->out_max;
        if (error > 0.0f) {
            i_new = pi->integral;
        }
    } else if (out < pi->out_min) {
        out = pi->out_min;
        if (error < 0.0f) {
            i_new = pi->integral;
        }
    }

    pi->integral = i_new;
    return out;
}

/* ================================================================== */
/* Controller setup                                                    */
/* ================================================================== */

void foc_ctrl_init(foc_ctrl_t *c)
{
    c->target_id  = 0.0f;
    c->target_iq  = 0.0f;
    c->target_vel = 0.0f;
    c->target_pos = 0.0f;

    c->id_meas  = 0.0f;
    c->iq_meas  = 0.0f;
    c->vel_meas = 0.0f;
    c->pos_meas = 0.0f;

    c->vd = 0.0f;
    c->vq = 0.0f;
    c->theta_e = 0.0f;

    /* debug-safe limits; raise them after the loops are stable */
    c->i_limit     = LIMIT_I_DEBUG;
    c->v_limit     = MOTOR_V_RATED * LIMIT_V_DUTY_PCT * 0.01f;
    c->speed_limit = LIMIT_SPEED_RAD_S;

    /* ---- current loop: fastest, widest bandwidth ----
     * d-axis and q-axis are symmetric machines-wise, but Id is often
     * given a stiffer gain because its reference is a constant 0. */
    foc_pi_init(&c->pi_id, 0.50f, 10.0f, -c->v_limit, c->v_limit);
    foc_pi_init(&c->pi_iq, 0.20f, 10.0f, -c->v_limit, c->v_limit);

    /* ---- velocity loop ----
     * Input  : rad/s error
     * Output : Amps  -> clamped to +/- i_limit inside the loop */
    foc_pi_init(&c->pi_vel, 0.80f, 8.00f, -c->i_limit, c->i_limit);

    /* ---- position loop ----
     * Input  : rad error
     * Output : rad/s -> clamped to +/- speed_limit inside the loop */
    foc_pi_init(&c->pi_pos, 3.0f, 0.40f, -c->speed_limit, c->speed_limit);

    c->mode = FOC_MODE_STOP;
}

void foc_ctrl_reset_integrators(foc_ctrl_t *c)
{
    foc_pi_reset(&c->pi_id);
    foc_pi_reset(&c->pi_iq);
    foc_pi_reset(&c->pi_vel);
    foc_pi_reset(&c->pi_pos);
}

void foc_ctrl_set_mode(foc_ctrl_t *c, foc_mode_t mode)
{
    if (c->mode != mode) {
        foc_ctrl_reset_integrators(c);
        c->vd = 0.0f;
        c->vq = 0.0f;
        c->theta_e = c->theta_e;   /* keep the angle, it must not jump */
    }
    c->mode = mode;
}

/* ================================================================== */
/* Inner loop - 20 kHz                                                 */
/* ================================================================== */

void foc_ctrl_current_loop(foc_ctrl_t *c, float dt)
{
    float vd_cmd, vq_cmd, vmag, scale;

    /* keep the output clamps in sync with the current voltage limit */
    c->pi_id.out_max =  c->v_limit;
    c->pi_id.out_min = -c->v_limit;
    c->pi_iq.out_max =  c->v_limit;
    c->pi_iq.out_min = -c->v_limit;

    /* Id is normally regulated to zero: it only makes heat */
    vd_cmd = foc_pi_step(&c->pi_id, c->target_id - c->id_meas, dt);
    /* Iq produces torque: T = Kt * Iq */
    vq_cmd = foc_pi_step(&c->pi_iq, c->target_iq - c->iq_meas, dt);

    /* circular voltage limiting - never exceed the linear SVPWM range.
     * Scaling the whole (vd,vq) vector preserves the commanded angle,
     * which is what the current loop cares about. */
    vmag = sqrtf(vd_cmd * vd_cmd + vq_cmd * vq_cmd);
    if (vmag > c->v_limit && vmag > 1e-6f) {
        scale = c->v_limit / vmag;
        vd_cmd *= scale;
        vq_cmd *= scale;
    }

    c->vd = vd_cmd;
    c->vq = vq_cmd;
}

/* ================================================================== */
/* Outer loops - 1 kHz                                                 */
/* ================================================================== */

void foc_ctrl_outer_loops(foc_ctrl_t *c, float dt)
{
    switch (c->mode) {

    case FOC_MODE_STOP:
        c->target_vel = 0.0f;
        c->target_iq  = 0.0f;
        break;

    case FOC_MODE_TORQUE:
    {
        /* A constant torque command on an UNLOADED motor accelerates it
         * until back-EMF limits the current - on this motor roughly
         * 2600 rpm.  That is normal physics, not a bug, but on the bench
         * it looks like a runaway and keeps the windings hot.  Fold the
         * speed back into the current command above a knee so the demo
         * stays tame. */
        float f  = 1.0f;
        float av = c->vel_meas < 0.0f ? -c->vel_meas : c->vel_meas;
        const float v_knee = 120.0f;          /* rad/s = 1146 rpm */

        if (av > v_knee) {
            f = v_knee / av;
            if (f < 0.05f) {
                f = 0.05f;
            }
        }
        c->target_iq = foc_clampf(c->target_iq * f, -c->i_limit, c->i_limit);
        break;
    }

    case FOC_MODE_VELOCITY:
        c->pi_vel.out_max =  c->i_limit;
        c->pi_vel.out_min = -c->i_limit;
        c->target_iq = foc_pi_step(&c->pi_vel, c->target_vel - c->vel_meas, dt);
        break;

    case FOC_MODE_POSITION:
    {
        float perr;
        float vcmd;

        /* stage 1: position error -> speed command (speed is limited) */
        perr = foc_clampf(c->target_pos - c->pos_meas,
                          -LIMIT_POS_ERR_RAD, LIMIT_POS_ERR_RAD);

        c->pi_pos.out_max =  c->speed_limit;
        c->pi_pos.out_min = -c->speed_limit;
        vcmd = foc_pi_step(&c->pi_pos, perr, dt);

        /* FRICTION COMPENSATION.
         * A small DC motor has a static friction threshold: below about
         * 0.8 rad/s the shaft will not move at all.  As the position error
         * shrinks the PI asks for less and less speed, eventually landing
         * inside that dead band: the shaft stops, the integrator builds up,
         * the motor lurches, stops again.  That stick-slip crawl is what
         * you see at the end of a large move.  Fix: while still outside the
         * position dead band, never ask for a speed between zero and the
         * threshold - jump straight to it. */
        /* ============================================================
         *  FRICTION COMPENSATION     [EXPERIMENTAL - DISABLED]
         *
         *  Idea  : outside a position dead band, never command a speed
         *          between 0 and the static-friction threshold (~0.8 rad/s
         *          here) - jump straight to it so the shaft cannot stick
         *          in the stick-slip crawl seen at the end of a move.
         *  Result: TRIED AND REJECTED.  The forced minimum speed fought
         *          the PI term and produced a runaway (57 revolutions when
         *          asked for 1 radian).
         *  State : disabled, kept as a documented idea.
         *  Fix   : a trapezoidal velocity profile.
         * ============================================================ */
        (void)0;
        /* Integral separation: 0 = disabled.  See the annotated block in
         * foc_pi_step() - it was tried on the bench and rejected because
         * it prevented the residual error from being cleared. */
        c->pi_pos.i_sep_thresh = 0.0f;

        /* stage 2: speed error -> current command (current is limited) */
        c->pi_vel.out_max =  c->i_limit;
        c->pi_vel.out_min = -c->i_limit;
        c->target_iq = foc_pi_step(&c->pi_vel, vcmd - c->vel_meas, dt);
        break;
    }

    default:
        c->target_iq = 0.0f;
        break;
    }

    /* Id reference is always zero for surface-mount PMSM */
    c->target_id = 0.0f;
}

/* ================================================================== */
/* Diagnostics                                                         */
/* ================================================================== */

float foc_ctrl_torque_estimate(const foc_ctrl_t *c)
{
    return MOTOR_KT * c->iq_meas;
}
