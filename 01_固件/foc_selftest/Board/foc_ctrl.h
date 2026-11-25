/**
 * @file    foc_ctrl.h
 * @brief   Cascade control: current loop (20 kHz) -> velocity -> position
 *
 *  Control structure (inner to outer):
 *
 *      position PI (1 kHz)  ->  velocity target
 *      velocity PI (1 kHz)  ->  Iq target
 *      current  PI (20 kHz) ->  Vd / Vq
 *      inverse Park + SVPWM ->  three duties
 *
 *  Three-stage limiting protects the machine, the driver and the load:
 *      position error -> speed limit
 *      speed output   -> current limit
 *      current output -> voltage limit
 */

#ifndef __FOC_CTRL_H
#define __FOC_CTRL_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Generic PI(D) controller with anti-windup                           */
/* ------------------------------------------------------------------ */
typedef struct {
    float kp;
    float ki;
    float integral;         /* accumulated I term                */
    float out_min;          /* output clamp                      */
    float out_max;
    float i_term_min;       /* integral clamp (anti-windup)      */
    float i_term_max;
    float i_sep_thresh;     /* integral separation: above this |error| the
                             * integrator is frozen (0 = disabled) */
} foc_pi_t;

void  foc_pi_init(foc_pi_t *pi, float kp, float ki, float out_min, float out_max);
void  foc_pi_reset(foc_pi_t *pi);
float foc_pi_step(foc_pi_t *pi, float error, float dt);

/* ------------------------------------------------------------------ */
/* Control modes                                                       */
/* ------------------------------------------------------------------ */
typedef enum {
    FOC_MODE_STOP      = 0,     /* PWM off, zero torque request    */
    FOC_MODE_TORQUE    = 1,     /* direct Iq command [A]           */
    FOC_MODE_VELOCITY  = 2,     /* speed loop [rad/s]              */
    FOC_MODE_POSITION  = 3      /* position loop [rad]             */
} foc_mode_t;

/* ------------------------------------------------------------------ */
/* Controller set                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    /* --- setpoints --- */
    float target_id;            /* usually 0                       */
    float target_iq;            /* from torque mode or velocity PI */
    float target_vel;           /* from position PI or velocity mode */
    float target_pos;           /* from user                       */

    /* --- feedback --- */
    float id_meas;
    float iq_meas;
    float vel_meas;
    float pos_meas;

    /* --- controllers --- */
    foc_pi_t pi_id;
    foc_pi_t pi_iq;
    foc_pi_t pi_vel;
    foc_pi_t pi_pos;

    /* --- outputs --- */
    float vd;
    float vq;
    float theta_e;              /* electrical angle used for Park */

    /* --- limits --- */
    float i_limit;              /* [A]   */
    float v_limit;              /* [V]   */
    float speed_limit;          /* [rad/s] */

    /* --- state --- */
    foc_mode_t mode;
} foc_ctrl_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Initialise with conservative defaults (debug limits). */
void foc_ctrl_init(foc_ctrl_t *c);

/** Reset all integrators - call on every mode change. */
void foc_ctrl_reset_integrators(foc_ctrl_t *c);

/** Switch mode and clear the integrators. */
void foc_ctrl_set_mode(foc_ctrl_t *c, foc_mode_t mode);

/**
 * Run the current loop (called from the 20 kHz PWM ISR).
 * Inputs : c->id_meas, c->iq_meas, c->theta_e, c->v_limit
 * Outputs: c->vd, c->vq
 */
void foc_ctrl_current_loop(foc_ctrl_t *c, float dt);

/**
 * Run the velocity / position loops (called from a 1 kHz tick).
 * Chooses the Iq setpoint according to c->mode.
 */
void foc_ctrl_outer_loops(foc_ctrl_t *c, float dt);

/** Convenience: read back the torque estimate [N.m] */
float foc_ctrl_torque_estimate(const foc_ctrl_t *c);

#endif /* __FOC_CTRL_H */
