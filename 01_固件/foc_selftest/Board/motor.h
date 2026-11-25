#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

#define MOTOR_POLE_PAIRS          7U
#define MOTOR_SLOT_COUNT          12U
#define MOTOR_POLE_COUNT          14U

#define MOTOR_PWM_FREQ            20000.0f
#define MOTOR_PWM_ARR             4199U
#define MOTOR_PWM_CENTER          2100U

#define MOTOR_SVPWM_MODULATION    0.20f

#define MOTOR_START_FREQ          40.0f
#define MOTOR_MAX_ELEC_FREQ       40.0f

#define MOTOR_RAMP_STEP           0.001f

#define MOTOR_U_CHANNEL           TIM_CHANNEL_1
#define MOTOR_V_CHANNEL           TIM_CHANNEL_2
#define MOTOR_W_CHANNEL           TIM_CHANNEL_3

#define MOTOR_PHASE_120           0x55555555UL
#define MOTOR_PHASE_240           0xAAAAAAAAUL

void Motor_Init(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetElectricalFrequency(float freq);
float Motor_GetElectricalFrequency(void);
float Motor_GetMechanicalRPM(void);
void Motor_Update(void);

#endif
