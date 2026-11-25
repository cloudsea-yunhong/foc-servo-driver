#include "motor.h"
#include <math.h>

#define SQRT3_BY_2       0.8660254037844386f
#define TWO_BY_SQRT3     1.1547005383792515f
#define TWO_PI           6.283185307179586f

static volatile uint32_t phase_accumulator = 0;
static volatile uint32_t phase_step = 0;
static volatile float electrical_frequency = 0.0f;
static volatile float target_frequency = 0.0f;
static volatile uint8_t motor_running = 0;
static volatile uint8_t motor_sector = 0;

static void Motor_UpdatePhaseStep(void)
{
    phase_step = (uint32_t)((electrical_frequency * 4294967296.0f) / MOTOR_PWM_FREQ);
}

static uint8_t SVPWM_GetSector(float Valpha, float Vbeta)
{
    float u1 = Vbeta;
    float u2 = -0.5f * Vbeta + SQRT3_BY_2 * Valpha;
    float u3 = -0.5f * Vbeta - SQRT3_BY_2 * Valpha;
    uint8_t code = 0;

    if (u1 > 0.0f) code |= 1U;
    if (u2 > 0.0f) code |= 2U;
    if (u3 > 0.0f) code |= 4U;

    switch (code)
    {
        case 3: return 1;
        case 1: return 2;
        case 5: return 3;
        case 4: return 4;
        case 6: return 5;
        case 2: return 6;
        default: return 0;
    }
}

static void SVPWM_Calc(float Valpha, float Vbeta, float *duty_u, float *duty_v, float *duty_w)
{
    float X;
    float Y;
    float Z;
    float T1;
    float T2;
    float T0;
    float Ta;
    float Tb;
    float Tc;

    uint8_t sector;

    X = Vbeta;
    Y = SQRT3_BY_2 * Valpha + 0.5f * Vbeta;
    Z = -SQRT3_BY_2 * Valpha + 0.5f * Vbeta;

    sector = SVPWM_GetSector(Valpha, Vbeta);
    motor_sector = sector;

		switch (sector)
		{
				case 1:  T1 = -Z; T2 =  X; break;
				case 2:  T1 =  Y; T2 =  Z; break;   // 
				case 3:  T1 =  X; T2 = -Y; break;
				case 4:  T1 =  Z; T2 = -X; break;
				case 5:  T1 = -Y; T2 = -Z; break;
				case 6:  T1 = -X; T2 =  Y; break;
				default: T1 = 0.0f; T2 = 0.0f; break;
		}


    T1 *= TWO_BY_SQRT3;
    T2 *= TWO_BY_SQRT3;

    if (T1 < 0.0f) T1 = 0.0f;
    if (T2 < 0.0f) T2 = 0.0f;

    T0 = 1.0f - T1 - T2;

    if (T0 < 0.0f)
    {
        float sum = T1 + T2;

        if (sum > 0.0f)
        {
            T1 /= sum;
            T2 /= sum;
        }

        T0 = 0.0f;
    }
		
		

    switch (sector)
    {
        case 1:
            Ta = T1 + T2 + T0 * 0.5f;
            Tb = T2 + T0 * 0.5f;
            Tc = T0 * 0.5f;
            break;

        case 2:
            Ta = T1 + T0 * 0.5f;
            Tb = T1 + T2 + T0 * 0.5f;
            Tc = T0 * 0.5f;
            break;

        case 3:
            Ta = T0 * 0.5f;
            Tb = T1 + T2 + T0 * 0.5f;
            Tc = T2 + T0 * 0.5f;
            break;

        case 4:
            Ta = T0 * 0.5f;
            Tb = T1 + T0 * 0.5f;
            Tc = T1 + T2 + T0 * 0.5f;
            break;

        case 5:
            Ta = T2 + T0 * 0.5f;
            Tb = T0 * 0.5f;
            Tc = T1 + T2 + T0 * 0.5f;
            break;

        case 6:
            Ta = T1 + T2 + T0 * 0.5f;
            Tb = T0 * 0.5f;
            Tc = T1 + T0 * 0.5f;
            break;

        default:
            Ta = 0.5f;
            Tb = 0.5f;
            Tc = 0.5f;
            break;
    }

    if (Ta < 0.0f) Ta = 0.0f;
    if (Ta > 1.0f) Ta = 1.0f;
    if (Tb < 0.0f) Tb = 0.0f;
    if (Tb > 1.0f) Tb = 1.0f;
    if (Tc < 0.0f) Tc = 0.0f;
    if (Tc > 1.0f) Tc = 1.0f;

    *duty_u = Ta;
    *duty_v = Tb;
    *duty_w = Tc;
}

void Motor_Init(void)
{
    phase_accumulator = 0;
    phase_step = 0;
    electrical_frequency = 0.0f;
    target_frequency = 0.0f;
    motor_running = 0;
    motor_sector = 0;

    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_U_CHANNEL, MOTOR_PWM_CENTER);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_V_CHANNEL, MOTOR_PWM_CENTER);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_W_CHANNEL, MOTOR_PWM_CENTER);
}

void Motor_SetElectricalFrequency(float freq)
{
    if (freq < 0.0f) freq = 0.0f;
    if (freq > MOTOR_MAX_ELEC_FREQ) freq = MOTOR_MAX_ELEC_FREQ;
    target_frequency = freq;
}

float Motor_GetElectricalFrequency(void)
{
    return electrical_frequency;
}

float Motor_GetMechanicalRPM(void)
{
    return (electrical_frequency * 60.0f) / (float)MOTOR_POLE_PAIRS;
}

void Motor_Start(void)
{
    phase_accumulator = 0;
    electrical_frequency = 0.0f;
    Motor_SetElectricalFrequency(MOTOR_START_FREQ);
    motor_running = 1;

    HAL_TIM_PWM_Start(&htim3, MOTOR_U_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, MOTOR_V_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, MOTOR_W_CHANNEL);
    HAL_TIM_Base_Start_IT(&htim2);
}

void Motor_Stop(void)
{
    motor_running = 0;

    HAL_TIM_Base_Stop_IT(&htim2);

    HAL_TIM_PWM_Stop(&htim3, MOTOR_U_CHANNEL);
    HAL_TIM_PWM_Stop(&htim3, MOTOR_V_CHANNEL);
    HAL_TIM_PWM_Stop(&htim3, MOTOR_W_CHANNEL);

    phase_accumulator = 0;
    phase_step = 0;
    electrical_frequency = 0.0f;
    target_frequency = 0.0f;
    motor_sector = 0;

    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_U_CHANNEL, MOTOR_PWM_CENTER);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_V_CHANNEL, MOTOR_PWM_CENTER);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_W_CHANNEL, MOTOR_PWM_CENTER);
}

void Motor_Update(void)
{
    float theta;
    float Valpha;
    float Vbeta;
    float duty_u;
    float duty_v;
    float duty_w;

    uint16_t ccr_u;
    uint16_t ccr_v;
    uint16_t ccr_w;

    if (!motor_running) return;

    if (electrical_frequency < target_frequency)
    {
        electrical_frequency += MOTOR_RAMP_STEP;

        if (electrical_frequency > target_frequency)
            electrical_frequency = target_frequency;
    }
    else if (electrical_frequency > target_frequency)
    {
        electrical_frequency -= MOTOR_RAMP_STEP;

        if (electrical_frequency < target_frequency)
            electrical_frequency = target_frequency;
    }

    Motor_UpdatePhaseStep();

    phase_accumulator += phase_step;

    theta = ((float)phase_accumulator / 4294967296.0f) * TWO_PI;

    Valpha = MOTOR_SVPWM_MODULATION * cosf(theta);
    Vbeta = MOTOR_SVPWM_MODULATION * sinf(theta);

    SVPWM_Calc(Valpha, Vbeta, &duty_u, &duty_v, &duty_w);

    ccr_u = (uint16_t)(duty_u * (float)MOTOR_PWM_ARR);
    ccr_v = (uint16_t)(duty_v * (float)MOTOR_PWM_ARR);
    ccr_w = (uint16_t)(duty_w * (float)MOTOR_PWM_ARR);

    if (ccr_u > MOTOR_PWM_ARR) ccr_u = MOTOR_PWM_ARR;
    if (ccr_v > MOTOR_PWM_ARR) ccr_v = MOTOR_PWM_ARR;
    if (ccr_w > MOTOR_PWM_ARR) ccr_w = MOTOR_PWM_ARR;

    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_U_CHANNEL, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_V_CHANNEL, ccr_v);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_W_CHANNEL, ccr_w);
}



