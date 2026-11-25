/**
 * @file    openloop.c
 * @brief   Open-loop spinning test for the DRV8316 board + 2804 motor
 *
 *  What it does
 *  ------------
 *   Sweeps a rotating voltage vector at a fixed ELECTRICAL frequency.
 *   The three duties are pure sine waves 120 degrees apart, so the motor
 *   should turn smoothly at f_elec / pole_pairs revolutions per second.
 *
 *     f_elec = 1 Hz, pole pairs = 7  ->  1/7 rev/s  ~=  8.6 rpm
 *     amplitude = 5 % of Vdc         ->  0.6 V phase swing
 *                                         ~100 mA peak with 5.9 ohm phases
 *
 *   The duties are updated inside the TIM1 update interrupt at 20 kHz -
 *   the same place the real FOC current loop will run.  So this test also
 *   proves that the interrupt path, the NVIC priority and the PWM update
 *   sequence are all sane.
 *
 *  Safety
 *  ------
 *   - runs for RUN_TIME_MS, then forces 50 % duty (zero voltage)
 *   - aborts immediately if any phase current exceeds I_ABORT
 *   - EN (PC4) is dropped low on abort, so the bridge goes passive
 */

#include "openloop.h"
#include "main.h"
#include "hw_config.h"

#include <stdarg.h>
#include <stdio.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Test parameters                                                     */
/* ------------------------------------------------------------------ */
#define FOC_2PI_LOCAL       6.28318530717959f

#define OL_F_START          0.10f       /* starting electrical freq   */
#define OL_F_RAMP_MS        2500u       /* ramp time to OL_F_ELEC_HZ  */
#define OL_F_ELEC_HZ        1.0f        /* electrical frequency         */
#define OL_AMPLITUDE        0.25f       /* fraction of Vdc (5 %)        */
#define OL_TARGET_REV       3.0f        /* stop after this many revs   */
#define OL_TARGET_CNT       ((int32_t)(OL_TARGET_REV * (float)ENC_CPR))
#define OL_TIMEOUT_MS       40000u      /* safety timeout              */
#define OL_PRINT_PERIOD_MS  100u
#define OL_I_ABORT          1.50f       /* abort above this phase current */

/* ------------------------------------------------------------------ */
/* Handles                                                             */
/* ------------------------------------------------------------------ */
static TIM_HandleTypeDef  s_htim1;
static TIM_HandleTypeDef  s_htim3;
static UART_HandleTypeDef s_huart2;

/* ------------------------------------------------------------------ */
/* Runtime state (shared with the ISR)                                 */
/* ------------------------------------------------------------------ */
static volatile float    s_theta;        /* electrical angle [rad]       */
static volatile float    s_dtheta;
static volatile float    s_freq;        /* ramping electrical freq [Hz] */       /* angle increment per ISR      */
static volatile uint8_t  s_running;      /* 1 = generating sine          */
static volatile uint8_t  s_aborted;
static volatile uint16_t s_ccr[3];
static volatile uint32_t s_isr_count;       /* last values written          */

static uint16_t s_zero[3];
static int16_t  s_last_enc;
static int32_t  s_total_enc;
static uint32_t s_start_tick;

/* ================================================================== */
/* USART2                                                              */
/* ================================================================== */
static void uart2_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = PIN_UART_TX | PIN_UART_RX;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = UART_AF;
    HAL_GPIO_Init(GPIO_UART_PORT, &gpio);

    s_huart2.Instance          = USART2;
    s_huart2.Init.BaudRate     = UART_BAUD;
    s_huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart2.Init.StopBits     = UART_STOPBITS_1;
    s_huart2.Init.Parity       = UART_PARITY_NONE;
    s_huart2.Init.Mode         = UART_MODE_TX_RX;
    s_huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_huart2);
}

static char s_txbuf[210];

static void log_printf(const char *fmt, ...)
{
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(s_txbuf, sizeof(s_txbuf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof(s_txbuf) - 1) {
            n = (int)sizeof(s_txbuf) - 1;
        }
        HAL_UART_Transmit(&s_huart2, (uint8_t *)s_txbuf, (uint16_t)n, 200);
    }
}

/* ================================================================== */
/* EN                                                                  */
/* ================================================================== */
static void en_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin   = PIN_EN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_EN_PORT, &gpio);
    HAL_GPIO_WritePin(GPIO_EN_PORT, PIN_EN, GPIO_PIN_SET);
}

/* ================================================================== */
/* TIM1 - 3 phase PWM + update interrupt @ 20 kHz                      */
/* ================================================================== */
static void tim1_pwm_init(void)
{
    GPIO_InitTypeDef        gpio = {0};
    TIM_OC_InitTypeDef      oc   = {0};
    TIM_ClockConfigTypeDef  csc  = {0};
    TIM_MasterConfigTypeDef msc  = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = PIN_PWM_U | PIN_PWM_V | PIN_PWM_W;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = PWM_AF;
    HAL_GPIO_Init(GPIO_PWM_PORT, &gpio);

    s_htim1.Instance               = TIM1;
    s_htim1.Init.Prescaler         = 0;
    s_htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    s_htim1.Init.Period            = PWM_ARR;
    s_htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim1.Init.RepetitionCounter = 1;   /* 1 UEV per PWM period -> 20 kHz ISR */
    s_htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&s_htim1);

    csc.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&s_htim1, &csc);

    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = PWM_CENTER;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&s_htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&s_htim1, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&s_htim1, &oc, TIM_CHANNEL_3);

    msc.MasterOutputTrigger = TIM_TRGO_RESET;
    msc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&s_htim1, &msc);

    /* update interrupt at the PWM rate - this is where the FOC current
     * loop will live, so exercise it here as well */
    __HAL_TIM_CLEAR_FLAG(&s_htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&s_htim1, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/* ================================================================== */
/* TIM3 encoder                                                        */
/* ================================================================== */
static void tim3_encoder_init(void)
{
    GPIO_InitTypeDef        gpio = {0};
    TIM_Encoder_InitTypeDef enc  = {0};
    TIM_MasterConfigTypeDef msc  = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin       = PIN_ENC_A | PIN_ENC_B;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = ENC_AF;
    HAL_GPIO_Init(GPIO_ENC_PORT, &gpio);

    s_htim3.Instance               = TIM3;
    s_htim3.Init.Prescaler         = 0;
    s_htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim3.Init.Period            = 65535u;
    s_htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = ENC_INPUT_FILTER;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = ENC_INPUT_FILTER;
    HAL_TIM_Encoder_Init(&s_htim3, &enc);

    msc.MasterOutputTrigger = TIM_TRGO_RESET;
    msc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&s_htim3, &msc);
}

/* ================================================================== */
/* ADC1 - IN10/IN11/IN12, software triggered, one channel at a time    */
/* ================================================================== */
static void adc1_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin  = PIN_CS_U | PIN_CS_V | PIN_CS_W;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_CS_PORT, &gpio);

    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SMPR1 = (6u << 0) | (6u << 3) | (6u << 6);
    ADC1->SMPR2 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = ADC_CH_U;

    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t i = 0; i < 30000u; i++) {
        __NOP();
    }
    ADC1->CR2 |= (1u << 3);
    for (volatile uint32_t w = 0; (ADC1->CR2 & (1u << 3)) && w < 2000000u; w++) {
        ;
    }
    ADC1->CR2 |= (1u << 2);
    for (volatile uint32_t w = 0; (ADC1->CR2 & (1u << 2)) && w < 2000000u; w++) {
        ;
    }
}

static uint16_t adc1_read_ch(uint32_t ch)
{
    volatile uint32_t w;

    ADC1->SQR3 = ch;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    for (w = 0; w < 300000u; w++) {
        if (ADC1->SR & ADC_SR_EOC) {
            return (uint16_t)(ADC1->DR & 0x0FFFu);
        }
    }
    return 0xFFFFu;
}

static int cs_zero_calibrate(void)
{
    uint32_t acc[3] = {0, 0, 0};
    uint32_t ch[3]  = {ADC_CH_U, ADC_CH_V, ADC_CH_W};
    uint32_t i, k;
    int bad = 0;

    for (k = 0; k < 3u; k++) {
        for (i = 0; i < CS_CAL_SAMPLES; i++) {
            uint16_t v = adc1_read_ch(ch[k]);
            if (v == 0xFFFFu) {
                return -1;
            }
            acc[k] += v;
        }
        s_zero[k] = (uint16_t)(acc[k] / CS_CAL_SAMPLES);
        if (s_zero[k] < 1500u || s_zero[k] > 2600u) {
            bad++;
        }
    }
    return bad;
}

static float cs_amp(uint16_t raw, int ch)
{
    int32_t d = (int32_t)raw - (int32_t)s_zero[ch];
    return ((float)d * ADC_VREF_V / ADC_MAX_COUNT) / CS_GAIN_V_PER_A;
}

/* ================================================================== */
/* 20 kHz update ISR - the open-loop sine generator                    */
/*                                                                     */
/* This is deliberately the same place the real FOC current loop will  */
/* run, and it obeys the same rule: no HAL calls, no UART, no printf.  */
/* ================================================================== */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&s_htim1, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&s_htim1, TIM_FLAG_UPDATE);
        s_isr_count++;

        /* keep this ISR as short as possible: no sinf(), no float math
         * beyond what is needed, no HAL calls.  The sine wave is computed
         * in the 1 kHz main loop and published through s_ccr[]. */
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, s_ccr[0]);
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, s_ccr[1]);
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_3, s_ccr[2]);
    }
}
/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */
void Openloop_Init(void)
{
    int cal;
    uint16_t i;

    en_init();
    uart2_init();

    log_printf("\r\n\r\n");
    log_printf("==========================================\r\n");
    log_printf("  OPEN LOOP SPIN TEST  (SimpleFOC mini V2)\r\n");
    log_printf("==========================================\r\n");

    tim1_pwm_init();
    tim3_encoder_init();
    adc1_init();

    /* start with 50 % duty = zero voltage, in case the sine generator
     * is enabled before the first ISR fires */
    s_ccr[0] = PWM_CENTER; s_ccr[1] = PWM_CENTER; s_ccr[2] = PWM_CENTER;
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, PWM_CENTER);
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, PWM_CENTER);
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_3, PWM_CENTER);

    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&s_htim1);

    HAL_TIM_Encoder_Start(&s_htim3, TIM_CHANNEL_ALL);

    log_printf("PWM   : TIM1 PA8/PA9/PA10  20kHz  center aligned\r\n");
    log_printf("ISR   : TIM1_UP @20kHz  NVIC prio 4\r\n");
    log_printf("ENC   : TIM3 PC6/PC7  x4\r\n");
    log_printf("EN    : PC4 high\r\n");
    log_printf("------------------------------------------\r\n");

    HAL_Delay(200);

    cal = cs_zero_calibrate();
    if (cal < 0) {
        log_printf("!! ZERO CAL FAILED\r\n");
        return;
    }
    for (i = 0u; i < 3u; i++) {
        log_printf("  CS%u zero = %u\r\n", (unsigned)(i + 1u), (unsigned)s_zero[i]);
    }
    if (cal > 0) {
        log_printf("!! %d channel(s) out of range\r\n", cal);
        return;
    }

    /* --- test parameters --- */
    s_theta  = 0.0f;
    s_freq   = OL_F_START;
    s_dtheta = (2.0f * FOC_2PI_LOCAL * s_freq) / CURRENT_LOOP_HZ;
    s_aborted = 0u;

    log_printf("------------------------------------------\r\n");
    log_printf("f_elec = %.1f Hz  ->  %.1f rpm mechanical\r\n",
               OL_F_ELEC_HZ, OL_F_ELEC_HZ * 60.0f / (float)MOTOR_POLE_PAIRS);
    log_printf("amplitude = %.0f %% Vdc   target = %u rev\r\n", (float)(OL_AMPLITUDE * 100.0f), (unsigned)OL_TARGET_REV);
    log_printf("abort if |I| > %.2f A\r\n", OL_I_ABORT);
    log_printf("------------------------------------------\r\n");
    log_printf("SPINNING...\r\n");
    log_printf("------------------------------------------\r\n");

    s_last_enc  = (int16_t)TIM3->CNT;
    s_total_enc = 0;
    s_start_tick = HAL_GetTick();
    s_running = 1u;
}

void Openloop_Loop(void)
{
    static uint32_t last_print = 0;
    static uint32_t last_sine  = 0;
    static uint32_t last_isr   = 0;
    uint16_t raw[3];
    float ia, ib, ic;
    uint32_t elapsed;
    int16_t cnt;

    elapsed = HAL_GetTick() - s_start_tick;

    /* frequency soft start: ramp from OL_F_START to OL_F_ELEC_HZ so the
     * rotor can follow the field without losing sync at t = 0 */
    if (s_running) {
        float t = (float)elapsed;
        float f;
        if (t >= (float)OL_F_RAMP_MS) {
            f = OL_F_ELEC_HZ;
        } else {
            f = OL_F_START + (OL_F_ELEC_HZ - OL_F_START) * (t / (float)OL_F_RAMP_MS);
        }
        s_freq   = f;
        s_dtheta = (2.0f * FOC_2PI_LOCAL * f) / 1000.0f;
    }

    /* ---- 1 kHz sine generation (moved OUT of the 20 kHz ISR) ----
     * The ISR used to call sinf() three times, which takes longer than the
     * 50 us interrupt period on this part and effectively locks the CPU up.
     * Now the waveform is computed here at 1 kHz (1000 points per electrical
     * revolution at 1 Hz - plenty) and only published to the ISR. */
    if (s_running && (HAL_GetTick() - last_sine) >= 1u) {
        float th, a1, a2, a3;
        float amp = OL_AMPLITUDE;

        last_sine = HAL_GetTick();

        th = s_theta;
        s_theta = th + s_dtheta;
        if (s_theta >= FOC_2PI_LOCAL) {
            s_theta -= FOC_2PI_LOCAL;
        }

        a1 = 0.5f + amp * sinf(th);
        a2 = 0.5f + amp * sinf(th - 2.0943951f);
        a3 = 0.5f + amp * sinf(th + 2.0943951f);

        s_ccr[0] = (uint16_t)(a1 * (float)PWM_ARR);
        s_ccr[1] = (uint16_t)(a2 * (float)PWM_ARR);
        s_ccr[2] = (uint16_t)(a3 * (float)PWM_ARR);
    }

    /* ---- run window over -> stop and report ---- */
    if (s_running) {
        int16_t nowenc = (int16_t)TIM3->CNT;
        s_total_enc += (int16_t)(nowenc - s_last_enc);
        s_last_enc = nowenc;
    }

    if (s_running && (((s_total_enc >= OL_TARGET_CNT) || (-s_total_enc >= OL_TARGET_CNT)) || elapsed >= OL_TIMEOUT_MS)) {
        s_running = 0u;
        s_ccr[0] = PWM_CENTER; s_ccr[1] = PWM_CENTER; s_ccr[2] = PWM_CENTER;
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, PWM_CENTER);
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, PWM_CENTER);
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_3, PWM_CENTER);
        log_printf("------------------------------------------\r\n");
        log_printf("DONE. counts=%ld  revs=%ld.%02ld  duty -> 50%%\r\n", (long)s_total_enc, (long)(s_total_enc / (int32_t)ENC_CPR), (long)(((s_total_enc % (int32_t)ENC_CPR) * 100) / (int32_t)ENC_CPR));
        log_printf("Reset to run again.\r\n");
        log_printf("------------------------------------------\r\n");
        return;
    }

    /* ---- periodic telemetry ---- */
    if ((HAL_GetTick() - last_print) < OL_PRINT_PERIOD_MS) {
        return;
    }
    last_print = HAL_GetTick();

    raw[0] = adc1_read_ch(ADC_CH_U);
    raw[1] = adc1_read_ch(ADC_CH_V);
    raw[2] = adc1_read_ch(ADC_CH_W);

    ia = cs_amp(raw[0], 0);
    ib = cs_amp(raw[1], 1);
    ic = cs_amp(raw[2], 2);

    cnt = (int16_t)TIM3->CNT;

    log_printf("t=%5lu ms | th=%4.1f | duty=%4u,%4u,%4u | I=%5d,%5d,%5d mA | CNT=%6d | ISR=%lu Hz\r\n",
               (unsigned long)elapsed,
               (double)(s_theta * 57.29578f),
               (unsigned)s_ccr[0], (unsigned)s_ccr[1], (unsigned)s_ccr[2],
               (int)(ia * 1000.0f), (int)(ib * 1000.0f), (int)(ic * 1000.0f),
               (int)cnt,
               (unsigned long)((s_isr_count - last_isr) * 10u));
    last_isr = s_isr_count;

    /* ---- over-current guard ---- */
    if (s_running) {
        float a0 = ia < 0.0f ? -ia : ia;
        float a1 = ib < 0.0f ? -ib : ib;
        float a2 = ic < 0.0f ? -ic : ic;
        float m  = a0;
        if (a1 > m) m = a1;
        if (a2 > m) m = a2;

        if (m > OL_I_ABORT) {
            s_running = 0u;
            s_aborted = 1u;
            s_ccr[0] = PWM_CENTER; s_ccr[1] = PWM_CENTER; s_ccr[2] = PWM_CENTER;
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, PWM_CENTER);
            __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, PWM_CENTER);
            __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_3, PWM_CENTER);
            HAL_GPIO_WritePin(GPIO_EN_PORT, PIN_EN, GPIO_PIN_RESET);
            log_printf("!! OVER CURRENT %.2f A - ABORTED, EN dropped\r\n", (double)m);
            return;
        }
    }
}
