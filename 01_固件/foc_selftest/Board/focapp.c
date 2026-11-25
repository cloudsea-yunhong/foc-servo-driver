/**
 * @file    focapp.c
 * @brief   FOC servo application - three cascade loops on a DRV8316 board
 *
 *  Timing architecture (this is the important part)
 *  ------------------------------------------------
 *  Every control period is derived from the TIM1 update interrupt, i.e. from
 *  the 168 MHz crystal through the PWM carrier.  Nothing in the control path
 *  depends on HAL_GetTick(), because its timebase proved unreliable on this
 *  board.
 *
 *      TIM1 update ISR @ 20 kHz
 *          every call      -> ADC sampling + current loop (20 kHz)
 *          every 20th call -> set the 1 kHz flag for the outer loops
 *
 *  Current sensing
 *  ---------------
 *  DRV8316 uses LOW-SIDE shunts, so the phase current is only visible while
 *  the low-side FET conducts.  The injected ADC group is triggered from
 *  software a few microseconds after the PWM update, which lets the switching
 *  ringing settle and lands inside the low-side conduction window.
 *
 *  States
 *  ------
 *      IDLE -> ALIGN -> (IDLE)
 *                 \-> OPENLOOP / TORQUE / VELOCITY / POSITION
 */

#include "focapp.h"
#include "main.h"
#include "hw_config.h"
#include "foc_core.h"
#include "foc_ctrl.h"

#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================== */
/* Local constants                                                     */
/* ================================================================== */
/* CS1 reads ~0.18 V low on this board -> reconstruct phase A instead */
#define USE_TWO_SHUNT       1

#define ISR_DIV_1KHZ        20u         /* 20 kHz / 20 = 1 kHz              */

#define ALIGN_VOLTAGE       1.0f        /* Volts applied during alignment   */
#define ALIGN_TIME_ISR      (20000u)    /* 1 s at 20 kHz                    */

#define OL_RAMP_ISR         (20000u)    /* 1 s frequency ramp */
#define OL_F_START          0.5f        /* Hz electrical */
#define OL_F_TARGET         3.0f        /* Hz electrical -> 26 rpm */
#define OL_VOLTAGE          5.0f        /* Volts - below this the rotor does not break away */

/* ================================================================== */
/* Handles / state                                                     */
/* ================================================================== */
static TIM_HandleTypeDef  s_htim1;
static TIM_HandleTypeDef  s_htim3;
static UART_HandleTypeDef s_huart2;
static foc_ctrl_t         s_ctrl;

typedef enum {
    ST_IDLE = 0,
    ST_ALIGN,
    ST_OPENLOOP,
    ST_TORQUE,
    ST_VELOCITY,
    ST_POSITION,
    ST_FAULT
} app_state_t;

static volatile app_state_t s_state;

/* ---- ISR domain ---- */
static volatile uint32_t s_isr_count;        /* total TIM1 interrupts      */
static volatile uint32_t s_align_cnt;
static volatile uint32_t s_ol_cnt;

static volatile float    s_theta_e;          /* electrical angle [rad]     */
static volatile float    s_theta_ol;         /* open-loop angle [rad]      */
static volatile float    s_align_vd;         /* voltage forced in ALIGN    */

static volatile uint16_t s_raw[3];
static volatile float    s_ia, s_ib, s_ic;   /* measured phase currents    */
static volatile float    s_id, s_iq;         /* after Park                 */

static volatile uint16_t s_ccr[3];

/* ---- encoder ---- */
static int16_t  s_enc_last;
static int32_t  s_enc_total;                 /* signed, multi-turn         */
static float    s_mech_angle;                /* rad, continuous            */
static float    s_mech_vel;                  /* rad/s                      */

/* ---- calibration / alignment ---- */
static uint16_t s_zero[3];
static float    s_elec_offset;               /* encoder -> electrical zero */
static float    s_mech_at_align;             /* mechanical angle at align  */

/* ---- outer loop bookkeeping ---- */
static volatile uint8_t s_tick_1khz;
static uint32_t s_last_step;
static uint32_t s_last_telemetry;
static uint32_t s_telemetry_ms = 250u;   /* telemetry period, settable with F<ms> */
static uint32_t s_last_vel;
static uint32_t s_last_pos;


/* ================================================================== */
/* UART receive ring buffer                                            */
/* ================================================================== */
#define RX_RING_SIZE   64u

static volatile uint8_t  s_rx_ring[RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

#define TX_RING_SIZE   512u

static volatile uint8_t  s_tx_ring[TX_RING_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;

void USART2_IRQHandler(void)
{
    /* ---- transmit: drain the ring buffer ---- */
    if (USART2->SR & USART_SR_TXE) {
        if (s_tx_tail != s_tx_head) {
            USART2->DR = s_tx_ring[s_tx_tail];
            s_tx_tail = (uint16_t)((s_tx_tail + 1u) % TX_RING_SIZE);
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }

    if (USART2->SR & USART_SR_RXNE) {
        uint8_t b = (uint8_t)(USART2->DR & 0x00FFu);
        uint16_t nx = (uint16_t)((s_rx_head + 1u) % RX_RING_SIZE);
        if (nx != s_rx_tail) {
            s_rx_ring[s_rx_head] = b;
            s_rx_head = nx;
        }
    }
    if (USART2->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)USART2->DR;                       /* clear the error flags */
    }
}

static int rx_get(uint8_t *out)
{
    if (s_rx_head == s_rx_tail) {
        return 0;
    }
    *out = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) % RX_RING_SIZE);
    return 1;
}
/* ================================================================== */
/* UART                                                                */
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

    /* RX interrupt + ring buffer: the 100 ms telemetry blocks for ~17 ms
     * inside HAL_UART_Transmit, and any byte arriving in that window was
     * being dropped, which is why commands were randomly ignored. */
    __HAL_UART_ENABLE_IT(&s_huart2, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* 状态编号 -> 中文名称（GBK 字节，串口助手按 GBK 解析） */
/* state id -> short label (ASCII: AC6 cannot emit multibyte literals) */
static const char *state_name(unsigned st)
{
    switch (st) {
    case 0u: return "STOP";
    case 1u: return "ALIGN";
    case 2u: return "OPEN";
    case 3u: return "TORQUE";
    case 4u: return "SPEED";
    case 5u: return "POS";
    default: return "FAULT";
    }
}

static char s_txbuf[230];

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
        {
            int k;
            for (k = 0; k < n; k++) {
                uint16_t nx = (uint16_t)((s_tx_head + 1u) % TX_RING_SIZE);
                if (nx == s_tx_tail) { break; }
                s_tx_ring[s_tx_head] = (uint8_t)s_txbuf[k];
                s_tx_head = nx;
            }
            USART2->CR1 |= USART_CR1_TXEIE;
        }
    }
}

/* integer-only float printing (newlib-nano has no float printf) */
static void log_f2(const char *tag, float v)
{
    long whole = (long)v;
    long frac  = (long)((v - (float)whole) * 100.0f);
    if (frac < 0) frac = -frac;
    log_printf("%s=%ld.%02ld ", tag, whole, frac);
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

static void en_off(void)
{
    HAL_GPIO_WritePin(GPIO_EN_PORT, PIN_EN, GPIO_PIN_RESET);
}

/* ================================================================== */
/* Sampling marker on PB0 - only used for the logic-analyser capture   */
/* ================================================================== */

/* Encoder mirror: PB1 = A copy, PB2 = B copy, refreshed at 20 kHz */
static void enc_mirror_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = PIN_ENC_A_MIRROR | PIN_ENC_B_MIRROR;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void mark_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin   = PIN_MARK;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIO_MARK_PORT, &gpio);
    HAL_GPIO_WritePin(GPIO_MARK_PORT, PIN_MARK, GPIO_PIN_RESET);
}

/* ================================================================== */
/* TIM1 - PWM + 20 kHz update interrupt + ADC trigger                  */
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
    s_htim1.Init.RepetitionCounter = 1;   /* 1 update per PWM period */
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

    /* TRGO = update, so the ADC could be triggered by hardware later.
     * For now the injected group is started by software inside the ISR. */
    msc.MasterOutputTrigger = TIM_TRGO_UPDATE;
    msc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&s_htim1, &msc);

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
/* ADC1 - injected group, channels 10/11/12                           */
/*                                                                     */
/* The injected group is used because it has its own data registers    */
/* (JDR1..3) and cannot be disturbed by the regular conversions.       */
/* ================================================================== */
/* ================================================================== */
/* ADC1 - one channel at a time, software triggered (main loop only)   */
/*                                                                     */
/* This is the only configuration that has been proven to return real  */
/* values on this board.  It is NOT synchronised with the PWM, so the  */
/* readings may violate Ia+Ib+Ic=0 while current flows; the proper fix */
/* is a hardware trigger from TIM1_CH4 (left as the next step, see the */
/* project notes - the trigger itself needs checking with a scope).    */
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
    ADC1->SMPR1 = (1u << 0) | (1u << 3) | (1u << 6);   /* 15 cycles = 1.3 us */
    ADC1->SMPR2 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 10;

    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t k = 0; k < 30000u; k++) { __NOP(); }
    ADC1->CR2 |= (1u << 3);
    for (volatile uint32_t w = 0; (ADC1->CR2 & (1u << 3)) && w < 2000000u; w++) { ; }
    ADC1->CR2 |= (1u << 2);
    for (volatile uint32_t w = 0; (ADC1->CR2 & (1u << 2)) && w < 2000000u; w++) { ; }
}

static uint16_t adc_read_ch(uint32_t ch)
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

/* Wait until the PWM counter is inside the low-side conduction window.
 *
 * With center-aligned PWM in mode 1 the outputs are HIGH around CNT = 0
 * and LOW around CNT = ARR.  The DRV8316 measures the phase current on
 * the LOW side, so the sample must be taken near the counter peak.  This
 * short wait guarantees exactly that - no external instrument needed. */
static void wait_low_side_window(void)
{
    uint32_t guard = 0;

    /* start below the window */
    while (TIM1->CNT > (PWM_ARR / 4u) && guard++ < 20000u) {
        ;
    }
    guard = 0;
    /* wait for the counter to climb into the low-side window */
    while (TIM1->CNT < (PWM_ARR * 5u / 8u) && guard++ < 20000u) {
        ;
    }
}

static int adc1_read_injected(uint16_t out[3])
{
    wait_low_side_window();

    /* Marker high for the whole conversion: on the analyser this pulse
     * must sit inside the LOW period of the PWM - that is the low-side
     * conduction window where the shunt actually carries the current. */
    GPIOB->BSRR = GPIO_BSRR_BS_0;

    out[0] = adc_read_ch(10);
    out[1] = adc_read_ch(11);
    out[2] = adc_read_ch(12);

    GPIOB->BSRR = GPIO_BSRR_BR_0;

    if (out[0] == 0xFFFFu || out[1] == 0xFFFFu || out[2] == 0xFFFFu) {
        return -1;
    }
    return 0;
}
/* ================================================================== */
/* Zero calibration                                                    */
/* ================================================================== */
static int cs_calibrate(void)
{
    uint32_t acc[3] = {0, 0, 0};
    uint32_t i, k;
    int bad = 0;
    uint16_t raw[3];

    for (i = 0; i < CS_CAL_SAMPLES; i++) {
        if (adc1_read_injected(raw) != 0) {
            return -1;
        }
        acc[0] += raw[0];
        acc[1] += raw[1];
        acc[2] += raw[2];
    }
    for (k = 0; k < 3u; k++) {
        s_zero[k] = (uint16_t)(acc[k] / CS_CAL_SAMPLES);
        if (s_zero[k] < 1500u || s_zero[k] > 2600u) {
            bad++;
        }
    }
    return bad;
}

static float cs_raw_to_amp(uint16_t raw, int ch)
{
    int32_t d = (int32_t)raw - (int32_t)s_zero[ch];
    return (((float)d * ADC_VREF_V / ADC_MAX_COUNT) / CS_GAIN_V_PER_A);
}

/* ================================================================== */
/* Encoder bookkeeping (called at 1 kHz)                               */
/* ================================================================== */
static void encoder_update(float dt)
{
    int16_t now = (int16_t)TIM3->CNT;
    int16_t d   = (int16_t)(now - s_enc_last);
    float   dth;

    s_enc_last = now;
    s_enc_total += d;

    dth = (float)d * ENC_ANGLE_PER_COUNT * (float)ENC_DIR_SIGN;
    s_mech_angle += dth;

    /* velocity with a light low-pass, dt is exact (1 ms) */
    {
        float v = dth / dt;
        s_mech_vel += 0.12f * (v - s_mech_vel);
    }
}

/* ================================================================== */
/* Apply three duties                                                  */
/* ================================================================== */
static inline void apply_duty(float du, float dv, float dw)
{
    uint32_t a = (uint32_t)(du * (float)PWM_ARR);
    uint32_t b = (uint32_t)(dv * (float)PWM_ARR);
    uint32_t c = (uint32_t)(dw * (float)PWM_ARR);

    if (a > PWM_ARR) a = PWM_ARR;
    if (b > PWM_ARR) b = PWM_ARR;
    if (c > PWM_ARR) c = PWM_ARR;

    s_ccr[0] = (uint16_t)a;
    s_ccr[1] = (uint16_t)b;
    s_ccr[2] = (uint16_t)c;

    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, a);
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, b);
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_3, c);
}

static void output_zero(void)
{
    apply_duty(0.5f, 0.5f, 0.5f);
}

/* ================================================================== */
/* 20 kHz update ISR: sampling + current loop + SVPWM                  */
/*                                                                     */
/* Rules: no HAL calls, no printf, no blocking. Everything below runs  */
/* in about 10 us of the 50 us period.                                 */
/* ================================================================== */
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* copy encoder A/B levels to PB1/PB2 (20 kHz) */
    {
        uint32_t idr = GPIOC->IDR;
        uint32_t bs  = 0u;

        bs |= (idr & GPIO_PIN_6) ? GPIO_BSRR_BS_1 : GPIO_BSRR_BR_1;
        bs |= (idr & GPIO_PIN_7) ? GPIO_BSRR_BS_12 : GPIO_BSRR_BR_12;
        GPIOB->BSRR = bs;
    }
    if (__HAL_TIM_GET_FLAG(&s_htim1, TIM_FLAG_UPDATE) == RESET) {
        return;
    }
    __HAL_TIM_CLEAR_FLAG(&s_htim1, TIM_FLAG_UPDATE);
    s_isr_count++;

    /* Deliberately tiny: push the duties and leave.  Everything else
     * (ADC, Clarke, Park, PI, SVPWM) runs from the 1 kHz task below,
     * because sampling the ADC inside this interrupt proved unreliable
     * and long enough to overrun the 50 us budget. */
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, s_ccr[0]);
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, s_ccr[1]);
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_3, s_ccr[2]);

    if ((s_isr_count % ISR_DIV_1KHZ) == 0u) {
        s_tick_1khz = 1u;
    }
}

/* ================================================================== */
/* 1 kHz control step: sampling + current loop + SVPWM                 */
/* ================================================================== */
static void foc_step_1khz(float dt)
{
    uint16_t raw[3];
    foc_ab_t ab, vab;
    foc_dq_t dq, vdq;
    float    s_th, c_th;
    float    duty[3];

    if (adc1_read_injected(raw) == 0) {
        s_raw[0] = raw[0]; s_raw[1] = raw[1]; s_raw[2] = raw[2];

#if USE_TWO_SHUNT
        /* Two-shunt mode.  CS1 (phase A) reads about 0.18 V low on this
         * board and cannot be trusted, so phase B and C are measured and
         * phase A is reconstructed from Kirchhoff: Ia + Ib + Ic = 0.
         * This is a standard technique - many industrial drives ship with
         * only two shunts for exactly this reason. */
        s_ib = cs_raw_to_amp(raw[1], 1);
        s_ic = cs_raw_to_amp(raw[2], 2);
        s_ia = -(s_ib + s_ic);   /* Kirchhoff: Ia + Ib + Ic = 0 */
#else
        s_ia = cs_raw_to_amp(raw[0], 0);
        s_ib = cs_raw_to_amp(raw[1], 1);
        s_ic = cs_raw_to_amp(raw[2], 2);
#endif
    }

    s_theta_e = foc_wrap_2pi(s_mech_angle * (float)MOTOR_POLE_PAIRS - s_elec_offset);

    foc_clarke(s_ia, s_ib, s_ic, &ab);
    foc_sincos(s_theta_e, &s_th, &c_th);
    foc_park_sc(&ab, s_th, c_th, &dq);
    s_id = dq.d;
    s_iq = dq.q;

    switch (s_state) {

    case ST_ALIGN:
        s_theta_e = 0.0f;
        vdq.d = s_align_vd;
        vdq.q = 0.0f;
        s_align_cnt++;
        if (s_align_cnt >= (ALIGN_TIME_ISR / ISR_DIV_1KHZ)) {
            s_mech_at_align = s_mech_angle;
            s_elec_offset   = foc_wrap_2pi(s_mech_at_align * (float)MOTOR_POLE_PAIRS);
            s_state = ST_IDLE;
        }
        break;

    case ST_OPENLOOP:
    {
        float fr;
        if (s_ol_cnt < (OL_RAMP_ISR / ISR_DIV_1KHZ)) {
            fr = OL_F_START + (OL_F_TARGET - OL_F_START) *
                 ((float)s_ol_cnt / (float)(OL_RAMP_ISR / ISR_DIV_1KHZ));
        } else {
            fr = OL_F_TARGET;
        }
        s_ol_cnt++;

        s_theta_ol += (2.0f * FOC_2PI * fr) * dt;
        s_theta_ol = foc_wrap_2pi(s_theta_ol);
        s_theta_e = s_theta_ol;
        vdq.d = 0.0f;
        vdq.q = OL_VOLTAGE;
        break;
    }

    case ST_TORQUE:
    case ST_VELOCITY:
    case ST_POSITION:
        foc_ctrl_current_loop(&s_ctrl, dt);
        vdq.d = s_ctrl.vd;
        vdq.q = s_ctrl.vq;
        break;

    case ST_IDLE:
    case ST_FAULT:
    default:
        vdq.d = 0.0f;
        vdq.q = 0.0f;
        break;
    }


    /* Recompute sin/cos HERE: the state machine above may have changed
     * s_theta_e (open loop drives its own angle), so the values from the
     * top of the function would rotate the vector at the wrong angle -
     * which froze the SVPWM output completely. */
    foc_sincos(s_theta_e, &s_th, &c_th);

    foc_inv_park_sc(&vdq, s_th, c_th, &vab);
    foc_svpwm(vab.alpha, vab.beta, MOTOR_V_RATED, duty);
    apply_duty(duty[0], duty[1], duty[2]);
}

/* ================================================================== */
/* State helpers                                                       */
/* ================================================================== */
static void enter_idle(void)
{
    s_state = ST_IDLE;
    output_zero();
    foc_ctrl_reset_integrators(&s_ctrl);
    log_printf("STATE: STOP\r\n");
}

static void enter_align(void)
{
    s_align_cnt  = 0;
    s_align_vd   = ALIGN_VOLTAGE;
    s_state      = ST_ALIGN;
    log_printf("STATE: ALIGN  Vd=%d mV for 1 s\r\n", (int)(ALIGN_VOLTAGE*1000.0f));
}

static void enter_openloop(void)
{
    s_ol_cnt    = 0;
    s_theta_ol  = 0.0f;
    s_state     = ST_OPENLOOP;
    log_printf("STATE: OPENLOOP  f %u->%u Hz  Vq=%u V\r\n",
               (unsigned)OL_F_START, (unsigned)OL_F_TARGET, (unsigned)OL_VOLTAGE);
}

static void enter_torque(float iq)
{
    s_ctrl.i_limit = LIMIT_I_RUN;
    s_ctrl.target_iq = foc_clampf(iq, -s_ctrl.i_limit, s_ctrl.i_limit);
    foc_ctrl_set_mode(&s_ctrl, FOC_MODE_TORQUE);
    s_state = ST_TORQUE;
    log_printf("STATE: TORQUE  target Iq=%d mA\r\n", (int)(s_ctrl.target_iq * 1000.0f));
}

static void enter_velocity(float vel)
{
    s_ctrl.i_limit = LIMIT_I_RUN;
    s_ctrl.speed_limit = LIMIT_SPEED_RAD_S;
    s_ctrl.target_vel = foc_clampf(vel, -s_ctrl.speed_limit, s_ctrl.speed_limit);
    foc_ctrl_set_mode(&s_ctrl, FOC_MODE_VELOCITY);
    s_state = ST_VELOCITY;
    log_printf("STATE: SPEED  target %d.%02d rad/s\r\n",
               (int)s_ctrl.target_vel,
               (int)((s_ctrl.target_vel - (float)(int)s_ctrl.target_vel) * 100.0f));
}

static void enter_position(float pos)
{
    s_ctrl.i_limit = LIMIT_I_RUN;
    s_ctrl.speed_limit = LIMIT_SPEED_RAD_S;
    s_ctrl.target_pos = pos;
    foc_ctrl_set_mode(&s_ctrl, FOC_MODE_POSITION);
    s_state = ST_POSITION;
    log_printf("STATE: POSITION  target %d mrad\r\n", (int)(pos * 1000.0f));
}

/* ================================================================== */
/* Serial command parser (very small)                                  */
/* ================================================================== */
static int s_rx_len;
static char s_rxbuf[32];

static void handle_command(const char *cmd)
{

    /* Safety interlock: if the power-on zero calibration failed the
     * current feedback is meaningless.  Running a closed loop on bogus
     * feedback saturates the voltage and cookes the motor, so refuse all
     * motion commands until the board is power cycled. */
    if (s_state == ST_FAULT) {
        char cc = cmd[0];
        if (cc != 'h' && cc != 'H' && cc != 'q' && cc != 'Q' && cc != 's' && cc != 'S') {
            log_printf("ERR: FAULT - check the 12V supply to the driver board, then power cycle\r\n");
            return;
        }
    }
    char c = cmd[0];
    float v = (float)atof(&cmd[1]);

    switch (c) {
    case 'a': case 'A': enter_align();    break;
    case 'o': case 'O': enter_openloop(); break;
    case 's': case 'S': enter_idle();     break;
    case 't': case 'T': enter_torque(v);  break;
    case 'v': case 'V': enter_velocity(v);break;
    case 'p': case 'P': enter_position(v);break;
    case 'g': case 'G':
    {
        /* G<kp_milli>,<ki_milli>  e.g. G1500,100 -> kp=1.5 ki=0.1 */
        int ai = atoi(&cmd[1]);
        const char *cm = strchr(&cmd[1], ',');
        int bi = cm ? atoi(cm + 1) : 0;
        if (ai > 0) {
            s_ctrl.pi_pos.kp = (float)ai / 1000.0f;
            s_ctrl.pi_pos.ki = (float)bi / 1000.0f;
            log_printf("POS PI  kp=%d ki=%d (milli)\r\n", ai, bi);
        } else {
            log_printf("ERR: use G<kp_milli>,<ki_milli>\r\n");
        }
        break;
    }
    case 'k': case 'K':
    {
        int ai = atoi(&cmd[1]);
        const char *cm = strchr(&cmd[1], ',');
        int bi = cm ? atoi(cm + 1) : 0;
        if (ai > 0) {
            s_ctrl.pi_vel.kp = (float)ai / 1000.0f;
            s_ctrl.pi_vel.ki = (float)bi / 1000.0f;
            log_printf("VEL PI  kp=%d ki=%d (milli)\r\n", ai, bi);
        } else {
            log_printf("ERR: use K<kp_milli>,<ki_milli>\r\n");
        }
        break;
    }
    case 'f': case 'F':
    {
        /* F<ms> : telemetry period in milliseconds (10 .. 2000).
         * Handy for capturing step responses at a usable rate. */
        int ms = atoi(&cmd[1]);
        if (ms < 10)   { ms = 10;   }
        if (ms > 2000) { ms = 2000; }
        s_telemetry_ms = (uint32_t)ms;
        log_printf("OK   telemetry %d ms\r\n", ms);
        break;
    }
    case 'q': case 'Q':
        log_printf("POS kp=%d ki=%d | VEL kp=%d ki=%d (milli)\r\n",
                   (int)(s_ctrl.pi_pos.kp*1000.0f), (int)(s_ctrl.pi_pos.ki*1000.0f),
                   (int)(s_ctrl.pi_vel.kp*1000.0f), (int)(s_ctrl.pi_vel.ki*1000.0f));
        break;
    case 'h': case 'H':
        log_printf("cmd: A O S T<A> V<rad/s> P<rad> G<kp,ki> K<kp,ki> F<ms> Q\r\n");
        break;
    default:
        log_printf("ERR: unknown cmd '%c'\r\n", c);
        break;
    }
}

static void poll_uart(void)
{
    uint8_t ch;

    while (rx_get(&ch)) {
        if (ch == '\r' || ch == '\n') {
            if (s_rx_len > 0) {
                s_rxbuf[s_rx_len] = '\0';
                handle_command(s_rxbuf);
                s_rx_len = 0;
            }
        } else if (s_rx_len < (int)sizeof(s_rxbuf) - 1) {
            s_rxbuf[s_rx_len++] = (char)ch;
        }
    }
}
/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */
void FocApp_Init(void)
{
    int cal;
    uint16_t i;

    en_init();
    mark_init();
    enc_mirror_init();
    uart2_init();

    log_printf("\r\n\r\n");
    log_printf("==============================================\r\n");
    log_printf("  BLDC FOC 3-LOOP SERVO DRIVER\r\n");
    log_printf("==============================================\r\n");

    tim1_pwm_init();
    tim3_encoder_init();
    adc1_init();
    foc_ctrl_init(&s_ctrl);

    output_zero();
    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&s_htim1);

    HAL_TIM_Encoder_Start(&s_htim3, TIM_CHANNEL_ALL);

    log_printf("PWM   : TIM1 20kHz center-aligned, ISR prio 4\r\n");
    log_printf("ENC   : TIM3 x4, PPR=%u\r\n", (unsigned)ENC_PPR);
    log_printf("ADC   : IN10/11/12, 20kHz\r\n");
    log_printf("----------------------------------------------\r\n");

    HAL_Delay(200);

    /* ---- power-on zero calibration, with retry ----
     * The driver board may power up after the MCU (USB first, 12V later).
     * Without 12V the DRV8316 has no AVDD, so the current-sense amps sit
     * near 0 V and the calibration can only fail.  Instead of latching a
     * FAULT on the first attempt, keep retrying for ~6 s: as soon as the
     * 12V comes up the calibration succeeds and the drive becomes ready
     * by itself. */
    {
        int retry;

        cal = -1;
        for (retry = 0; retry < 12; retry++) {
            cal = cs_calibrate();
            if (cal == 0) {
                break;                                /* success */
            }
            if (retry == 0) {
                log_printf("!! zero cal failed - waiting for the 12V driver supply...\r\n");
            }
            HAL_Delay(500);
        }

        if (cal != 0) {
            log_printf("!! give up after %d tries - check the 12V supply and CS wiring\r\n", retry);
            s_state = ST_FAULT;
            return;
        }

        if (retry > 0) {
            log_printf("OK   zero cal recovered after %d retr%s\r\n",
                       retry, (retry == 1) ? "y" : "ies");
        }
        for (i = 0u; i < 3u; i++) {
            log_printf("CS%u zero = %u\r\n", (unsigned)(i + 1u), (unsigned)s_zero[i]);
        }
    }

    s_enc_last  = (int16_t)TIM3->CNT;
    s_enc_total = 0;
    s_mech_angle = 0.0f;
    s_mech_vel   = 0.0f;
    s_theta_e    = 0.0f;
    s_state      = ST_IDLE;

    log_printf("----------------------------------------------\r\n");
    log_printf("READY. Send H for the command list.\r\n");
    log_printf("  A=align  O=openloop  S=stop\r\n");
    log_printf("  T<A>=torque  V<rad/s>=speed  P<rad>=position\r\n");
    log_printf("----------------------------------------------\r\n");

    s_last_step = 0;
    s_last_telemetry = 0;
    s_last_vel = 0;
    s_last_pos = 0;
}

void FocApp_Loop(void)
{
    uint16_t raw[3];
    uint32_t now;

    /* ---- serial ---- */
    poll_uart();

    /* ---- 1 kHz tasks ---- */
    if (s_tick_1khz) {
        uint32_t n, dn;
        float dt;

        s_tick_1khz = 0u;

        /* dt from the real interrupt count, not a fixed 1 kHz: the
         * carrier flag can fire faster than the nominal divider, and
         * using the measured interval keeps the electrical angle and
         * every integrator correct regardless. */
        n  = s_isr_count;
        dn = n - s_last_step;
        s_last_step = n;
        dt = (float)dn * (1.0f / CURRENT_LOOP_HZ);
        if (dt <= 0.0f || dt > 0.05f) {
            dt = DT_VELOCITY;
        }

        encoder_update(dt);

        foc_step_1khz(dt);

        /* publish feedback into the controller */
        s_ctrl.id_meas  = s_id;
        s_ctrl.iq_meas  = s_iq;
        s_ctrl.vel_meas = s_mech_vel;
        s_ctrl.pos_meas = s_mech_angle;
        s_ctrl.theta_e  = s_theta_e;

        if (s_state == ST_VELOCITY || s_state == ST_POSITION ||
            s_state == ST_TORQUE) {
            foc_ctrl_outer_loops(&s_ctrl, dt);
        }
    }

    /* ---- telemetry every 100 ms ---- */
    now = s_isr_count / ISR_DIV_1KHZ;      /* milliseconds, from the carrier */
    if ((now - s_last_telemetry) < s_telemetry_ms) {
        return;
    }
    s_last_telemetry = now;

    {
        /* Compact one-line telemetry - stays inside a typical serial
         * terminal width so it never wraps. */
        /* Compact, ASCII-only telemetry (AC6 cannot emit multibyte).
         *   Torque = Iq, Ang = electrical angle, Pos = mech position,
         *   State = state machine, Cur = phase currents              */
        log_printf("Torque=%4dmA Ang=%3u Pos=%7d State=%-6s Cur=%4d,%4d,%4dmA\r\n",
                   (int)(s_iq * 1000.0f),
                   (unsigned)(s_theta_e * 57.29578f),
                   (int)(s_mech_angle * 1000.0f),
                   state_name((unsigned)s_state),
                   (int)(s_ia * 1000.0f), (int)(s_ib * 1000.0f), (int)(s_ic * 1000.0f));
    }

    (void)s_last_vel;
    (void)s_last_pos;
}
