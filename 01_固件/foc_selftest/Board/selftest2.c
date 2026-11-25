/**
 * @file    selftest2.c
 * @brief   Hardware self-test for the SimpleFOC mini V2 (DRV8316) board
 *
 *  What it does
 *  ------------
 *   1. drives EN high so the DRV8316 wakes up (its nSLEEP)
 *   2. starts TIM1 3-phase PWM at 20 kHz, 50 % duty  -> zero phase voltage
 *   3. starts TIM3 in encoder mode on PC6/PC7
 *   4. configures ADC1 on PC0 / PC1 / PC2 (IN10 / IN11 / IN12)
 *   5. runs the POWER-ON ZERO CALIBRATION (64 samples per channel)
 *   6. prints raw counts, zero counts and computed currents over USART2
 *
 *  Expected values on this board (measured with a multimeter):
 *	 AVDD (driver)      = 3.32 V
 *	 CS zero voltage    = 1.66 V
 *	 F407 ADC reference = 3.30 V
 *	 => zero ADC count  ~= 1.66 / 3.30 * 4095 = 2060
 *
 *  Current conversion:
 *	 I[A] = (adc_count - zero_count) * ADC_VREF_V / 4095 / CS_GAIN_V_PER_A
 */

#include "selftest2.h"
#include "main.h"
#include "hw_config.h"

#include <stdarg.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Handles                                                             */
/* ------------------------------------------------------------------ */
static TIM_HandleTypeDef  s_htim1;      /* PWM   */
static TIM_HandleTypeDef  s_htim3;      /* encoder */
static UART_HandleTypeDef s_huart2;

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static uint16_t s_zero[3];              /* per-channel zero counts    */
static int16_t  s_last_cnt;
static int32_t  s_total_cnt;
static uint32_t s_tick;

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

static char s_txbuf[220];

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
/* EN - wakes the DRV8316 (its nSLEEP input)                           */
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

    /* active high: the chip stays asleep until this goes high */
    HAL_GPIO_WritePin(GPIO_EN_PORT, PIN_EN, GPIO_PIN_SET);
}

/* ================================================================== */
/* TIM1 - 3 phase PWM, 20 kHz center aligned, 50 % duty                */
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
    s_htim1.Init.RepetitionCounter = 0;
    s_htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&s_htim1);

    csc.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&s_htim1, &csc);

    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = PWM_CENTER;          /* 50 % -> zero phase voltage */
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
}

/* ================================================================== */
/* TIM3 - quadrature encoder, x4                                       */
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
/* ADC1 - IN10 / IN11 / IN12 on PC0 / PC1 / PC2, software triggered    */
/*                                                                     */
/* The HAL_ADC driver is not present in this project, and the earlier  */
/* multi-channel scan attempt was unreliable, so each channel is       */
/* converted on its own with SCAN disabled.  Slow but bullet proof -   */
/* good enough for a self test.  The 20 kHz FOC loop will use the      */
/* injected group + DMA instead.                                       */
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

    /* ADC clock = PCLK2 / 4 = 84 / 4 = 21 MHz (max 36 MHz) */
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

    ADC1->CR1 = 0;                          /* scan off */
    ADC1->CR2 = 0;

    /* sampling time 144 cycles for channels 10..12 (SMPR1 bits 0..8) */
    ADC1->SMPR1 = (6u << 0) | (6u << 3) | (6u << 6);
    ADC1->SMPR2 = 0;

    ADC1->SQR1 = 0;                         /* one conversion at a time */
    ADC1->SQR2 = 0;
    ADC1->SQR3 = ADC_CH_U;

    /* power on + calibrate */
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t i = 0; i < 30000u; i++) {
        __NOP();
    }

    ADC1->CR2 |= (1u << 3);                 /* RSTCAL */
    for (volatile uint32_t w = 0; (ADC1->CR2 & (1u << 3)) && w < 2000000u; w++) {
        ;
    }
    ADC1->CR2 |= (1u << 2);                 /* CAL */
    for (volatile uint32_t w = 0; (ADC1->CR2 & (1u << 2)) && w < 2000000u; w++) {
        ;
    }
}

/** Read one channel (10, 11 or 12). Returns 0xFFFF on timeout. */
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

/* ================================================================== */
/* Power-on zero calibration                                           */
/* ================================================================== */
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

        /* sanity: the zero must sit near mid-scale */
        if (s_zero[k] < 1500u || s_zero[k] > 2600u) {
            bad++;
        }
    }
    return bad;
}

/** Convert a raw count to Amps using the measured per-channel zero */
static float cs_count_to_amp(uint16_t raw, int ch)
{
    int32_t d = (int32_t)raw - (int32_t)s_zero[ch];
    return ((float)d * ADC_VREF_V / ADC_MAX_COUNT) / CS_GAIN_V_PER_A;
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */
void Selftest2_Init(void)
{
    int cal;

    /* EN first: the DRV8316 must be awake before anything else works */
    en_init();
    uart2_init();

    log_printf("\r\n\r\n");
    log_printf("==========================================\r\n");
    log_printf("  DRV8316 (SimpleFOC mini V2) SELF TEST\r\n");
    log_printf("==========================================\r\n");
    log_printf("SYSCLK = %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());

    tim1_pwm_init();
    tim3_encoder_init();
    adc1_init();

    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&s_htim1);

    HAL_TIM_Encoder_Start(&s_htim3, TIM_CHANNEL_ALL);

    log_printf("PWM : TIM1 PA8/PA9/PA10  20kHz  50%% (zero voltage)\r\n");
    log_printf("ENC : TIM3 PC6/PC7  x4  filter=%u\r\n", (unsigned)ENC_INPUT_FILTER);
    log_printf("ADC : PC0/PC1/PC2 = IN10/IN11/IN12\r\n");
    log_printf("EN  : PC4 driven high (chip awake)\r\n");
    log_printf("------------------------------------------\r\n");

    /* let the current-sense amplifiers settle after power-up */
    HAL_Delay(200);

    cal = cs_zero_calibrate();
    if (cal < 0) {
        log_printf("!! ZERO CAL FAILED (ADC timeout)\r\n");
    } else {
        log_printf("ZERO CAL done, %u samples/ch\r\n", (unsigned)CS_CAL_SAMPLES);
        log_printf("  CS1(U) zero = %u\r\n", (unsigned)s_zero[0]);
        log_printf("  CS2(V) zero = %u\r\n", (unsigned)s_zero[1]);
        log_printf("  CS3(W) zero = %u\r\n", (unsigned)s_zero[2]);
        log_printf("  expected    ~ %u  (=1.66V/3.30V*4095)\r\n",
                   (unsigned)CS_ZERO_COUNT);
        log_printf("  spread      = %u counts\r\n",
                   (unsigned)((s_zero[0] > s_zero[1] ? s_zero[0] : s_zero[1]) > s_zero[2]
                              ? ((s_zero[0] > s_zero[1] ? s_zero[0] : s_zero[1]) - s_zero[2])
                              : (s_zero[2] - (s_zero[0] > s_zero[1] ? s_zero[0] : s_zero[1]))));
        if (cal > 0) {
            log_printf("!! %d channel(s) OUT OF RANGE (expect 1500..2600)\r\n", cal);
            log_printf("!! check the CS wiring / pin order\r\n");
        }
    }

    log_printf("------------------------------------------\r\n");
    log_printf("Turn the motor by hand, watch CNT change.\r\n");
    log_printf("Currents should stay near 0 mA when idle.\r\n");
    log_printf("------------------------------------------\r\n");

    s_last_cnt  = (int16_t)TIM3->CNT;
    s_total_cnt = 0;
    s_tick      = HAL_GetTick();
}

void Selftest2_Loop(void)
{
    uint16_t raw[3];
    int16_t  now, delta;
    int      k;

    if ((HAL_GetTick() - s_tick) < 300u) {
        return;
    }
    s_tick = HAL_GetTick();

    raw[0] = adc1_read_ch(ADC_CH_U);
    raw[1] = adc1_read_ch(ADC_CH_V);
    raw[2] = adc1_read_ch(ADC_CH_W);

    now   = (int16_t)TIM3->CNT;
    delta = (int16_t)(now - s_last_cnt);
    s_last_cnt = now;
    s_total_cnt += delta;

    log_printf("CNT=%6d d=%5d tot=%8ld | I=%5d,%5d,%5d mA | ADC=%4u,%4u,%4u\r\n",
               (int)now, (int)delta, (long)s_total_cnt,
               (int)(cs_count_to_amp(raw[0], 0) * 1000.0f),
               (int)(cs_count_to_amp(raw[1], 1) * 1000.0f),
               (int)(cs_count_to_amp(raw[2], 2) * 1000.0f),
               (unsigned)raw[0], (unsigned)raw[1], (unsigned)raw[2]);

    (void)k;
}
