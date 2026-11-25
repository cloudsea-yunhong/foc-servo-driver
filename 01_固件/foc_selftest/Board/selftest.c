/**
 * @file    selftest.c
 * @brief   Hardware self-test for FOC servo project
 *
 *  Purpose: measure the two unknowns before writing the real FOC firmware
 *    1) Encoder PPR  -> turn the motor by hand one revolution, read CNT delta
 *    2) ADC full-scale -> read raw ADC with zero current, judge 3.3V / 5V range
 *
 *  Also verifies:
 *    - TIM1 3-phase PWM on PA8 / PA9 / PA10 at 20 kHz center-aligned
 *    - TIM3 hardware encoder mode on PC6 / PC7 (4x)
 *    - ADC1 IN10 / IN11 on PC0 / PC1 (register level, HAL_ADC not available)
 *    - USART2 on PA2 / PA3 at 115200
 *
 *  NOTE: bare-metal (no FreeRTOS) on purpose - simplest bring-up.
 */

#include "selftest.h"
#include "main.h"

#include <stdarg.h>
#include <stdio.h>

/* debug: write execution step to RAM marker */
#define MARK(v)   (*(volatile unsigned int *)0x20010020u = (unsigned int)(v))

/* ------------------------------------------------------------------ */
/* Pin map                                                             */
/* ------------------------------------------------------------------ */
#define PWM_U_PIN        GPIO_PIN_8      /* TIM1_CH1  AF1 */
#define PWM_V_PIN        GPIO_PIN_9      /* TIM1_CH2  AF1 */
#define PWM_W_PIN        GPIO_PIN_10     /* TIM1_CH3  AF1 */

#define ENC_A_PIN        GPIO_PIN_6      /* TIM3_CH1  AF2 */
#define ENC_B_PIN        GPIO_PIN_7      /* TIM3_CH2  AF2 */

#define CUR_A_PIN        GPIO_PIN_0      /* ADC1_IN10 */
#define CUR_B_PIN        GPIO_PIN_1      /* ADC1_IN11 */

#define UART_TX_PIN      GPIO_PIN_2      /* USART2_TX AF7 */
#define UART_RX_PIN      GPIO_PIN_3      /* USART2_RX AF7 */

/* ------------------------------------------------------------------ */
/* PWM timing                                                          */
/* ------------------------------------------------------------------ */
#define PWM_ARR          4199u           /* 168MHz / (2*4200) = 20 kHz */
#define PWM_CENTER       2100u           /* 50 % -> zero average voltage */
#define PWM_MAX_DUTY_PCT 20u             /* safety limit for bring-up */

/* ------------------------------------------------------------------ */
/* Handles                                                             */
/* ------------------------------------------------------------------ */
static TIM_HandleTypeDef  htim1;
static TIM_HandleTypeDef  htim3;
static UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */
static int16_t  s_last_cnt;
static int32_t  s_total_cnt;
static uint32_t s_tick;
static uint16_t s_pwm_cmp = PWM_CENTER;
static volatile uint32_t s_tx_count = 0;

/* ================================================================== */
/* USART2                                                              */
/* ================================================================== */
static void uart2_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = UART_TX_PIN | UART_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    *(volatile unsigned int*)0x20010004u = 0x5A5A5A5Au;  /* MARK-B: UART2 ready */
}

/* Small formatted output (integer only - no float in newlib-nano) */
static char s_txbuf[192];

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
        HAL_UART_Transmit(&huart2, (uint8_t *)s_txbuf, (uint16_t)n, 200);
        s_tx_count += (uint32_t)n;
    }
}

/* ================================================================== */
/* TIM1 - 3 phase PWM, 20 kHz, center aligned                          */
/* ================================================================== */
static void tim1_pwm_init(void)
{
    GPIO_InitTypeDef        gpio = {0};
    TIM_OC_InitTypeDef      oc   = {0};
    TIM_ClockConfigTypeDef  csc  = {0};
    TIM_MasterConfigTypeDef msc  = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = PWM_U_PIN | PWM_V_PIN | PWM_W_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period            = PWM_ARR;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    csc.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &csc);

    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = PWM_CENTER;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3);

    msc.MasterOutputTrigger = TIM_TRGO_RESET;
    msc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &msc);
}

/* ================================================================== */
/* TIM3 - hardware encoder mode (quadrature x4)                        */
/* ================================================================== */
static void tim3_encoder_init(void)
{
    GPIO_InitTypeDef        gpio = {0};
    TIM_Encoder_InitTypeDef enc  = {0};
    TIM_MasterConfigTypeDef msc  = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin       = ENC_A_PIN | ENC_B_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &gpio);

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535u;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;   /* both edges -> x4 */
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 6;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 6;
    HAL_TIM_Encoder_Init(&htim3, &enc);

    msc.MasterOutputTrigger = TIM_TRGO_RESET;
    msc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &msc);
}

/* ================================================================== */
/* ADC1 - CH10 (PC0) + CH11 (PC1), register level                      */
/* HAL_ADC driver (.c) is not present in this project, so do it by hand */
/* ================================================================== */
static void adc1_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin  = CUR_A_PIN | CUR_B_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* ADC clock = PCLK2 / 4 = 84 / 4 = 21 MHz (max 36 MHz) */
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    /* sampling time: 144 cycles for CH10 and CH11 (SMPR1) */
    ADC1->SMPR1 = (6u << 0) | (6u << 3);
    ADC1->SMPR2 = 0;

    /* regular sequence: L=1 -> 2 conversions, SQ1=CH10, SQ2=CH11 */
    ADC1->SQR1 = (1u << 20);
    ADC1->SQR2 = 0;
    ADC1->SQR3 = (10u << 0) | (11u << 5);
    /* single channel, single conversion per SWSTART (SCAN off) */
    ADC1->CR1 &= ~ADC_CR1_SCAN;

    /* power on + calibrate */
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t i = 0; i < 20000u; i++) {
        __NOP();
    }

    ADC1->CR2 |= (1u << 3);   /* RSTCAL */
    for (volatile uint32_t w = 0; (ADC1->CR2 & (1u << 3)) && w < 2000000u; w++) { ; }
    ADC1->CR2 |= (1u << 2);   /* CAL */
    for (volatile uint32_t w = 0; (ADC1->CR2 & (1u << 2)) && w < 2000000u; w++) { ; }
}

/* read one channel (10 or 11) with a single SWSTART - robust without DMA */
static uint16_t adc1_read_ch(uint32_t ch)
{
    volatile uint32_t w;

    ADC1->SQR1 = 0;                    /* L = 0 -> one conversion */
    ADC1->SQR3 = ch;                   /* SQ1 = ch */
    ADC1->CR2 |= ADC_CR2_SWSTART;

    for (w = 0; w < 300000u; w++) {
        if (ADC1->SR & ADC_SR_EOC) {
            return (uint16_t)(ADC1->DR & 0x0FFFu);
        }
    }
    return 0xFFFFu;                    /* timeout */
}
/* single software-triggered conversion pair */
static void adc1_read(uint16_t *ch10, uint16_t *ch11)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;

    for (volatile uint32_t w = 0; !(ADC1->SR & ADC_SR_EOC) && w < 200000u; w++) { ; } if (!(ADC1->SR & ADC_SR_EOC)) { return; }
    *ch10 = (uint16_t)(ADC1->DR & 0x0FFFu);

    for (volatile uint32_t w = 0; !(ADC1->SR & ADC_SR_EOC) && w < 200000u; w++) { ; } if (!(ADC1->SR & ADC_SR_EOC)) { return; }
    *ch11 = (uint16_t)(ADC1->DR & 0x0FFFu);
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */
void Selftest_Init(void)
{
    uart2_init();
    MARK(1);

    log_printf("\r\n\r\n");
    log_printf("======================================\r\n");
    log_printf("  FOC SERVO  HARDWARE SELF TEST  v1  \r\n");
    log_printf("======================================\r\n");
    log_printf("SYSCLK = %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
    log_printf("PCLK2  = %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK2Freq());

    tim1_pwm_init();
    MARK(2);
    tim3_encoder_init();
    MARK(3);
    adc1_init();
    MARK(4);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&htim1);

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    MARK(5);

    /* capture the encoder baseline AFTER starting the counter */
    s_last_cnt  = (int16_t)TIM3->CNT;
    s_total_cnt = 0;
    s_tick      = HAL_GetTick();

    log_printf("PWM    : TIM1 PA8/PA9/PA10  20kHz  ARR=%u  CMP=%u\r\n",
               (unsigned)PWM_ARR, (unsigned)s_pwm_cmp);
    log_printf("ENCODER: TIM3 PC6/PC7  x4  filter=6\r\n");
    log_printf("ADC    : ADC1 CH10(PC0) CH11(PC1)  SW trigger\r\n");
    log_printf("UART   : USART2 PA2/PA3  115200 8N1\r\n");
    log_printf("--------------------------------------\r\n");
    log_printf("Turn the motor by hand EXACTLY one turn.\r\n");
    log_printf("Watch 'd' and 'tot'. tot after 1 turn = PPR x 4\r\n");
    log_printf("--------------------------------------\r\n");
}

void Selftest_Loop(void)
{
    MARK(6);
    /* publish UART state into RAM for debugger inspection */
    *(volatile unsigned int *)0x20010008u = (unsigned int)USART2->CR1;
    *(volatile unsigned int *)0x2001000Cu = (unsigned int)USART2->BRR;
    *(volatile unsigned int *)0x20010010u = (unsigned int)USART2->SR;
    *(volatile unsigned int *)0x20010014u = s_tx_count;
    *(volatile unsigned int *)0x20010018u = (unsigned int)TIM3->CNT;
    int16_t  now;
    int16_t  d;
    uint16_t a0;
    uint16_t a1;
    uint32_t mv0;
    uint32_t mv1;

    if ((HAL_GetTick() - s_tick) < 300u) {
        return;
    }
    s_tick = HAL_GetTick();

    now = (int16_t)TIM3->CNT;
    d   = (int16_t)(now - s_last_cnt);
    s_last_cnt = now;
    s_total_cnt += d;

    a0 = adc1_read_ch(10);
    a1 = adc1_read_ch(11);
    MARK(7);
    mv0 = ((uint32_t)a0 * 3300u) / 4095u;
    mv1 = ((uint32_t)a1 * 3300u) / 4095u;

    log_printf("CNT=%6d d=%5d tot=%8ld | A0=%4u(%4lumV) A2=%4u(%4lumV) | PWM=%u%%\r\n",
               (int)now, (int)d, (long)s_total_cnt,
               (unsigned)a0, (unsigned long)mv0,
               (unsigned)a1, (unsigned long)mv1,
               (unsigned)((s_pwm_cmp * 100u) / (PWM_ARR + 1u)));
}
