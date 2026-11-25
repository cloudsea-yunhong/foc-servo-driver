/**
 * @file    hw_config.h
 * @brief   FOC Servo - Hardware configuration constants
 *
 *  Board : SimpleFOC mini V2.0 (TI DRV8316CTRGFR)
 *  MCU   : STM32F407VET6 @ 168 MHz
 *  Motor : 2804 BLDC, 7 pole pairs, MT6701 encoder (ABZ, 1024 PPR)
 *
 *  IMPORTANT
 *  ---------
 *  Every value marked [MEASURE] must be verified with a multimeter /
 *  test firmware after the board arrives.  Do NOT trust the defaults.
 */

#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* ================================================================== */
/* 1. Current sensing  (DRV8316 low-side CSA)                         */
/* ================================================================== */

/** Gain [V/A].
 *  DRV8316 RGAIN pin tied to AGND -> 0.15 V/A (per schematic note).
 *  Other options: 0.30 / 0.60 / 1.20.
 *  Status: theoretical - still needs a known-current check. */
#define CS_GAIN_V_PER_A         0.15f

/** Zero-current output voltage [V] - NOMINAL value only.
 *  Status: MEASURED on the real board with a multimeter.
 *    AVDD (3.3V LDO)  = 3.30 V  (the LDO tolerance puts it at ~3.32 V)
 *    CS1 = CS2 = CS3  = 1.66 V  -> zero = AVDD/2
 *
 *  IMPORTANT: 1.66 V vs the nominal 1.65 V is only 10 mV, but that is
 *  10 mV / 0.15 V/A = 67 mA of current offset - 13 % of this motor's
 *  0.5 A rating.  So the firmware MUST run the power-on zero
 *  calibration (see bsp_driver.c) and use the measured per-channel
 *  offsets instead of this constant.  The macro below is only used
 *  before calibration completes, or if calibration fails.
 *
 *  Note: the Buck output (VCC pin) measured only 1.13 V on this board
 *  revision and is NOT used by this firmware - the CSA reference is
 *  AVDD, not the Buck. Do not connect the VCC pin. */
#define CS_ZERO_VOLT            1.66f

/** Number of ADC samples averaged during the power-on zero calibration */
#define CS_CAL_SAMPLES          64

/** ADC full-scale reference [V] - the F407's OWN 3.3 V rail (VDDA),
 *  NOT the driver board's AVDD.  These are two independent regulators.
 *  Status: MEASURED = 3.30 V.
 *  The zero point below is an absolute voltage from the driver board,
 *  so both numbers are needed and they are not the same rail. */
#define ADC_VREF_V              3.30f
#define ADC_MAX_COUNT           4095.0f

/** Zero-current ADC count (derived) */
#define CS_ZERO_COUNT           ((uint16_t)((CS_ZERO_VOLT / ADC_VREF_V) * ADC_MAX_COUNT + 0.5f))

/** Convert an ADC count into Amps */
#define CS_COUNT_TO_AMP(c)      ((((float)(c) * ADC_VREF_V / ADC_MAX_COUNT) - CS_ZERO_VOLT) / CS_GAIN_V_PER_A)

/** Number of samples averaged per current reading (1 = none) */
#define CS_OVERSAMPLE           4

/* ================================================================== */
/* 2. Motor parameters (2804 datasheet)                               */
/* ================================================================== */

#define MOTOR_POLE_PAIRS        7U          /* 12N14P */
#define MOTOR_PHASE_R           2.55f       /* phase resistance [ohm]  */
#define MOTOR_PHASE_L           0.00086f    /* phase inductance [H]    */
#define MOTOR_KT                0.0434f     /* torque constant [N.m/A] */
#define MOTOR_KE                0.0434f     /* back-EMF const [V/(rad/s)] */
#define MOTOR_I_RATED           0.50f       /* rated current [A]       */
#define MOTOR_I_MAX             2.00f       /* max current [A]         */
#define MOTOR_V_RATED           12.0f       /* rated voltage [V]       */
#define MOTOR_V_MAX             16.0f       /* absolute max voltage [V] */

/* ================================================================== */
/* 3. Encoder (MT6701, ABZ mode, quadrature x4)                       */
/* ================================================================== */

#define ENC_PPR                 1024U
#define ENC_CPR                 (ENC_PPR * 4U)              /* 4096 counts/rev */
#define ENC_ANGLE_PER_COUNT     (6.283185307f / (float)ENC_CPR)
/** [MEASURE] turning the shaft "forward" produced negative counts -> -1 */
#define ENC_DIR_SIGN        (-1)
/** Input filter for TIM3 encoder mode (0..15) */
#define ENC_INPUT_FILTER       15U

/* ================================================================== */
/* 4. PWM / carrier                                                   */
/* ================================================================== */

#define PWM_FREQ_HZ             20000.0f
/** TIM1 center-aligned: 168MHz / (2 * (ARR+1)) = 20 kHz  -> ARR = 4199 */
#define PWM_ARR                 4199U
#define PWM_CENTER              (PWM_ARR / 2U)              /* 50 % duty */
#define PWM_MIN_CMP             0U
#define PWM_MAX_CMP             PWM_ARR

/* ================================================================== */
/* 5. Control loop rates                                              */
/* ================================================================== */

#define CURRENT_LOOP_HZ         20000.0f    /* == PWM carrier            */
#define VELOCITY_LOOP_HZ        1000.0f
#define POSITION_LOOP_HZ        1000.0f
#define DT_CURRENT              (1.0f / CURRENT_LOOP_HZ)
#define DT_VELOCITY             (1.0f / VELOCITY_LOOP_HZ)
#define DT_POSITION             (1.0f / POSITION_LOOP_HZ)

/* ================================================================== */
/* 6. Limits  (start conservative, raise after tuning)                */
/* ================================================================== */

/** Phase current limit - DEBUG (well below the 0.5 A rating) */
#define LIMIT_I_DEBUG           0.30f
/** Phase current limit - NORMAL operation */
#define LIMIT_I_RUN             MOTOR_I_RATED   /* 0.5 A - never exceed rated on hold */
/** Absolute current limit */
#define LIMIT_I_MAX             MOTOR_I_MAX

/** Voltage limit = Vbus / sqrt(3) * this percentage */
#define LIMIT_V_DUTY_PCT        60.0f

/** Speed limit [rad/s]  (2600 rpm = 272 rad/s) */
#define LIMIT_SPEED_RAD_S       300.0f
/** Position error clamp [rad] */
#define LIMIT_POS_ERR_RAD       6.283185307f

/** PWM duty hard cap [%] during bring-up */
#define LIMIT_DUTY_PCT_DEBUG    20.0f

/* ================================================================== */
/* 7. Pin map                                                         */
/* ================================================================== */

/* --- TIM1 3-phase PWM --- */
#define GPIO_PWM_PORT           GPIOA
#define PIN_PWM_U               GPIO_PIN_8      /* PA8  TIM1_CH1  -> board IN1 */
#define PIN_PWM_V               GPIO_PIN_9      /* PA9  TIM1_CH2  -> board IN2 */
#define PIN_PWM_W               GPIO_PIN_10     /* PA10 TIM1_CH3  -> board IN3 */
#define PWM_AF                  GPIO_AF1_TIM1

/* --- Driver enable (board EN pin) --- */
#define GPIO_EN_PORT            GPIOC
#define PIN_EN                  GPIO_PIN_4      /* PC4  -> board EN */

/* --- TIM3 quadrature encoder --- */
#define GPIO_ENC_PORT           GPIOC
#define PIN_ENC_A               GPIO_PIN_6      /* PC6  TIM3_CH1 */
#define PIN_ENC_B               GPIO_PIN_7      /* PC7  TIM3_CH2 */
#define ENC_AF                  GPIO_AF2_TIM3

/* --- ADC current sense --- */
#define GPIO_CS_PORT            GPIOC
#define PIN_CS_U                GPIO_PIN_0      /* PC0  ADC1_IN10 -> CS1 */
#define PIN_CS_V                GPIO_PIN_1      /* PC1  ADC1_IN11 -> CS2 */
#define PIN_CS_W                GPIO_PIN_2      /* PC2  ADC1_IN12 -> CS3 */
#define ADC_CH_U                10U
#define ADC_CH_V                11U
#define ADC_CH_W                12U

/* --- USART2 (CH340) --- */
#define GPIO_UART_PORT          GPIOA
#define PIN_UART_TX             GPIO_PIN_2      /* PA2 USART2_TX */
#define PIN_UART_RX             GPIO_PIN_3      /* PA3 USART2_RX */
#define UART_AF                 GPIO_AF7_USART2
/* --- ADC sampling marker: toggles high while the injected conversions run,
 *     so a logic analyser can prove the sample lands inside the low-side
 *     conduction window.  PB0 is free on this board. --- */
#define GPIO_MARK_PORT          GPIOB
#define PIN_MARK                GPIO_PIN_0

/* Encoder A/B MIRROR outputs for logic-analyser observation.
 * PC6/PC7 keep the real encoder connection (it may be open-drain and
 * needs the MCU pull-ups); PB1/PB2 are a 20 kHz copy of those levels. */
#define PIN_ENC_A_MIRROR        GPIO_PIN_1     /* PB1 */
#define PIN_ENC_B_MIRROR        GPIO_PIN_12    /* PB12 */


#define UART_BAUD               115200U

/* --- Optional: nFAULT (NOT broken out on the 10-pin board revision) --- */
#define GPIO_FAULT_PORT         GPIOB
#define PIN_FAULT               GPIO_PIN_0      /* PB0 - optional */

/* --- On-board LED of the F407 core board --- */
#define GPIO_LED_PORT           GPIOC
#define PIN_LED                 GPIO_PIN_13

/* ================================================================== */
/* 8. Board-specific notes                                            */
/* ================================================================== */

/* 10-pin header, CONFIRMED against the physical board silkscreen.
 *
 *  Orientation : Power terminal on the LEFT, Motor terminal at the
 *                BOTTOM, 2x5 header along the TOP.
 *
 *           left  ->                              ->  right
 *   row A :  IN1     IN2     IN3     EN      GND   (board-edge side)
 *   row B :  VCC     CS1     CS2     CS3     3V3   (chip side)
 *
 *  Column pairing : IN1-VCC, IN2-CS1, IN3-CS2, EN-CS3, GND-3V3
 *
 * VM and GND for power come from the 2-pin screw terminal ("Power").
 * Motor U/V/W come from the 3-pin screw terminal ("Motor").
 *
 * Wiring used by this firmware:
 *   IN1 -> PA8    IN2 -> PA9    IN3 -> PA10   EN -> PC4   GND -> GND
 *   CS1 -> PC0    CS2 -> PC1    CS3 -> PC2
 *   VCC and 3V3 are measured, NOT connected.
 *
 * NOT broken out on this revision:
 *   nSLEEP (assumed pulled up on-board)
 *   nFAULT (only the FLT LED indicates a fault)
 *   VREF   (tied to Buck output on-board)
 */


/* ================================================================== */
/* 9. LOCKED-IN BENCH CONFIGURATION  (do not change without re-testing) */
/* ================================================================== */
/* This block records the exact combination that was verified on the   */
/* bench.  Every number here was measured, not guessed.  Changing any  */
/* of them means the alignment, the current-loop gain or the rotation  */
/* direction has to be re-verified.                                    */
/*
 * HEADER (SimpleFOC mini V2 / DRV8316), left-to-right in the photo
 *   top row    : IN1  IN2  IN3  EN  GND
 *   bottom row : VCC  CS1  CS2  CS3 3V3
 *
 *   IN1 -> PA8    IN2 -> PA9    IN3 -> PA10   EN -> PC4   GND -> GND
 *   CS2 -> PC1    (CS1 -> PC0 is NOT used, see note)
 *   VCC and 3V3 are measured, never connected.
 *
 * MOTOR (2804, 12N14P, 7 pole pairs)
 *   three phase wires -> Motor screw terminal, order is arbitrary BUT
 *   once a combination makes the rotor turn in the +direction it must
 *   be kept.  Write the physical colours here:
 *       wire 1 (______) -> Motor hole 1
 *       wire 2 (______) -> Motor hole 2
 *       wire 3 (______) -> Motor hole 3
 *
 * ENCODER (MT6701, ABZ used as A/B only)
 *   VCC -> 3V3   GND -> GND   A -> PC6   B -> PC7   Z -> not connected
 *
 * KNOWNS
 *   ENC_DIR_SIGN = -1        open loop makes s_mech_angle DECREASE
 *   CS_GAIN_V_PER_A = 0.15   measured: vq scales linearly with Iq
 *   CS_ZERO_VOLT = 1.66      CS2/CS3 read 2057..2058 counts
 *
 * TWO-SHUNT MODE (IMPORTANT)
 *   CS1 (PC0) never tracked the real phase current on this board: in
 *   open loop phase C showed 1.36 A while CS1 still sat at its zero
 *   point, which is impossible for a three-wire motor.  The firmware
 *   therefore measures phases B and C only and reconstructs phase A:
 *       Ia = -(Ib + Ic)
 *   This is why the CS1 wire is not needed.  If a future board has a
 *   working CS1, set USE_TWO_SHUNT to 0 in focapp.c.
 *
 * STATUS
 *   current loop  : verified +-1..4 % from 100 mA to 500 mA
 *   open loop     : rotor spins continuously
 *   velocity loop : gains raised, but the torque direction is opposite
 *                   to the encoder direction.  Fix by swapping any two
 *                   MOTOR phase wires (NOT by flipping ENC_DIR_SIGN,
 *                   which breaks the FOC angle), then re-align.
 */
#endif /* __HW_CONFIG_H */
