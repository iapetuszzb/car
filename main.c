/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"
#include "main.h"

#include "clock.h"
#include "interrupt.h"

#include "mpu6500.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "oled_software_i2c.h"
#include "stepper_gimbal.h"
#include "k210_face.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define STEPPER_GIMBAL_RUN_TEST_ON_BOOT 0
#define GPIO_TOGGLE_TEST_ENABLED 0
#define HCSR04_MEASUREMENT_ENABLED 0
#define USB_ANGLE_UART_INTERVAL_MS          (100UL)

#define HCSR04_TRIG_PORT                    GPIOB
#define HCSR04_TRIG_PIN                     DL_GPIO_PIN_10
#define HCSR04_TRIG_IOMUX                   IOMUX_PINCM27

#define HCSR04_ECHO_PORT                    GPIOB
#define HCSR04_ECHO_PIN                     DL_GPIO_PIN_11
#define HCSR04_ECHO_IOMUX                   IOMUX_PINCM28

#define HCSR04_TRIGGER_US                   (10UL)
#define HCSR04_SETTLE_US                    (2UL)
#define HCSR04_ECHO_TIMEOUT_US              (30000UL)
#define HCSR04_MEASURE_INTERVAL_MS          (150UL)
#define HCSR04_AB_ALPHA                     (0.45f)
#define HCSR04_AB_BETA                      (0.08f)
#define HCSR04_AB_DEFAULT_DT_S              (0.15f)
#define OLED_UPDATE_INTERVAL_MS             (100UL)
#define OLED_CONFIG_REPAIR_UPDATES          (50U)
#define OLED_RX_TEXT_LEN                     (46U)

#define GPIO_TEST_PORT                      GPIOB
#define GPIO_TEST_PB10_PIN                  DL_GPIO_PIN_10
#define GPIO_TEST_PB10_IOMUX                IOMUX_PINCM27
#define GPIO_TEST_PB11_PIN                  DL_GPIO_PIN_11
#define GPIO_TEST_PB11_IOMUX                IOMUX_PINCM28
#define GPIO_TEST_TOGGLE_INTERVAL_MS        (1000UL)

#define SERVO_PORT                         GPIOB
#define SERVO_PIN                          DL_GPIO_PIN_26
#define SERVO_IOMUX                        IOMUX_PINCM57
#define SERVO_PERIOD_US                    (20000UL)
#define SERVO_TICK_US                      (100UL)
#define SERVO_PERIOD_TICKS                 (SERVO_PERIOD_US / SERVO_TICK_US)
#define SERVO_LEFT_90_US                   (500UL)
#define SERVO_CENTER_US                    (1500UL)
#define SERVO_RIGHT_90_US                  (2500UL)
#define SERVO_FIRST_LEFT_MS                (3000UL)
#define SERVO_FULL_SWEEP_MS                (6000UL)

bool start = false;
uint8_t Circle_Count, turncount, paramSelcet;
float TargetSpeed, TargetLine, basespeed;
pid_t pidMotorA, pidMotorB, pidLine;
float P, I, D;

#if !GPIO_TOGGLE_TEST_ENABLED && HCSR04_MEASUREMENT_ENABLED
typedef struct {
    float distance_mm;
    float velocity_mm_s;
    unsigned long last_update_ms;
    bool initialized;
} alpha_beta_filter_t;

static alpha_beta_filter_t hcsr04_filter;
#endif

typedef enum {
    SERVO_PHASE_CENTER_TO_LEFT = 0,
    SERVO_PHASE_LEFT_TO_RIGHT,
    SERVO_PHASE_RIGHT_TO_LEFT,
} servo_sweep_phase_t;

typedef struct {
    servo_sweep_phase_t phase;
    unsigned long phase_start_ms;
    uint32_t from_us;
    uint32_t to_us;
    uint32_t duration_ms;
    bool initialized;
} servo_sweep_t;

static servo_sweep_t servo_sweep;
static volatile uint16_t servo_pwm_pulse_ticks = SERVO_CENTER_US / SERVO_TICK_US;
static volatile uint16_t servo_pwm_period_ticks = 0U;
static volatile bool servo_pwm_enabled = false;

static void Servo_SetPulseUs(uint32_t pulse_us)
{
    if (pulse_us < SERVO_LEFT_90_US) {
        pulse_us = SERVO_LEFT_90_US;
    } else if (pulse_us > SERVO_RIGHT_90_US) {
        pulse_us = SERVO_RIGHT_90_US;
    }

    servo_pwm_pulse_ticks = (uint16_t)(pulse_us / SERVO_TICK_US);
}

static void Servo_StartPhase(servo_sweep_phase_t phase,
                             unsigned long now_ms)
{
    servo_sweep.phase = phase;
    servo_sweep.phase_start_ms = now_ms;

    switch (phase) {
    case SERVO_PHASE_CENTER_TO_LEFT:
        servo_sweep.from_us = SERVO_CENTER_US;
        servo_sweep.to_us = SERVO_LEFT_90_US;
        servo_sweep.duration_ms = SERVO_FIRST_LEFT_MS;
        break;
    case SERVO_PHASE_LEFT_TO_RIGHT:
        servo_sweep.from_us = SERVO_LEFT_90_US;
        servo_sweep.to_us = SERVO_RIGHT_90_US;
        servo_sweep.duration_ms = SERVO_FULL_SWEEP_MS;
        break;
    default:
        servo_sweep.from_us = SERVO_RIGHT_90_US;
        servo_sweep.to_us = SERVO_LEFT_90_US;
        servo_sweep.duration_ms = SERVO_FULL_SWEEP_MS;
        break;
    }

    Servo_SetPulseUs(servo_sweep.from_us);
}

static void Servo_Init(void)
{
    DL_GPIO_initDigitalOutput(SERVO_IOMUX);
    DL_GPIO_clearPins(SERVO_PORT, SERVO_PIN);
    DL_GPIO_enableOutput(SERVO_PORT, SERVO_PIN);

    Servo_SetPulseUs(SERVO_CENTER_US);
    servo_pwm_period_ticks = 0U;
    servo_pwm_enabled = true;
    servo_sweep.initialized = false;
}

void Servo_Tick100us(void)
{
    if (!servo_pwm_enabled) {
        return;
    }

    if (servo_pwm_period_ticks == 0U) {
        DL_GPIO_setPins(SERVO_PORT, SERVO_PIN);
    }

    if (servo_pwm_period_ticks >= servo_pwm_pulse_ticks) {
        DL_GPIO_clearPins(SERVO_PORT, SERVO_PIN);
    }

    servo_pwm_period_ticks++;
    if (servo_pwm_period_ticks >= SERVO_PERIOD_TICKS) {
        servo_pwm_period_ticks = 0U;
    }
}

static void Servo_Task(unsigned long now_ms)
{
    uint32_t elapsed_ms;
    int32_t pulse_us;

    if (!servo_sweep.initialized) {
        servo_sweep.initialized = true;
        Servo_StartPhase(SERVO_PHASE_CENTER_TO_LEFT, now_ms);
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - servo_sweep.phase_start_ms);
    if (elapsed_ms >= servo_sweep.duration_ms) {
        if (servo_sweep.phase == SERVO_PHASE_CENTER_TO_LEFT) {
            Servo_StartPhase(SERVO_PHASE_LEFT_TO_RIGHT, now_ms);
        } else if (servo_sweep.phase == SERVO_PHASE_LEFT_TO_RIGHT) {
            Servo_StartPhase(SERVO_PHASE_RIGHT_TO_LEFT, now_ms);
        } else {
            Servo_StartPhase(SERVO_PHASE_LEFT_TO_RIGHT, now_ms);
        }
        return;
    }

    pulse_us = (int32_t)servo_sweep.from_us +
        ((((int32_t)servo_sweep.to_us - (int32_t)servo_sweep.from_us) *
          (int32_t)elapsed_ms) / (int32_t)servo_sweep.duration_ms);
    Servo_SetPulseUs((uint32_t)pulse_us);
}

#if !GPIO_TOGGLE_TEST_ENABLED && HCSR04_MEASUREMENT_ENABLED
static void HCSR04_Init(void)
{
    DL_GPIO_initDigitalOutput(HCSR04_TRIG_IOMUX);
    DL_GPIO_clearPins(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    DL_GPIO_enableOutput(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    DL_GPIO_initDigitalInputFeatures(HCSR04_ECHO_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static uint8_t HCSR04_ReadEchoLevel(void)
{
    return (DL_GPIO_readPins(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) &
            HCSR04_ECHO_PIN) ? 1U : 0U;
}

static bool HCSR04_WaitEchoLevel(uint8_t target_level,
                                 unsigned long timeout_us,
                                 unsigned long *hit_time_us)
{
    unsigned long start_us;
    unsigned long now_us;

    (void)mspm0_get_clock_us(&start_us);
    do {
        (void)mspm0_get_clock_us(&now_us);
        if (HCSR04_ReadEchoLevel() == target_level) {
            if (hit_time_us != NULL) {
                *hit_time_us = now_us;
            }
            return true;
        }
    } while ((unsigned long)(now_us - start_us) < timeout_us);

    return false;
}

static bool HCSR04_ReadDistanceMm(uint32_t *distance_mm)
{
    unsigned long rise_us;
    unsigned long fall_us;
    unsigned long pulse_us;

    if (distance_mm == NULL) {
        return false;
    }

    if (!HCSR04_WaitEchoLevel(0U, HCSR04_ECHO_TIMEOUT_US, NULL)) {
        return false;
    }

    DL_GPIO_clearPins(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    (void)mspm0_delay_us(HCSR04_SETTLE_US);
    DL_GPIO_setPins(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    (void)mspm0_delay_us(HCSR04_TRIGGER_US);
    DL_GPIO_clearPins(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    if (!HCSR04_WaitEchoLevel(1U, HCSR04_ECHO_TIMEOUT_US, &rise_us)) {
        return false;
    }

    if (!HCSR04_WaitEchoLevel(0U, HCSR04_ECHO_TIMEOUT_US, &fall_us)) {
        return false;
    }

    pulse_us = fall_us - rise_us;
    *distance_mm = (uint32_t)((pulse_us * 343UL + 1000UL) / 2000UL);
    return true;
}

static uint32_t HCSR04_FilterDistanceMm(uint32_t raw_distance_mm,
                                        unsigned long now_ms)
{
    float dt_s;
    float predicted_distance;
    float residual;

    if (!hcsr04_filter.initialized) {
        hcsr04_filter.distance_mm = (float)raw_distance_mm;
        hcsr04_filter.velocity_mm_s = 0.0f;
        hcsr04_filter.last_update_ms = now_ms;
        hcsr04_filter.initialized = true;
        return raw_distance_mm;
    }

    dt_s = (float)(now_ms - hcsr04_filter.last_update_ms) / 1000.0f;
    if (dt_s <= 0.0f || dt_s > 1.0f) {
        dt_s = HCSR04_AB_DEFAULT_DT_S;
    }
    hcsr04_filter.last_update_ms = now_ms;

    predicted_distance =
        hcsr04_filter.distance_mm + (hcsr04_filter.velocity_mm_s * dt_s);
    residual = (float)raw_distance_mm - predicted_distance;

    hcsr04_filter.distance_mm =
        predicted_distance + (HCSR04_AB_ALPHA * residual);
    hcsr04_filter.velocity_mm_s +=
        (HCSR04_AB_BETA * residual) / dt_s;

    if (hcsr04_filter.distance_mm < 0.0f) {
        hcsr04_filter.distance_mm = 0.0f;
        hcsr04_filter.velocity_mm_s = 0.0f;
    }

    return (uint32_t)(hcsr04_filter.distance_mm + 0.5f);
}
#endif

static void FormatYaw(char *buffer, uint32_t buffer_len, float value)
{
    long yaw_milli;
    long yaw_int;
    long yaw_frac;

    if (value >= 0.0f) {
        yaw_milli = (long)(value * 1000.0f + 0.5f);
    } else {
        yaw_milli = (long)(value * 1000.0f - 0.5f);
    }

    yaw_int = yaw_milli / 1000L;
    yaw_frac = yaw_milli % 1000L;
    if (yaw_frac < 0) {
        yaw_frac = -yaw_frac;
    }

    (void)snprintf(buffer, buffer_len, "yaw:%ld.%03ld   ",
                   yaw_int, yaw_frac);
}

static void FormatOdometer(char *buffer, uint32_t buffer_len,
                           float distance_mm)
{
    uint32_t rounded_mm;
    uint32_t meters;
    uint32_t millimeters;

    if(distance_mm < 0.0f) {
        distance_mm = 0.0f;
    }
    rounded_mm = (uint32_t)(distance_mm + 0.5f);
    meters = rounded_mm / 1000UL;
    millimeters = rounded_mm % 1000UL;
    (void)snprintf(buffer, buffer_len, "odo:%lu.%03lum",
                   (unsigned long)meters,
                   (unsigned long)millimeters);
}

static void USB_UART_SendString(const char *text)
{
    while(*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_USB_INST, (uint8_t)*text);
        text++;
    }
}

static void FormatAngleValue(char *buffer, uint32_t buffer_len, float value)
{
    long milli;
    long integer;
    long fraction;

    if(value >= 0.0f) {
        milli = (long)(value * 1000.0f + 0.5f);
    } else {
        milli = (long)(value * 1000.0f - 0.5f);
    }

    integer = milli / 1000L;
    fraction = milli % 1000L;
    if(fraction < 0L) {
        fraction = -fraction;
    }

    (void)snprintf(buffer, buffer_len, "%ld.%03ld", integer, fraction);
}

static void USB_UART_PrintAngles(void)
{
    char value[24];

    FormatAngleValue(value, sizeof(value), roll);
    USB_UART_SendString(value);
    USB_UART_SendString(",");
    FormatAngleValue(value, sizeof(value), pitch);
    USB_UART_SendString(value);
    USB_UART_SendString(",");
    FormatAngleValue(value, sizeof(value), yaw);
    USB_UART_SendString(value);
    USB_UART_SendString("\r\n");
}

static void OLED_CopyAsciiLine(char *line, uint32_t line_len,
                               const char *text, uint32_t text_offset,
                               const char *prefix)
{
    uint32_t i;
    uint32_t dst = 0U;
    uint32_t text_len = 0U;

    if (line_len < 17U) {
        return;
    }

    for (i = 0U; i < 16U; i++) {
        line[i] = ' ';
    }

    if (prefix != NULL) {
        while ((prefix[dst] != '\0') && (dst < 16U)) {
            line[dst] = prefix[dst];
            dst++;
        }
    }

    while (text[text_len] != '\0') {
        text_len++;
    }
    while ((dst < 16U) && (text_offset < text_len)) {
        line[dst] = text[text_offset];
        dst++;
        text_offset++;
    }
    line[16] = '\0';
}

static void OLED_ShowLineIfChanged(uint8_t y, uint8_t cache_index,
                                   const char *text)
{
    static char line_cache[4][17];
    static uint8_t refresh_age[4];
    char padded[17];
    uint8_t i;
    bool changed = false;

    if(cache_index >= 4U || text == NULL) {
        return;
    }

    for(i = 0U; i < 16U; i++) {
        if(text[i] != '\0') {
            padded[i] = text[i];
        } else {
            break;
        }
    }
    while(i < 16U) {
        padded[i++] = ' ';
    }
    padded[16] = '\0';

    for(i = 0U; i < 17U; i++) {
        if(line_cache[cache_index][i] != padded[i]) {
            changed = true;
        }
        line_cache[cache_index][i] = padded[i];
    }

    /* Periodically repair a line if an earlier software-I2C write was noisy. */
    if(refresh_age[cache_index] >= 10U) {
        changed = true;
    }

    if(changed) {
        OLED_ShowString16(y, padded);
        refresh_age[cache_index] = 0U;
    } else {
        refresh_age[cache_index]++;
    }
}

static void OLED_ShowStatus(void)
{
    char line[20];
    char rx_text[OLED_RX_TEXT_LEN];
    encoder_odometry_t odometry;
    static uint8_t config_repair_age;

    config_repair_age++;
    if(config_repair_age >= OLED_CONFIG_REPAIR_UPDATES) {
        config_repair_age = 0U;
        OLED_RefreshConfig();
    }

    FormatYaw(line, sizeof(line), yaw);
    OLED_ShowLineIfChanged(0, 0U, line);

    Encoder_GetOdometry(&odometry);
    FormatOdometer(line, sizeof(line), odometry.distance_center_mm);
    OLED_ShowLineIfChanged(2, 1U, line);

    (void)snprintf(line, sizeof(line), "A:%05lu B:%05lu",
                   (unsigned long)(odometry.travel_counts_a % 100000UL),
                   (unsigned long)(odometry.travel_counts_b % 100000UL));
    OLED_ShowLineIfChanged(4, 2U, line);

    Bluetooth_GetRecentAscii(rx_text, sizeof(rx_text));
    if (rx_text[0] == '\0') {
        (void)snprintf(line, sizeof(line), "CPR:%lu X%lu",
                       (unsigned long)ENCODER_COUNTS_PER_WHEEL_REV,
                       (unsigned long)ENCODER_DECODE_MULTIPLIER);
        OLED_ShowLineIfChanged(6, 3U, line);
    } else {
        OLED_CopyAsciiLine(line, sizeof(line), rx_text, 0U, "RX:");
        OLED_ShowLineIfChanged(6, 3U, line);
    }
}

static void GPIO_ToggleTest_Init(void)
{
    DL_GPIO_initDigitalOutput(GPIO_TEST_PB10_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_TEST_PB11_IOMUX);

    DL_GPIO_clearPins(GPIO_TEST_PORT, GPIO_TEST_PB10_PIN);
    DL_GPIO_setPins(GPIO_TEST_PORT, GPIO_TEST_PB11_PIN);
    DL_GPIO_enableOutput(GPIO_TEST_PORT,
                         GPIO_TEST_PB10_PIN | GPIO_TEST_PB11_PIN);
}

static bool GPIO_ToggleTest_Task(uint8_t *pb10_state, uint8_t *pb11_state)
{
    static unsigned long last_toggle_ms = 0UL;
    unsigned long now_ms = 0UL;
    uint32_t pins;

    (void)mspm0_get_clock_ms(&now_ms);
    if ((unsigned long)(now_ms - last_toggle_ms) >=
        GPIO_TEST_TOGGLE_INTERVAL_MS) {
        last_toggle_ms = now_ms;
        DL_GPIO_togglePins(GPIO_TEST_PORT,
                           GPIO_TEST_PB10_PIN | GPIO_TEST_PB11_PIN);
    }

    pins = DL_GPIO_readPins(GPIO_TEST_PORT,
                            GPIO_TEST_PB10_PIN | GPIO_TEST_PB11_PIN);
    if (pb10_state != NULL) {
        *pb10_state = (pins & GPIO_TEST_PB10_PIN) ? 1U : 0U;
    }
    if (pb11_state != NULL) {
        *pb11_state = (pins & GPIO_TEST_PB11_PIN) ? 1U : 0U;
    }

    return true;
}

static void OLED_ShowToggleStatus(uint8_t pb10_state, uint8_t pb11_state)
{
    char line[20];

    if (MPU6500_GetInitStatus() != 0U) {
        (void)snprintf(line, sizeof(line), "mpu err:%u   ",
                       (unsigned int)MPU6500_GetInitStatus());
    } else if ((MPU6500_GetUpdateCount() == 0U) &&
               (MPU6500_GetLastReadStatus() != 0U)) {
        (void)snprintf(line, sizeof(line), "mpu wait:%u  ",
                       (unsigned int)MPU6500_GetLastReadStatus());
    } else {
        FormatYaw(line, sizeof(line), yaw);
    }
    OLED_ShowString(0, 0, (uint8_t *)line, 16);

    OLED_ShowString(0, 2, (uint8_t *)"GPIO flip test ", 16);
    (void)snprintf(line, sizeof(line), "PB10:%u        ",
                   (unsigned int)pb10_state);
    OLED_ShowString(0, 4, (uint8_t *)line, 16);
    (void)snprintf(line, sizeof(line), "PB11:%u        ",
                   (unsigned int)pb11_state);
    OLED_ShowString(0, 6, (uint8_t *)line, 16);
}

int main(void)
{
    static unsigned long last_mpu_update_ms = 0UL;
#if HCSR04_MEASUREMENT_ENABLED
    static unsigned long last_hcsr04_ms = 0UL;
#endif
    static unsigned long last_oled_update_ms = 0UL;
    static unsigned long last_usb_angle_uart_ms = 0UL;
    uint8_t mpu_init_status;

    SYSCFG_DL_init();
    SysTick_Init();

    OLED_Init();
    Motor_Init();
    Encoder_OdometryReset();
    Load(0, 0);
    OLED_ShowLineIfChanged(0, 0U, "BOOT RESET OK");
    OLED_ShowLineIfChanged(2, 1U, "MPU CAL WAIT");
    OLED_ShowLineIfChanged(4, 2U, "KEEP CAR STILL");
    OLED_ShowLineIfChanged(6, 3U, "");
#if GPIO_TOGGLE_TEST_ENABLED
    GPIO_ToggleTest_Init();
#elif HCSR04_MEASUREMENT_ENABLED
    HCSR04_Init();
#endif
    mpu_init_status = MPU6500_Init();
    if(mpu_init_status == 0U) {
        OLED_ShowLineIfChanged(2, 1U, "MPU READY");
    } else {
        char mpu_error_line[20];
        (void)snprintf(mpu_error_line, sizeof(mpu_error_line),
                       "MPU ERROR %u", (unsigned int)mpu_init_status);
        OLED_ShowLineIfChanged(2, 1U, mpu_error_line);
    }
#if APP_ENABLE_SERVO_SWEEP
    Servo_Init();
#endif
#if APP_ENABLE_STEPPER_GIMBAL
    StepperGimbal_Init();
#endif
#if APP_ENABLE_VISION_UART
    K210Face_Init();
    K210Face_SetTrackingEnabled(false);
#endif
    Bluetooth_Init();

#if APP_ENABLE_STEPPER_GIMBAL && STEPPER_GIMBAL_RUN_TEST_ON_BOOT
    StepperGimbal_TestSmallMove();
#endif

    DL_TimerG_startCounter(TIMER_0_INST);
    Interrupt_Init();

    pid_init(&pidMotorA, DELTA_PID, 18, 35, 0);
    pid_init(&pidMotorB, DELTA_PID, 18, 35, 0);

    P = 2500.0f;
    I = 0;
    D = 800.0f;
    pid_init(&pidLine, POSITION_PID, P, I, D);

    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    Load(0, 0);

    basespeed = 10.0f;
    TargetSpeed = basespeed;
    TargetLine = 4.5f;
    Circle_Count = 1;
    turncount = 0;

    while (1) {
        unsigned long now_ms = 0UL;

        (void)mspm0_get_clock_ms(&now_ms);
        if ((unsigned long)(now_ms - last_mpu_update_ms) >= 10UL) {
            last_mpu_update_ms = now_ms;
            (void)Read_MPU6500();
        }

#if APP_ENABLE_VISION_UART
        K210Face_Task();
#endif
#if APP_ENABLE_SERVO_SWEEP
        Servo_Task(now_ms);
#endif
        Bluetooth_Task();

        if((unsigned long)(now_ms - last_usb_angle_uart_ms) >=
           USB_ANGLE_UART_INTERVAL_MS) {
            last_usb_angle_uart_ms = now_ms;
            USB_UART_PrintAngles();
        }

#if GPIO_TOGGLE_TEST_ENABLED
        {
            static unsigned long last_oled_update_ms = 0UL;
            uint8_t pb10_state;
            uint8_t pb11_state;

            (void)GPIO_ToggleTest_Task(&pb10_state, &pb11_state);
            if ((unsigned long)(now_ms - last_oled_update_ms) >= 100UL) {
                last_oled_update_ms = now_ms;
                OLED_ShowToggleStatus(pb10_state, pb11_state);
            }
        }
#else
#if HCSR04_MEASUREMENT_ENABLED
        if ((unsigned long)(now_ms - last_hcsr04_ms) >=
            HCSR04_MEASURE_INTERVAL_MS) {
            bool ok;
            uint32_t distance_mm = 0U;

            last_hcsr04_ms = now_ms;
            ok = HCSR04_ReadDistanceMm(&distance_mm);
            if (ok) {
                distance_mm = HCSR04_FilterDistanceMm(distance_mm, now_ms);
            }
        }
#endif

        if ((unsigned long)(now_ms - last_oled_update_ms) >=
            OLED_UPDATE_INTERVAL_MS) {
            last_oled_update_ms = now_ms;
            OLED_ShowStatus();
        }
#endif
    }
}
