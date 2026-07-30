#include "uart.h"

#define ENABLE_BLUETOOTH                  (1)

#if ENABLE_BLUETOOTH

#include "clock.h"
#include "encoder.h"
#include "k210_face.h"
#include "mpu6500.h"
#include "motor.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* UART1 is dedicated to Bluetooth in this build. The former K210 UART1
 * receiver is disabled by APP_ENABLE_VISION_UART in main.h. */
#define BT_UART_INST                       (UART_0_INST)
#define BT_UART_IRQN                       (UART_0_INST_INT_IRQN)
#define BT_UART_TX_IOMUX                   (GPIO_UART_0_IOMUX_TX)
#define BT_UART_TX_IOMUX_FUNC              (GPIO_UART_0_IOMUX_TX_FUNC)
#define BT_UART_RX_IOMUX                   (GPIO_UART_0_IOMUX_RX)
#define BT_UART_RX_IOMUX_FUNC              (GPIO_UART_0_IOMUX_RX_FUNC)
#define BT_UART_MANUAL_INIT                (0)
#define BT_UART_ENABLE_TX                  (1)
#define BT_UART_DIRECTION                  DL_UART_MAIN_DIRECTION_TX_RX
#define BT_UART_READY_TEXT                 "BT UART1 PA17-TX PA18-RX 9600 ready\r\n"
#define BT_UART_LINK_TEST                  (0)
#define BT_LINK_TEST_INTERVAL_MS           (500UL)

#define BT_UART_IBRD_40_MHZ_115200         (21U)
#define BT_UART_FBRD_40_MHZ_115200         (45U)

#define BT_RX_LINE_LEN                     (64U)
#define BT_RX_QUEUE_LEN                    (4U)
#define BT_TX_BUFFER_LEN                   (512U)
#define BT_RX_DISPLAY_LEN                  (46U)
#define BT_START_SPEED_SCALE               (0.85f)
#define BT_MIN_BASE_SPEED                  (0.0f)
#define BT_MAX_BASE_SPEED                  (30.0f)
#define BT_RECENT_TIMEOUT_MS               (3000UL)
#define BT_HEARTBEAT_INTERVAL_MS           (1000UL)
#define BT_COMMAND_IDLE_FLUSH_MS           (40UL)

#define BT_ENCODER_PULSES_PER_MM           (ENCODER_COUNTS_PER_MM)

#define BT_FORWARD_MIN_DISTANCE_M          (0.02f)
#define BT_FORWARD_MAX_DISTANCE_M          (10.0f)
#define BT_FORWARD_CRUISE_PULSES_TICK      (5.0f)
#define BT_FORWARD_MIN_PULSES_TICK         (1.5f)
#define BT_FORWARD_BRAKE_DISTANCE_MM       (250.0f)
#define BT_FORWARD_STOP_TOLERANCE_MM       (8.0f)
#define BT_FORWARD_ACCEL_PULSES_TICK2      (0.25f)
#define BT_FORWARD_DECEL_PULSES_TICK2      (0.40f)
#define BT_FORWARD_YAW_KP                  (0.12f)
#define BT_FORWARD_YAW_KD_RATE             (0.008f)
#define BT_FORWARD_YAW_CORR_LIMIT          (2.0f)
#define BT_FORWARD_YAW_SIGN                (1.0f)

#define BT_TURN_MIN_ANGLE_DEG              (1.0f)
#define BT_TURN_MAX_ANGLE_DEG              (360.0f)
#define BT_TURN_YAW_KP                     (0.07f)
#define BT_TURN_YAW_KD_RATE                (0.010f)
#define BT_TURN_MOTOR_SIGN                 (1.0f)
#define BT_TURN_MAX_PULSES_TICK            (4.5f)
#define BT_TURN_MIN_PULSES_TICK            (1.3f)
#define BT_TURN_ACCEL_PULSES_TICK2         (0.25f)
#define BT_TURN_DECEL_PULSES_TICK2         (0.45f)
#define BT_TURN_YAW_TOLERANCE_DEG          (1.5f)
#define BT_TURN_RATE_TOLERANCE_DPS         (8.0f)
#define BT_TURN_SETTLE_TICKS               (15U)
#define BT_TURN_TIMEOUT_TICKS              (2000UL)

#define BT_SPEED_PI_KP                     (700.0f)
#define BT_SPEED_PI_KI                     (80.0f)
#define BT_SPEED_PI_INTEGRAL_LIMIT         (12000.0f)
#define BT_SPEED_MIN_PWM                   (10500.0f)
#define BT_SPEED_FF_PWM_PER_PULSE          (900.0f)
#define BT_SPEED_STALL_BOOST_PWM           (6000.0f)
#define BT_SPEED_MAX_PWM                   (30000.0f)

typedef enum {
    BT_MOTION_IDLE = 0,
    BT_MOTION_TURN_LEFT,
    BT_MOTION_TURN_RIGHT,
    BT_MOTION_FORWARD,
} BtMotionMode;

typedef enum {
    BT_MOTION_RESULT_NONE = 0,
    BT_MOTION_RESULT_DONE,
    BT_MOTION_RESULT_STOPPED,
    BT_MOTION_RESULT_TIMEOUT,
} BtMotionResult;

typedef struct {
    volatile BtMotionMode mode;
    float requested_value;
    float target_yaw_deg;
    float turn_target_delta_deg;
    float turn_progress_deg;
    float turn_last_yaw_deg;
    int32_t target_pulses;
    int32_t accum_pulses_a;
    int32_t accum_pulses_b;
    float command_base_speed;
    float command_speed_a;
    float command_speed_b;
    float speed_integral_a;
    float speed_integral_b;
    uint16_t settle_ticks;
    uint32_t elapsed_ticks;
    uint32_t timeout_ticks;
    volatile BtMotionResult pending_result;
    BtMotionMode completed_mode;
    float completed_value;
} BtMotionControl;

static volatile char g_bt_rx_line[BT_RX_LINE_LEN];
static volatile uint8_t g_bt_rx_index;
static volatile char g_bt_rx_display[BT_RX_DISPLAY_LEN];
static volatile uint8_t g_bt_rx_display_len;
static volatile bool g_bt_rx_display_new_line;
static volatile char g_bt_cmd_queue[BT_RX_QUEUE_LEN][BT_RX_LINE_LEN];
static volatile uint8_t g_bt_cmd_head;
static volatile uint8_t g_bt_cmd_tail;
static volatile uint8_t g_bt_cmd_count;

static char g_bt_tx_buffer[BT_TX_BUFFER_LEN];
static volatile uint16_t g_bt_tx_head;
static volatile uint16_t g_bt_tx_tail;

static volatile uint32_t g_bt_rx_byte_count;
static volatile uint32_t g_bt_command_count;
static volatile uint32_t g_bt_dropped_command_count;
static volatile uint32_t g_bt_rx_error_count;
static volatile uint32_t g_bt_tx_byte_count;
static volatile unsigned long g_bt_last_rx_ms;
static unsigned long g_bt_last_heartbeat_ms;
static bool g_bt_initialized;
static BtMotionControl g_bt_motion;

extern bool start, turnMark;
extern uint8_t turncount;
extern float TargetSpeed, basespeed;
extern pid_t pidMotorA, pidMotorB, pidLine;
extern float yaw;

static float bt_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t bt_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float bt_clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float bt_wrap_angle_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float bt_slew(float current, float target,
                     float accel_step, float decel_step)
{
    float step = (bt_abs_float(target) > bt_abs_float(current)) ?
        accel_step : decel_step;

    if (target > current + step) {
        return current + step;
    }
    if (target < current - step) {
        return current - step;
    }
    return target;
}

static void bt_motion_reset_control_locked(void)
{
    g_bt_motion.command_base_speed = 0.0f;
    g_bt_motion.command_speed_a = 0.0f;
    g_bt_motion.command_speed_b = 0.0f;
    g_bt_motion.speed_integral_a = 0.0f;
    g_bt_motion.speed_integral_b = 0.0f;
    g_bt_motion.settle_ticks = 0U;
    g_bt_motion.elapsed_ticks = 0UL;
    g_bt_motion.accum_pulses_a = 0;
    g_bt_motion.accum_pulses_b = 0;
    g_bt_motion.turn_progress_deg = 0.0f;
}

static void bt_motion_stop_locked(BtMotionResult result,
                                  float completed_value)
{
    BtMotionMode completed_mode = g_bt_motion.mode;

    Load(0, 0);
    g_bt_motion.mode = BT_MOTION_IDLE;
    g_bt_motion.command_speed_a = 0.0f;
    g_bt_motion.command_speed_b = 0.0f;
    g_bt_motion.speed_integral_a = 0.0f;
    g_bt_motion.speed_integral_b = 0.0f;
    g_bt_motion.settle_ticks = 0U;
    g_bt_motion.completed_mode = completed_mode;
    g_bt_motion.completed_value = completed_value;
    g_bt_motion.pending_result = result;
    start = false;
    TargetSpeed = 0.0f;
}

static float bt_speed_measure_for_target(float target, int16_t raw_delta)
{
    float magnitude = (float)((raw_delta < 0) ? -raw_delta : raw_delta);

    return (target < 0.0f) ? -magnitude : magnitude;
}

static int bt_speed_pi_update(float target, int16_t raw_delta,
                              float *integral)
{
    float measured;
    float error;
    float feedforward;
    float output;
    float sign;

    if (bt_abs_float(target) < 0.05f) {
        *integral = 0.0f;
        return 0;
    }

    sign = (target < 0.0f) ? -1.0f : 1.0f;
    measured = bt_speed_measure_for_target(target, raw_delta);
    error = target - measured;
    *integral += BT_SPEED_PI_KI * error;
    *integral = bt_clamp_float(*integral,
                               -BT_SPEED_PI_INTEGRAL_LIMIT,
                               BT_SPEED_PI_INTEGRAL_LIMIT);

    feedforward = sign *
        (BT_SPEED_MIN_PWM +
         BT_SPEED_FF_PWM_PER_PULSE * bt_abs_float(target));
    output = feedforward + BT_SPEED_PI_KP * error + *integral;
    if(raw_delta == 0) {
        output += sign * BT_SPEED_STALL_BOOST_PWM;
    }
    output = bt_clamp_float(output, -BT_SPEED_MAX_PWM, BT_SPEED_MAX_PWM);

    if ((sign > 0.0f) && (output < 0.0f)) {
        output = 0.0f;
    } else if ((sign < 0.0f) && (output > 0.0f)) {
        output = 0.0f;
    }
    return (int)output;
}

static void bt_configure_uart(void)
{
#if BT_UART_MANUAL_INIT
    static const DL_UART_Main_ClockConfig clock_config = {
        .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    static const DL_UART_Main_Config uart_config = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = BT_UART_DIRECTION,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_MAIN_PARITY_NONE,
        .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_MAIN_STOP_BITS_ONE,
    };

    DL_UART_Main_reset(BT_UART_INST);
    DL_UART_Main_enablePower(BT_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_UART_Main_setClockConfig(BT_UART_INST,
                                (DL_UART_Main_ClockConfig *)&clock_config);
    DL_UART_Main_init(BT_UART_INST,
                      (DL_UART_Main_Config *)&uart_config);
    DL_UART_Main_setOversampling(BT_UART_INST,
                                 DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(BT_UART_INST,
        BT_UART_IBRD_40_MHZ_115200,
        BT_UART_FBRD_40_MHZ_115200);
#endif

#if BT_UART_ENABLE_TX
    DL_GPIO_initPeripheralOutputFunction(BT_UART_TX_IOMUX,
                                         BT_UART_TX_IOMUX_FUNC);
#endif
    DL_GPIO_initPeripheralInputFunctionFeatures(BT_UART_RX_IOMUX,
        BT_UART_RX_IOMUX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_UART_Main_setRXFIFOThreshold(BT_UART_INST,
                                    DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(BT_UART_INST,
                                    DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);
    DL_UART_Main_enable(BT_UART_INST);
}

static bool ascii_equal_ignore_case(const char *a, const char *b)
{
    char ca;
    char cb;

    while ((*a != '\0') && (*b != '\0')) {
        ca = *a;
        cb = *b;
        if ((ca >= 'a') && (ca <= 'z')) {
            ca = (char)(ca - ('a' - 'A'));
        }
        if ((cb >= 'a') && (cb <= 'z')) {
            cb = (char)(cb - ('a' - 'A'));
        }
        if (ca != cb) {
            return false;
        }
        a++;
        b++;
    }

    return (*a == '\0') && (*b == '\0');
}

static bool ascii_starts_with_ignore_case(const char *text, const char *prefix)
{
    char ct;
    char cp;

    while (*prefix != '\0') {
        ct = *text;
        cp = *prefix;
        if ((ct >= 'a') && (ct <= 'z')) {
            ct = (char)(ct - ('a' - 'A'));
        }
        if ((cp >= 'a') && (cp <= 'z')) {
            cp = (char)(cp - ('a' - 'A'));
        }
        if (ct != cp) {
            return false;
        }
        text++;
        prefix++;
    }

    return true;
}

static const char *skip_spaces(const char *text)
{
    while ((*text == ' ') || (*text == '\t')) {
        text++;
    }
    return text;
}

static bool parse_float_value(const char *text, float *value)
{
    bool negative = false;
    bool have_digit = false;
    float result = 0.0f;
    float scale = 0.1f;

    text = skip_spaces(text);
    if (*text == '-') {
        negative = true;
        text++;
    } else if (*text == '+') {
        text++;
    }

    while ((*text >= '0') && (*text <= '9')) {
        have_digit = true;
        result = (result * 10.0f) + (float)(*text - '0');
        text++;
    }

    if (*text == '.') {
        text++;
        while ((*text >= '0') && (*text <= '9')) {
            have_digit = true;
            result += (float)(*text - '0') * scale;
            scale *= 0.1f;
            text++;
        }
    }

    if (!have_digit) {
        return false;
    }

    text = skip_spaces(text);
    if (*text != '\0') {
        return false;
    }

    *value = negative ? -result : result;
    return true;
}

static bool bt_imu_ready(void)
{
    return (MPU6500_GetInitStatus() == 0U) &&
           (MPU6500_GetUpdateCount() > 0U);
}

static void bt_motion_prepare_common_locked(void)
{
    start = false;
    TargetSpeed = 0.0f;
    PID_Reset(&pidMotorA);
    PID_Reset(&pidMotorB);
    PID_Reset(&pidLine);
    PID_LineControlReset();
    Load(0, 0);
    bt_motion_reset_control_locked();
}

static bool bt_motion_start_turn(bool left, float angle_deg)
{
    if ((angle_deg < BT_TURN_MIN_ANGLE_DEG) ||
        (angle_deg > BT_TURN_MAX_ANGLE_DEG) || !bt_imu_ready()) {
        return false;
    }

    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    if (g_bt_motion.mode != BT_MOTION_IDLE) {
        NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
        return false;
    }

    bt_motion_prepare_common_locked();
    g_bt_motion.mode = left ? BT_MOTION_TURN_LEFT : BT_MOTION_TURN_RIGHT;
    g_bt_motion.requested_value = angle_deg;
    g_bt_motion.target_yaw_deg = bt_wrap_angle_deg(
        yaw + (left ? angle_deg : -angle_deg));
    g_bt_motion.turn_target_delta_deg = left ? angle_deg : -angle_deg;
    g_bt_motion.turn_progress_deg = 0.0f;
    g_bt_motion.turn_last_yaw_deg = yaw;
    g_bt_motion.target_pulses = 0;
    g_bt_motion.timeout_ticks = BT_TURN_TIMEOUT_TICKS;
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    return true;
}

static bool bt_motion_start_forward(float distance_m)
{
    float distance_mm;
    uint32_t timeout_ticks;

    if ((distance_m < BT_FORWARD_MIN_DISTANCE_M) ||
        (distance_m > BT_FORWARD_MAX_DISTANCE_M) || !bt_imu_ready()) {
        return false;
    }

    distance_mm = distance_m * 1000.0f;
    timeout_ticks = 500UL + (uint32_t)(distance_m * 2000.0f);

    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    if (g_bt_motion.mode != BT_MOTION_IDLE) {
        NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
        return false;
    }

    bt_motion_prepare_common_locked();
    g_bt_motion.mode = BT_MOTION_FORWARD;
    g_bt_motion.requested_value = distance_m;
    g_bt_motion.target_yaw_deg = yaw;
    g_bt_motion.target_pulses =
        (int32_t)(distance_mm * BT_ENCODER_PULSES_PER_MM + 0.5f);
    if (g_bt_motion.target_pulses < 1) {
        g_bt_motion.target_pulses = 1;
    }
    g_bt_motion.timeout_ticks = timeout_ticks;
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    return true;
}

static void bt_motion_stop_from_task(void)
{
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    if (g_bt_motion.mode != BT_MOTION_IDLE) {
        bt_motion_stop_locked(BT_MOTION_RESULT_STOPPED,
                              g_bt_motion.requested_value);
    } else {
        Load(0, 0);
        start = false;
        TargetSpeed = 0.0f;
    }
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

bool Bluetooth_MotionIsActive(void)
{
    return g_bt_motion.mode != BT_MOTION_IDLE;
}

bool Bluetooth_MotionControlTick10ms(int16_t encoder_delta_a,
                                     int16_t encoder_delta_b)
{
    BtMotionMode mode = g_bt_motion.mode;
    float yaw_error;
    float yaw_delta;
    float yaw_rate;
    float desired_turn_speed;
    float remaining_pulses;
    float remaining_mm;
    float desired_forward_speed;
    float yaw_correction;
    float progress_value;
    int32_t average_pulses;
    int pwm_a;
    int pwm_b;

    if (mode == BT_MOTION_IDLE) {
        return false;
    }

    g_bt_motion.elapsed_ticks++;
    if (g_bt_motion.elapsed_ticks > g_bt_motion.timeout_ticks) {
        bt_motion_stop_locked(BT_MOTION_RESULT_TIMEOUT,
                              g_bt_motion.requested_value);
        return true;
    }

    yaw_rate = MPU6500_GetYawRateDps();

    if ((mode == BT_MOTION_TURN_LEFT) ||
        (mode == BT_MOTION_TURN_RIGHT)) {
        yaw_delta = bt_wrap_angle_deg(yaw - g_bt_motion.turn_last_yaw_deg);
        g_bt_motion.turn_last_yaw_deg = yaw;
        g_bt_motion.turn_progress_deg += yaw_delta;
        yaw_error = g_bt_motion.turn_target_delta_deg -
                    g_bt_motion.turn_progress_deg;
        desired_turn_speed = BT_TURN_MOTOR_SIGN *
            (BT_TURN_YAW_KP * yaw_error -
             BT_TURN_YAW_KD_RATE * yaw_rate);
        desired_turn_speed = bt_clamp_float(
            desired_turn_speed,
            -BT_TURN_MAX_PULSES_TICK,
            BT_TURN_MAX_PULSES_TICK);

        if ((bt_abs_float(yaw_error) > BT_TURN_YAW_TOLERANCE_DEG) &&
            (bt_abs_float(desired_turn_speed) < BT_TURN_MIN_PULSES_TICK)) {
            desired_turn_speed = BT_TURN_MOTOR_SIGN *
                ((yaw_error < 0.0f) ?
                 -BT_TURN_MIN_PULSES_TICK : BT_TURN_MIN_PULSES_TICK);
        }

        g_bt_motion.command_speed_a = bt_slew(
            g_bt_motion.command_speed_a,
            desired_turn_speed,
            BT_TURN_ACCEL_PULSES_TICK2,
            BT_TURN_DECEL_PULSES_TICK2);
        g_bt_motion.command_speed_b = -g_bt_motion.command_speed_a;

        if ((bt_abs_float(yaw_error) <= BT_TURN_YAW_TOLERANCE_DEG) &&
            (bt_abs_float(yaw_rate) <= BT_TURN_RATE_TOLERANCE_DPS)) {
            if (g_bt_motion.settle_ticks < BT_TURN_SETTLE_TICKS) {
                g_bt_motion.settle_ticks++;
            }
        } else {
            g_bt_motion.settle_ticks = 0U;
        }

        if (g_bt_motion.settle_ticks >= BT_TURN_SETTLE_TICKS) {
            progress_value = bt_abs_float(g_bt_motion.turn_progress_deg);
            bt_motion_stop_locked(BT_MOTION_RESULT_DONE, progress_value);
            return true;
        }
    } else {
        yaw_error = bt_wrap_angle_deg(g_bt_motion.target_yaw_deg - yaw);
        g_bt_motion.accum_pulses_a += bt_abs_i32(encoder_delta_a);
        g_bt_motion.accum_pulses_b += bt_abs_i32(encoder_delta_b);
        average_pulses = (g_bt_motion.accum_pulses_a +
                          g_bt_motion.accum_pulses_b) / 2;
        remaining_pulses = (float)(g_bt_motion.target_pulses - average_pulses);
        remaining_mm = remaining_pulses / BT_ENCODER_PULSES_PER_MM;

        if (remaining_mm <= BT_FORWARD_STOP_TOLERANCE_MM) {
            progress_value = (float)average_pulses /
                             BT_ENCODER_PULSES_PER_MM / 1000.0f;
            bt_motion_stop_locked(BT_MOTION_RESULT_DONE, progress_value);
            return true;
        }

        desired_forward_speed = BT_FORWARD_CRUISE_PULSES_TICK;
        if (remaining_mm < BT_FORWARD_BRAKE_DISTANCE_MM) {
            desired_forward_speed = BT_FORWARD_MIN_PULSES_TICK +
                (BT_FORWARD_CRUISE_PULSES_TICK -
                 BT_FORWARD_MIN_PULSES_TICK) *
                (remaining_mm / BT_FORWARD_BRAKE_DISTANCE_MM);
        }
        desired_forward_speed = bt_clamp_float(
            desired_forward_speed,
            BT_FORWARD_MIN_PULSES_TICK,
            BT_FORWARD_CRUISE_PULSES_TICK);

        g_bt_motion.command_base_speed = bt_slew(
            g_bt_motion.command_base_speed,
            desired_forward_speed,
            BT_FORWARD_ACCEL_PULSES_TICK2,
            BT_FORWARD_DECEL_PULSES_TICK2);

        yaw_correction = BT_FORWARD_YAW_SIGN *
            (BT_FORWARD_YAW_KP * yaw_error -
             BT_FORWARD_YAW_KD_RATE * yaw_rate);
        yaw_correction = bt_clamp_float(
            yaw_correction,
            -BT_FORWARD_YAW_CORR_LIMIT,
            BT_FORWARD_YAW_CORR_LIMIT);

        g_bt_motion.command_speed_b = bt_clamp_float(
            g_bt_motion.command_base_speed - yaw_correction,
            BT_FORWARD_MIN_PULSES_TICK,
            BT_FORWARD_CRUISE_PULSES_TICK + BT_FORWARD_YAW_CORR_LIMIT);
        g_bt_motion.command_speed_a = bt_clamp_float(
            g_bt_motion.command_base_speed + yaw_correction,
            BT_FORWARD_MIN_PULSES_TICK,
            BT_FORWARD_CRUISE_PULSES_TICK + BT_FORWARD_YAW_CORR_LIMIT);
    }

    pwm_a = bt_speed_pi_update(g_bt_motion.command_speed_a,
                               encoder_delta_a,
                               &g_bt_motion.speed_integral_a);
    pwm_b = bt_speed_pi_update(g_bt_motion.command_speed_b,
                               encoder_delta_b,
                               &g_bt_motion.speed_integral_b);
    Load(pwm_a, pwm_b);
    return true;
}

static void bt_queue_command_from_isr(const char *cmd)
{
    uint8_t tail;
    uint8_t i;

    if (g_bt_cmd_count >= BT_RX_QUEUE_LEN) {
        g_bt_dropped_command_count++;
        return;
    }

    tail = g_bt_cmd_tail;
    for (i = 0U; i < (BT_RX_LINE_LEN - 1U); i++) {
        g_bt_cmd_queue[tail][i] = cmd[i];
        if (cmd[i] == '\0') {
            break;
        }
    }
    g_bt_cmd_queue[tail][BT_RX_LINE_LEN - 1U] = '\0';

    g_bt_cmd_tail = (uint8_t)((tail + 1U) % BT_RX_QUEUE_LEN);
    g_bt_cmd_count++;
    g_bt_command_count++;
}

static void bt_finish_line_from_isr(void)
{
    char cmd[BT_RX_LINE_LEN];
    uint8_t i;

    if (g_bt_rx_index == 0U) {
        return;
    }

    for (i = 0U; i < g_bt_rx_index; i++) {
        cmd[i] = (char)g_bt_rx_line[i];
    }
    cmd[g_bt_rx_index] = '\0';
    g_bt_rx_index = 0U;
    bt_queue_command_from_isr(cmd);
}

static void bt_append_display_ascii_from_isr(char c)
{
    uint8_t i;

    if (g_bt_rx_display_new_line) {
        g_bt_rx_display_len = 0U;
        g_bt_rx_display[0] = '\0';
        g_bt_rx_display_new_line = false;
    }

    if (g_bt_rx_display_len < (BT_RX_DISPLAY_LEN - 1U)) {
        g_bt_rx_display[g_bt_rx_display_len] = c;
        g_bt_rx_display_len++;
    } else {
        for (i = 1U; i < (BT_RX_DISPLAY_LEN - 1U); i++) {
            g_bt_rx_display[i - 1U] = g_bt_rx_display[i];
        }
        g_bt_rx_display[BT_RX_DISPLAY_LEN - 2U] = c;
    }
    g_bt_rx_display[g_bt_rx_display_len] = '\0';
}

static void bt_rx_byte_from_isr(uint8_t byte)
{
    unsigned long now = 0UL;
    char c = (char)byte;

    g_bt_rx_byte_count++;
    (void)mspm0_get_clock_ms(&now);
    g_bt_last_rx_ms = now;

    if ((c == '\r') || (c == '\n') || (c == ';') || (c == '#')) {
        g_bt_rx_display_new_line = true;
        bt_finish_line_from_isr();
        return;
    }

    if ((byte < 0x20U) || (byte > 0x7EU)) {
        return;
    }

    bt_append_display_ascii_from_isr(c);

    if ((g_bt_rx_index == 0U) && (c == '1')) {
        g_bt_rx_display_new_line = true;
        bt_queue_command_from_isr("START");
        return;
    }
    if ((g_bt_rx_index == 0U) && (c == '0')) {
        g_bt_rx_display_new_line = true;
        bt_queue_command_from_isr("STOP");
        return;
    }
    if ((g_bt_rx_index == 0U) && (c == '?')) {
        g_bt_rx_display_new_line = true;
        bt_queue_command_from_isr("STATUS");
        return;
    }

    if (g_bt_rx_index < (BT_RX_LINE_LEN - 1U)) {
        g_bt_rx_line[g_bt_rx_index] = c;
        g_bt_rx_index++;
    } else {
        bt_finish_line_from_isr();
    }
}

static bool bt_pop_command(char *cmd)
{
    uint8_t head;
    uint8_t i;

    NVIC_DisableIRQ(BT_UART_IRQN);
    if (g_bt_cmd_count == 0U) {
        NVIC_EnableIRQ(BT_UART_IRQN);
        return false;
    }

    head = g_bt_cmd_head;
    for (i = 0U; i < BT_RX_LINE_LEN; i++) {
        cmd[i] = (char)g_bt_cmd_queue[head][i];
        if (cmd[i] == '\0') {
            break;
        }
    }
    cmd[BT_RX_LINE_LEN - 1U] = '\0';

    g_bt_cmd_head = (uint8_t)((head + 1U) % BT_RX_QUEUE_LEN);
    g_bt_cmd_count--;
    NVIC_EnableIRQ(BT_UART_IRQN);
    return true;
}

static void bt_tx_drain(void)
{
#if BT_UART_ENABLE_TX
    while (g_bt_tx_tail != g_bt_tx_head) {
        if (!DL_UART_Main_transmitDataCheck(
                BT_UART_INST, (uint8_t)g_bt_tx_buffer[g_bt_tx_tail])) {
            break;
        }
        g_bt_tx_byte_count++;
        g_bt_tx_tail = (uint16_t)((g_bt_tx_tail + 1U) % BT_TX_BUFFER_LEN);
    }
#else
    g_bt_tx_tail = g_bt_tx_head;
#endif
}

#if BT_UART_LINK_TEST
static void bt_link_test_send_blocking(const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        DL_UART_Main_transmitDataBlocking(BT_UART_INST, (uint8_t)*text);
        text++;
    }
}
#endif

static void bt_tx_push_char(char c)
{
#if BT_UART_ENABLE_TX
    uint16_t next_head = (uint16_t)((g_bt_tx_head + 1U) % BT_TX_BUFFER_LEN);

    if (next_head == g_bt_tx_tail) {
        bt_tx_drain();
        next_head = (uint16_t)((g_bt_tx_head + 1U) % BT_TX_BUFFER_LEN);
        if (next_head == g_bt_tx_tail) {
            return;
        }
    }

    g_bt_tx_buffer[g_bt_tx_head] = c;
    g_bt_tx_head = next_head;
#else
    (void)c;
#endif
}

static void bt_rx_poll_fifo(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(BT_UART_INST)) {
        bt_rx_byte_from_isr((uint8_t)DL_UART_Main_receiveData(BT_UART_INST));
    }
}

void Bluetooth_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

#if BT_UART_ENABLE_TX
    while (*text != '\0') {
        bt_tx_push_char(*text);
        text++;
    }
    bt_tx_drain();
#else
    (void)text;
#endif
}

static void bt_reply_ok(const char *message)
{
    Bluetooth_SendString("OK ");
    Bluetooth_SendString(message);
    Bluetooth_SendString("\r\n");
}

static void bt_reply_err(const char *message)
{
    Bluetooth_SendString("ERR ");
    Bluetooth_SendString(message);
    Bluetooth_SendString("\r\n");
}

static void bt_start_line_mode(void)
{
    bt_motion_stop_from_task();
    PID_Reset(&pidMotorA);
    PID_Reset(&pidMotorB);
    PID_Reset(&pidLine);
    PID_LineControlReset();
    turnMark = false;
    turncount = 0U;
    TargetSpeed = basespeed * BT_START_SPEED_SCALE;
    start = true;
    DL_GPIO_togglePins(GPIO_RGB_PORT, GPIO_RGB_USER_LED_2_PIN);
}

static void bt_stop_line_mode(void)
{
    bt_motion_stop_from_task();
    start = false;
    TargetSpeed = 0.0f;
    PID_Reset(&pidMotorA);
    PID_Reset(&pidMotorB);
    PID_Reset(&pidLine);
    PID_LineControlReset();
    Load(0, 0);
}

static void bt_send_status(void)
{
    char buffer[160];
    const char *motion_name = "IDLE";
    BtMotionMode motion_mode;
    int32_t average_pulses;
    float turn_progress;
    float progress;
    int len;

    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    motion_mode = g_bt_motion.mode;
    average_pulses = (g_bt_motion.accum_pulses_a +
                      g_bt_motion.accum_pulses_b) / 2;
    turn_progress = g_bt_motion.turn_progress_deg;
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    if (motion_mode == BT_MOTION_TURN_LEFT) {
        motion_name = "LEFT";
        progress = bt_abs_float(turn_progress);
    } else if (motion_mode == BT_MOTION_TURN_RIGHT) {
        motion_name = "RIGHT";
        progress = bt_abs_float(turn_progress);
    } else if (motion_mode == BT_MOTION_FORWARD) {
        motion_name = "FORWARD";
        progress = (float)average_pulses /
                   BT_ENCODER_PULSES_PER_MM / 1000.0f;
    } else {
        progress = 0.0f;
    }

    len = snprintf(buffer, sizeof(buffer),
                   "RUN=%u MOTION=%s VALUE=%.3f YAW=%.3f TX=%lu RX=%lu CMD=%lu DROP=%lu ERR=%lu\r\n",
                   start ? 1U : 0U,
                   motion_name,
                   (double)progress,
                   (double)yaw,
                   (unsigned long)g_bt_tx_byte_count,
                   (unsigned long)g_bt_rx_byte_count,
                   (unsigned long)g_bt_command_count,
                   (unsigned long)g_bt_dropped_command_count,
                   (unsigned long)g_bt_rx_error_count);
    if (len > 0) {
        Bluetooth_SendString(buffer);
    }
}

static void bt_send_help(void)
{
    Bluetooth_SendString(
        "CMD: left 90, right 45, forward 1.5, stop, status, start, SPD=10\r\n");
}

static void bt_handle_speed_command(const char *value_text)
{
    float value;

    if (!parse_float_value(value_text, &value)) {
        bt_reply_err("SPD");
        return;
    }

    if (value < BT_MIN_BASE_SPEED) {
        value = BT_MIN_BASE_SPEED;
    } else if (value > BT_MAX_BASE_SPEED) {
        value = BT_MAX_BASE_SPEED;
    }

    basespeed = value;
    if (start) {
        TargetSpeed = basespeed * BT_START_SPEED_SCALE;
    }
    bt_reply_ok("SPD");
}

static void bt_handle_track_command(const char *value_text)
{
    (void)value_text;
    K210Face_SetTrackingEnabled(false);
    bt_reply_err("GIMBAL_OFF");
}

static void bt_handle_turn_command(const char *value_text, bool left)
{
    char reply[48];
    float angle_deg;

    if (!parse_float_value(value_text, &angle_deg) ||
        (angle_deg < BT_TURN_MIN_ANGLE_DEG) ||
        (angle_deg > BT_TURN_MAX_ANGLE_DEG)) {
        bt_reply_err("ANGLE_1_TO_360");
        return;
    }
    if (Bluetooth_MotionIsActive()) {
        bt_reply_err("BUSY_USE_STOP");
        return;
    }
    if (!bt_imu_ready()) {
        bt_reply_err("IMU_NOT_READY");
        return;
    }
    if (!bt_motion_start_turn(left, angle_deg)) {
        bt_reply_err("TURN_START");
        return;
    }

    (void)snprintf(reply, sizeof(reply), "%s %.3f",
                   left ? "LEFT" : "RIGHT", (double)angle_deg);
    bt_reply_ok(reply);
}

static void bt_handle_forward_command(const char *value_text)
{
    char reply[48];
    float distance_m;

    if (!parse_float_value(value_text, &distance_m) ||
        (distance_m < BT_FORWARD_MIN_DISTANCE_M) ||
        (distance_m > BT_FORWARD_MAX_DISTANCE_M)) {
        bt_reply_err("DIST_0.02_TO_10M");
        return;
    }
    if (Bluetooth_MotionIsActive()) {
        bt_reply_err("BUSY_USE_STOP");
        return;
    }
    if (!bt_imu_ready()) {
        bt_reply_err("IMU_NOT_READY");
        return;
    }
    if (!bt_motion_start_forward(distance_m)) {
        bt_reply_err("FORWARD_START");
        return;
    }

    (void)snprintf(reply, sizeof(reply), "FORWARD %.3fM",
                   (double)distance_m);
    bt_reply_ok(reply);
}

static void bt_handle_command(char *cmd)
{
    const char *arg;

    cmd = (char *)skip_spaces(cmd);
    if (*cmd == '\0') {
        return;
    }

#if BT_UART_LINK_TEST
    if (ascii_equal_ignore_case(cmd, "ACK")) {
        bt_link_test_send_blocking("ack1\r\n");
    }
    return;
#endif

    if (ascii_equal_ignore_case(cmd, "START") ||
        ascii_equal_ignore_case(cmd, "RUN")) {
        if (Bluetooth_MotionIsActive()) {
            bt_reply_err("BUSY_USE_STOP");
            return;
        }
        bt_start_line_mode();
        bt_reply_ok("START");
    } else if (ascii_equal_ignore_case(cmd, "STOP") ||
               ascii_equal_ignore_case(cmd, "HALT")) {
        bt_stop_line_mode();
        bt_reply_ok("STOP");
    } else if (ascii_equal_ignore_case(cmd, "STATUS") ||
               ascii_equal_ignore_case(cmd, "STAT")) {
        bt_send_status();
    } else if (ascii_equal_ignore_case(cmd, "HELP")) {
        bt_send_help();
    } else if (ascii_starts_with_ignore_case(cmd, "SPD=")) {
        bt_handle_speed_command(cmd + 4);
    } else if (ascii_starts_with_ignore_case(cmd, "SPEED=")) {
        bt_handle_speed_command(cmd + 6);
    } else if (ascii_starts_with_ignore_case(cmd, "BASE=")) {
        bt_handle_speed_command(cmd + 5);
    } else if (ascii_starts_with_ignore_case(cmd, "TRACK")) {
        arg = cmd + 5;
        bt_handle_track_command(arg);
    } else if (ascii_starts_with_ignore_case(cmd, "LEFT") &&
               ((cmd[4] == ' ') || (cmd[4] == '\t'))) {
        bt_handle_turn_command(cmd + 5, true);
    } else if (ascii_starts_with_ignore_case(cmd, "RIGHT") &&
               ((cmd[5] == ' ') || (cmd[5] == '\t'))) {
        bt_handle_turn_command(cmd + 6, false);
    } else if (ascii_starts_with_ignore_case(cmd, "FORWARD") &&
               ((cmd[7] == ' ') || (cmd[7] == '\t'))) {
        bt_handle_forward_command(cmd + 8);
    } else if (ascii_starts_with_ignore_case(cmd, "FWD") &&
               ((cmd[3] == ' ') || (cmd[3] == '\t'))) {
        bt_handle_forward_command(cmd + 4);
    } else {
        bt_reply_err("UNKNOWN");
    }
}

static void bt_report_motion_result(void)
{
    BtMotionResult result;
    BtMotionMode mode;
    float value;
    const char *name = "MOTION";
    char message[80];

    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    result = g_bt_motion.pending_result;
    mode = g_bt_motion.completed_mode;
    value = g_bt_motion.completed_value;
    g_bt_motion.pending_result = BT_MOTION_RESULT_NONE;
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    if (result == BT_MOTION_RESULT_NONE) {
        return;
    }
    if (mode == BT_MOTION_TURN_LEFT) {
        name = "LEFT";
    } else if (mode == BT_MOTION_TURN_RIGHT) {
        name = "RIGHT";
    } else if (mode == BT_MOTION_FORWARD) {
        name = "FORWARD";
    }

    if (result == BT_MOTION_RESULT_DONE) {
        if (mode == BT_MOTION_FORWARD) {
            (void)snprintf(message, sizeof(message),
                           "DONE %s %.3fM\r\n", name, (double)value);
        } else {
            (void)snprintf(message, sizeof(message),
                           "DONE %s %.3fDEG\r\n", name, (double)value);
        }
        Bluetooth_SendString(message);
    } else if (result == BT_MOTION_RESULT_TIMEOUT) {
        (void)snprintf(message, sizeof(message),
                       "ERR TIMEOUT %s %.3f\r\n", name, (double)value);
        Bluetooth_SendString(message);
    } else {
        (void)snprintf(message, sizeof(message),
                       "STOPPED %s\r\n", name);
        Bluetooth_SendString(message);
    }
}

void Bluetooth_Init(void)
{
    uint32_t rx_interrupts =
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR;

    NVIC_DisableIRQ(BT_UART_IRQN);
    bt_configure_uart();
    DL_UART_Main_disableInterrupt(BT_UART_INST, rx_interrupts);
    while (!DL_UART_Main_isRXFIFOEmpty(BT_UART_INST)) {
        (void)DL_UART_Main_receiveData(BT_UART_INST);
    }
    DL_UART_Main_clearInterruptStatus(BT_UART_INST, rx_interrupts);

    g_bt_rx_index = 0U;
    g_bt_rx_display_len = 0U;
    g_bt_rx_display[0] = '\0';
    g_bt_rx_display_new_line = true;
    g_bt_cmd_head = 0U;
    g_bt_cmd_tail = 0U;
    g_bt_cmd_count = 0U;
    g_bt_tx_head = 0U;
    g_bt_tx_tail = 0U;
    g_bt_rx_byte_count = 0UL;
    g_bt_command_count = 0UL;
    g_bt_dropped_command_count = 0UL;
    g_bt_rx_error_count = 0UL;
    g_bt_tx_byte_count = 0UL;
    g_bt_last_rx_ms = 0UL;
    g_bt_last_heartbeat_ms = 0UL;
    g_bt_motion.mode = BT_MOTION_IDLE;
    g_bt_motion.requested_value = 0.0f;
    g_bt_motion.target_yaw_deg = 0.0f;
    g_bt_motion.turn_target_delta_deg = 0.0f;
    g_bt_motion.turn_progress_deg = 0.0f;
    g_bt_motion.turn_last_yaw_deg = 0.0f;
    g_bt_motion.target_pulses = 0;
    g_bt_motion.timeout_ticks = 0UL;
    g_bt_motion.pending_result = BT_MOTION_RESULT_NONE;
    g_bt_motion.completed_mode = BT_MOTION_IDLE;
    g_bt_motion.completed_value = 0.0f;
    bt_motion_reset_control_locked();
    g_bt_initialized = true;

    DL_UART_Main_enableInterrupt(BT_UART_INST, rx_interrupts);
    NVIC_ClearPendingIRQ(BT_UART_IRQN);
    NVIC_EnableIRQ(BT_UART_IRQN);

#if BT_UART_LINK_TEST
    bt_link_test_send_blocking("hello\r\n");
#else
    Bluetooth_SendString(BT_UART_READY_TEXT);
    bt_send_help();
#endif
}

void Bluetooth_Task(void)
{
    char cmd[BT_RX_LINE_LEN];
#if !BT_UART_LINK_TEST
    char heartbeat[48];
#endif
    uint8_t processed = 0U;
    unsigned long now_ms = 0UL;
#if !BT_UART_LINK_TEST
    int len;
#endif

    if (!g_bt_initialized) {
        return;
    }

    bt_tx_drain();
    bt_rx_poll_fifo();
    (void)mspm0_get_clock_ms(&now_ms);
    if ((g_bt_rx_index > 0U) && (g_bt_last_rx_ms != 0UL) &&
        ((unsigned long)(now_ms - g_bt_last_rx_ms) >=
         BT_COMMAND_IDLE_FLUSH_MS)) {
        NVIC_DisableIRQ(BT_UART_IRQN);
        bt_finish_line_from_isr();
        NVIC_EnableIRQ(BT_UART_IRQN);
    }
    while ((processed < BT_RX_QUEUE_LEN) && bt_pop_command(cmd)) {
        bt_handle_command(cmd);
        processed++;
        bt_tx_drain();
        bt_rx_poll_fifo();
    }

    bt_report_motion_result();
#if BT_UART_LINK_TEST
    if ((unsigned long)(now_ms - g_bt_last_heartbeat_ms) >=
        BT_LINK_TEST_INTERVAL_MS) {
        g_bt_last_heartbeat_ms = now_ms;
        bt_link_test_send_blocking("hello\r\n");
    }
#else
    if ((unsigned long)(now_ms - g_bt_last_heartbeat_ms) >=
        BT_HEARTBEAT_INTERVAL_MS) {
        g_bt_last_heartbeat_ms = now_ms;
        len = snprintf(heartbeat, sizeof(heartbeat),
                       "BT TX=%lu RX=%lu\r\n",
                       (unsigned long)g_bt_tx_byte_count,
                       (unsigned long)g_bt_rx_byte_count);
        if (len > 0) {
            Bluetooth_SendString(heartbeat);
        }
    }
#endif
}

void Bluetooth_UART_IRQHandler(void)
{
    for (;;) {
        switch (DL_UART_Main_getPendingInterrupt(BT_UART_INST)) {
            case DL_UART_MAIN_IIDX_RX:
                while (!DL_UART_Main_isRXFIFOEmpty(BT_UART_INST)) {
                    bt_rx_byte_from_isr(
                        (uint8_t)DL_UART_Main_receiveData(BT_UART_INST));
                }
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                g_bt_rx_error_count++;
                while (!DL_UART_Main_isRXFIFOEmpty(BT_UART_INST)) {
                    (void)DL_UART_Main_receiveData(BT_UART_INST);
                }
                DL_UART_Main_clearInterruptStatus(
                    BT_UART_INST,
                    DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                    DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                    DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                    DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                    DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
                break;

            case DL_UART_MAIN_IIDX_NO_INTERRUPT:
                return;

            default:
                return;
        }
    }
}

uint32_t Bluetooth_GetRxByteCount(void)
{
    return g_bt_rx_byte_count;
}

uint32_t Bluetooth_GetTxByteCount(void)
{
    return g_bt_tx_byte_count;
}

uint32_t Bluetooth_GetCommandCount(void)
{
    return g_bt_command_count;
}

uint32_t Bluetooth_GetDroppedCommandCount(void)
{
    return g_bt_dropped_command_count;
}

uint32_t Bluetooth_GetRxErrorCount(void)
{
    return g_bt_rx_error_count;
}

void Bluetooth_GetRecentAscii(char *buffer, uint32_t buffer_len)
{
    uint32_t i;
    uint32_t copy_len;

    if ((buffer == NULL) || (buffer_len == 0U)) {
        return;
    }

    for (i = 0U; i < buffer_len; i++) {
        buffer[i] = '\0';
    }

    NVIC_DisableIRQ(BT_UART_IRQN);
    copy_len = g_bt_rx_display_len;
    if (copy_len >= buffer_len) {
        copy_len = buffer_len - 1U;
    }
    for (i = 0U; i < copy_len; i++) {
        buffer[i] = (char)g_bt_rx_display[i];
    }
    NVIC_EnableIRQ(BT_UART_IRQN);
}

bool Bluetooth_IsConnectedRecent(void)
{
    unsigned long now = 0UL;

    (void)mspm0_get_clock_ms(&now);
    return ((g_bt_last_rx_ms != 0UL) &&
            ((unsigned long)(now - g_bt_last_rx_ms) <= BT_RECENT_TIMEOUT_MS));
}

void updateUARTData(void)
{
    bt_send_status();
}

void uartNonBlockingSend(void)
{
    bt_tx_drain();
}

void UART1_IRQHandler(void)
{
    Bluetooth_UART_IRQHandler();
}

#else

void Bluetooth_Init(void)
{
}

void Bluetooth_Task(void)
{
}

void Bluetooth_SendString(const char *text)
{
    (void)text;
}

void Bluetooth_UART_IRQHandler(void)
{
}

uint32_t Bluetooth_GetRxByteCount(void)
{
    return 0UL;
}

uint32_t Bluetooth_GetTxByteCount(void)
{
    return 0UL;
}

uint32_t Bluetooth_GetCommandCount(void)
{
    return 0UL;
}

uint32_t Bluetooth_GetDroppedCommandCount(void)
{
    return 0UL;
}

uint32_t Bluetooth_GetRxErrorCount(void)
{
    return 0UL;
}

void Bluetooth_GetRecentAscii(char *buffer, uint32_t buffer_len)
{
    uint32_t i;

    if (buffer != NULL) {
        for (i = 0U; i < buffer_len; i++) {
            buffer[i] = '\0';
        }
    }
}

bool Bluetooth_IsConnectedRecent(void)
{
    return false;
}

bool Bluetooth_MotionIsActive(void)
{
    return false;
}

bool Bluetooth_MotionControlTick10ms(int16_t encoder_delta_a,
                                     int16_t encoder_delta_b)
{
    (void)encoder_delta_a;
    (void)encoder_delta_b;
    return false;
}

void updateUARTData(void)
{
}

void uartNonBlockingSend(void)
{
}

#endif
