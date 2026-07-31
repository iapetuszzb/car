#include <Arduino.h>
#include <HardwareTimer.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "ball_controller.h"
#include "control_config.h"

namespace {

// C06B schematic:
//   USART1 PA10(RX)/PA9(TX) is connected to the onboard CH9102 USB-UART.
//   USART3 PB11(RX)/PB10(TX) is exposed on the Bluetooth header H3.
HardwareSerial DebugSerial(PA10, PA9);
HardwareSerial K230Serial(K230_UART_RX_PIN, K230_UART_TX_PIN);

HardwareTimer* motion_timer = nullptr;
BallController ball_controller;

constexpr int32_t kQ16One = 1L << 16;
constexpr int32_t kMaximumSpeedQ16 =
    config::kMaximumMotorSpeedStepsS * kQ16One;
constexpr int32_t kAccelerationPerTickQ16 =
    (config::kMaximumMotorAccelerationStepsS2 * kQ16One) /
    static_cast<int32_t>(config::kMotionTickHz);

static_assert(kAccelerationPerTickQ16 > 0,
              "Motion tick is too fast for the configured acceleration");

volatile int32_t target_steps = 0;
volatile int32_t current_steps = 0;
volatile int32_t velocity_q16 = 0;
volatile int32_t phase_q16 = 0;
volatile int8_t output_direction = 0;
volatile uint16_t direction_setup_ticks = 0;
volatile uint16_t step_high_ticks = 0;
volatile bool motion_fault = false;

uint32_t last_vision_ms = 0;
uint32_t last_diagnostic_ms = 0;
uint32_t last_timeout_service_ms = 0;
bool have_received_vision = false;
bool vision_timed_out = true;
int32_t latest_raw_error_px = 0;
BallControlOutput latest_control_output{};

char line_buffer[20]{};
uint8_t line_length = 0;

inline int32_t absoluteInt32(int32_t value) {
    return value >= 0 ? value : -value;
}

inline int8_t signOf(int32_t value) {
    return value > 0 ? 1 : (value < 0 ? -1 : 0);
}

inline int32_t approach(int32_t current, int32_t requested,
                        int32_t max_delta) {
    if (current < requested) {
        const int32_t next = current + max_delta;
        return next > requested ? requested : next;
    }
    if (current > requested) {
        const int32_t next = current - max_delta;
        return next < requested ? requested : next;
    }
    return current;
}

inline void writeStepInactive() {
    digitalWrite(MOTOR_STEP_PIN,
                 MOTOR_STEP_ACTIVE_HIGH ? LOW : HIGH);
}

inline void writeStepActive() {
    digitalWrite(MOTOR_STEP_PIN,
                 MOTOR_STEP_ACTIVE_HIGH ? HIGH : LOW);
}

inline void writeDirection(int8_t direction) {
    // Electrical DIR inversion and plant direction are deliberately separate.
    // MOTOR_DIRECTION_INVERTED matches the interface voltage/polarity;
    // kMotorPlantSign matches how pipe tilt changes the measured ball error.
    const bool positive_direction_is_high =
        MOTOR_DIRECTION_INVERTED == 0;
    const bool output_high = direction > 0
        ? positive_direction_is_high
        : !positive_direction_is_high;
    digitalWrite(MOTOR_DIR_PIN, output_high ? HIGH : LOW);
}

void enterMotionFaultFromIsr() {
    motion_fault = true;
    target_steps = current_steps;
    velocity_q16 = 0;
    phase_q16 = 0;
    writeStepInactive();
    step_high_ticks = 0;
}

void motionTick() {
    // Complete an already-started STEP pulse. Returning here guarantees at
    // least one full timer tick of low time before another pulse can start.
    if (step_high_ticks > 0) {
        --step_high_ticks;
        if (step_high_ticks == 0) {
            writeStepInactive();
        }
        return;
    }

    if (motion_fault) {
        velocity_q16 = 0;
        phase_q16 = 0;
        return;
    }

    if (absoluteInt32(current_steps) > config::kHardLimitSteps) {
        enterMotionFaultFromIsr();
        return;
    }

    int32_t error_steps = target_steps - current_steps;
    const int8_t requested_direction = signOf(error_steps);

    int32_t desired_velocity_q16 = 0;
    if (requested_direction != 0) {
        const int32_t speed_steps_s =
            absoluteInt32(velocity_q16) >> 16;
        const int32_t stopping_distance_steps =
            (speed_steps_s * speed_steps_s +
             2 * config::kMaximumMotorAccelerationStepsS2 - 1) /
            (2 * config::kMaximumMotorAccelerationStepsS2);

        // Begin deceleration when the remaining distance reaches the integer
        // braking distance. A target reversal immediately requests velocity in
        // the new direction; acceleration limiting brings it through zero.
        if (signOf(velocity_q16) == requested_direction &&
            stopping_distance_steps >= absoluteInt32(error_steps)) {
            desired_velocity_q16 = 0;
        } else {
            desired_velocity_q16 =
                requested_direction * kMaximumSpeedQ16;
        }
    }

    const int8_t previous_velocity_direction = signOf(velocity_q16);
    int32_t next_velocity_q16 = approach(
        velocity_q16, desired_velocity_q16, kAccelerationPerTickQ16);
    const int8_t next_velocity_direction = signOf(next_velocity_q16);

    // Force every reversal through an exact zero-speed tick. Without this,
    // fixed-point acceleration can jump from a small positive value directly
    // to a small negative value and carry old-direction phase into the first
    // pulse in the new direction.
    if (previous_velocity_direction != 0 &&
        next_velocity_direction != 0 &&
        previous_velocity_direction != next_velocity_direction) {
        next_velocity_q16 = 0;
    }
    velocity_q16 = next_velocity_q16;

    if (velocity_q16 == 0) {
        phase_q16 = 0;
        return;
    }

    if (requested_direction == 0) {
        // Never emit an extra pulse after reaching the absolute target.
        phase_q16 = 0;
        return;
    }

    // During a commanded reversal, STEP pulses keep following the current
    // velocity direction while that velocity ramps down. This makes the real
    // pulse train obey the deceleration limit instead of stopping abruptly;
    // after one exact zero-speed tick it accelerates in the new direction.
    const int8_t step_direction = signOf(velocity_q16);

    phase_q16 +=
        absoluteInt32(velocity_q16) /
        static_cast<int32_t>(config::kMotionTickHz);

    if (phase_q16 < kQ16One) {
        return;
    }

    if (output_direction != step_direction) {
        output_direction = step_direction;
        writeDirection(output_direction);
        direction_setup_ticks = config::kDirectionSetupTicks;
        return;
    }

    if (direction_setup_ticks > 0) {
        --direction_setup_ticks;
        return;
    }

    const int32_t next_position = current_steps + step_direction;
    if (absoluteInt32(next_position) > config::kHardLimitSteps) {
        enterMotionFaultFromIsr();
        return;
    }

    writeStepActive();
    step_high_ticks = config::kStepHighTicks;
    current_steps = next_position;
    phase_q16 -= kQ16One;
}

int32_t clampSteps(int32_t value, int32_t limit) {
    if (value < -limit) return -limit;
    if (value > limit) return limit;
    return value;
}

int32_t angleDegToSteps(float angle_deg) {
    const float raw_steps =
        angle_deg * static_cast<float>(config::kStepsPerMotorRev) / 360.0f;
    int32_t steps = static_cast<int32_t>(std::lround(raw_steps));
    return clampSteps(steps, config::kControlLimitSteps);
}

void setAbsoluteTargetAngle(float angle_deg) {
    const float control_limit_deg =
        static_cast<float>(config::kControlLimitMilliDeg) / 1000.0f;
    if (angle_deg > control_limit_deg) angle_deg = control_limit_deg;
    if (angle_deg < -control_limit_deg) angle_deg = -control_limit_deg;

    const int32_t new_target = angleDegToSteps(angle_deg);
    noInterrupts();
    target_steps = new_target;
    interrupts();
}

void setMotorEnabled(bool enabled) {
#if USE_MOTOR_ENABLE_PIN
    const bool output_high =
        MOTOR_ENABLE_ACTIVE_HIGH ? enabled : !enabled;
    digitalWrite(MOTOR_ENABLE_PIN, output_high ? HIGH : LOW);
#else
    (void)enabled;
#endif
}

bool parsePixelErrorLine(const char* text, int32_t& result) {
    if (text == nullptr || text[0] == '\0') return false;

    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0') return false;
    if (parsed < -config::kMaximumAcceptedPixelError ||
        parsed > config::kMaximumAcceptedPixelError) {
        return false;
    }

    result = static_cast<int32_t>(parsed);
    return true;
}

void acceptPixelError(int32_t error_px, uint32_t now_ms) {
    latest_raw_error_px = error_px;
    latest_control_output =
        ball_controller.updatePixelError(error_px, now_ms);
    setAbsoluteTargetAngle(latest_control_output.target_angle_deg);
    last_vision_ms = now_ms;
    last_timeout_service_ms = now_ms;
    have_received_vision = true;
    vision_timed_out = false;
}

void serviceK230Serial() {
    while (K230Serial.available() > 0) {
        const char ch = static_cast<char>(K230Serial.read());

        if (ch == '\n' || ch == '\r') {
            if (line_length == 0) continue;

            line_buffer[line_length] = '\0';
            int32_t error_px = 0;
            if (parsePixelErrorLine(line_buffer, error_px)) {
                acceptPixelError(error_px, millis());
            }
            line_length = 0;
            continue;
        }

        const bool allowed =
            (ch >= '0' && ch <= '9') ||
            ((ch == '+' || ch == '-') && line_length == 0);
        if (!allowed || line_length >= sizeof(line_buffer) - 1) {
            line_length = 0;
            continue;
        }

        line_buffer[line_length++] = ch;
    }
}

void serviceVisionTimeout(uint32_t now_ms) {
    if (!have_received_vision ||
        now_ms - last_vision_ms <= config::kVisionTimeoutMs) {
        return;
    }

    // Service return-to-zero at about 100 Hz. The controller uses real elapsed
    // time and applies a 6 deg/s slew, while the motion timer still enforces its
    // independent 6 deg/s and 30 deg/s^2 physical limits.
    if (now_ms - last_timeout_service_ms < 10) return;
    last_timeout_service_ms = now_ms;
    latest_control_output = ball_controller.onMeasurementTimeout(now_ms);
    setAbsoluteTargetAngle(latest_control_output.target_angle_deg);
    vision_timed_out = true;
}

void serviceDebugZeroCommand() {
    while (DebugSerial.available() > 0) {
        const char ch = static_cast<char>(DebugSerial.read());
        if (ch != 'z' && ch != 'Z') continue;

        noInterrupts();
        const bool stopped = absoluteInt32(velocity_q16) < (1L << 14);
        if (stopped) {
            current_steps = 0;
            target_steps = 0;
            velocity_q16 = 0;
            phase_q16 = 0;
            motion_fault = false;
        }
        interrupts();

        if (stopped) {
            ball_controller.reset(millis());
            DebugSerial.println("ZERO_OK");
        } else {
            DebugSerial.println("ZERO_REJECTED_MOTOR_MOVING");
        }
    }
}

void printDiagnostics(uint32_t now_ms) {
    if (now_ms - last_diagnostic_ms < 200) return;
    last_diagnostic_ms = now_ms;

    noInterrupts();
    const int32_t current = current_steps;
    const int32_t target = target_steps;
    const int32_t speed_q16 = velocity_q16;
    const bool fault = motion_fault;
    interrupts();

    DebugSerial.print("err_px=");
    DebugSerial.print(latest_raw_error_px);
    DebugSerial.print(" pos=");
    DebugSerial.print(latest_control_output.filtered_position_norm, 4);
    DebugSerial.print(" vel=");
    DebugSerial.print(latest_control_output.filtered_velocity_norm_s, 4);
    DebugSerial.print(" angle=");
    DebugSerial.print(latest_control_output.target_angle_deg, 3);
    DebugSerial.print(" target_steps=");
    DebugSerial.print(target);
    DebugSerial.print(" current_steps=");
    DebugSerial.print(current);
    DebugSerial.print(" speed_sps=");
    DebugSerial.print(static_cast<float>(speed_q16) /
                      static_cast<float>(kQ16One), 2);
    DebugSerial.print(" centered=");
    DebugSerial.print(latest_control_output.centered ? 1 : 0);
    DebugSerial.print(" timeout=");
    DebugSerial.print(vision_timed_out ? 1 : 0);
    DebugSerial.print(" fault=");
    DebugSerial.println(fault ? 1 : 0);
}

}  // namespace

void setup() {
    pinMode(MOTOR_STEP_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    writeStepInactive();
    writeDirection(+1);
    output_direction = +1;

#if USE_MOTOR_ENABLE_PIN
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
#endif
    setMotorEnabled(true);

    DebugSerial.begin(config::kDebugBaud);
    K230Serial.begin(config::kK230Baud);

    const uint32_t now_ms = millis();
    ball_controller.reset(now_ms);
    latest_control_output = ball_controller.onMeasurementTimeout(now_ms);
    last_vision_ms = now_ms;
    last_timeout_service_ms = now_ms;

    motion_timer = new HardwareTimer(TIM2);
    motion_timer->setOverflow(config::kMotionTickHz, HERTZ_FORMAT);
    motion_timer->attachInterrupt(motionTick);
    motion_timer->resume();

    DebugSerial.println();
    DebugSerial.println("C06B BALL BALANCE CONTROLLER READY");
    DebugSerial.print("steps/rev=");
    DebugSerial.println(config::kStepsPerMotorRev);
    DebugSerial.print("control_limit_steps=+/-");
    DebugSerial.println(config::kControlLimitSteps);
    DebugSerial.print("hard_limit_steps=+/-");
    DebugSerial.println(config::kHardLimitSteps);
    DebugSerial.print("max_speed_steps_s=");
    DebugSerial.println(config::kMaximumMotorSpeedStepsS);
    DebugSerial.println(
        "LEVEL THE PIPE BEFORE POWER-UP; send Z only while stationary to rezero");
}

void loop() {
    serviceK230Serial();
    serviceDebugZeroCommand();

    const uint32_t now_ms = millis();
    serviceVisionTimeout(now_ms);
    printDiagnostics(now_ms);

    // No blocking motor movement and no long delay. A tiny yield keeps Arduino
    // background services responsive without reducing command reaction speed.
    delay(1);
}
