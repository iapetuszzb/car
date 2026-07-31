#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// C06B physical pins (confirmed from the schematic in tmp/pdfs).
// H3 Bluetooth header: PB11 = USART3 RX, PB10 = USART3 TX.
// H4/H5/H6 servo signal pins: PB6/PB7/PB8.
// ---------------------------------------------------------------------------
#define K230_UART_RX_PIN PB11
#define K230_UART_TX_PIN PB10
#define MOTOR_STEP_PIN PB6
#define MOTOR_DIR_PIN PB7
#define MOTOR_ENABLE_PIN PB8

// The MS42C external input pinout and enable polarity are not present in the
// supplied manual. Keep ENABLE disabled until the real terminal definition is
// confirmed. STEP and DIR must also pass through the correct level/open-
// collector interface required by the MS42C input.
#define USE_MOTOR_ENABLE_PIN 0
#define MOTOR_ENABLE_ACTIVE_HIGH 1

// These are electrical-interface assumptions, separate from the ball-control
// direction kMotorPlantSign below. Confirm the MS42C terminal/common wiring
// and the interface circuit before changing or energizing the connection.
#define MOTOR_STEP_ACTIVE_HIGH 1
#define MOTOR_DIRECTION_INVERTED 0

namespace config {

constexpr uint32_t kDebugBaud = 115200;
constexpr uint32_t kK230Baud = 115200;

// K230 sends one signed pixel difference per camera-loop iteration:
//     ball_x - screen_center_x
// Example wire data: "-37\n", "0\n", "42\n".
// A valid frame sends the new error. A missed frame retransmits the last valid
// error until detection resumes; before the first valid frame it sends
// nothing. Therefore this timeout detects a stopped K230 program or broken
// UART stream, not an individual detector miss. A real centered value is 0.
constexpr uint32_t kVisionTimeoutMs = 150;
constexpr int32_t kMaximumAcceptedPixelError = 2000;

// Set this to the useful half-width of the pipe/ROI, not blindly to the whole
// display width. For a 320 px wide ROI centered on screen, 160 is a good start.
constexpr float kErrorFullScalePx = 160.0f;

// Ball controller. Start with PD; integral is deliberately disabled because
// ball-and-beam systems commonly oscillate badly when integral is introduced
// before P and D are stable.
constexpr float kKpDegPerNormalizedError = 1.80f;
constexpr float kKdDegPerNormalizedVelocity = 0.45f;
constexpr float kKiDegPerNormalizedErrorSecond = 0.0f;
constexpr float kIntegralAngleLimitDeg = 0.25f;

// +1 means a positive motor angle tends to move the measured ball error in the
// positive direction. If the first safe direction test is wrong, change this
// to -1. Never tune gains while this sign is wrong: that creates positive
// feedback and drives the ball away from center.
constexpr int kMotorPlantSign = +1;

// Position and velocity filtering. These values retain fast response but stop
// one- or two-pixel detection noise from causing repeated direction changes.
constexpr float kPositionFilterTauS = 0.05f;
constexpr float kVelocityFilterTauS = 0.10f;
constexpr float kMinimumVisionDtS = 0.010f;
constexpr float kMaximumVisionDtS = 0.080f;

// Center deadband with hysteresis. At 160 px full scale these are 3 px entering
// and 6 px exiting. Velocity is included so a ball crossing center quickly is
// still actively braked instead of being mistaken for a stable centered ball.
constexpr float kCenterEnterPositionNorm = 3.0f / kErrorFullScalePx;
constexpr float kCenterExitPositionNorm = 6.0f / kErrorFullScalePx;
constexpr float kCenterEnterVelocityNormS = 10.0f / kErrorFullScalePx;
constexpr float kCenterExitVelocityNormS = 20.0f / kErrorFullScalePx;

// Outer target-angle slew. This prevents one noisy frame from changing the
// requested angle from one extreme to the other. The motor planner below has
// an independent, stricter physical speed and acceleration limit.
constexpr float kTargetAngleSlewDegS = 8.0f;
constexpr float kReturnToZeroSlewDegS = 6.0f;

// ---------------------------------------------------------------------------
// Motor limits.
// These defaults assume the external MS42C STEP input is configured for 3200
// pulses/revolution (commonly a 1.8-degree motor at 16 microsteps). Some closed
// loop drives apply a separate electronic gear ratio, so confirm the external
// pulse/revolution value rather than relying only on the internal microstep
// label. A mismatch here changes every angle and speed.
// ---------------------------------------------------------------------------
constexpr int32_t kMotorFullStepsPerRev = 200;
constexpr int32_t kMotorMicrosteps = 16;
constexpr int32_t kStepsPerMotorRev =
    kMotorFullStepsPerRev * kMotorMicrosteps;

// Normal control is limited to +/-2.7 degrees. The independent software hard
// boundary is floor(+/-3 degrees), never rounded upward.
constexpr int32_t kControlLimitMilliDeg = 2700;
constexpr int32_t kHardLimitMilliDeg = 3000;
constexpr int32_t kControlLimitSteps =
    (kStepsPerMotorRev * kControlLimitMilliDeg) / 360000;
constexpr int32_t kHardLimitSteps =
    (kStepsPerMotorRev * kHardLimitMilliDeg) / 360000;

// Conservative first-run motion limits: <=6 deg/s and <=30 deg/s^2.
// Integer division deliberately rounds down so the configured speed cannot
// exceed the requested upper bound.
constexpr int32_t kMaximumMotorSpeedMilliDegS = 6000;
constexpr int32_t kMaximumMotorAccelerationMilliDegS2 = 30000;
constexpr int32_t kMaximumMotorSpeedStepsS =
    (kStepsPerMotorRev * kMaximumMotorSpeedMilliDegS) / 360000;
constexpr int32_t kMaximumMotorAccelerationStepsS2 =
    (kStepsPerMotorRev * kMaximumMotorAccelerationMilliDegS2) / 360000;

// Hardware-timer motion service. At 2 kHz, a one-tick STEP pulse is 500 us.
// This is intentionally slow/conservative and far below the 53-step/s default
// maximum. Confirm the MS42C minimum/maximum pulse widths before raising speed.
constexpr uint32_t kMotionTickHz = 2000;
constexpr uint16_t kDirectionSetupTicks = 2;
constexpr uint16_t kStepHighTicks = 1;

static_assert(kMotorPlantSign == 1 || kMotorPlantSign == -1,
              "kMotorPlantSign must be +1 or -1");
static_assert(MOTOR_STEP_ACTIVE_HIGH == 0 || MOTOR_STEP_ACTIVE_HIGH == 1,
              "MOTOR_STEP_ACTIVE_HIGH must be 0 or 1");
static_assert(MOTOR_DIRECTION_INVERTED == 0 ||
                  MOTOR_DIRECTION_INVERTED == 1,
              "MOTOR_DIRECTION_INVERTED must be 0 or 1");
static_assert(kMotorMicrosteps >= 8,
              "Use at least 8 microsteps; 16 is recommended to reduce jitter");
static_assert(kControlLimitSteps <= kHardLimitSteps,
              "Control limit must not exceed the hard limit");
static_assert(kMaximumMotorSpeedStepsS > 0,
              "Configured motor speed rounds down to zero");
static_assert(kMaximumMotorAccelerationStepsS2 > 0,
              "Configured motor acceleration rounds down to zero");

}  // namespace config
