#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "ball_controller.h"
#include "control_config.h"

int main() {
    static_assert(config::kStepsPerMotorRev == 3200);
    static_assert(config::kControlLimitSteps == 24);
    static_assert(config::kHardLimitSteps == 26);
    static_assert(config::kMaximumMotorSpeedStepsS == 53);
    static_assert(config::kMaximumMotorAccelerationStepsS2 == 266);

    BallController controller;
    controller.reset(0);

    float previous_angle = 0.0f;
    BallControlOutput output{};

    // A ball far to the positive-x side must request a negative correcting
    // angle with the default plant sign. Angle slew is capped at 8 deg/s.
    for (uint32_t now_ms = 20; now_ms <= 1000; now_ms += 20) {
        output = controller.updatePixelError(160, now_ms);
        assert(output.target_angle_deg <= 0.0001f);
        assert(std::fabs(output.target_angle_deg) <= 2.7001f);
        assert(std::fabs(output.target_angle_deg - previous_angle) <= 0.1601f);
        previous_angle = output.target_angle_deg;
    }
    assert(output.target_angle_deg < -1.0f);

    // Reversing the measured error must not jump instantaneously from one
    // extreme to the other.
    output = controller.updatePixelError(-160, 1020);
    assert(std::fabs(output.target_angle_deg - previous_angle) <= 0.1601f);
    assert(std::fabs(output.target_angle_deg) <= 2.7001f);

    // A timeout must move the requested angle toward zero, never away from it.
    const float before_timeout = std::fabs(output.target_angle_deg);
    output = controller.onMeasurementTimeout(1040);
    assert(!output.measurement_valid);
    assert(std::fabs(output.target_angle_deg) <= before_timeout + 0.0001f);

    for (uint32_t now_ms = 1060; now_ms <= 2000; now_ms += 20) {
        output = controller.onMeasurementTimeout(now_ms);
        assert(std::fabs(output.target_angle_deg) <= 2.7001f);
    }
    assert(std::fabs(output.target_angle_deg) < 0.001f);

    // A freshly reset, centered measurement produces no command.
    controller.reset(3000);
    output = controller.updatePixelError(0, 3020);
    assert(output.centered);
    assert(std::fabs(output.target_angle_deg) < 0.001f);

    std::cout << "control_logic_test: PASS\n";
    return 0;
}

