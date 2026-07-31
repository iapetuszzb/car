#include <cassert>
#include <cstdint>
#include <iostream>

// Include the firmware implementation into this native test translation unit.
// Arduino and HardwareTimer are provided by tests/stubs for syntax/logic tests.
#include "../c06b_controller/src/main.cpp"

int main() {
    arduinoStubResetPins();

    // STEP polarity, pulse width, and DIR setup are configuration-driven.
    writeStepInactive();
    assert(arduinoStubPinValue(MOTOR_STEP_PIN) ==
           (MOTOR_STEP_ACTIVE_HIGH ? LOW : HIGH));

    current_steps = 0;
    target_steps = 1;
    velocity_q16 = kMaximumSpeedQ16;
    phase_q16 = kQ16One;
    output_direction = +1;
    direction_setup_ticks = 0;
    step_high_ticks = 0;
    motion_fault = false;
    motionTick();
    assert(current_steps == 1);
    assert(arduinoStubPinValue(MOTOR_STEP_PIN) ==
           (MOTOR_STEP_ACTIVE_HIGH ? HIGH : LOW));
    assert(step_high_ticks == config::kStepHighTicks);
    motionTick();
    assert(current_steps == 1);
    assert(arduinoStubPinValue(MOTOR_STEP_PIN) ==
           (MOTOR_STEP_ACTIVE_HIGH ? LOW : HIGH));

    current_steps = 0;
    target_steps = 1;
    velocity_q16 = kMaximumSpeedQ16;
    phase_q16 = kQ16One;
    output_direction = -1;
    direction_setup_ticks = 0;
    step_high_ticks = 0;
    motion_fault = false;
    motionTick();
    assert(current_steps == 0);
    assert(output_direction == +1);
    assert(direction_setup_ticks == config::kDirectionSetupTicks);
    assert(arduinoStubPinValue(MOTOR_DIR_PIN) ==
           (MOTOR_DIRECTION_INVERTED ? LOW : HIGH));
    for (uint16_t i = 0; i < config::kDirectionSetupTicks; ++i) {
        motionTick();
        assert(current_steps == 0);
    }
    motionTick();
    assert(current_steps == 1);

    current_steps = 0;
    target_steps = 0;
    velocity_q16 = 0;
    phase_q16 = 0;
    output_direction = +1;
    direction_setup_ticks = 0;
    step_high_ticks = 0;
    motion_fault = false;

    // The public target conversion clamps even a wildly excessive command to
    // the +/-2.7-degree normal-control boundary (+/-24 steps by default).
    setAbsoluteTargetAngle(100.0f);
    assert(target_steps == config::kControlLimitSteps);

    int32_t observed_max_speed_q16 = 0;
    for (int i = 0; i < 200000; ++i) {
        motionTick();
        if (absoluteInt32(velocity_q16) > observed_max_speed_q16) {
            observed_max_speed_q16 = absoluteInt32(velocity_q16);
        }
        assert(current_steps <= config::kControlLimitSteps);
        assert(current_steps >= -config::kHardLimitSteps);
    }
    assert(current_steps == config::kControlLimitSteps);
    assert(observed_max_speed_q16 <= kMaximumSpeedQ16);

    // Reverse a real in-flight move. The pulse position must keep moving a
    // short distance in the old direction while the configured acceleration
    // ramps velocity down, then pass through zero and head to the new target.
    current_steps = 0;
    target_steps = config::kControlLimitSteps;
    velocity_q16 = 0;
    phase_q16 = 0;
    output_direction = +1;
    direction_setup_ticks = 0;
    step_high_ticks = 0;
    motion_fault = false;

    for (int i = 0; i < 200000; ++i) {
        motionTick();
        if (velocity_q16 == kMaximumSpeedQ16 && current_steps >= 8) break;
    }
    assert(velocity_q16 == kMaximumSpeedQ16);
    const int32_t reversal_position = current_steps;
    target_steps = -config::kControlLimitSteps;

    bool emitted_decelerating_forward_step = false;
    bool reached_zero_velocity = false;
    bool emitted_reverse_step = false;
    int32_t previous_position = current_steps;
    for (int i = 0; i < 300000; ++i) {
        motionTick();
        assert(absoluteInt32(current_steps) <= config::kHardLimitSteps);
        assert(absoluteInt32(velocity_q16) <= kMaximumSpeedQ16);

        if (current_steps > reversal_position) {
            emitted_decelerating_forward_step = true;
        }
        if (velocity_q16 == 0) {
            reached_zero_velocity = true;
        }
        if (current_steps < previous_position) {
            emitted_reverse_step = true;
        }
        previous_position = current_steps;

        if (current_steps == -config::kControlLimitSteps &&
            step_high_ticks == 0) {
            break;
        }
    }
    assert(emitted_decelerating_forward_step);
    assert(reached_zero_velocity);
    assert(emitted_reverse_step);
    assert(current_steps == -config::kControlLimitSteps);

    // Also retain the stopped-endpoint reversal case.
    current_steps = config::kControlLimitSteps;
    target_steps = config::kControlLimitSteps;
    velocity_q16 = 0;
    phase_q16 = 0;
    output_direction = +1;
    direction_setup_ticks = 0;
    step_high_ticks = 0;
    motion_fault = false;
    setAbsoluteTargetAngle(-100.0f);
    for (int i = 0; i < 300000; ++i) {
        motionTick();
        assert(current_steps >= -config::kControlLimitSteps);
        assert(current_steps <= config::kHardLimitSteps);
        assert(absoluteInt32(velocity_q16) <= kMaximumSpeedQ16);
    }
    assert(current_steps == -config::kControlLimitSteps);

    // Independently test the ISR hard guard by bypassing the normal setter.
    // It may reach the floor(3-degree) boundary, but the next outward pulse
    // must latch a fault instead of crossing it.
    current_steps = config::kHardLimitSteps;
    target_steps = config::kHardLimitSteps + 100;
    velocity_q16 = kMaximumSpeedQ16;
    phase_q16 = kQ16One;
    output_direction = +1;
    direction_setup_ticks = 0;
    step_high_ticks = 0;
    motion_fault = false;
    motionTick();
    assert(motion_fault);
    assert(current_steps == config::kHardLimitSteps);

    // The independent guard is symmetric at the negative boundary.
    current_steps = -config::kHardLimitSteps;
    target_steps = -config::kHardLimitSteps - 100;
    velocity_q16 = -kMaximumSpeedQ16;
    phase_q16 = kQ16One;
    output_direction = -1;
    direction_setup_ticks = 0;
    step_high_ticks = 0;
    motion_fault = false;
    motionTick();
    assert(motion_fault);
    assert(current_steps == -config::kHardLimitSteps);

    std::cout << "motion_planner_test: PASS\n";
    return 0;
}
