#pragma once

#include <cmath>
#include <cstdint>

#include "control_config.h"

struct BallControlOutput {
    float target_angle_deg;
    float filtered_position_norm;
    float filtered_velocity_norm_s;
    bool centered;
    bool measurement_valid;
};

class BallController {
public:
    BallController() { reset(0); }

    void reset(uint32_t now_ms) {
        sample_count_ = 0;
        sample_index_ = 0;
        median_samples_[0] = 0.0f;
        median_samples_[1] = 0.0f;
        median_samples_[2] = 0.0f;
        position_norm_ = 0.0f;
        previous_position_norm_ = 0.0f;
        velocity_norm_s_ = 0.0f;
        integral_angle_deg_ = 0.0f;
        target_angle_deg_ = 0.0f;
        last_update_ms_ = now_ms;
        has_filter_state_ = false;
        centered_ = true;
        lost_ = true;
    }

    BallControlOutput updatePixelError(int32_t error_px, uint32_t now_ms) {
        const float dt = elapsedSeconds(now_ms);
        lost_ = false;

        error_px = clampInt(error_px,
                            -config::kMaximumAcceptedPixelError,
                            config::kMaximumAcceptedPixelError);

        float measurement =
            static_cast<float>(error_px) / config::kErrorFullScalePx;
        measurement = clampFloat(measurement, -1.0f, 1.0f);
        measurement = median3(measurement);

        if (!has_filter_state_) {
            position_norm_ = measurement;
            previous_position_norm_ = measurement;
            velocity_norm_s_ = 0.0f;
            has_filter_state_ = true;
        } else {
            const float position_alpha =
                dt / (config::kPositionFilterTauS + dt);
            position_norm_ +=
                position_alpha * (measurement - position_norm_);

            const float raw_velocity =
                (position_norm_ - previous_position_norm_) / dt;
            previous_position_norm_ = position_norm_;

            const float velocity_alpha =
                dt / (config::kVelocityFilterTauS + dt);
            velocity_norm_s_ +=
                velocity_alpha * (raw_velocity - velocity_norm_s_);
        }

        updateCenterHysteresis();

        float requested_angle_deg = 0.0f;
        if (centered_) {
            // Remove residual integral when the ball is genuinely stable.
            integral_angle_deg_ *= 0.90f;
        } else {
            const float effective_position = softDeadzone(
                position_norm_, config::kCenterEnterPositionNorm);

            // Conditional integration and anti-windup are retained even though
            // Ki is zero by default, so a later small Ki cannot wind up while
            // the output is saturated.
            const float provisional_without_i =
                -static_cast<float>(config::kMotorPlantSign) *
                (config::kKpDegPerNormalizedError * effective_position +
                 config::kKdDegPerNormalizedVelocity * velocity_norm_s_);

            const bool output_saturated =
                std::fabs(provisional_without_i + integral_angle_deg_) >=
                milliDegToDeg(config::kControlLimitMilliDeg);
            const bool error_drives_back =
                (provisional_without_i > 0.0f && effective_position > 0.0f) ||
                (provisional_without_i < 0.0f && effective_position < 0.0f);

            if (config::kKiDegPerNormalizedErrorSecond > 0.0f &&
                (!output_saturated || error_drives_back)) {
                integral_angle_deg_ +=
                    -static_cast<float>(config::kMotorPlantSign) *
                    config::kKiDegPerNormalizedErrorSecond *
                    effective_position * dt;
                integral_angle_deg_ = clampFloat(
                    integral_angle_deg_,
                    -config::kIntegralAngleLimitDeg,
                    config::kIntegralAngleLimitDeg);
            }

            requested_angle_deg =
                provisional_without_i + integral_angle_deg_;
        }

        const float soft_limit_deg =
            milliDegToDeg(config::kControlLimitMilliDeg);
        requested_angle_deg = clampFloat(
            requested_angle_deg, -soft_limit_deg, soft_limit_deg);
        target_angle_deg_ = slewToward(
            target_angle_deg_, requested_angle_deg,
            config::kTargetAngleSlewDegS * dt);

        return output(true);
    }

    BallControlOutput onMeasurementTimeout(uint32_t now_ms) {
        const float dt = elapsedSeconds(now_ms);
        if (!lost_) {
            lost_ = true;
            has_filter_state_ = false;
            sample_count_ = 0;
            sample_index_ = 0;
            integral_angle_deg_ = 0.0f;
            centered_ = true;
        }

        target_angle_deg_ = slewToward(
            target_angle_deg_, 0.0f,
            config::kReturnToZeroSlewDegS * dt);
        return output(false);
    }

    float targetAngleDeg() const { return target_angle_deg_; }

private:
    static float clampFloat(float value, float low, float high) {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    static int32_t clampInt(int32_t value, int32_t low, int32_t high) {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    static float milliDegToDeg(int32_t milli_deg) {
        return static_cast<float>(milli_deg) / 1000.0f;
    }

    static float slewToward(float current, float requested, float max_delta) {
        const float delta = requested - current;
        if (delta > max_delta) return current + max_delta;
        if (delta < -max_delta) return current - max_delta;
        return requested;
    }

    static float softDeadzone(float value, float width) {
        const float magnitude = std::fabs(value);
        if (magnitude <= width) return 0.0f;
        const float reduced = magnitude - width;
        return value >= 0.0f ? reduced : -reduced;
    }

    float elapsedSeconds(uint32_t now_ms) {
        uint32_t elapsed_ms = now_ms - last_update_ms_;
        last_update_ms_ = now_ms;
        float dt = static_cast<float>(elapsed_ms) * 0.001f;
        return clampFloat(dt,
                          config::kMinimumVisionDtS,
                          config::kMaximumVisionDtS);
    }

    float median3(float value) {
        if (sample_count_ < 3) {
            median_samples_[sample_count_] = value;
            ++sample_count_;
            while (sample_count_ < 3) {
                median_samples_[sample_count_] = value;
                ++sample_count_;
            }
            sample_index_ = 0;
        } else {
            median_samples_[sample_index_] = value;
            sample_index_ = (sample_index_ + 1) % 3;
        }

        float a = median_samples_[0];
        float b = median_samples_[1];
        float c = median_samples_[2];
        if (a > b) { const float t = a; a = b; b = t; }
        if (b > c) { const float t = b; b = c; c = t; }
        if (a > b) { const float t = a; a = b; b = t; }
        return b;
    }

    void updateCenterHysteresis() {
        if (centered_) {
            if (std::fabs(position_norm_) >=
                    config::kCenterExitPositionNorm ||
                std::fabs(velocity_norm_s_) >=
                    config::kCenterExitVelocityNormS) {
                centered_ = false;
            }
        } else {
            if (std::fabs(position_norm_) <=
                    config::kCenterEnterPositionNorm &&
                std::fabs(velocity_norm_s_) <=
                    config::kCenterEnterVelocityNormS) {
                centered_ = true;
            }
        }
    }

    BallControlOutput output(bool valid) const {
        BallControlOutput result{};
        result.target_angle_deg = target_angle_deg_;
        result.filtered_position_norm = position_norm_;
        result.filtered_velocity_norm_s = velocity_norm_s_;
        result.centered = centered_;
        result.measurement_valid = valid;
        return result;
    }

    float median_samples_[3]{};
    uint8_t sample_count_ = 0;
    uint8_t sample_index_ = 0;
    float position_norm_ = 0.0f;
    float previous_position_norm_ = 0.0f;
    float velocity_norm_s_ = 0.0f;
    float integral_angle_deg_ = 0.0f;
    float target_angle_deg_ = 0.0f;
    uint32_t last_update_ms_ = 0;
    bool has_filter_state_ = false;
    bool centered_ = true;
    bool lost_ = true;
};

