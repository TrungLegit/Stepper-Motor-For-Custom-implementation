#pragma once

#include <cstdint>
#include <string>
#include <array>
#include "hal/HalTypes.hpp"

namespace motion_core {

/**
 * @brief Invert motion direction.
 */
inline constexpr Direction invertDirection(Direction dir) noexcept {
    return (dir == Direction::POSITIVE) ? Direction::NEGATIVE : Direction::POSITIVE;
}

/**
 * @brief Velocity profile type selector.
 */
enum class ProfileType : uint8_t {
    SCURVE = 0,
    TRAPEZOIDAL = 1
};

/**
 * @brief Motion command operation types.
 */
enum class MotionType : uint8_t {
    MOVE_ABS = 0,
    MOVE_REL = 1,
    HOME = 2,
    JOG = 3,
    SET_SPEED = 4,
    WAIT = 5
};

/**
 * @brief Configuration parameters for a single physical or kinematic axis.
 */
struct AxisConfig {
    uint8_t axis_id{0};
    std::string name{"A"};
    float steps_per_mm{80.0f};         // or steps_per_degree for rotary axes
    float max_speed_mm_s{100.0f};
    float max_accel_mm_s2{500.0f};
    float max_jerk_mm_s3{2000.0f};     // S-curve jerk limit
    float backlash_mm{0.0f};           // Backlash compensation distance
    float travel_min_mm{0.0f};
    float travel_max_mm{200.0f};
    bool is_rotary{false};
    bool invert_dir{false};
};

/**
 * @brief Multi-axis coordinated motion command.
 */
struct MotionCommand {
    MotionType type{MotionType::MOVE_ABS};
    float targets[MAX_AXES]{0.0f};      // Target positions in mm or degrees
    bool axis_mask[MAX_AXES]{false};     // Which axes participate in this move
    float feedrate_mm_s{0.0f};          // Coordinated feed rate (0 = use axis limits)
    ProfileType profile{ProfileType::SCURVE};
    uint32_t wait_ms{0};
    bool is_backlash_premove{false};    // Internal flag for backlash take-up segment
};

/**
 * @brief Real-time status of the motion planner and axes.
 */
struct MotionStatus {
    float current_pos[MAX_AXES]{0.0f};  // Current position in user units (mm/deg)
    float current_vel[MAX_AXES]{0.0f};  // Instantaneous velocity (mm/s or deg/s)
    int32_t current_steps[MAX_AXES]{0}; // Motor step counter
    bool is_moving{false};
    bool is_homing{false};
    bool estop_active{false};
    int32_t queue_depth{0};
    float estimated_time_remaining_s{0.0f};
    bool is_homed[MAX_AXES]{false};
};

} // namespace motion_core
