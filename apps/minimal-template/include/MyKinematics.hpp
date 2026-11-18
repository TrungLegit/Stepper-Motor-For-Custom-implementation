#pragma once

#include "motion_core.hpp"
#include <array>

namespace my_rig {

using namespace motion_core;

/**
 * @brief Machine configuration and kinematic parameters for a 3-axis scanning rig.
 *        Cloners only need to edit this file to configure steps/mm, limits, and jerk.
 */
struct MyMachineConfig {
    static constexpr uint8_t NUM_AXES = 3;

    static inline const std::array<AxisConfig, NUM_AXES>& getAxesConfig() {
        static const std::array<AxisConfig, NUM_AXES> s_axes = {{
            // Axis 0: Azimuth Turntable (Rotary)
            // 200 full steps * 16 microsteps = 3200 steps/rev. Gear reduction 4:1 -> 35.55556 steps/deg
            AxisConfig{
                0,                  // axis_id
                "A",                // name
                35.555556f,         // steps_per_degree
                180.0f,             // max_speed_deg_s
                360.0f,             // max_accel_deg_s2
                1500.0f,            // max_jerk_deg_s3
                0.05f,              // backlash_deg
                0.0f,               // travel_min_deg
                360.0f,             // travel_max_deg
                true                // is_rotary
            },
            // Axis 1: Elevation Gimbal (Rotary)
            // 200 full steps * 16 microsteps = 3200 steps/rev. Gear reduction 4:1 -> 35.55556 steps/deg
            AxisConfig{
                1,                  // axis_id
                "B",                // name
                35.555556f,         // steps_per_degree
                90.0f,              // max_speed_deg_s
                180.0f,             // max_accel_deg_s2
                800.0f,             // max_jerk_deg_s3
                0.02f,              // backlash_deg
                0.0f,               // travel_min_deg
                90.0f,              // travel_max_deg
                true                // is_rotary
            },
            // Axis 2: Focus Z (Linear stage)
            // 200 full steps * 16 microsteps = 3200 steps/rev. Lead screw pitch 8mm -> 400 steps/mm
            AxisConfig{
                2,                  // axis_id
                "Z",                // name
                400.0f,             // steps_per_mm
                25.0f,              // max_speed_mm_s
                100.0f,             // max_accel_mm_s2
                500.0f,             // max_jerk_mm_s3
                0.01f,              // backlash_mm
                -25.0f,             // travel_min_mm
                25.0f,              // travel_max_mm
                false               // is_rotary
            }
        }};
        return s_axes;
    }
};

} // namespace my_rig
