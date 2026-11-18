#pragma once

#include "planner/VelocityProfile.hpp"
#include <array>

namespace motion_core {

/**
 * @brief 7-Phase Jerk-Limited S-Curve Velocity Profile.
 *
 * Implements smooth jerk-bounded trajectories:
 * - Phase 1: Jerk ramp-up (+J) to peak acceleration
 * - Phase 2: Constant acceleration (A_max)
 * - Phase 3: Jerk ramp-down (-J) to zero acceleration at cruise speed
 * - Phase 4: Constant velocity (V_max / V_peak)
 * - Phase 5: Jerk ramp-up (-J) to peak deceleration
 * - Phase 6: Constant deceleration (-A_max)
 * - Phase 7: Jerk ramp-down (+J) to zero acceleration at target speed
 *
 * Gracefully collapses phases:
 * - 5-phase: Constant acceleration/deceleration collapse when distance/V too small
 * - 4-phase: Constant cruise collapses when distance is too short to reach V_max
 * - 3-phase/pure-jerk: Constant accel and cruise both collapse for microscopic moves
 */
class SCurveProfile : public VelocityProfile {
public:
    struct PhaseInfo {
        float duration{0.0f};      // Phase duration (seconds)
        float start_time{0.0f};    // Start time of phase (seconds)
        float end_time{0.0f};      // End time of phase (seconds)
        float start_pos{0.0f};     // Position at start of phase (mm)
        float end_pos{0.0f};       // Position at end of phase (mm)
        float start_vel{0.0f};     // Velocity at start of phase (mm/s)
        float end_vel{0.0f};       // Velocity at end of phase (mm/s)
        float start_accel{0.0f};   // Acceleration at start of phase (mm/s^2)
        float end_accel{0.0f};     // Acceleration at end of phase (mm/s^2)
        float jerk{0.0f};          // Jerk during this phase (mm/s^3)
    };

    SCurveProfile() = default;
    ~SCurveProfile() override = default;

    void calculate(float distance, float v_start, float v_target,
                   float v_max, float a_max, float j_max) override;

    float getDuration() const override { return m_total_duration; }
    float getVelocity(float t) const override;
    float getPosition(float t) const override;
    float getAcceleration(float t) const override;
    float getJerk(float t) const override;
    uint8_t getPhase(float t) const override;
    float getDistance() const override { return m_distance; }
    float getPeakVelocity() const override { return m_peak_velocity; }
    float getPeakAcceleration() const override { return m_peak_accel; }

    void generateStepTable(float steps_per_unit, uint32_t timer_freq_hz,
                           std::vector<StepInterval>& out_table,
                           uint32_t max_segments = 256) const override;

    const std::array<PhaseInfo, 7>& getPhases() const { return m_phases; }

private:
    float m_distance{0.0f};
    float m_v_start{0.0f};
    float m_v_target{0.0f};
    float m_v_max{0.0f};
    float m_a_max{0.0f};
    float m_j_max{0.0f};
    float m_peak_velocity{0.0f};
    float m_peak_accel{0.0f};
    float m_total_duration{0.0f};

    std::array<PhaseInfo, 7> m_phases{};

    void computePhases(float T1, float T2, float T3, float T4, float T5, float T6, float T7,
                       float J_accel, float J_decel);
};

} // namespace motion_core
