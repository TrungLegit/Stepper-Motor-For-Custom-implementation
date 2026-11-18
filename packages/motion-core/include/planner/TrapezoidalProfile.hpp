#pragma once

#include "planner/VelocityProfile.hpp"
#include <array>

namespace motion_core {

/**
 * @brief Classical 3-phase Trapezoidal (constant acceleration) Velocity Profile.
 * 
 * Phases:
 * - Phase 1: Constant acceleration (+a_max)
 * - Phase 2: Constant cruise velocity (v_peak)
 * - Phase 3: Constant deceleration (-a_max)
 * 
 * Supports triangular collapse when distance is too short to reach v_max.
 */
class TrapezoidalProfile : public VelocityProfile {
public:
    struct PhaseInfo {
        float duration{0.0f};
        float start_time{0.0f};
        float end_time{0.0f};
        float start_pos{0.0f};
        float end_pos{0.0f};
        float start_vel{0.0f};
        float end_vel{0.0f};
        float accel{0.0f};
    };

    TrapezoidalProfile() = default;
    ~TrapezoidalProfile() override = default;

    void calculate(float distance, float v_start, float v_target,
                   float v_max, float a_max, float j_max = 0.0f) override;

    float getDuration() const override { return m_total_duration; }
    float getVelocity(float t) const override;
    float getPosition(float t) const override;
    float getAcceleration(float t) const override;
    float getJerk(float /* t */) const override { return 0.0f; }
    uint8_t getPhase(float t) const override;
    float getDistance() const override { return m_distance; }
    float getPeakVelocity() const override { return m_peak_velocity; }
    float getPeakAcceleration() const override { return m_a_max; }

    void generateStepTable(float steps_per_unit, uint32_t timer_freq_hz,
                           std::vector<StepInterval>& out_table,
                           uint32_t max_segments = 256) const override;

    const std::array<PhaseInfo, 3>& getPhases() const { return m_phases; }

private:
    float m_distance{0.0f};
    float m_v_start{0.0f};
    float m_v_target{0.0f};
    float m_v_max{0.0f};
    float m_a_max{0.0f};
    float m_peak_velocity{0.0f};
    float m_total_duration{0.0f};

    std::array<PhaseInfo, 3> m_phases{};
};

} // namespace motion_core
