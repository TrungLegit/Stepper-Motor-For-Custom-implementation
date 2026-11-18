#pragma once

#include <cstdint>
#include <vector>
#include "planner/Axis.hpp"

namespace motion_core {

/**
 * @brief Pre-computed step-interval entry for non-blocking step generation.
 * 
 * Consumed by StepGenerator in timer ISR:
 * - step_count: Number of consecutive steps at this interval
 * - isr_ticks_per_step: Number of timer ISR ticks per step pulse
 */
struct StepInterval {
    uint32_t step_count{1};
    uint32_t isr_ticks_per_step{1};
};

/**
 * @brief Abstract base class for motion velocity profiles.
 */
class VelocityProfile {
public:
    virtual ~VelocityProfile() = default;

    /**
     * @brief Compute profile parameters for a move.
     * @param distance Total travel distance (positive scalar)
     * @param v_start Starting velocity (>= 0)
     * @param v_target Target exit velocity (>= 0)
     * @param v_max Maximum allowable cruise velocity (> 0)
     * @param a_max Maximum allowable acceleration (> 0)
     * @param j_max Maximum allowable jerk (> 0, used by S-curve)
     */
    virtual void calculate(float distance, float v_start, float v_target,
                           float v_max, float a_max, float j_max) = 0;

    /**
     * @brief Total planned duration of the profile in seconds.
     */
    virtual float getDuration() const = 0;

    /**
     * @brief Instantaneous velocity at time t in mm/s.
     */
    virtual float getVelocity(float t) const = 0;

    /**
     * @brief Instantaneous position at time t in mm.
     */
    virtual float getPosition(float t) const = 0;

    /**
     * @brief Instantaneous acceleration at time t in mm/s^2.
     */
    virtual float getAcceleration(float t) const = 0;

    /**
     * @brief Instantaneous jerk at time t in mm/s^3.
     */
    virtual float getJerk(float t) const = 0;

    /**
     * @brief Current active profile phase index at time t.
     */
    virtual uint8_t getPhase(float t) const = 0;

    /**
     * @brief Total planned distance in mm.
     */
    virtual float getDistance() const = 0;

    /**
     * @brief Peak velocity achieved during this move in mm/s.
     */
    virtual float getPeakVelocity() const = 0;

    /**
     * @brief Peak acceleration achieved during this move in mm/s^2.
     */
    virtual float getPeakAcceleration() const = 0;

    /**
     * @brief Pre-compute step-interval table for StepGenerator timer ISR consumption.
     * @param steps_per_unit Scale factor (steps per mm/degree)
     * @param timer_freq_hz Frequency of the step timer ISR in Hz (e.g. 100,000)
     * @param out_table Destination vector for StepInterval entries
     * @param max_segments Maximum number of profile segments to discretize into
     */
    virtual void generateStepTable(float steps_per_unit, uint32_t timer_freq_hz,
                                  std::vector<StepInterval>& out_table,
                                  uint32_t max_segments = 256) const = 0;
};

} // namespace motion_core
