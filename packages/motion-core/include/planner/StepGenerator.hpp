#pragma once

#include <cstdint>
#include <array>
#include <functional>
#include "planner/Axis.hpp"
#include "planner/VelocityProfile.hpp"
#include "hal/IDriverHAL.hpp"
#include "hal/ITimerHAL.hpp"

namespace motion_core {

/**
 * @brief High-frequency, deterministic non-blocking multi-axis step generator.
 *
 * Consumes pre-computed StepInterval tables produced by VelocityProfile.
 * Designed to execute inside a hardware timer ISR tick (typically 50kHz - 200kHz).
 *
 * CRITICAL INVARIANTS INSIDE stepTimerTick():
 * - Zero division (/)
 * - Zero square root (sqrt)
 * - Zero memory allocation (malloc/new)
 * - Pure integer DDA addition/subtraction and pin toggles
 */
class StepGenerator {
public:
    static constexpr size_t MAX_SEGMENTS = 512;

    StepGenerator();
    ~StepGenerator();

    /**
     * @brief Initialize generator with HAL driver and timer interfaces.
     */
    void init(IDriverHAL* driver_hal, ITimerHAL* timer_hal = nullptr, uint32_t timer_freq_hz = 100000);

    /**
     * @brief Load a multi-axis coordinated move into the generator.
     * @param lead_axis Index of the dominant axis with the largest step count
     * @param axis_steps Array of total steps per axis
     * @param directions Array of motion directions per axis
     * @param table Pointer to pre-computed StepInterval segments
     * @param table_size Number of segments in table
     * @return true if successfully loaded, false if table too large or invalid
     */
    bool loadMove(uint8_t lead_axis,
                  const std::array<uint32_t, MAX_AXES>& axis_steps,
                  const std::array<Direction, MAX_AXES>& directions,
                  const StepInterval* table,
                  size_t table_size);

    /**
     * @brief Start executing the loaded move.
     */
    void start();

    /**
     * @brief Abruptly halt stepping (e.g. for emergency stop or endstop trip).
     */
    void stop();

    /**
     * @brief Pause motion without losing current step position.
     */
    void pause();

    /**
     * @brief Resume paused motion.
     */
    void resume();

    /**
     * @brief Check if step generator is actively running a move.
     */
    bool isRunning() const { return m_is_running; }

    /**
     * @brief Query current absolute step count for an axis.
     */
    int32_t getPosition(uint8_t axis) const {
        return (axis < MAX_AXES) ? m_current_steps[axis] : 0;
    }

    /**
     * @brief Set or override the current step count for an axis (e.g. on homing).
     */
    void setPosition(uint8_t axis, int32_t steps) {
        if (axis < MAX_AXES) {
            m_current_steps[axis] = steps;
        }
    }

    /**
     * @brief Periodic timer ISR tick entry point.
     * MUST satisfy all hard real-time invariants.
     */
    void stepTimerTick();

    /**
     * @brief Register callback invoked when the active move finishes all steps.
     */
    void setOnMoveCompleteCallback(std::function<void()> cb) {
        m_on_move_complete = std::move(cb);
    }

    /**
     * @brief Register callback invoked on each step pulse (primarily for simulation/tests).
     */
    void setOnStepCallback(std::function<void(uint8_t axis, int32_t steps)> cb) {
        m_on_step_cb = std::move(cb);
    }

private:
    IDriverHAL* m_driver_hal{nullptr};
    ITimerHAL* m_timer_hal{nullptr};
    TimerHandle m_timer_handle{INVALID_TIMER_HANDLE};
    uint32_t m_timer_freq_hz{100000};

    // Motion execution state
    volatile bool m_is_running{false};
    volatile bool m_pulse_active{false};

    uint8_t m_lead_axis{0};
    uint32_t m_lead_steps{0};
    std::array<uint32_t, MAX_AXES> m_axis_steps{};
    std::array<Direction, MAX_AXES> m_direction{};
    std::array<int32_t, MAX_AXES> m_dda_accum{};
    std::array<int32_t, MAX_AXES> m_current_steps{};

    // Pre-computed step-interval table (fixed-size, zero heap allocation)
    std::array<StepInterval, MAX_SEGMENTS> m_table{};
    size_t m_total_segments{0};
    size_t m_current_segment_idx{0};
    uint32_t m_segment_steps_remaining{0};
    uint32_t m_current_interval_ticks{0};

    std::function<void()> m_on_move_complete;
    std::function<void(uint8_t axis, int32_t steps)> m_on_step_cb;
};

} // namespace motion_core
