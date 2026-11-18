#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <functional>
#include "planner/Axis.hpp"
#include "planner/RingBuffer.hpp"
#include "planner/StepGenerator.hpp"
#include "planner/SCurveProfile.hpp"
#include "planner/TrapezoidalProfile.hpp"
#include "planner/BacklashCompensator.hpp"
#include "planner/HomingEngine.hpp"
#include "hal/IDriverHAL.hpp"
#include "hal/IEndstopHAL.hpp"
#include "hal/ITimerHAL.hpp"

namespace motion_core {

/**
 * @brief Top-level motion planning and multi-axis trajectory coordination engine.
 *
 * Coordinates:
 * - Command queuing via thread-safe RingBuffer
 * - Automatic backlash compensation injection
 * - 7-Phase S-Curve and Trapezoidal trajectory computation
 * - StepGenerator execution via high-speed timer ISR
 * - Sensorless (StallGuard4), Hall-effect, and physical switch homing
 * - Immediate safety E-STOP preemption
 */
class MotionPlanner {
public:
    static constexpr size_t QUEUE_CAPACITY = 32;

    MotionPlanner();
    ~MotionPlanner();

    /**
     * @brief Initialize planner with HAL interfaces.
     */
    void init(IDriverHAL* driver_hal,
              IEndstopHAL* endstop_hal = nullptr,
              ITimerHAL* timer_hal = nullptr,
              uint32_t timer_freq_hz = 100000);

    /**
     * @brief Configure kinematic limits and scaling for an axis.
     */
    void configureAxis(const AxisConfig& config);
    const AxisConfig& getAxisConfig(uint8_t axis) const;

    /**
     * @brief Queue a generic MotionCommand for execution.
     * @return true if successfully queued, false if queue full or ESTOP active
     */
    bool queueCommand(const MotionCommand& cmd);

    /**
     * @brief Queue coordinated absolute move across up to 4 axes.
     */
    bool moveTo(const std::array<float, MAX_AXES>& targets,
                float feedrate_mm_s = 0.0f,
                ProfileType profile = ProfileType::SCURVE);

    /**
     * @brief Queue single-axis absolute move.
     */
    bool moveToAxis(uint8_t axis,
                    float target,
                    float feedrate_mm_s = 0.0f,
                    ProfileType profile = ProfileType::SCURVE);

    /**
     * @brief Queue coordinated relative move.
     */
    bool moveRelative(const std::array<float, MAX_AXES>& deltas,
                      float feedrate_mm_s = 0.0f,
                      ProfileType profile = ProfileType::SCURVE);

    /**
     * @brief Jog a single axis by relative distance.
     */
    bool jog(uint8_t axis,
             float delta,
             float feedrate_mm_s = 0.0f);

    /**
     * @brief Begin homing procedure on a single axis.
     */
    bool home(uint8_t axis);

    /**
     * @brief Begin sequential homing across all configured axes.
     */
    bool homeAll();

    /**
     * @brief Highest priority Emergency Stop: immediately halts step generator,
     * disables drivers, empties queue, and enters ESTOP_LOCKED state.
     */
    void emergencyStop();

    /**
     * @brief Reset Emergency Stop lock, re-enabling motion commands.
     */
    void resetEstop();

    /**
     * @brief Non-blocking update loop. Call regularly from main loop or RTOS task.
     * @param current_time_us Current system time in microseconds
     */
    void update(uint64_t current_time_us = 0);

    /**
     * @brief Check if any axis is currently executing motion.
     */
    bool isMoving() const;

    /**
     * @brief Check if homing sequence is actively running.
     */
    bool isHoming() const { return m_homing.isHoming(); }

    /**
     * @brief Check if Emergency Stop is locked.
     */
    bool isEstopActive() const { return m_estop_active; }

    /**
     * @brief Get comprehensive real-time status telemetry snapshot.
     */
    MotionStatus getStatus() const;

    /**
     * @brief Force-set current logical position for an axis (e.g. after manual zeroing).
     */
    void setPosition(uint8_t axis, float position);

    /**
     * @brief Get logical coordinate for an axis.
     */
    float getPosition(uint8_t axis) const {
        return (axis < MAX_AXES) ? m_current_pos[axis] : 0.0f;
    }

    // Subsystem accessors
    StepGenerator& getStepGenerator() { return m_step_gen; }
    HomingEngine& getHomingEngine() { return m_homing; }
    BacklashCompensator& getBacklashCompensator() { return m_backlash; }

    void setOnMotionCompleteCallback(std::function<void()> cb) {
        m_on_motion_complete = std::move(cb);
    }

private:
    IDriverHAL* m_driver_hal{nullptr};
    IEndstopHAL* m_endstop_hal{nullptr};
    ITimerHAL* m_timer_hal{nullptr};
    uint32_t m_timer_freq_hz{100000};

    std::array<AxisConfig, MAX_AXES> m_axes{};
    std::array<float, MAX_AXES> m_current_pos{};

    RingBuffer<MotionCommand, QUEUE_CAPACITY> m_queue;
    std::vector<MotionCommand> m_active_subcommands;
    size_t m_subcommand_idx{0};

    StepGenerator m_step_gen;
    SCurveProfile m_scurve;
    TrapezoidalProfile m_trapezoidal;
    BacklashCompensator m_backlash;
    HomingEngine m_homing;

    std::vector<StepInterval> m_step_table;

    volatile bool m_estop_active{false};
    bool m_is_executing_command{false};
    MotionCommand m_current_cmd;

    std::function<void()> m_on_motion_complete;

    bool planAndExecuteCommand(const MotionCommand& cmd);
    void onStepGeneratorFinished();
};

} // namespace motion_core
