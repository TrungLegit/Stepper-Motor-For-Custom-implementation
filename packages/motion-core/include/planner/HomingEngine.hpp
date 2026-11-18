#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <functional>
#include "planner/Axis.hpp"
#include "hal/IEndstopHAL.hpp"
#include "hal/IDriverHAL.hpp"

namespace motion_core {

/**
 * @brief Homing sensor hardware technology selection.
 */
enum class HomingSensorType : uint8_t {
    LIMIT_SWITCH_NO = 0,   // Mechanical Normally Open switch
    LIMIT_SWITCH_NC = 1,   // Mechanical Normally Closed switch (fail-safe)
    HALL_ACTIVE_LOW = 2,   // Magnetic Hall effect sensor, active LOW (A3144)
    HALL_ACTIVE_HIGH = 3,  // Magnetic Hall effect sensor, active HIGH
    STALLGUARD = 4         // TMC2209 sensorless stall detection (StallGuard4)
};

/**
 * @brief Configuration parameters for homing an axis.
 */
struct HomingConfig {
    HomingSensorType sensor_type{HomingSensorType::LIMIT_SWITCH_NC};
    Direction homing_dir{Direction::NEGATIVE};
    float fast_speed_mm_s{25.0f};          // Fast approach feedrate
    float slow_speed_mm_s{5.0f};           // Precision slow approach feedrate
    float backoff_distance_mm{5.0f};       // Distance to back off after initial tap
    float max_travel_mm{300.0f};           // Safety travel distance before abort
    float home_offset_mm{0.0f};            // Logical coordinate to set at home position
    uint8_t stallguard_threshold{50};      // TMC2209 SGTHRS sensitivity (0-255)
    uint32_t debounce_us{5000};            // Endstop software debounce in microseconds
};

/**
 * @brief State machine stages for the double-tap homing sequence.
 */
enum class HomingState : uint8_t {
    IDLE = 0,
    FAST_APPROACH,
    WAIT_FAST_TRIGGER,
    BACKOFF,
    WAIT_BACKOFF_DONE,
    SLOW_APPROACH,
    WAIT_SLOW_TRIGGER,
    SET_ZERO,
    COMPLETE,
    FAILED
};

/**
 * @brief Non-blocking, multi-sensor double-tap homing state machine.
 *
 * Sequence:
 * 1. Fast approach towards limit sensor at fast_speed_mm_s
 * 2. On trigger: stop immediately
 * 3. Backoff away from sensor by backoff_distance_mm
 * 4. Verify sensor release
 * 5. Slow precision approach at slow_speed_mm_s
 * 6. On trigger: stop immediately and zero axis coordinate
 */
class HomingEngine {
public:
    using MoveRequestFn = std::function<bool(uint8_t axis, Direction dir, float dist_mm, float speed_mm_s)>;
    using StopFn = std::function<void()>;
    using SetZeroFn = std::function<void(uint8_t axis, float offset_mm)>;
    using HomingFinishedFn = std::function<void(uint8_t axis, bool success)>;

    HomingEngine();

    void init(IEndstopHAL* endstops, IDriverHAL* drivers);

    void setConfig(uint8_t axis, const HomingConfig& config);
    const HomingConfig& getConfig(uint8_t axis) const;

    void setMoveCallbacks(MoveRequestFn move_fn, StopFn stop_fn, SetZeroFn zero_fn) {
        m_move_fn = std::move(move_fn);
        m_stop_fn = std::move(stop_fn);
        m_zero_fn = std::move(zero_fn);
    }

    void setOnFinishedCallback(HomingFinishedFn cb) {
        m_finished_cb = std::move(cb);
    }

    /**
     * @brief Start homing a single axis.
     */
    bool startHoming(uint8_t axis);

    /**
     * @brief Start sequential homing across multiple axes.
     */
    bool startHomingSequence(const std::vector<uint8_t>& axis_order);

    /**
     * @brief Abort active homing routine immediately.
     */
    void abort();

    /**
     * @brief Non-blocking state machine periodic update.
     * @param current_time_us Current system time in microseconds
     * @param is_move_running Status flag indicating whether the last commanded move is actively stepping
     */
    void update(uint64_t current_time_us, bool is_move_running);

    bool isHoming() const { return m_state != HomingState::IDLE && m_state != HomingState::COMPLETE && m_state != HomingState::FAILED; }
    bool isAxisHomed(uint8_t axis) const { return (axis < MAX_AXES) ? m_is_homed[axis] : false; }
    void setAxisHomed(uint8_t axis, bool homed) { if (axis < MAX_AXES) m_is_homed[axis] = homed; }

    HomingState getState() const { return m_state; }
    uint8_t getCurrentAxis() const { return m_current_axis; }

    /**
     * @brief Query if endstop or stall sensor is currently triggered for axis.
     */
    bool isTriggered(uint8_t axis) const;

private:
    IEndstopHAL* m_endstop_hal{nullptr};
    IDriverHAL* m_driver_hal{nullptr};

    std::array<HomingConfig, MAX_AXES> m_configs{};
    std::array<bool, MAX_AXES> m_is_homed{};

    HomingState m_state{HomingState::IDLE};
    uint8_t m_current_axis{0};
    std::vector<uint8_t> m_sequence;
    size_t m_sequence_idx{0};

    MoveRequestFn m_move_fn;
    StopFn m_stop_fn;
    SetZeroFn m_zero_fn;
    HomingFinishedFn m_finished_cb;

    void advanceSequence();
};

} // namespace motion_core
