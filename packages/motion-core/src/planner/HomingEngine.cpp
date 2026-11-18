#include "planner/HomingEngine.hpp"
#include <algorithm>

namespace motion_core {

HomingEngine::HomingEngine() {
    m_configs.fill(HomingConfig{});
    m_is_homed.fill(false);
}

void HomingEngine::init(IEndstopHAL* endstops, IDriverHAL* drivers) {
    m_endstop_hal = endstops;
    m_driver_hal = drivers;
}

void HomingEngine::setConfig(uint8_t axis, const HomingConfig& config) {
    if (axis < MAX_AXES) {
        m_configs[axis] = config;
    }
}

const HomingConfig& HomingEngine::getConfig(uint8_t axis) const {
    static const HomingConfig default_cfg{};
    return (axis < MAX_AXES) ? m_configs[axis] : default_cfg;
}

bool HomingEngine::startHoming(uint8_t axis) {
    if (axis >= MAX_AXES) {
        return false;
    }

    m_sequence.clear();
    m_sequence.push_back(axis);
    m_sequence_idx = 0;
    m_current_axis = axis;

    const auto& cfg = m_configs[axis];

    // Configure hardware layer for this sensor
    if (cfg.sensor_type == HomingSensorType::STALLGUARD) {
        if (m_driver_hal) {
            m_driver_hal->setStallGuardThreshold(axis, cfg.stallguard_threshold);
        }
    } else if (m_endstop_hal) {
        EndstopType type = EndstopType::SWITCH_NC;
        switch (cfg.sensor_type) {
            case HomingSensorType::LIMIT_SWITCH_NO: type = EndstopType::SWITCH_NO; break;
            case HomingSensorType::LIMIT_SWITCH_NC: type = EndstopType::SWITCH_NC; break;
            case HomingSensorType::HALL_ACTIVE_LOW: type = EndstopType::HALL_ACTIVE_LOW; break;
            case HomingSensorType::HALL_ACTIVE_HIGH: type = EndstopType::HALL_ACTIVE_HIGH; break;
            default: break;
        }
        m_endstop_hal->configureEndstop(axis, type, cfg.debounce_us);
    }

    // Check if endstop is already triggered initially
    if (isTriggered(axis)) {
        // Back off first
        m_state = HomingState::BACKOFF;
        const Direction backoff_dir = invertDirection(cfg.homing_dir);
        if (m_move_fn) {
            m_move_fn(axis, backoff_dir, cfg.backoff_distance_mm, cfg.fast_speed_mm_s);
        }
        m_state = HomingState::WAIT_BACKOFF_DONE;
    } else {
        // Normal fast approach
        m_state = HomingState::FAST_APPROACH;
        if (m_move_fn) {
            m_move_fn(axis, cfg.homing_dir, cfg.max_travel_mm, cfg.fast_speed_mm_s);
        }
        m_state = HomingState::WAIT_FAST_TRIGGER;
    }

    return true;
}

bool HomingEngine::startHomingSequence(const std::vector<uint8_t>& axis_order) {
    if (axis_order.empty()) {
        return false;
    }
    m_sequence = axis_order;
    m_sequence_idx = 0;
    return startHoming(m_sequence[0]);
}

void HomingEngine::abort() {
    if (m_stop_fn) {
        m_stop_fn();
    }
    m_state = HomingState::FAILED;
    m_sequence.clear();
}

bool HomingEngine::isTriggered(uint8_t axis) const {
    if (axis >= MAX_AXES) {
        return false;
    }

    const auto& cfg = m_configs[axis];
    if (cfg.sensor_type == HomingSensorType::STALLGUARD) {
        if (!m_driver_hal) {
            return false;
        }
        uint16_t sg_val = 0;
        if (m_driver_hal->readStallGuard(axis, sg_val)) {
            return sg_val < static_cast<uint16_t>(cfg.stallguard_threshold);
        }
        return false;
    } else {
        if (!m_endstop_hal) {
            return false;
        }
        return m_endstop_hal->isTriggered(axis);
    }
}

void HomingEngine::update(uint64_t current_time_us, bool is_move_running) {
    if (m_endstop_hal) {
        m_endstop_hal->update(current_time_us);
    }

    if (m_state == HomingState::IDLE || m_state == HomingState::COMPLETE || m_state == HomingState::FAILED) {
        return;
    }

    const auto& cfg = m_configs[m_current_axis];

    switch (m_state) {
        case HomingState::WAIT_FAST_TRIGGER:
            if (isTriggered(m_current_axis)) {
                if (m_stop_fn) {
                    m_stop_fn();
                }
                const Direction backoff_dir = invertDirection(cfg.homing_dir);
                m_state = HomingState::BACKOFF;
                if (m_move_fn) {
                    m_move_fn(m_current_axis, backoff_dir, cfg.backoff_distance_mm, cfg.fast_speed_mm_s);
                }
                m_state = HomingState::WAIT_BACKOFF_DONE;
            } else if (!is_move_running) {
                // Traveled max distance without trigger -> fail
                m_state = HomingState::FAILED;
                if (m_finished_cb) {
                    m_finished_cb(m_current_axis, false);
                }
                advanceSequence();
            }
            break;

        case HomingState::WAIT_BACKOFF_DONE:
            if (!is_move_running) {
                if (!isTriggered(m_current_axis)) {
                    // Released! Start slow precision approach
                    m_state = HomingState::SLOW_APPROACH;
                    if (m_move_fn) {
                        m_move_fn(m_current_axis, cfg.homing_dir, cfg.backoff_distance_mm * 2.0f, cfg.slow_speed_mm_s);
                    }
                    m_state = HomingState::WAIT_SLOW_TRIGGER;
                } else {
                    // Still triggered after backoff -> failure
                    m_state = HomingState::FAILED;
                    if (m_finished_cb) {
                        m_finished_cb(m_current_axis, false);
                    }
                    advanceSequence();
                }
            }
            break;

        case HomingState::WAIT_SLOW_TRIGGER:
            if (isTriggered(m_current_axis)) {
                if (m_stop_fn) {
                    m_stop_fn();
                }
                m_state = HomingState::SET_ZERO;
                if (m_zero_fn) {
                    m_zero_fn(m_current_axis, cfg.home_offset_mm);
                }
                m_is_homed[m_current_axis] = true;
                m_state = HomingState::COMPLETE;
                if (m_finished_cb) {
                    m_finished_cb(m_current_axis, true);
                }
                advanceSequence();
            } else if (!is_move_running) {
                m_state = HomingState::FAILED;
                if (m_finished_cb) {
                    m_finished_cb(m_current_axis, false);
                }
                advanceSequence();
            }
            break;

        default:
            break;
    }
}

void HomingEngine::advanceSequence() {
    if (m_sequence_idx + 1 < m_sequence.size()) {
        ++m_sequence_idx;
        startHoming(m_sequence[m_sequence_idx]);
    }
}

} // namespace motion_core
