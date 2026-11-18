#include "planner/MotionPlanner.hpp"
#include <cmath>
#include <algorithm>

namespace motion_core {

MotionPlanner::MotionPlanner() {
    m_current_pos.fill(0.0f);
    // Initialize default axis parameters
    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        m_axes[i].axis_id = i;
        m_axes[i].steps_per_mm = 80.0f;
        m_axes[i].max_speed_mm_s = 100.0f;
        m_axes[i].max_accel_mm_s2 = 500.0f;
        m_axes[i].max_jerk_mm_s3 = 2000.0f;
        m_axes[i].backlash_mm = 0.0f;
        m_axes[i].travel_min_mm = -500.0f;
        m_axes[i].travel_max_mm = 500.0f;
    }
}

MotionPlanner::~MotionPlanner() {
    emergencyStop();
}

void MotionPlanner::init(IDriverHAL* driver_hal,
                         IEndstopHAL* endstop_hal,
                         ITimerHAL* timer_hal,
                         uint32_t timer_freq_hz) {
    m_driver_hal = driver_hal;
    m_endstop_hal = endstop_hal;
    m_timer_hal = timer_hal;
    m_timer_freq_hz = (timer_freq_hz > 0) ? timer_freq_hz : 100000;

    m_step_gen.init(m_driver_hal, m_timer_hal, m_timer_freq_hz);
    m_step_gen.setOnMoveCompleteCallback([this]() {
        onStepGeneratorFinished();
    });

    m_homing.init(m_endstop_hal, m_driver_hal);
    m_homing.setMoveCallbacks(
        [this](uint8_t axis, Direction dir, float dist_mm, float speed_mm_s) -> bool {
            if (axis >= MAX_AXES || m_estop_active) return false;
            const float steps_per_unit = m_axes[axis].steps_per_mm;
            const uint32_t steps = static_cast<uint32_t>(std::round(dist_mm * steps_per_unit));
            if (steps == 0) return true;

            std::array<uint32_t, MAX_AXES> axis_steps{};
            axis_steps.fill(0);
            axis_steps[axis] = steps;

            std::array<Direction, MAX_AXES> dirs{};
            dirs.fill(Direction::POSITIVE);
            dirs[axis] = (m_axes[axis].invert_dir) ? invertDirection(dir) : dir;

            m_step_table.clear();
            const float a_max = m_axes[axis].max_accel_mm_s2;
            m_trapezoidal.calculate(dist_mm, 0.0f, 0.0f, speed_mm_s, a_max);
            m_trapezoidal.generateStepTable(steps_per_unit, m_timer_freq_hz, m_step_table);

            if (m_step_table.empty()) return false;
            if (!m_step_gen.loadMove(axis, axis_steps, dirs, m_step_table.data(), m_step_table.size())) {
                return false;
            }
            m_step_gen.start();
            return true;
        },
        [this]() {
            m_step_gen.stop();
        },
        [this](uint8_t axis, float offset_mm) {
            setPosition(axis, offset_mm);
        }
    );
}

void MotionPlanner::configureAxis(const AxisConfig& config) {
    if (config.axis_id < MAX_AXES) {
        m_axes[config.axis_id] = config;
        m_backlash.setBacklash(config.axis_id, config.backlash_mm);
    }
}

const AxisConfig& MotionPlanner::getAxisConfig(uint8_t axis) const {
    static const AxisConfig default_cfg{};
    return (axis < MAX_AXES) ? m_axes[axis] : default_cfg;
}

bool MotionPlanner::queueCommand(const MotionCommand& cmd) {
    if (m_estop_active) {
        return false;
    }
    return m_queue.push(cmd);
}

bool MotionPlanner::moveTo(const std::array<float, MAX_AXES>& targets,
                           float feedrate_mm_s,
                           ProfileType profile) {
    MotionCommand cmd;
    cmd.type = MotionType::MOVE_ABS;
    cmd.profile = profile;
    cmd.feedrate_mm_s = feedrate_mm_s;
    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        cmd.targets[i] = targets[i];
        cmd.axis_mask[i] = true;
    }
    return queueCommand(cmd);
}

bool MotionPlanner::moveToAxis(uint8_t axis, float target, float feedrate_mm_s, ProfileType profile) {
    if (axis >= MAX_AXES) return false;
    MotionCommand cmd;
    cmd.type = MotionType::MOVE_ABS;
    cmd.profile = profile;
    cmd.feedrate_mm_s = feedrate_mm_s;
    cmd.targets[axis] = target;
    cmd.axis_mask[axis] = true;
    return queueCommand(cmd);
}

bool MotionPlanner::moveRelative(const std::array<float, MAX_AXES>& deltas,
                                float feedrate_mm_s,
                                ProfileType profile) {
    MotionCommand cmd;
    cmd.type = MotionType::MOVE_REL;
    cmd.profile = profile;
    cmd.feedrate_mm_s = feedrate_mm_s;
    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        cmd.targets[i] = deltas[i];
        cmd.axis_mask[i] = true;
    }
    return queueCommand(cmd);
}

bool MotionPlanner::jog(uint8_t axis, float delta, float feedrate_mm_s) {
    if (axis >= MAX_AXES) return false;
    MotionCommand cmd;
    cmd.type = MotionType::JOG;
    cmd.profile = ProfileType::TRAPEZOIDAL;
    cmd.feedrate_mm_s = feedrate_mm_s;
    cmd.targets[axis] = delta;
    cmd.axis_mask[axis] = true;
    return queueCommand(cmd);
}

bool MotionPlanner::home(uint8_t axis) {
    if (m_estop_active || axis >= MAX_AXES) return false;
    return m_homing.startHoming(axis);
}

bool MotionPlanner::homeAll() {
    if (m_estop_active) return false;
    std::vector<uint8_t> all_axes = {0, 1, 2, 3};
    return m_homing.startHomingSequence(all_axes);
}

void MotionPlanner::emergencyStop() {
    m_estop_active = true;
    m_step_gen.stop();
    m_homing.abort();
    m_queue.clear();
    m_active_subcommands.clear();
    m_is_executing_command = false;

    if (m_driver_hal) {
        for (uint8_t i = 0; i < MAX_AXES; ++i) {
            m_driver_hal->setEnablePin(i, false);
            m_driver_hal->setStepPin(i, PinState::LOW);
        }
    }
}

void MotionPlanner::resetEstop() {
    m_estop_active = false;
}

bool MotionPlanner::isMoving() const {
    return m_step_gen.isRunning() || m_is_executing_command || !m_queue.empty() || m_homing.isHoming();
}

MotionStatus MotionPlanner::getStatus() const {
    MotionStatus st;
    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        st.current_pos[i] = m_current_pos[i];
        st.current_steps[i] = m_step_gen.getPosition(i);
        st.is_homed[i] = m_homing.isAxisHomed(i);
        st.current_vel[i] = 0.0f; // Instantaneous velocity from generator
    }
    st.is_moving = isMoving();
    st.is_homing = m_homing.isHoming();
    st.estop_active = m_estop_active;
    st.queue_depth = static_cast<int32_t>(m_queue.size() + m_active_subcommands.size());
    st.estimated_time_remaining_s = 0.0f;
    return st;
}

void MotionPlanner::setPosition(uint8_t axis, float position) {
    if (axis < MAX_AXES) {
        m_current_pos[axis] = position;
        const int32_t steps = static_cast<int32_t>(std::round(position * m_axes[axis].steps_per_mm));
        m_step_gen.setPosition(axis, steps);
    }
}

void MotionPlanner::update(uint64_t current_time_us) {
    if (m_estop_active) {
        return;
    }

    // Drive homing state machine if active
    if (m_homing.isHoming()) {
        m_homing.update(current_time_us, m_step_gen.isRunning());
        return;
    }

    // Check if currently executing subcommands (e.g. backlash pre-move + primary move)
    if (!m_active_subcommands.empty()) {
        if (!m_step_gen.isRunning()) {
            ++m_subcommand_idx;
            if (m_subcommand_idx < m_active_subcommands.size()) {
                planAndExecuteCommand(m_active_subcommands[m_subcommand_idx]);
            } else {
                m_active_subcommands.clear();
                m_subcommand_idx = 0;
                m_is_executing_command = false;
                if (m_on_motion_complete) {
                    m_on_motion_complete();
                }
            }
        }
        return;
    }

    // If step generator is idle, dequeue next user command from ring buffer
    if (!m_step_gen.isRunning()) {
        MotionCommand next_cmd;
        if (m_queue.pop(next_cmd)) {
            m_subcommand_idx = 0;
            m_is_executing_command = true;
            m_backlash.processMove(m_current_pos, next_cmd, m_active_subcommands);
            if (!m_active_subcommands.empty()) {
                planAndExecuteCommand(m_active_subcommands[0]);
            }
        }
    }
}

bool MotionPlanner::planAndExecuteCommand(const MotionCommand& cmd) {
    std::array<float, MAX_AXES> deltas{};
    std::array<uint32_t, MAX_AXES> axis_steps{};
    std::array<Direction, MAX_AXES> dirs{};

    uint8_t lead_axis = 0;
    uint32_t max_steps = 0;

    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        if (!cmd.axis_mask[i]) {
            deltas[i] = 0.0f;
            axis_steps[i] = 0;
            dirs[i] = Direction::POSITIVE;
            continue;
        }

        if (cmd.type == MotionType::MOVE_REL || cmd.type == MotionType::JOG) {
            deltas[i] = cmd.targets[i];
        } else {
            deltas[i] = cmd.targets[i] - m_current_pos[i];
        }

        const float abs_delta = std::abs(deltas[i]);
        axis_steps[i] = static_cast<uint32_t>(std::round(abs_delta * m_axes[i].steps_per_mm));
        dirs[i] = (deltas[i] >= 0.0f) ? Direction::POSITIVE : Direction::NEGATIVE;
        if (m_axes[i].invert_dir) {
            dirs[i] = invertDirection(dirs[i]);
        }

        if (axis_steps[i] > max_steps) {
            max_steps = axis_steps[i];
            lead_axis = i;
        }
    }

    if (max_steps == 0) {
        // Zero distance move: immediately update target position without stepping
        if (!cmd.is_backlash_premove) {
            for (uint8_t i = 0; i < MAX_AXES; ++i) {
                if (cmd.axis_mask[i]) {
                    m_current_pos[i] = (cmd.type == MotionType::MOVE_REL || cmd.type == MotionType::JOG)
                                           ? (m_current_pos[i] + cmd.targets[i])
                                           : cmd.targets[i];
                }
            }
        }
        return true;
    }

    // Kinematic constraints based on the lead axis
    const float lead_dist = static_cast<float>(max_steps) / m_axes[lead_axis].steps_per_mm;
    float feedrate = (cmd.feedrate_mm_s > 0.0f)
                         ? std::min(cmd.feedrate_mm_s, m_axes[lead_axis].max_speed_mm_s)
                         : m_axes[lead_axis].max_speed_mm_s;
    const float accel = m_axes[lead_axis].max_accel_mm_s2;
    const float jerk = m_axes[lead_axis].max_jerk_mm_s3;

    m_step_table.clear();
    if (cmd.profile == ProfileType::SCURVE) {
        m_scurve.calculate(lead_dist, 0.0f, 0.0f, feedrate, accel, jerk);
        m_scurve.generateStepTable(m_axes[lead_axis].steps_per_mm, m_timer_freq_hz, m_step_table);
    } else {
        m_trapezoidal.calculate(lead_dist, 0.0f, 0.0f, feedrate, accel);
        m_trapezoidal.generateStepTable(m_axes[lead_axis].steps_per_mm, m_timer_freq_hz, m_step_table);
    }

    if (m_step_table.empty()) {
        return false;
    }

    m_current_cmd = cmd;
    if (!m_step_gen.loadMove(lead_axis, axis_steps, dirs, m_step_table.data(), m_step_table.size())) {
        return false;
    }

    m_step_gen.start();
    return true;
}

void MotionPlanner::onStepGeneratorFinished() {
    // If this was a regular move (not backlash take-up pre-move), update logical workspace position
    if (!m_current_cmd.is_backlash_premove) {
        for (uint8_t i = 0; i < MAX_AXES; ++i) {
            if (m_current_cmd.axis_mask[i]) {
                if (m_current_cmd.type == MotionType::MOVE_REL || m_current_cmd.type == MotionType::JOG) {
                    m_current_pos[i] += m_current_cmd.targets[i];
                } else {
                    m_current_pos[i] = m_current_cmd.targets[i];
                }
            }
        }
    }
}

} // namespace motion_core
