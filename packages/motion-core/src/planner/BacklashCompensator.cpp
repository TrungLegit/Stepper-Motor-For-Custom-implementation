#include "planner/BacklashCompensator.hpp"
#include <cmath>
#include <algorithm>

namespace motion_core {

BacklashCompensator::BacklashCompensator() {
    reset();
}

void BacklashCompensator::reset() {
    m_backlash_mm.fill(0.0f);
    m_transition_speed.fill(10.0f);
    m_last_direction.fill(Direction::POSITIVE);
    m_direction_known.fill(false);
}

void BacklashCompensator::setBacklash(uint8_t axis, float backlash_mm, float transition_speed_mm_s) {
    if (axis < MAX_AXES) {
        m_backlash_mm[axis] = std::max(0.0f, backlash_mm);
        m_transition_speed[axis] = std::max(0.1f, transition_speed_mm_s);
    }
}

bool BacklashCompensator::needsCompensation(uint8_t axis, Direction new_dir) const {
    if (axis >= MAX_AXES || m_backlash_mm[axis] <= 1e-6f) {
        return false;
    }
    if (!m_direction_known[axis]) {
        // Initial move establishes direction baseline without compensation
        return false;
    }
    return m_last_direction[axis] != new_dir;
}

float BacklashCompensator::getCompensationDistance(uint8_t axis, Direction new_dir) const {
    return needsCompensation(axis, new_dir) ? m_backlash_mm[axis] : 0.0f;
}

void BacklashCompensator::recordDirection(uint8_t axis, Direction dir) {
    if (axis < MAX_AXES) {
        m_last_direction[axis] = dir;
        m_direction_known[axis] = true;
    }
}

bool BacklashCompensator::processMove(const std::array<float, MAX_AXES>& current_pos,
                                      const MotionCommand& cmd,
                                      std::vector<MotionCommand>& out_commands) {
    if (cmd.is_backlash_premove) {
        out_commands.push_back(cmd);
        return false;
    }

    bool compensation_required = false;
    MotionCommand pre_cmd;
    pre_cmd.type = MotionType::MOVE_REL;
    pre_cmd.is_backlash_premove = true;
    pre_cmd.profile = cmd.profile;
    pre_cmd.feedrate_mm_s = 1e6f; // Will be lowered to min transition speed

    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        if (!cmd.axis_mask[i]) {
            continue;
        }

        float delta = 0.0f;
        if (cmd.type == MotionType::MOVE_REL || cmd.type == MotionType::JOG) {
            delta = cmd.targets[i];
        } else {
            delta = cmd.targets[i] - current_pos[i];
        }

        if (std::abs(delta) > 1e-5f) {
            const Direction new_dir = (delta > 0.0f) ? Direction::POSITIVE : Direction::NEGATIVE;
            if (needsCompensation(i, new_dir)) {
                compensation_required = true;
                pre_cmd.axis_mask[i] = true;
                const float comp_dist = m_backlash_mm[i];
                pre_cmd.targets[i] = (new_dir == Direction::POSITIVE) ? comp_dist : -comp_dist;
                pre_cmd.feedrate_mm_s = std::min(pre_cmd.feedrate_mm_s, m_transition_speed[i]);
            }
            recordDirection(i, new_dir);
        }
    }

    if (compensation_required) {
        out_commands.push_back(pre_cmd);
    }
    out_commands.push_back(cmd);
    return compensation_required;
}

} // namespace motion_core
