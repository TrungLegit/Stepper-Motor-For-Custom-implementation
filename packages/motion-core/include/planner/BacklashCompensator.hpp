#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include "planner/Axis.hpp"

namespace motion_core {

/**
 * @brief Automatic backlash compensation engine.
 *
 * Tracks motion direction reversals per axis. When a reversal is detected,
 * transparently inserts a pre-move take-up segment at a controlled transition
 * speed without altering the user-facing coordinate system.
 */
class BacklashCompensator {
public:
    BacklashCompensator();

    /**
     * @brief Configure backlash parameters for an axis.
     * @param axis Axis index (0 to MAX_AXES-1)
     * @param backlash_mm Clearance distance in mm/deg to take up on reversal
     * @param transition_speed_mm_s Speed in mm/s or deg/s to perform backlash pre-move
     */
    void setBacklash(uint8_t axis, float backlash_mm, float transition_speed_mm_s = 10.0f);

    float getBacklash(uint8_t axis) const {
        return (axis < MAX_AXES) ? m_backlash_mm[axis] : 0.0f;
    }

    float getTransitionSpeed(uint8_t axis) const {
        return (axis < MAX_AXES) ? m_transition_speed[axis] : 10.0f;
    }

    /**
     * @brief Check if moving in new_dir constitutes a direction reversal with non-zero backlash.
     */
    bool needsCompensation(uint8_t axis, Direction new_dir) const;

    /**
     * @brief Get backlash clearance distance required for an axis moving in new_dir.
     * @return Positive backlash_mm if reversal detected, or 0.0f if no reversal.
     */
    float getCompensationDistance(uint8_t axis, Direction new_dir) const;

    /**
     * @brief Update the recorded direction for an axis.
     */
    void recordDirection(uint8_t axis, Direction dir);

    /**
     * @brief Reset recorded directions to uninitialized.
     */
    void reset();

    /**
     * @brief Inspect an incoming commanded move, generating any necessary backlash
     * pre-move commands before the primary move.
     * 
     * @param current_pos Current axis coordinates
     * @param cmd Commanded move
     * @param out_commands Output vector containing pre-moves (if any) followed by the primary move
     * @return true if backlash compensation pre-moves were generated
     */
    bool processMove(const std::array<float, MAX_AXES>& current_pos,
                     const MotionCommand& cmd,
                     std::vector<MotionCommand>& out_commands);

private:
    std::array<float, MAX_AXES> m_backlash_mm{};
    std::array<float, MAX_AXES> m_transition_speed{};
    std::array<Direction, MAX_AXES> m_last_direction{};
    std::array<bool, MAX_AXES> m_direction_known{};
};

} // namespace motion_core
