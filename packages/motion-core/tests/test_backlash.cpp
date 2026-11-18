#include "planner/BacklashCompensator.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

using namespace motion_core;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

// 1. Basic configuration and queries
bool test_backlash_config_and_defaults() {
    std::cout << "[TEST] Backlash configuration and queries...\n";

    BacklashCompensator comp;
    TEST_ASSERT(comp.getBacklash(0) == 0.0f, "Default backlash is 0");
    TEST_ASSERT(comp.getBacklash(1) == 0.0f, "Default backlash is 0");

    comp.setBacklash(0, 0.25f, 15.0f);
    comp.setBacklash(1, 0.10f, 8.0f);

    TEST_ASSERT(std::abs(comp.getBacklash(0) - 0.25f) < 1e-5f, "Axis 0 backlash set");
    TEST_ASSERT(std::abs(comp.getTransitionSpeed(0) - 15.0f) < 1e-5f, "Axis 0 transition speed set");
    TEST_ASSERT(std::abs(comp.getBacklash(1) - 0.10f) < 1e-5f, "Axis 1 backlash set");
    TEST_ASSERT(std::abs(comp.getTransitionSpeed(1) - 8.0f) < 1e-5f, "Axis 1 transition speed set");

    return true;
}

// 2. Initial move baseline (no compensation before direction is known)
bool test_backlash_initial_move_baseline() {
    std::cout << "[TEST] Backlash initial move establishes baseline without pre-move...\n";

    BacklashCompensator comp;
    comp.setBacklash(0, 0.20f, 10.0f);

    std::array<float, MAX_AXES> current_pos{0.0f, 0.0f, 0.0f, 0.0f};

    MotionCommand cmd;
    cmd.type = MotionType::MOVE_ABS;
    cmd.axis_mask[0] = true;
    cmd.targets[0] = 50.0f; // Move positive
    cmd.feedrate_mm_s = 50.0f;

    std::vector<MotionCommand> out_cmds;
    bool compensated = comp.processMove(current_pos, cmd, out_cmds);

    TEST_ASSERT(!compensated, "First move establishes direction baseline without compensation");
    TEST_ASSERT(out_cmds.size() == 1, "Only single primary command emitted");
    TEST_ASSERT(!out_cmds[0].is_backlash_premove, "Command is not a backlash pre-move");
    TEST_ASSERT(out_cmds[0].targets[0] == 50.0f, "Target matches requested 50mm");

    return true;
}

// 3. Same direction move (no compensation)
bool test_backlash_same_direction_continuation() {
    std::cout << "[TEST] Backlash same direction move requires no compensation...\n";

    BacklashCompensator comp;
    comp.setBacklash(0, 0.20f, 10.0f);

    std::array<float, MAX_AXES> current_pos{0.0f, 0.0f, 0.0f, 0.0f};

    // First move: 0 -> 20 (POSITIVE)
    MotionCommand cmd1;
    cmd1.type = MotionType::MOVE_ABS;
    cmd1.axis_mask[0] = true;
    cmd1.targets[0] = 20.0f;
    std::vector<MotionCommand> out1;
    comp.processMove(current_pos, cmd1, out1);

    // Second move: 20 -> 40 (still POSITIVE)
    current_pos[0] = 20.0f;
    MotionCommand cmd2;
    cmd2.type = MotionType::MOVE_ABS;
    cmd2.axis_mask[0] = true;
    cmd2.targets[0] = 40.0f;
    std::vector<MotionCommand> out2;
    bool comp2 = comp.processMove(current_pos, cmd2, out2);

    TEST_ASSERT(!comp2, "No compensation on continuation in same direction");
    TEST_ASSERT(out2.size() == 1, "Only 1 command generated");
    TEST_ASSERT(!out2[0].is_backlash_premove, "Not a pre-move");

    return true;
}

// 4. Direction reversal triggers compensation pre-move injection
bool test_backlash_reversal_injection() {
    std::cout << "[TEST] Backlash direction reversal triggers compensation pre-move...\n";

    BacklashCompensator comp;
    const float backlash_dist = 0.35f;
    const float transition_speed = 12.0f;
    comp.setBacklash(0, backlash_dist, transition_speed);

    std::array<float, MAX_AXES> current_pos{0.0f, 0.0f, 0.0f, 0.0f};

    // Move 1: Positive direction (0 -> 50)
    MotionCommand cmd1;
    cmd1.type = MotionType::MOVE_ABS;
    cmd1.axis_mask[0] = true;
    cmd1.targets[0] = 50.0f;
    std::vector<MotionCommand> out1;
    comp.processMove(current_pos, cmd1, out1);

    // Move 2: Reverse direction (50 -> 30, NEGATIVE)
    current_pos[0] = 50.0f;
    MotionCommand cmd2;
    cmd2.type = MotionType::MOVE_ABS;
    cmd2.axis_mask[0] = true;
    cmd2.targets[0] = 30.0f;
    cmd2.feedrate_mm_s = 60.0f;

    std::vector<MotionCommand> out2;
    bool comp2 = comp.processMove(current_pos, cmd2, out2);

    TEST_ASSERT(comp2, "Compensation required on reversal");
    TEST_ASSERT(out2.size() == 2, "Reversal generates 2 commands: pre-move + primary move");

    // Pre-move checks
    const auto& pre_move = out2[0];
    TEST_ASSERT(pre_move.is_backlash_premove, "First command is backlash pre-move");
    TEST_ASSERT(pre_move.type == MotionType::MOVE_REL, "Pre-move is relative");
    TEST_ASSERT(pre_move.axis_mask[0], "Pre-move targets axis 0");
    TEST_ASSERT(std::abs(pre_move.targets[0] - (-backlash_dist)) < 1e-5f, "Pre-move distance matches negative backlash");
    TEST_ASSERT(std::abs(pre_move.feedrate_mm_s - transition_speed) < 1e-5f, "Pre-move runs at transition speed");

    // Primary move checks
    const auto& primary = out2[1];
    TEST_ASSERT(!primary.is_backlash_premove, "Second command is primary move");
    TEST_ASSERT(primary.targets[0] == 30.0f, "Primary target is preserved");
    TEST_ASSERT(primary.feedrate_mm_s == 60.0f, "Primary feedrate is preserved");

    // Move 3: Reverse back to positive (30 -> 40)
    current_pos[0] = 30.0f;
    MotionCommand cmd3;
    cmd3.type = MotionType::MOVE_ABS;
    cmd3.axis_mask[0] = true;
    cmd3.targets[0] = 40.0f;
    std::vector<MotionCommand> out3;
    bool comp3 = comp.processMove(current_pos, cmd3, out3);

    TEST_ASSERT(comp3, "Reversal back to positive triggers compensation");
    TEST_ASSERT(out3.size() == 2, "2 commands generated");
    TEST_ASSERT(std::abs(out3[0].targets[0] - backlash_dist) < 1e-5f, "Pre-move in positive direction");

    return true;
}

// 5. Multi-axis selective compensation
bool test_backlash_multi_axis_selective() {
    std::cout << "[TEST] Backlash multi-axis selective compensation...\n";

    BacklashCompensator comp;
    // Axis 0 has backlash, Axis 1 has 0 backlash, Axis 2 has backlash
    comp.setBacklash(0, 0.20f, 10.0f);
    comp.setBacklash(1, 0.0f, 10.0f); // zero backlash
    comp.setBacklash(2, 0.15f, 8.0f);

    std::array<float, MAX_AXES> pos{0.0f, 0.0f, 0.0f, 0.0f};

    // Move 1: All axes move POSITIVE
    MotionCommand cmd1;
    cmd1.type = MotionType::MOVE_ABS;
    cmd1.axis_mask[0] = true;
    cmd1.axis_mask[1] = true;
    cmd1.axis_mask[2] = true;
    cmd1.targets[0] = 10.0f;
    cmd1.targets[1] = 10.0f;
    cmd1.targets[2] = 10.0f;
    std::vector<MotionCommand> out1;
    comp.processMove(pos, cmd1, out1);

    // Move 2: Axis 0 reverses (NEGATIVE), Axis 1 reverses (NEGATIVE, but zero backlash), Axis 2 continues (POSITIVE)
    pos = {10.0f, 10.0f, 10.0f, 0.0f};
    MotionCommand cmd2;
    cmd2.type = MotionType::MOVE_ABS;
    cmd2.axis_mask[0] = true;
    cmd2.axis_mask[1] = true;
    cmd2.axis_mask[2] = true;
    cmd2.targets[0] = 5.0f;   // Reversal (needs comp)
    cmd2.targets[1] = 5.0f;   // Reversal (0 backlash, NO comp)
    cmd2.targets[2] = 15.0f;  // Continuation (NO comp)
    std::vector<MotionCommand> out2;
    bool comp2 = comp.processMove(pos, cmd2, out2);

    TEST_ASSERT(comp2, "Compensation required for Axis 0");
    TEST_ASSERT(out2.size() == 2, "Pre-move + primary move generated");

    const auto& pre = out2[0];
    TEST_ASSERT(pre.axis_mask[0], "Axis 0 included in pre-move");
    TEST_ASSERT(!pre.axis_mask[1], "Axis 1 excluded (zero backlash)");
    TEST_ASSERT(!pre.axis_mask[2], "Axis 2 excluded (same direction)");
    TEST_ASSERT(std::abs(pre.targets[0] - (-0.20f)) < 1e-5f, "Axis 0 negative pre-move distance");

    return true;
}

// 6. Reset functionality
bool test_backlash_reset() {
    std::cout << "[TEST] Backlash compensator reset clears history...\n";

    BacklashCompensator comp;
    comp.setBacklash(0, 0.30f);
    comp.recordDirection(0, Direction::POSITIVE);
    TEST_ASSERT(comp.needsCompensation(0, Direction::NEGATIVE), "Needs compensation on reversal");

    comp.reset();
    TEST_ASSERT(!comp.needsCompensation(0, Direction::NEGATIVE), "After reset, no direction known -> no compensation");
    TEST_ASSERT(comp.getBacklash(0) == 0.0f, "Reset zeroes backlash values");

    return true;
}

bool run_all_backlash_tests() {
    std::cout << "========================================\n";
    std::cout << " RUNNING BACKLASH COMPENSATOR TESTS    \n";
    std::cout << "========================================\n";

    if (!test_backlash_config_and_defaults()) return false;
    if (!test_backlash_initial_move_baseline()) return false;
    if (!test_backlash_same_direction_continuation()) return false;
    if (!test_backlash_reversal_injection()) return false;
    if (!test_backlash_multi_axis_selective()) return false;
    if (!test_backlash_reset()) return false;

    std::cout << "========================================\n";
    std::cout << " ALL BACKLASH COMPENSATOR TESTS PASSED! \n";
    std::cout << "========================================\n";
    return true;
}

#ifndef TEST_COMBINED_RUNNER
int main() {
    return run_all_backlash_tests() ? 0 : 1;
}
#endif
