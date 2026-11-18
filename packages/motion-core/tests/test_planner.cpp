#include "planner/MotionPlanner.hpp"
#include "planner/RingBuffer.hpp"
#include "hal_impl/mock/MockHAL.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <thread>
#include <chrono>

using namespace motion_core;
using namespace motion_core::mock;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

// 1. Ring-buffer queue management & capacity limits
bool test_planner_ring_buffer() {
    std::cout << "[TEST] RingBuffer queue operations & capacity limits...\n";

    RingBuffer<int, 8> buffer; // Power of 2 capacity

    TEST_ASSERT(buffer.empty(), "Buffer starts empty");
    TEST_ASSERT(buffer.size() == 0, "Initial size is 0");
    TEST_ASSERT(buffer.capacity() == 8, "Capacity is 8");

    // Push 8 items
    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT(buffer.push(i * 10), "Push item succeeds");
    }

    TEST_ASSERT(buffer.full(), "Buffer is now full");
    TEST_ASSERT(buffer.size() == 8, "Buffer size is 8");
    TEST_ASSERT(!buffer.push(999), "Pushing to full buffer returns false");

    // Peek front
    const int* front = buffer.peek();
    TEST_ASSERT(front != nullptr && *front == 0, "Peek front returns first item");

    // Pop items in FIFO order
    for (int i = 0; i < 8; ++i) {
        int val = -1;
        TEST_ASSERT(buffer.pop(val), "Pop succeeds");
        TEST_ASSERT(val == i * 10, "FIFO ordering preserved");
    }

    TEST_ASSERT(buffer.empty(), "Buffer is empty after popping all");
    int dummy = 0;
    TEST_ASSERT(!buffer.pop(dummy), "Pop on empty returns false");

    return true;
}

// 2. MotionPlanner queue management & queue depth reporting
bool test_planner_queue_depth_and_estop_rejection() {
    std::cout << "[TEST] MotionPlanner queue depth & ESTOP preemption...\n";

    MockHAL hal;
    MotionPlanner planner;
    planner.init(&hal.driver(), &hal.endstop(), &hal.timer(), 100000);

    TEST_ASSERT(planner.getStatus().queue_depth == 0, "Initial queue depth is 0");
    TEST_ASSERT(!planner.isMoving(), "Not moving initially");

    // Queue 3 absolute moves
    std::array<float, MAX_AXES> targets1{10.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, MAX_AXES> targets2{20.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, MAX_AXES> targets3{30.0f, 0.0f, 0.0f, 0.0f};

    TEST_ASSERT(planner.moveTo(targets1, 50.0f), "Queue move 1");
    TEST_ASSERT(planner.moveTo(targets2, 50.0f), "Queue move 2");
    TEST_ASSERT(planner.moveTo(targets3, 50.0f), "Queue move 3");

    TEST_ASSERT(planner.getStatus().queue_depth == 3, "Queue depth is 3");

    // Trigger Emergency Stop
    planner.emergencyStop();
    TEST_ASSERT(planner.isEstopActive(), "ESTOP is active");
    TEST_ASSERT(!planner.isMoving(), "Motion stopped by ESTOP");
    TEST_ASSERT(planner.getStatus().queue_depth == 0, "Queue cleared on ESTOP");

    // Attempting to queue moves during ESTOP must be rejected
    std::array<float, MAX_AXES> targets4{40.0f, 0.0f, 0.0f, 0.0f};
    TEST_ASSERT(!planner.moveTo(targets4, 50.0f), "Move command rejected during ESTOP");

    // Reset ESTOP
    planner.resetEstop();
    TEST_ASSERT(!planner.isEstopActive(), "ESTOP cleared after reset");
    TEST_ASSERT(planner.moveTo(targets4, 50.0f), "Move command accepted after reset");
    TEST_ASSERT(planner.getStatus().queue_depth == 1, "Queue depth is 1");

    return true;
}

// 3. Multi-axis synchronized coordinated moves
bool test_planner_multiaxis_synchronized_moves() {
    std::cout << "[TEST] Multi-axis synchronized coordinated move (X + Y)...\n";

    MockHAL hal;
    MotionPlanner planner;
    const uint32_t timer_freq_hz = 100000; // 100 kHz
    planner.init(&hal.driver(), &hal.endstop(), &hal.timer(), timer_freq_hz);

    // Axis 0 (Lead): 80 steps/mm, move 20mm -> 1600 steps
    // Axis 1 (Slave): 80 steps/mm, move 10mm -> 800 steps (exact 2:1 ratio)
    AxisConfig cfg0{};
    cfg0.axis_id = 0;
    cfg0.steps_per_mm = 80.0f;
    cfg0.max_speed_mm_s = 100.0f;
    cfg0.max_accel_mm_s2 = 500.0f;
    cfg0.max_jerk_mm_s3 = 2000.0f;

    AxisConfig cfg1{};
    cfg1.axis_id = 1;
    cfg1.steps_per_mm = 80.0f;
    cfg1.max_speed_mm_s = 100.0f;
    cfg1.max_accel_mm_s2 = 500.0f;
    cfg1.max_jerk_mm_s3 = 2000.0f;

    planner.configureAxis(cfg0);
    planner.configureAxis(cfg1);

    std::array<float, MAX_AXES> targets{20.0f, 10.0f, 0.0f, 0.0f};
    TEST_ASSERT(planner.moveTo(targets, 40.0f, ProfileType::SCURVE), "Queue synchronized move");

    // Dequeue and start move via planner update
    planner.update(0);
    TEST_ASSERT(planner.isMoving(), "Planner started executing move");

    // Advance simulation time until move completes
    bool move_finished = false;
    planner.setOnMotionCompleteCallback([&move_finished]() {
        move_finished = true;
    });

    // Step time forward in chunks (e.g. 10ms per tick)
    const uint64_t step_us = 10000; // 10ms
    uint64_t elapsed_us = 0;
    const uint64_t timeout_us = 3000000; // 3 seconds timeout

    while (planner.isMoving() && elapsed_us < timeout_us) {
        hal.advanceTime(step_us);
        elapsed_us += step_us;
        planner.update(elapsed_us);
    }

    TEST_ASSERT(!planner.isMoving(), "Move completed within timeout");
    TEST_ASSERT(move_finished, "OnMotionComplete callback fired");

    // Verify step pulse counts in MockDriverHAL
    const uint32_t pulses_0 = hal.driver().getTotalPulses(0);
    const uint32_t pulses_1 = hal.driver().getTotalPulses(1);

    const uint32_t expected_pulses_0 = static_cast<uint32_t>(std::round(20.0f * 80.0f)); // 1600
    const uint32_t expected_pulses_1 = static_cast<uint32_t>(std::round(10.0f * 80.0f)); // 800

    TEST_ASSERT(pulses_0 == expected_pulses_0, "Axis 0 generated exact expected steps (1600)");
    TEST_ASSERT(pulses_1 == expected_pulses_1, "Axis 1 generated exact expected steps (800)");

    // Final logical positions
    TEST_ASSERT(std::abs(planner.getPosition(0) - 20.0f) < 1e-3f, "Axis 0 final pos is 20.0mm");
    TEST_ASSERT(std::abs(planner.getPosition(1) - 10.0f) < 1e-3f, "Axis 1 final pos is 10.0mm");

    return true;
}

// 4. Velocity scaling & feedrate override
bool test_planner_velocity_scaling() {
    std::cout << "[TEST] Velocity scaling and profile duration...\n";

    MockHAL hal;
    MotionPlanner planner;
    planner.init(&hal.driver(), &hal.endstop(), &hal.timer(), 100000);

    AxisConfig cfg{};
    cfg.axis_id = 0;
    cfg.steps_per_mm = 80.0f;
    cfg.max_speed_mm_s = 100.0f; // Max is 100 mm/s
    cfg.max_accel_mm_s2 = 400.0f;
    cfg.max_jerk_mm_s3 = 1000.0f;
    planner.configureAxis(cfg);

    // Fast move: feedrate 80 mm/s
    std::array<float, MAX_AXES> t_fast{40.0f, 0.0f, 0.0f, 0.0f};
    planner.moveTo(t_fast, 80.0f, ProfileType::TRAPEZOIDAL);
    planner.update(0);

    uint64_t elapsed_fast_us = 0;
    while (planner.isMoving() && elapsed_fast_us < 3000000) {
        hal.advanceTime(5000);
        elapsed_fast_us += 5000;
        planner.update(elapsed_fast_us);
    }
    TEST_ASSERT(!planner.isMoving(), "Fast move finished");

    // Slow move: feedrate 20 mm/s for same distance (40mm to 80mm)
    std::array<float, MAX_AXES> t_slow{80.0f, 0.0f, 0.0f, 0.0f};
    planner.moveTo(t_slow, 20.0f, ProfileType::TRAPEZOIDAL);
    planner.update(elapsed_fast_us);

    uint64_t elapsed_slow_us = 0;
    while (planner.isMoving() && elapsed_slow_us < 6000000) {
        hal.advanceTime(5000);
        elapsed_slow_us += 5000;
        planner.update(elapsed_fast_us + elapsed_slow_us);
    }
    TEST_ASSERT(!planner.isMoving(), "Slow move finished");

    // The slow move at 20mm/s should take noticeably longer than at 80mm/s
    TEST_ASSERT(elapsed_slow_us > elapsed_fast_us * 1.8f, "Slow move duration scales inversely with feedrate");

    return true;
}

// 5. 3-Axis coordinated 3D Scanner path (Turntable + Elevation + Focus Z)
bool test_planner_3axis_scanner_path() {
    std::cout << "[TEST] 3-Axis Coordinated 3D Scanner Move (Turntable, Elevation, Focus)...\n";

    MockHAL hal;
    MotionPlanner planner;
    planner.init(&hal.driver(), &hal.endstop(), &hal.timer(), 100000);

    // Configure 3 axes
    for (uint8_t i = 0; i < 3; ++i) {
        AxisConfig cfg{};
        cfg.axis_id = i;
        cfg.steps_per_mm = 100.0f;
        cfg.max_speed_mm_s = 50.0f;
        cfg.max_accel_mm_s2 = 200.0f;
        cfg.max_jerk_mm_s3 = 1000.0f;
        planner.configureAxis(cfg);
    }

    // Move: Turntable=45 deg, Elevation=30 deg, Focus=2.5 mm
    std::array<float, MAX_AXES> scan_pose{45.0f, 30.0f, 2.5f, 0.0f};
    TEST_ASSERT(planner.moveTo(scan_pose, 30.0f, ProfileType::SCURVE), "Queue 3-axis scan pose");

    planner.update(0);
    TEST_ASSERT(planner.isMoving(), "3-axis move started");

    uint64_t elapsed_us = 0;
    while (planner.isMoving() && elapsed_us < 5000000) {
        hal.advanceTime(10000);
        elapsed_us += 10000;
        planner.update(elapsed_us);
    }

    TEST_ASSERT(!planner.isMoving(), "3-axis scan move completed");

    // Verify all 3 axes reached exact target step counts
    TEST_ASSERT(hal.driver().getTotalPulses(0) == 4500, "Turntable reached 4500 steps (45 * 100)");
    TEST_ASSERT(hal.driver().getTotalPulses(1) == 3000, "Elevation reached 3000 steps (30 * 100)");
    TEST_ASSERT(hal.driver().getTotalPulses(2) == 250, "Focus Z reached 250 steps (2.5 * 100)");

    // Telemetry status check
    auto status = planner.getStatus();
    TEST_ASSERT(std::abs(status.current_pos[0] - 45.0f) < 1e-3f, "Telemetry pos A is 45.0");
    TEST_ASSERT(std::abs(status.current_pos[1] - 30.0f) < 1e-3f, "Telemetry pos B is 30.0");
    TEST_ASSERT(std::abs(status.current_pos[2] - 2.5f) < 1e-3f, "Telemetry pos Z is 2.5");
    TEST_ASSERT(!status.is_moving, "Telemetry moving flag is false");

    return true;
}

bool run_all_planner_tests() {
    std::cout << "========================================\n";
    std::cout << " RUNNING MOTION PLANNER UNIT TESTS      \n";
    std::cout << "========================================\n";

    if (!test_planner_ring_buffer()) return false;
    if (!test_planner_queue_depth_and_estop_rejection()) return false;
    if (!test_planner_multiaxis_synchronized_moves()) return false;
    if (!test_planner_velocity_scaling()) return false;
    if (!test_planner_3axis_scanner_path()) return false;

    std::cout << "========================================\n";
    std::cout << " ALL MOTION PLANNER TESTS PASSED!       \n";
    std::cout << "========================================\n";
    return true;
}

#ifndef TEST_COMBINED_RUNNER
int main() {
    return run_all_planner_tests() ? 0 : 1;
}
#endif
