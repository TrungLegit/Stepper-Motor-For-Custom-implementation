#include "planner/SCurveProfile.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>
#include <vector>
#include <numeric>

using namespace motion_core;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

// 1. Nominal 7-phase move verification (Long move with full cruise)
bool test_scurve_nominal_7phase() {
    std::cout << "[TEST] S-Curve nominal 7-phase profile (full cruise)...\n";

    SCurveProfile profile;
    const float distance = 100.0f; // 100 mm
    const float v_start = 0.0f;
    const float v_target = 0.0f;
    const float v_max = 50.0f;      // 50 mm/s
    const float a_max = 200.0f;     // 200 mm/s^2
    const float j_max = 1000.0f;    // 1000 mm/s^3

    profile.calculate(distance, v_start, v_target, v_max, a_max, j_max);

    TEST_ASSERT(profile.getDuration() > 0.0f, "Duration should be positive");
    TEST_ASSERT(std::abs(profile.getDistance() - distance) < 1e-4f, "Profile distance matches requested");
    TEST_ASSERT(profile.getPeakVelocity() <= v_max + 1e-4f, "Peak velocity respects v_max");
    TEST_ASSERT(std::abs(profile.getPeakVelocity() - v_max) < 0.5f, "Nominal move reaches v_max");
    TEST_ASSERT(profile.getPeakAcceleration() <= a_max + 1e-4f, "Peak acceleration respects a_max");

    const auto& phases = profile.getPhases();
    // Phase 4 (cruise) should have non-zero duration
    TEST_ASSERT(phases[3].duration > 0.0f, "Cruise phase (P4) duration must be positive");
    // Initial and final velocities
    TEST_ASSERT(std::abs(profile.getVelocity(0.0f) - v_start) < 1e-4f, "Start velocity is 0");
    TEST_ASSERT(std::abs(profile.getVelocity(profile.getDuration()) - v_target) < 1e-2f, "Exit velocity is 0");
    // Final position
    TEST_ASSERT(std::abs(profile.getPosition(profile.getDuration()) - distance) < 1e-2f, "Final position matches distance");

    return true;
}

// 2. Short move (Triangular / Cruise collapse: Phase 4 duration = 0)
bool test_scurve_short_move_cruise_collapse() {
    std::cout << "[TEST] S-Curve short move (cruise collapse, Phase 4 = 0)...\n";

    SCurveProfile profile;
    const float distance = 2.0f; // short move
    const float v_start = 0.0f;
    const float v_target = 0.0f;
    const float v_max = 100.0f;   // High v_max that cannot be reached
    const float a_max = 200.0f;
    const float j_max = 1000.0f;

    profile.calculate(distance, v_start, v_target, v_max, a_max, j_max);

    TEST_ASSERT(profile.getDuration() > 0.0f, "Duration must be positive");
    TEST_ASSERT(profile.getPeakVelocity() < v_max, "Peak velocity should be less than unreachable v_max");
    TEST_ASSERT(profile.getPeakVelocity() > 0.0f, "Peak velocity should be > 0");

    const auto& phases = profile.getPhases();
    TEST_ASSERT(phases[3].duration < 1e-5f, "Cruise phase (P4) should collapse to zero");
    TEST_ASSERT(std::abs(profile.getPosition(profile.getDuration()) - distance) < 1e-2f, "Target distance reached");

    return true;
}

// 3. Zero-distance move and boundary conditions
bool test_scurve_zero_and_micro_distance() {
    std::cout << "[TEST] S-Curve boundary conditions (zero & micro-distance)...\n";

    SCurveProfile profile;

    // Zero distance
    profile.calculate(0.0f, 0.0f, 0.0f, 50.0f, 200.0f, 1000.0f);
    TEST_ASSERT(profile.getDuration() == 0.0f, "Zero distance move has zero duration");
    TEST_ASSERT(profile.getPosition(0.0f) == 0.0f, "Position at zero distance is 0");
    TEST_ASSERT(profile.getVelocity(0.0f) == 0.0f, "Velocity at zero distance is 0");
    TEST_ASSERT(profile.getAcceleration(0.0f) == 0.0f, "Acceleration at zero distance is 0");

    // Negative distance input clamped to zero safely
    profile.calculate(-5.0f, 0.0f, 0.0f, 50.0f, 200.0f, 1000.0f);
    TEST_ASSERT(profile.getDuration() == 0.0f, "Negative distance clamped to zero");

    // Micro-distance (0.001 mm)
    const float micro_dist = 0.001f;
    profile.calculate(micro_dist, 0.0f, 0.0f, 50.0f, 200.0f, 1000.0f);
    TEST_ASSERT(profile.getDuration() > 0.0f, "Micro move has valid duration");
    TEST_ASSERT(std::abs(profile.getPosition(profile.getDuration()) - micro_dist) < 1e-4f,
                "Micro move reaches commanded distance");

    return true;
}

// 4. Jerk continuity and C0/C1 smoothness verification
bool test_scurve_jerk_and_accel_continuity() {
    std::cout << "[TEST] S-Curve jerk continuity & smoothness sampling...\n";

    SCurveProfile profile;
    const float distance = 50.0f;
    const float v_start = 0.0f;
    const float v_target = 0.0f;
    const float v_max = 40.0f;
    const float a_max = 150.0f;
    const float j_max = 800.0f;

    profile.calculate(distance, v_start, v_target, v_max, a_max, j_max);

    const float total_t = profile.getDuration();
    const int num_samples = 1000;
    const float dt = total_t / static_cast<float>(num_samples);

    float prev_pos = 0.0f;
    float prev_vel = profile.getVelocity(0.0f);
    float prev_acc = profile.getAcceleration(0.0f);

    for (int i = 1; i <= num_samples; ++i) {
        const float t = i * dt;
        const float pos = profile.getPosition(t);
        const float vel = profile.getVelocity(t);
        const float acc = profile.getAcceleration(t);
        const float jerk = profile.getJerk(t);

        // 1. Monotonic position (since direction is positive)
        TEST_ASSERT(pos >= prev_pos - 1e-5f, "Position must be monotonically non-decreasing");

        // 2. Velocity bounds
        TEST_ASSERT(vel >= -1e-4f && vel <= v_max + 1e-4f, "Velocity within [0, v_max]");

        // 3. Acceleration bounds
        TEST_ASSERT(acc >= -a_max - 1e-2f && acc <= a_max + 1e-2f, "Acceleration within [-a_max, a_max]");

        // 4. Jerk bounds
        TEST_ASSERT(std::abs(jerk) <= j_max + 1e-2f, "Jerk within [-j_max, j_max]");

        // 5. C0 continuity of acceleration (no sudden infinite spikes; delta acc <= j_max * dt * tolerance)
        const float d_acc = std::abs(acc - prev_acc);
        const float max_expected_d_acc = j_max * dt * 2.5f; // small tolerance for phase boundary transitions
        TEST_ASSERT(d_acc <= max_expected_d_acc, "Acceleration exhibits C0 continuity without jumps");

        // 6. C1 continuity of velocity
        const float d_vel = std::abs(vel - prev_vel);
        const float max_expected_d_vel = a_max * dt * 2.0f;
        TEST_ASSERT(d_vel <= max_expected_d_vel, "Velocity exhibits C1 continuity without jumps");

        prev_pos = pos;
        prev_vel = vel;
        prev_acc = acc;
    }

    return true;
}

// 5. Non-zero start & exit velocity transitions
bool test_scurve_nonzero_boundary_velocities() {
    std::cout << "[TEST] S-Curve non-zero boundary velocity transitions...\n";

    SCurveProfile profile;
    const float distance = 40.0f;
    const float v_start = 10.0f;  // Starting with initial speed
    const float v_target = 15.0f; // Exiting with non-zero speed
    const float v_max = 35.0f;
    const float a_max = 120.0f;
    const float j_max = 600.0f;

    profile.calculate(distance, v_start, v_target, v_max, a_max, j_max);

    TEST_ASSERT(std::abs(profile.getVelocity(0.0f) - v_start) < 1e-3f, "Start velocity matches v_start");
    TEST_ASSERT(std::abs(profile.getVelocity(profile.getDuration()) - v_target) < 1e-2f, "Exit velocity matches v_target");
    TEST_ASSERT(std::abs(profile.getPosition(profile.getDuration()) - distance) < 1e-2f, "Final position matches distance");

    return true;
}

// 6. Step table generation for StepGenerator ISR consumption
bool test_scurve_step_table_generation() {
    std::cout << "[TEST] S-Curve step interval table generation...\n";

    SCurveProfile profile;
    const float distance = 10.0f;       // 10 mm
    const float steps_per_mm = 80.0f;   // 800 total steps
    const uint32_t timer_freq_hz = 100000; // 100 kHz timer ISR

    profile.calculate(distance, 0.0f, 0.0f, 40.0f, 150.0f, 800.0f);

    std::vector<StepInterval> table;
    profile.generateStepTable(steps_per_mm, timer_freq_hz, table, 128);

    TEST_ASSERT(!table.empty(), "Generated step table is not empty");

    uint32_t sum_steps = 0;
    for (const auto& interval : table) {
        TEST_ASSERT(interval.step_count > 0, "Step count must be positive");
        TEST_ASSERT(interval.isr_ticks_per_step > 0, "ISR ticks per step must be positive");
        sum_steps += interval.step_count;
    }

    const uint32_t expected_steps = static_cast<uint32_t>(std::round(distance * steps_per_mm));
    TEST_ASSERT(sum_steps == expected_steps, "Total step count in table equals round(distance * steps_per_mm)");

    return true;
}

bool run_all_scurve_tests() {
    std::cout << "========================================\n";
    std::cout << " RUNNING S-CURVE PROFILE UNIT TESTS     \n";
    std::cout << "========================================\n";

    if (!test_scurve_nominal_7phase()) return false;
    if (!test_scurve_short_move_cruise_collapse()) return false;
    if (!test_scurve_zero_and_micro_distance()) return false;
    if (!test_scurve_jerk_and_accel_continuity()) return false;
    if (!test_scurve_nonzero_boundary_velocities()) return false;
    if (!test_scurve_step_table_generation()) return false;

    std::cout << "========================================\n";
    std::cout << " ALL S-CURVE PROFILE TESTS PASSED!      \n";
    std::cout << "========================================\n";
    return true;
}

#ifndef TEST_COMBINED_RUNNER
int main() {
    return run_all_scurve_tests() ? 0 : 1;
}
#endif
