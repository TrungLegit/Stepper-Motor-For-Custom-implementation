#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>

// Declarations of individual test suite runners
bool run_all_protocol_tests();
bool run_all_scurve_tests();
bool run_all_planner_tests();
bool run_all_backlash_tests();
bool run_all_homing_tests();

struct TestSuiteResult {
    std::string name;
    bool passed;
    double duration_ms;
};

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::cout << "\n";
    std::cout << "#################################################################\n";
    std::cout << "#      STEPPER-MOTION-CORE AUTOMATED TEST SUITE RUNNER         #\n";
    std::cout << "#       Target: Host PC (Simulation with MockHAL)               #\n";
    std::cout << "#################################################################\n\n";

    std::vector<TestSuiteResult> results;
    auto global_start = std::chrono::high_resolution_clock::now();

    auto run_suite = [&results](const std::string& name, bool (*fn)()) {
        std::cout << "\n>>> Starting Suite: " << name << " ...\n";
        auto t0 = std::chrono::high_resolution_clock::now();
        bool ok = false;
        try {
            ok = fn();
        } catch (const std::exception& e) {
            std::cerr << "EXCEPTION in suite " << name << ": " << e.what() << "\n";
            ok = false;
        } catch (...) {
            std::cerr << "UNKNOWN EXCEPTION in suite " << name << "\n";
            ok = false;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        results.push_back({name, ok, dt_ms});
        if (ok) {
            std::cout << ">>> [PASSED] " << name << " (" << std::fixed << std::setprecision(2) << dt_ms << " ms)\n";
        } else {
            std::cout << ">>> [FAILED] " << name << " (" << std::fixed << std::setprecision(2) << dt_ms << " ms)\n";
        }
    };

    // 1. Protocol & Safety Suite
    run_suite("Protocol & Safety (JSON-RPC 2.0, E-STOP, Telemetry, MeshLab 3D)", run_all_protocol_tests);

    // 2. 7-Phase S-Curve Profile Suite
    run_suite("S-Curve Velocity Profile (7-Phase, Jerk Continuity, Short Moves)", run_all_scurve_tests);

    // 3. Motion Planner Suite
    run_suite("Motion Planner (Multi-Axis Coordinated Moves, Queue, Scaling)", run_all_planner_tests);

    // 4. Backlash Compensator Suite
    run_suite("Backlash Compensator (Direction Change Pre-Moves)", run_all_backlash_tests);

    // 5. Homing Engine Suite
    run_suite("Homing Engine (NO/NC, Software Debounce, Hall Effect, StallGuard4)", run_all_homing_tests);

    auto global_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(global_end - global_start).count();

    // Summary Table
    std::cout << "\n";
    std::cout << "=================================================================\n";
    std::cout << "                     VERIFICATION SUMMARY REPORT                 \n";
    std::cout << "=================================================================\n";
    std::cout << std::left << std::setw(65) << "Test Suite"
              << std::setw(12) << "Result"
              << std::setw(12) << "Time (ms)" << "\n";
    std::cout << std::string(89, '-') << "\n";

    size_t passed_count = 0;
    for (const auto& r : results) {
        std::cout << std::left << std::setw(65) << r.name
                  << std::setw(12) << (r.passed ? "[PASS]" : "[FAIL]")
                  << std::fixed << std::setprecision(2) << r.duration_ms << "\n";
        if (r.passed) ++passed_count;
    }

    std::cout << std::string(89, '=') << "\n";
    std::cout << "Total Suites:  " << results.size() << "\n";
    std::cout << "Passed:        " << passed_count << "\n";
    std::cout << "Failed:        " << (results.size() - passed_count) << "\n";
    std::cout << "Total Time:    " << std::fixed << std::setprecision(2) << total_ms << " ms\n";
    std::cout << "Overall Status: " << ((passed_count == results.size()) ? "ALL TESTS PASSED (100% SUCCESS)" : "FAILURES DETECTED") << "\n";
    std::cout << "=================================================================\n\n";

    return (passed_count == results.size()) ? 0 : 1;
}
