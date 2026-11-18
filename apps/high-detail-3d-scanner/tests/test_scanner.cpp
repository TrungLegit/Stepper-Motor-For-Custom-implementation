#include "ScanPathGenerator.hpp"
#include "CaptureEngine.hpp"
#include "PoseExporter.hpp"
#include "ScannerKinematics.hpp"
#include "hal_impl/mock/MockHAL.hpp"
#include "planner/MotionPlanner.hpp"

#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <string>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <chrono>

using namespace scanner;
using namespace motion_core;
using namespace motion_core::mock;
using namespace motion_core::protocol;

constexpr double PI = 3.14159265358979323846;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "  FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

static void setupPlannerAxes(MotionPlanner& planner, ScannerKinematics& kin) {
    // Axis 0: Azimuth Turntable (Rotary, 0 to 360 deg)
    AxisConfig az_cfg;
    az_cfg.axis_id = 0;
    az_cfg.name = "A";
    az_cfg.steps_per_mm = kin.getAzimuthStepsPerDegree();
    az_cfg.max_speed_mm_s = 60.0f;
    az_cfg.max_accel_mm_s2 = 300.0f;
    az_cfg.max_jerk_mm_s3 = 1500.0f;
    az_cfg.backlash_mm = 0.05f;
    az_cfg.travel_min_mm = -360.0f;
    az_cfg.travel_max_mm = 360.0f;
    az_cfg.is_rotary = true;
    planner.configureAxis(az_cfg);

    // Axis 1: Elevation Tilt Gimbal (Arc, 0 to 90 deg)
    AxisConfig el_cfg;
    el_cfg.axis_id = 1;
    el_cfg.name = "B";
    el_cfg.steps_per_mm = kin.getElevationStepsPerDegree();
    el_cfg.max_speed_mm_s = 45.0f;
    el_cfg.max_accel_mm_s2 = 200.0f;
    el_cfg.max_jerk_mm_s3 = 1000.0f;
    el_cfg.backlash_mm = 0.03f;
    el_cfg.travel_min_mm = 0.0f;
    el_cfg.travel_max_mm = 90.0f;
    el_cfg.is_rotary = true;
    planner.configureAxis(el_cfg);

    // Axis 2: Focus Z Rail (Linear, +/- 25 mm)
    AxisConfig z_cfg;
    z_cfg.axis_id = 2;
    z_cfg.name = "Z";
    z_cfg.steps_per_mm = kin.getFocusStepsPerMm();
    z_cfg.max_speed_mm_s = 20.0f;
    z_cfg.max_accel_mm_s2 = 250.0f;
    z_cfg.max_jerk_mm_s3 = 1200.0f;
    z_cfg.backlash_mm = 0.02f;
    z_cfg.travel_min_mm = -25.0f;
    z_cfg.travel_max_mm = 25.0f;
    z_cfg.is_rotary = false;
    planner.configureAxis(z_cfg);
}

// ---------------------------------------------------------------------------
// Test 1: Fibonacci Sphere Pose Generator
// ---------------------------------------------------------------------------
static bool test_fibonacci_sphere() {
    std::cout << "[Test 1] Fibonacci Sphere Pose Generator...\n";

    // 1. Check pose count and boundary angles for 56 viewpoints
    constexpr uint32_t N56 = 56;
    constexpr double el_min = 5.0;
    constexpr double el_max = 85.0;
    constexpr double focus_z = 2.5;

    auto path56 = ScanPathGenerator::fibonacci(N56, el_min, el_max, focus_z);
    TEST_ASSERT(path56.size() == N56, "Fibonacci pose count must match requested N=56");

    for (size_t i = 0; i < path56.size(); ++i) {
        const auto& p = path56[i];
        TEST_ASSERT(p.elevation_deg >= el_min - 1e-4, "Elevation must be >= min elevation");
        TEST_ASSERT(p.elevation_deg <= el_max + 1e-4, "Elevation must be <= max elevation");
        TEST_ASSERT(p.azimuth_deg >= 0.0 && p.azimuth_deg <= 360.0, "Azimuth must be in [0, 360] deg");
        TEST_ASSERT(std::abs(p.focus_z_mm - focus_z) < 1e-6, "Focus Z must match requested value");

        if (i > 0) {
            TEST_ASSERT(p.elevation_deg >= path56[i - 1].elevation_deg - 1e-6,
                        "Raw Fibonacci poses should be sorted by elevation");
        }
    }

    // 2. Check uniform solid angle distribution on unit hemisphere
    // Ensure no two poses are identical or excessively clustered
    double min_pairwise_dist = 1e9;
    for (size_t i = 0; i < path56.size(); ++i) {
        double phi_i = path56[i].elevation_deg * (PI / 180.0);
        double theta_i = path56[i].azimuth_deg * (PI / 180.0);
        double xi = std::cos(phi_i) * std::cos(theta_i);
        double yi = std::cos(phi_i) * std::sin(theta_i);
        double zi = std::sin(phi_i);

        for (size_t j = i + 1; j < path56.size(); ++j) {
            double phi_j = path56[j].elevation_deg * (PI / 180.0);
            double theta_j = path56[j].azimuth_deg * (PI / 180.0);
            double xj = std::cos(phi_j) * std::cos(theta_j);
            double yj = std::cos(phi_j) * std::sin(theta_j);
            double zj = std::sin(phi_j);

            double d = std::sqrt((xi - xj) * (xi - xj) +
                                 (yi - yj) * (yi - yj) +
                                 (zi - zj) * (zi - zj));
            if (d < min_pairwise_dist) {
                min_pairwise_dist = d;
            }
        }
    }
    // For 56 points on upper hemisphere, minimum distance should be well-separated (> 0.08)
    TEST_ASSERT(min_pairwise_dist > 0.08, "Fibonacci points must have uniform solid angle spacing without clustering");

    // 3. Test different lattice sizes (16, 120)
    auto path16 = ScanPathGenerator::fibonacci(16, 10.0, 80.0, 0.0);
    TEST_ASSERT(path16.size() == 16, "Fibonacci pose count N=16");

    auto path120 = ScanPathGenerator::fibonacci(120, 5.0, 85.0, 0.0);
    TEST_ASSERT(path120.size() == 120, "Fibonacci pose count N=120");

    // 4. Test Path Optimizer (Traveling Salesman / Serpentine reordering)
    auto optimized56 = path56;
    ScanPathGenerator::optimizePath(optimized56);
    TEST_ASSERT(optimized56.size() == path56.size(), "Optimized path must preserve pose count");

    // All original points must be preserved in the optimized set
    for (const auto& orig : path56) {
        bool found = false;
        for (const auto& opt : optimized56) {
            if (std::abs(orig.azimuth_deg - opt.azimuth_deg) < 1e-4 &&
                std::abs(orig.elevation_deg - opt.elevation_deg) < 1e-4) {
                found = true;
                break;
            }
        }
        TEST_ASSERT(found, "Optimized path must contain all original poses");
    }

    std::cout << "  [PASS] Fibonacci lattice (N=56, N=120, N=16), angle bounds, solid angle spacing, optimizer\n";
    return true;
}

// ---------------------------------------------------------------------------
// Test 2: Uniform Grid & CSV Serialization
// ---------------------------------------------------------------------------
static bool test_uniform_grid_and_csv() {
    std::cout << "[Test 2] Uniform Grid & CSV Serialization...\n";

    // 1. Generate regular grid (18 azimuth x 6 elevation x 2 focus layers = 216 poses)
    auto grid = ScanPathGenerator::uniformGrid(18, 6, 10.0, 70.0, {0.0, 5.0}, true);
    TEST_ASSERT(grid.size() == 18 * 6 * 2, "Uniform grid size must equal az_steps * el_steps * layers");

    // Check serpentine azimuth reversal
    // Tier 0 (elevation min): azimuth increases
    // Tier 1: azimuth decreases
    TEST_ASSERT(grid[0].azimuth_deg < grid[1].azimuth_deg, "Tier 0 should scan forward");
    TEST_ASSERT(grid[18].azimuth_deg > grid[19].azimuth_deg, "Tier 1 should scan in reverse (serpentine)");

    // 2. CSV Export & Import roundtrip
    std::vector<SphericalPose> test_poses = {
        {0.0, 15.0, 0.0},
        {45.5, 30.2, 1.25},
        {180.0, 60.0, -2.5},
        {359.9, 85.0, 0.0}
    };

    std::string csv = ScanPathGenerator::toCsv(test_poses);
    TEST_ASSERT(!csv.empty(), "CSV export must not be empty");

    auto loaded = ScanPathGenerator::fromCsv(csv);
    TEST_ASSERT(loaded.size() == test_poses.size(), "CSV import must match export size");

    for (size_t i = 0; i < test_poses.size(); ++i) {
        TEST_ASSERT(std::abs(loaded[i].azimuth_deg - test_poses[i].azimuth_deg) < 1e-4, "Azimuth roundtrip");
        TEST_ASSERT(std::abs(loaded[i].elevation_deg - test_poses[i].elevation_deg) < 1e-4, "Elevation roundtrip");
        TEST_ASSERT(std::abs(loaded[i].focus_z_mm - test_poses[i].focus_z_mm) < 1e-4, "Focus Z roundtrip");
    }

    // 3. Robust CSV parsing with headers and comments
    std::string messy_csv =
        "# Scanner Waypoints Trajectory\n"
        "azimuth,elevation,focus_z\n"
        "  10.0 , 20.0 , 0.5 \n"
        "# Mid-scan pause\n"
        "\n"
        "90.0, 45.0, -1.0\n";

    auto parsed_messy = ScanPathGenerator::fromCsv(messy_csv);
    TEST_ASSERT(parsed_messy.size() == 2, "Should parse exactly 2 valid data rows ignoring comments/headers");
    TEST_ASSERT(std::abs(parsed_messy[0].azimuth_deg - 10.0) < 1e-4, "Messy parse row 0 azimuth");
    TEST_ASSERT(std::abs(parsed_messy[1].elevation_deg - 45.0) < 1e-4, "Messy parse row 1 elevation");

    std::cout << "  [PASS] Uniform grid generator, serpentine routing, CSV serialization roundtrip\n";
    return true;
}

// ---------------------------------------------------------------------------
// Test 3: CaptureEngine State Machine Simulation with MockHAL (56 Poses)
// ---------------------------------------------------------------------------
static bool test_capture_engine_full_scan() {
    std::cout << "[Test 3] CaptureEngine 56-Pose Automated Scan Simulation (MockHAL)...\n";

    MockHAL hal;
    ScannerKinematics::Config k_cfg;
    k_cfg.motor_full_steps_per_rev = 200.0f;
    k_cfg.microsteps = 16.0f;
    k_cfg.azimuth_gear_ratio = 4.0f;
    k_cfg.elevation_gear_ratio = 4.0f;
    k_cfg.focus_lead_screw_pitch_mm = 8.0f;
    k_cfg.default_scan_radius_mm = 150.0f;
    ScannerKinematics kin(k_cfg);

    MotionPlanner planner;
    planner.init(&hal.driver(), &hal.endstop(), &hal.timer(), 100000);
    setupPlannerAxes(planner, kin);

    constexpr uint8_t SHUTTER_PIN = 18;
    CaptureConfig cap_cfg;
    cap_cfg.settle_delay_ms = 50;         // 50 ms settling delay
    cap_cfg.shutter_pulse_us = 20000;     // 20 ms shutter pulse
    cap_cfg.ack_timeout_ms = 1000;
    cap_cfg.shutter_gpio_pin = SHUTTER_PIN;
    cap_cfg.auto_ack = true;
    cap_cfg.travel_feedrate_mm_s = 60.0f;

    CaptureEngine engine(planner, kin);
    engine.init(&hal.gpio(), &hal.timer());
    engine.setConfig(cap_cfg);

    // Track state progression and shutter pin pulses
    uint32_t captured_count = 0;
    bool scan_finished_called = false;
    bool scan_finished_success = false;

    engine.setOnPoseCaptured([&](const ScanPoseRecord& /*rec*/) {
        captured_count++;
    });

    engine.setOnScanFinished([&](bool success) {
        scan_finished_called = true;
        scan_finished_success = success;
    });

    // Generate 56-pose Fibonacci scan path
    auto path = ScanPathGenerator::fibonacci(56, 10.0, 80.0, 0.0);
    ScanPathGenerator::optimizePath(path);

    TEST_ASSERT(engine.startScan(path), "CaptureEngine startScan should return true");
    TEST_ASSERT(engine.isScanning(), "Engine should be in scanning state");
    TEST_ASSERT(engine.getState() == CaptureState::MOVE_TO_POSE, "Initial state should be MOVE_TO_POSE");

    // Advance simulation loop
    uint64_t sim_time_us = 0;
    constexpr uint64_t dt_us = 500; // 0.5ms step
    constexpr uint64_t max_sim_us = 300000000ULL; // 5 minute max virtual time

    uint32_t shutter_high_detections = 0;

    while (engine.isScanning() && sim_time_us < max_sim_us) {
        hal.advanceTime(dt_us);
        sim_time_us += dt_us;

        planner.update(sim_time_us);
        engine.update(sim_time_us);

        // Check if shutter GPIO is asserted HIGH
        if (hal.gpio().digitalRead(SHUTTER_PIN) == PinState::HIGH) {
            shutter_high_detections++;
        }
    }

    TEST_ASSERT(sim_time_us < max_sim_us, "Scan simulation completed without timeout or deadlock");
    TEST_ASSERT(engine.getState() == CaptureState::SCAN_COMPLETE, "Final state must be SCAN_COMPLETE");
    TEST_ASSERT(scan_finished_called, "Scan finished callback must be triggered");
    TEST_ASSERT(scan_finished_success, "Scan finished must report success");
    TEST_ASSERT(captured_count == 56, "Exactly 56 viewpoints must be captured");
    TEST_ASSERT(engine.getLoggedPoses().size() == 56, "56 pose records logged in engine");
    TEST_ASSERT(shutter_high_detections > 0, "Shutter GPIO pin must have pulsed HIGH during scan");

    // Verify logged pose records data fidelity
    const auto& logged = engine.getLoggedPoses();
    for (size_t i = 0; i < logged.size(); ++i) {
        TEST_ASSERT(logged[i].frame_id == static_cast<uint32_t>(i), "Frame ID sequentially indexed");
        TEST_ASSERT(std::abs(logged[i].azimuth_deg - path[i].azimuth_deg) < 1e-4, "Logged azimuth matches path");
        TEST_ASSERT(std::abs(logged[i].elevation_deg - path[i].elevation_deg) < 1e-4, "Logged elevation matches path");

        // Verify camera extrinsics are populated
        const auto& ext = logged[i].extrinsics;
        double q_norm = ext.quaternion_wxyz.w * ext.quaternion_wxyz.w +
                        ext.quaternion_wxyz.x * ext.quaternion_wxyz.x +
                        ext.quaternion_wxyz.y * ext.quaternion_wxyz.y +
                        ext.quaternion_wxyz.z * ext.quaternion_wxyz.z;
        TEST_ASSERT(std::abs(q_norm - 1.0) < 0.005, "Quaternion in logged pose must be normalized");
    }

    std::cout << "  [PASS] Full 56-pose scan completed successfully without deadlock in "
              << (sim_time_us / 1000000.0) << " simulated seconds\n";
    return true;
}

// ---------------------------------------------------------------------------
// Test 4: Pause, Resume, and Abort Mechanisms
// ---------------------------------------------------------------------------
static bool test_capture_engine_pause_resume_abort() {
    std::cout << "[Test 4] CaptureEngine Pause, Resume, and Abort...\n";

    MockHAL hal;
    ScannerKinematics kin;
    MotionPlanner planner;
    planner.init(&hal.driver(), &hal.endstop(), &hal.timer(), 100000);
    setupPlannerAxes(planner, kin);

    CaptureConfig cap_cfg;
    cap_cfg.settle_delay_ms = 20;
    cap_cfg.shutter_pulse_us = 10000;
    cap_cfg.ack_timeout_ms = 500;
    cap_cfg.shutter_gpio_pin = 18;
    cap_cfg.auto_ack = true;

    CaptureEngine engine(planner, kin);
    engine.init(&hal.gpio(), &hal.timer());
    engine.setConfig(cap_cfg);

    auto path = ScanPathGenerator::fibonacci(8, 10.0, 80.0, 0.0);

    // 1. Pause & Resume Test
    engine.startScan(path);

    uint64_t sim_time_us = 0;
    constexpr uint64_t dt_us = 500;

    // Run until 2 poses captured
    while (engine.isScanning() && engine.getLoggedPoses().size() < 2) {
        hal.advanceTime(dt_us);
        sim_time_us += dt_us;
        planner.update(sim_time_us);
        engine.update(sim_time_us);
    }

    TEST_ASSERT(engine.getLoggedPoses().size() == 2, "2 poses logged before pause");

    // Request pause
    engine.pause();

    // Advance time until state reaches PAUSED
    while (engine.isScanning() && engine.getState() != CaptureState::PAUSED) {
        hal.advanceTime(dt_us);
        sim_time_us += dt_us;
        planner.update(sim_time_us);
        engine.update(sim_time_us);
    }

    TEST_ASSERT(engine.getState() == CaptureState::PAUSED, "Engine state must be PAUSED");

    // Advance time during pause -> no further poses should be logged
    for (int k = 0; k < 100; ++k) {
        hal.advanceTime(dt_us);
        sim_time_us += dt_us;
        planner.update(sim_time_us);
        engine.update(sim_time_us);
    }
    TEST_ASSERT(engine.getLoggedPoses().size() == 2, "No new poses logged while paused");

    // Resume scan
    engine.resume();
    TEST_ASSERT(engine.getState() == CaptureState::MOVE_TO_POSE, "Resuming transitions back to MOVE_TO_POSE");

    while (engine.isScanning()) {
        hal.advanceTime(dt_us);
        sim_time_us += dt_us;
        planner.update(sim_time_us);
        engine.update(sim_time_us);
    }

    TEST_ASSERT(engine.getState() == CaptureState::SCAN_COMPLETE, "Resumed scan finishes all poses");
    TEST_ASSERT(engine.getLoggedPoses().size() == 8, "All 8 poses logged after resume");

    // 2. Abort Test
    engine.startScan(path);
    TEST_ASSERT(engine.isScanning(), "Second scan started");

    for (int k = 0; k < 50; ++k) {
        hal.advanceTime(dt_us);
        sim_time_us += dt_us;
        planner.update(sim_time_us);
        engine.update(sim_time_us);
    }

    engine.abort();
    TEST_ASSERT(engine.getState() == CaptureState::ABORTED, "Abort immediately sets ABORTED");
    TEST_ASSERT(!engine.isScanning(), "Engine is no longer scanning after abort");
    TEST_ASSERT(hal.gpio().digitalRead(cap_cfg.shutter_gpio_pin) == PinState::LOW, "Shutter GPIO pulled LOW on abort");

    std::cout << "  [PASS] Pause holds at viewpoint boundary, resume completes all poses, abort halts immediately\n";
    return true;
}

// ---------------------------------------------------------------------------
// Test 5: Software Trigger Hook & Retries
// ---------------------------------------------------------------------------
static bool test_capture_engine_software_ack_and_retries() {
    std::cout << "[Test 5] Software Trigger Hook & Camera ACK Retries...\n";

    MockHAL hal;
    ScannerKinematics kin;
    MotionPlanner planner;
    planner.init(&hal.driver(), &hal.endstop(), &hal.timer(), 100000);
    setupPlannerAxes(planner, kin);

    CaptureConfig cap_cfg;
    cap_cfg.settle_delay_ms = 20;
    cap_cfg.shutter_pulse_us = 10000;
    cap_cfg.ack_timeout_ms = 200;
    cap_cfg.max_retries = 2;
    cap_cfg.auto_ack = false; // Require software hook to ACK

    CaptureEngine engine(planner, kin);
    engine.init(&hal.gpio(), &hal.timer());
    engine.setConfig(cap_cfg);

    auto path = ScanPathGenerator::fibonacci(3, 10.0, 80.0, 0.0);

    // 1. Success with retry recovery on frame 1
    int trigger_calls = 0;
    int retry_injected = 0;

    engine.setSoftwareTriggerHook([&](uint32_t frame_id, const SphericalPose& /*pose*/, std::function<void(bool)> ack_fn) {
        trigger_calls++;
        if (frame_id == 1 && retry_injected == 0) {
            // Simulate capture failure on first attempt of frame 1
            retry_injected++;
            ack_fn(false);
        } else {
            // Successful capture
            ack_fn(true);
        }
    });

    engine.startScan(path);

    uint64_t sim_time_us = 0;
    constexpr uint64_t dt_us = 500;

    while (engine.isScanning() && sim_time_us < 100000000ULL) {
        hal.advanceTime(dt_us);
        sim_time_us += dt_us;
        planner.update(sim_time_us);
        engine.update(sim_time_us);
    }

    TEST_ASSERT(engine.getState() == CaptureState::SCAN_COMPLETE, "Scan completed despite single failure retry");
    TEST_ASSERT(engine.getLoggedPoses().size() == 3, "All 3 poses captured after retry");
    TEST_ASSERT(trigger_calls == 4, "Total trigger calls should be 4 (frame 0, frame 1 attempt 1, frame 1 attempt 2, frame 2)");

    // 2. Failure when max retries exceeded
    CaptureEngine engine_fail(planner, kin);
    engine_fail.init(&hal.gpio(), &hal.timer());
    engine_fail.setConfig(cap_cfg);

    engine_fail.setSoftwareTriggerHook([](uint32_t /*frame_id*/, const SphericalPose& /*pose*/, std::function<void(bool)> ack_fn) {
        // Always fail
        ack_fn(false);
    });

    engine_fail.startScan(path);

    while (engine_fail.isScanning() && sim_time_us < 200000000ULL) {
        hal.advanceTime(dt_us);
        sim_time_us += dt_us;
        planner.update(sim_time_us);
        engine_fail.update(sim_time_us);
    }

    TEST_ASSERT(engine_fail.getState() == CaptureState::FAILED, "Engine transitions to FAILED on max retries exceeded");

    std::cout << "  [PASS] Software trigger hook dispatches ACK, retries on failure, transitions to FAILED on persistent fault\n";
    return true;
}

// ---------------------------------------------------------------------------
// Test 6: PoseExporter MeshLab PLY Format Verification
// ---------------------------------------------------------------------------
static bool test_pose_exporter_meshlab_ply() {
    std::cout << "[Test 6] PoseExporter MeshLab PLY Format Verification...\n";

    ScannerKinematics kin;
    auto path = ScanPathGenerator::fibonacci(56, 10.0, 80.0, 0.0);

    std::vector<ScanPoseRecord> records;
    records.reserve(path.size());
    for (uint32_t i = 0; i < path.size(); ++i) {
        records.push_back(kin.createPoseRecord(i, "frame_" + std::to_string(i) + ".jpg", path[i]));
    }

    PoseExporter exporter(records);
    TEST_ASSERT(exporter.size() == 56, "Exporter holds 56 records");

    // 1. Point Cloud PLY (include_frustums = false)
    std::string ply_cloud = exporter.toMeshLabPly(false);
    TEST_ASSERT(ply_cloud.rfind("ply\nformat ascii 1.0", 0) == 0, "PLY header format ascii 1.0");
    TEST_ASSERT(ply_cloud.find("element vertex 56\n") != std::string::npos, "Header element vertex 56");
    TEST_ASSERT(ply_cloud.find("property float x\n") != std::string::npos, "Property float x");
    TEST_ASSERT(ply_cloud.find("property float nx\n") != std::string::npos, "Property float nx");
    TEST_ASSERT(ply_cloud.find("property uchar red\n") != std::string::npos, "Property uchar red");
    TEST_ASSERT(ply_cloud.find("end_header\n") != std::string::npos, "Header end_header");

    // Parse and verify vertex normals have unit length: nx*nx + ny*ny + nz*nz = 1.0
    std::istringstream iss(ply_cloud);
    std::string line;
    bool in_data = false;
    uint32_t vertex_count = 0;

    while (std::getline(iss, line)) {
        if (!in_data) {
            if (line == "end_header") {
                in_data = true;
            }
            continue;
        }

        if (line.empty()) continue;

        std::istringstream line_stream(line);
        double x, y, z, nx, ny, nz;
        int r, g, b;
        if (line_stream >> x >> y >> z >> nx >> ny >> nz >> r >> g >> b) {
            vertex_count++;
            double n_len = std::sqrt(nx * nx + ny * ny + nz * nz);
            TEST_ASSERT(std::abs(n_len - 1.0) < 0.01, "Normal vector must have unit length");

            // Camera center distance from origin should match default radius (150 mm = 0.15 m)
            double c_dist = std::sqrt(x * x + y * y + z * z);
            TEST_ASSERT(std::abs(c_dist - 0.15) < 0.01, "Camera center distance from origin ~ 0.15m");
        }
    }
    TEST_ASSERT(vertex_count == 56, "PLY file contains exactly 56 vertex rows");

    // 2. Frustum Pyramid PLY (include_frustums = true)
    std::string ply_frustums = exporter.toMeshLabPly(true);
    TEST_ASSERT(ply_frustums.find("element vertex 280\n") != std::string::npos, "Frustum PLY has 56 * 5 = 280 vertices");
    TEST_ASSERT(ply_frustums.find("element face 280\n") != std::string::npos, "Frustum PLY has 56 * 5 = 280 faces");
    TEST_ASSERT(ply_frustums.find("property list uchar int vertex_indices\n") != std::string::npos, "Face property list");

    std::cout << "  [PASS] MeshLab ASCII .ply generated with valid headers, 56 vertices, unit normal vectors, and 3D frustums\n";
    return true;
}

// ---------------------------------------------------------------------------
// Test 7: PoseExporter COLMAP, RealityCapture, JSON & Disk Export
// ---------------------------------------------------------------------------
static bool test_pose_exporter_colmap_and_rc() {
    std::cout << "[Test 7] PoseExporter COLMAP, RealityCapture, JSON & Disk Export...\n";

    ScannerKinematics kin;
    auto path = ScanPathGenerator::fibonacci(20, 15.0, 75.0, 0.0);

    std::vector<ScanPoseRecord> records;
    for (uint32_t i = 0; i < path.size(); ++i) {
        records.push_back(kin.createPoseRecord(i, "frame_" + std::to_string(i) + ".jpg", path[i]));
    }

    PoseExporter exporter(records);

    // 1. COLMAP images.txt
    std::string images_txt = exporter.toColmapImagesTxt(1);
    TEST_ASSERT(images_txt.find("# Image list with two lines of data per image:") != std::string::npos, "COLMAP header comment");

    std::istringstream img_iss(images_txt);
    std::string line;
    uint32_t colmap_img_count = 0;

    while (std::getline(img_iss, line)) {
        if (line.empty() || line[0] == '#') continue;

        // Image line format: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
        std::istringstream ls(line);
        uint32_t img_id, cam_id;
        double qw, qx, qy, qz, tx, ty, tz;
        std::string name;

        if (ls >> img_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> cam_id >> name) {
            colmap_img_count++;
            double q_norm = qw * qw + qx * qx + qy * qy + qz * qz;
            TEST_ASSERT(std::abs(q_norm - 1.0) < 0.005, "COLMAP quaternion must have unit norm");
            TEST_ASSERT(cam_id == 1, "Camera ID matches parameter");

            // Empty line follows points2D line in COLMAP format
            std::string pts_line;
            std::getline(img_iss, pts_line);
        }
    }
    TEST_ASSERT(colmap_img_count == 20, "COLMAP images.txt contains 20 image records");

    // 2. COLMAP cameras.txt
    std::string cameras_txt = exporter.toColmapCamerasTxt(3840, 2160, 3000.0, 1);
    TEST_ASSERT(cameras_txt.find("1 PINHOLE 3840 2160 3000.0000 3000.0000 1920.0000 1080.0000") != std::string::npos,
                "COLMAP cameras.txt PINHOLE model parameters");

    // 3. RealityCapture CSV
    std::string rc_csv = exporter.toRealityCaptureCsv();
    TEST_ASSERT(rc_csv.find("#name, x, y, z, heading, pitch, roll\n") != std::string::npos, "RealityCapture CSV header");

    std::istringstream rc_iss(rc_csv);
    uint32_t rc_count = 0;
    while (std::getline(rc_iss, line)) {
        if (!line.empty() && line[0] != '#') {
            rc_count++;
        }
    }
    TEST_ASSERT(rc_count == 20, "RealityCapture CSV contains 20 pose rows");

    // 4. Native JSON
    std::string json_str = exporter.toJson(true);
    TEST_ASSERT(!json_str.empty(), "JSON output non-empty");
    TEST_ASSERT(json_str.find("\"frame_id\": 0") != std::string::npos, "JSON contains frame_id 0");
    TEST_ASSERT(json_str.find("\"quaternion_wxyz\"") != std::string::npos, "JSON contains quaternion");

    // 5. Full Disk Export via exportAll()
    const std::string test_dir = "./test_export_artifacts";
    std::filesystem::remove_all(test_dir);

    bool export_ok = exporter.exportAll(test_dir);
    TEST_ASSERT(export_ok, "exportAll returns true");

    TEST_ASSERT(std::filesystem::exists(test_dir + "/poses.ply"), "poses.ply generated on disk");
    TEST_ASSERT(std::filesystem::file_size(test_dir + "/poses.ply") > 0, "poses.ply non-empty");

    TEST_ASSERT(std::filesystem::exists(test_dir + "/colmap/images.txt"), "colmap/images.txt generated");
    TEST_ASSERT(std::filesystem::file_size(test_dir + "/colmap/images.txt") > 0, "images.txt non-empty");

    TEST_ASSERT(std::filesystem::exists(test_dir + "/colmap/cameras.txt"), "colmap/cameras.txt generated");
    TEST_ASSERT(std::filesystem::file_size(test_dir + "/colmap/cameras.txt") > 0, "cameras.txt non-empty");

    TEST_ASSERT(std::filesystem::exists(test_dir + "/rc_poses.csv"), "rc_poses.csv generated");
    TEST_ASSERT(std::filesystem::file_size(test_dir + "/rc_poses.csv") > 0, "rc_poses.csv non-empty");

    TEST_ASSERT(std::filesystem::exists(test_dir + "/poses.json"), "poses.json generated");
    TEST_ASSERT(std::filesystem::file_size(test_dir + "/poses.json") > 0, "poses.json non-empty");

    // Clean up temporary files
    std::filesystem::remove_all(test_dir);

    std::cout << "  [PASS] COLMAP images/cameras.txt, RealityCapture CSV, JSON, and exportAll() validated\n";
    return true;
}

// ---------------------------------------------------------------------------
// Consolidated Test Runner
// ---------------------------------------------------------------------------
int main() {
    std::cout << "========================================================\n";
    std::cout << "  High-Detail 3D Scanner Application Test Suite         \n";
    std::cout << "  Kinematics: 3-Axis Spherical Rig (Azimuth/Elevation/Z)\n";
    std::cout << "========================================================\n\n";

    int passed = 0;
    int failed = 0;

    auto run_test = [&](const char* name, bool (*fn)()) {
        auto t0 = std::chrono::high_resolution_clock::now();
        bool ok = fn();
        auto t1 = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (ok) {
            std::cout << ">>> " << name << " SUCCEEDED (" << elapsed_ms << " ms)\n\n";
            passed++;
        } else {
            std::cout << ">>> " << name << " FAILED (" << elapsed_ms << " ms)\n\n";
            failed++;
        }
    };

    run_test("test_fibonacci_sphere", test_fibonacci_sphere);
    run_test("test_uniform_grid_and_csv", test_uniform_grid_and_csv);
    run_test("test_capture_engine_full_scan", test_capture_engine_full_scan);
    run_test("test_capture_engine_pause_resume_abort", test_capture_engine_pause_resume_abort);
    run_test("test_capture_engine_software_ack_and_retries", test_capture_engine_software_ack_and_retries);
    run_test("test_pose_exporter_meshlab_ply", test_pose_exporter_meshlab_ply);
    run_test("test_pose_exporter_colmap_and_rc", test_pose_exporter_colmap_and_rc);

    std::cout << "========================================================\n";
    std::cout << "  Scanner Test Summary: " << passed << " passed, " << failed << " failed\n";
    std::cout << "========================================================\n";

    return (failed == 0) ? 0 : 1;
}
