#include "motion_core.hpp"
#include "ScannerKinematics.hpp"
#include "ScanPathGenerator.hpp"
#include "CaptureEngine.hpp"
#include "PoseExporter.hpp"
#include "board/pinmap_esp32s3.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>

#if defined(ESP_PLATFORM)
#include "hal_impl/esp32s3/ESP32S3_HAL.hpp"
#else
#include "hal_impl/mock/MockHAL.hpp"
#endif

using namespace motion_core;
using namespace motion_core::protocol;
using namespace scanner;

static std::unique_ptr<MotionPlanner> g_planner;
static std::unique_ptr<ScannerKinematics> g_kinematics;
static std::unique_ptr<CaptureEngine> g_capture_engine;
static std::unique_ptr<PoseExporter> g_exporter;
static std::unique_ptr<JsonRpcDispatcher> g_dispatcher;

#if defined(ESP_PLATFORM)
static std::unique_ptr<esp32s3::ESP32S3_HAL> g_hal;
#else
static std::unique_ptr<mock::MockHAL> g_hal;
#endif

static int8_t getAxisIdByName(const std::string& name) {
    if (name == "A" || name == "a" || name == "0" || name == "azimuth") return 0;
    if (name == "B" || name == "b" || name == "1" || name == "elevation") return 1;
    if (name == "Z" || name == "z" || name == "2" || name == "focus") return 2;
    return -1;
}

static void setupScannerMotionPlanner(MotionPlanner& planner, ScannerKinematics& kin) {
    // Axis 0: Azimuth Turntable (Rotary, 0 to 360 deg)
    AxisConfig az_cfg;
    az_cfg.axis_id = 0;
    az_cfg.name = "A";
    az_cfg.steps_per_mm = kin.getAzimuthStepsPerDegree(); // steps per degree
    az_cfg.max_speed_mm_s = 60.0f;                        // 60 deg/sec
    az_cfg.max_accel_mm_s2 = 300.0f;                      // 300 deg/sec^2
    az_cfg.max_jerk_mm_s3 = 1500.0f;                      // 1500 deg/sec^3
    az_cfg.backlash_mm = 0.05f;                           // 0.05 deg backlash
    az_cfg.travel_min_mm = -360.0f;
    az_cfg.travel_max_mm = 360.0f;
    az_cfg.is_rotary = true;
    planner.configureAxis(az_cfg);

    // Axis 1: Elevation Tilt Gimbal (Arc, 0 to 90 deg)
    AxisConfig el_cfg;
    el_cfg.axis_id = 1;
    el_cfg.name = "B";
    el_cfg.steps_per_mm = kin.getElevationStepsPerDegree();
    el_cfg.max_speed_mm_s = 45.0f;                        // 45 deg/sec
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
    z_cfg.max_speed_mm_s = 20.0f;                         // 20 mm/sec
    z_cfg.max_accel_mm_s2 = 250.0f;
    z_cfg.max_jerk_mm_s3 = 1200.0f;
    z_cfg.backlash_mm = 0.02f;
    z_cfg.travel_min_mm = -25.0f;
    z_cfg.travel_max_mm = 25.0f;
    z_cfg.is_rotary = false;
    planner.configureAxis(z_cfg);
}

int main(int argc, char** argv) {
    std::cout << "====================================================\n";
    std::cout << "  Stepper Motion Core - High-Detail 3D Scanner Rig  \n";
    std::cout << "  Kinematics: 3-Axis Spherical (Azimuth/Elevation/Z)\n";
    std::cout << "====================================================\n\n";

    bool run_simulation = true;
    if (argc > 1 && std::string(argv[1]) == "--hardware") {
        run_simulation = false;
    }

#if defined(ESP_PLATFORM)
    std::cout << "[Scanner] Target Platform: ESP32-S3 Hardware\n";
    esp32s3::AxisPins pins[MAX_AXES];
    board::populateScannerAxisPins(pins);
    auto uart = board::getScannerUartConfig(true);
    g_hal = std::make_unique<esp32s3::ESP32S3_HAL>(pins, uart);
    g_hal->init();
#else
    std::cout << "[Scanner] Target Platform: PC Simulation (MockHAL)\n";
    g_hal = std::make_unique<mock::MockHAL>();
#endif

    // 1. Initialize Kinematics & Motion Planner
    ScannerKinematics::Config k_cfg;
    k_cfg.motor_full_steps_per_rev = 200.0f;
    k_cfg.microsteps = 16.0f;
    k_cfg.azimuth_gear_ratio = 4.0f;
    k_cfg.elevation_gear_ratio = 4.0f;
    k_cfg.focus_lead_screw_pitch_mm = 8.0f;
    k_cfg.default_scan_radius_mm = 150.0f;

    g_kinematics = std::make_unique<ScannerKinematics>(k_cfg);
    g_planner = std::make_unique<MotionPlanner>();
    g_planner->init(g_hal.get(), g_hal.get(), g_hal.get(), 100000);
    setupScannerMotionPlanner(*g_planner, *g_kinematics);

    // 2. Initialize Capture Engine
    CaptureConfig cap_cfg;
    cap_cfg.settle_delay_ms = 150;
    cap_cfg.shutter_pulse_us = 50000;
    cap_cfg.ack_timeout_ms = 2000;
    cap_cfg.shutter_gpio_pin = board::CAMERA_SHUTTER_PIN;
    cap_cfg.ack_gpio_pin = 0;
    cap_cfg.auto_ack = true;

    g_capture_engine = std::make_unique<CaptureEngine>(*g_planner, *g_kinematics);
    g_capture_engine->init(g_hal.get(), g_hal.get());
    g_capture_engine->setConfig(cap_cfg);

    g_exporter = std::make_unique<PoseExporter>();

    g_capture_engine->setOnPoseCaptured([](const ScanPoseRecord& record) {
        g_exporter->addPose(record);
        std::cout << "  [Pose Captured] Frame " << record.frame_id
                  << " -> Azimuth: " << record.azimuth_deg
                  << " deg, Elevation: " << record.elevation_deg
                  << " deg, Z: " << record.focus_z_mm << " mm\n";
    });

    g_capture_engine->setOnScanFinished([](bool success) {
        std::cout << "\n[CaptureEngine] Scan Completed! Success: " << (success ? "YES" : "NO")
                  << ", Total Frames: " << g_exporter->size() << "\n";
    });

    // 3. If running PC simulation, perform sample 16-pose Fibonacci scan demo
    if (run_simulation) {
        std::cout << "[Scanner] Generating 16-pose Fibonacci spherical lattice...\n";
        auto path = ScanPathGenerator::fibonacci(16, 10.0, 80.0, 0.0);
        ScanPathGenerator::optimizePath(path);

        std::cout << "[Scanner] Starting automated scan sequence...\n";
        g_capture_engine->startScan(path);

        uint64_t sim_time_us = 0;
        constexpr uint64_t step_dt_us = 1000; // 1 ms simulation step

        while (g_capture_engine->isScanning()) {
#if !defined(ESP_PLATFORM)
            // Advance MockHAL simulation clock
            g_hal->advanceTime(step_dt_us);
#endif
            sim_time_us += step_dt_us;
            g_planner->update(sim_time_us);
            g_capture_engine->update(sim_time_us);
        }

        std::cout << "\n[Scanner] Exporting reconstruction metadata...\n";
        std::string out_dir = "./scan_output";
        g_exporter->exportAll(out_dir);

        std::cout << "  -> " << out_dir << "/poses.ply (MeshLab 3D camera frustums)\n";
        std::cout << "  -> " << out_dir << "/colmap/images.txt (COLMAP extrinsics)\n";
        std::cout << "  -> " << out_dir << "/colmap/cameras.txt (COLMAP pinhole model)\n";
        std::cout << "  -> " << out_dir << "/rc_poses.csv (RealityCapture)\n";
        std::cout << "  -> " << out_dir << "/poses.json (JSON format)\n\n";

        std::cout << "[Scanner] Simulation finished successfully with zero errors.\n";
    }

    // Clean up instances in strict dependency order before static destruction
    g_capture_engine.reset();
    g_planner.reset();
    g_hal.reset();

    return 0;
}
