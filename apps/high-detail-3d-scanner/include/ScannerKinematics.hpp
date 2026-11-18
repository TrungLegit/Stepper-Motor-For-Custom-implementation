#pragma once

#include "motion_core.hpp"
#include <cmath>
#include <cstdint>
#include <string>

namespace scanner {

using namespace motion_core;
using namespace motion_core::protocol;

/**
 * @brief Spherical coordinate pose for 3D photogrammetry and scanning.
 */
struct SphericalPose {
    double azimuth_deg{0.0};   // theta: 0 to 360 deg turntable rotation
    double elevation_deg{0.0}; // phi: 0 to 90 deg gimbal tilt
    double focus_z_mm{0.0};    // z: linear focus-stack or distance adjustment
};

/**
 * @brief Kinematic configuration and forward/inverse transforms for 3D scanning rig.
 *        Implements the multi-axis spherical kinematic model defined in ARCHITECTURE.md.
 */
class ScannerKinematics {
public:
    struct Config {
        float motor_full_steps_per_rev{200.0f};  // 1.8 deg NEMA 17
        float microsteps{16.0f};                // TMC2209 native microstep setting
        float azimuth_gear_ratio{4.0f};         // 4:1 pulley/gearbox reduction
        float elevation_gear_ratio{4.0f};       // 4:1 reduction
        float focus_lead_screw_pitch_mm{8.0f};  // T8 8mm pitch per rev
        float default_scan_radius_mm{150.0f};   // Fixed distance from center of turntable
    };

    ScannerKinematics() = default;
    explicit ScannerKinematics(Config config) : m_cfg(config) {}

    /**
     * @brief Steps per degree for Azimuth axis.
     */
    float getAzimuthStepsPerDegree() const noexcept {
        return (m_cfg.motor_full_steps_per_rev * m_cfg.microsteps * m_cfg.azimuth_gear_ratio) / 360.0f;
    }

    /**
     * @brief Steps per degree for Elevation axis.
     */
    float getElevationStepsPerDegree() const noexcept {
        return (m_cfg.motor_full_steps_per_rev * m_cfg.microsteps * m_cfg.elevation_gear_ratio) / 360.0f;
    }

    /**
     * @brief Steps per millimeter for Focus Z axis.
     */
    float getFocusStepsPerMm() const noexcept {
        return (m_cfg.motor_full_steps_per_rev * m_cfg.microsteps) / m_cfg.focus_lead_screw_pitch_mm;
    }

    /**
     * @brief Inverse Kinematics: Convert spherical pose to discrete motor steps.
     */
    void sphericalToSteps(const SphericalPose& pose,
                          int32_t& steps_azimuth,
                          int32_t& steps_elevation,
                          int32_t& steps_focus_z) const noexcept {
        steps_azimuth = static_cast<int32_t>(std::round(pose.azimuth_deg * getAzimuthStepsPerDegree()));
        steps_elevation = static_cast<int32_t>(std::round(pose.elevation_deg * getElevationStepsPerDegree()));
        steps_focus_z = static_cast<int32_t>(std::round(pose.focus_z_mm * getFocusStepsPerMm()));
    }

    /**
     * @brief Forward Kinematics: Convert motor steps to spherical pose.
     */
    SphericalPose stepsToSpherical(int32_t steps_azimuth,
                                   int32_t steps_elevation,
                                   int32_t steps_focus_z) const noexcept {
        SphericalPose pose;
        pose.azimuth_deg = steps_azimuth / static_cast<double>(getAzimuthStepsPerDegree());
        pose.elevation_deg = steps_elevation / static_cast<double>(getElevationStepsPerDegree());
        pose.focus_z_mm = steps_focus_z / static_cast<double>(getFocusStepsPerMm());
        return pose;
    }

    /**
     * @brief Generate populated ScanPoseRecord for MeshLab / COLMAP export.
     */
    ScanPoseRecord createPoseRecord(uint32_t frame_id,
                                    const std::string& image_file,
                                    const SphericalPose& pose,
                                    const std::string& timestamp_iso = "") const {
        ScanPoseRecord rec;
        rec.frame_id = frame_id;
        rec.image_file = image_file;
        rec.timestamp_iso = timestamp_iso;
        rec.azimuth_deg = pose.azimuth_deg;
        rec.elevation_deg = pose.elevation_deg;
        rec.focus_z_mm = pose.focus_z_mm;

        int32_t step_a = 0, step_b = 0, step_z = 0;
        sphericalToSteps(pose, step_a, step_b, step_z);
        rec.motor_steps_abs = {{"A", step_a}, {"B", step_b}, {"Z", step_z}};
        rec.motor_pos_calibrated = {{"A_deg", pose.azimuth_deg}, {"B_deg", pose.elevation_deg}, {"Z_mm", pose.focus_z_mm}};

        // Solve 6-DoF camera extrinsics
        double radius_m = (m_cfg.default_scan_radius_mm + pose.focus_z_mm) / 1000.0;
        rec.extrinsics = SphericalExtrinsicCalculator::calculate(pose.azimuth_deg, pose.elevation_deg, radius_m);

        return rec;
    }

    const Config& getConfig() const noexcept { return m_cfg; }

private:
    Config m_cfg;
};

} // namespace scanner
