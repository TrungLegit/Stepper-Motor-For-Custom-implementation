#pragma once

#include "protocol/Json.hpp"
#include <cstdint>
#include <string>
#include <vector>
#include <utility>

namespace motion_core::protocol {

struct Vector3d {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct QuaternionWXYZ {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct CameraExtrinsics {
    QuaternionWXYZ quaternion_wxyz;  // [w, x, y, z] rotation from world to camera
    Vector3d       translation_xyz_m; // [tx, ty, tz] translation vector
    Vector3d       camera_center_m;   // [cx, cy, cz] camera center in world space
};

/**
 * @brief Structured pose record for 3D photogrammetry, structured light scanning,
 *        MeshLab visualization, and reconstruction pipelines (COLMAP, RealityCapture).
 */
struct ScanPoseRecord {
    uint32_t frame_id{0};
    std::string image_file;
    std::string timestamp_iso;

    // Kinematic angles and stage position
    double azimuth_deg{0.0};   // theta (0-360)
    double elevation_deg{0.0}; // phi (0-90)
    double focus_z_mm{0.0};    // linear focus/distance in mm

    // Motor telemetry
    std::vector<std::pair<std::string, int32_t>> motor_steps_abs;
    std::vector<std::pair<std::string, double>>  motor_pos_calibrated;

    // 6-DoF Camera Extrinsics
    CameraExtrinsics extrinsics;

    /**
     * @brief Serialize to standard JSON schema (ARCHITECTURE.md Section 6).
     */
    JsonValue toJson() const;
    std::string toJsonString(bool pretty = false) const;

    /**
     * @brief Format single image line for COLMAP `images.txt`:
     *        IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
     */
    std::string toColmapImageLine(uint32_t camera_id = 1) const;

    /**
     * @brief Format single CSV record for RealityCapture `rc_poses.csv`:
     *        #name, x, y, z, heading, pitch, roll
     */
    std::string toRealityCaptureCsvLine() const;

    /**
     * @brief Format single PLY vertex line for MeshLab point cloud:
     *        x y z nx ny nz red green blue
     */
    std::string toMeshLabPlyVertex(uint8_t red = 0, uint8_t green = 255, uint8_t blue = 128) const;
};

/**
 * @brief Utility to solve spherical kinematics to 6-DoF camera extrinsics.
 */
class SphericalExtrinsicCalculator {
public:
    /**
     * @brief Compute camera center and world-to-camera rotation quaternion
     *        assuming the camera is aimed directly at the origin (0, 0, 0).
     * @param azimuth_deg Turntable azimuth angle in degrees.
     * @param elevation_deg Gimbal elevation angle in degrees above equator.
     * @param radius_m Distance from origin to camera in meters.
     * @param focus_z_m Additional Z stage offset in meters.
     */
    static CameraExtrinsics calculate(double azimuth_deg,
                                     double elevation_deg,
                                     double radius_m = 0.20,
                                     double focus_z_m = 0.0);
};

/**
 * @brief Generates complete MeshLab compatible ASCII .ply file
 *        containing camera positions and viewing directions.
 */
class MeshLabPlyBuilder {
public:
    static std::string buildCameraCloudPly(const std::vector<ScanPoseRecord>& poses);
};

} // namespace motion_core::protocol
