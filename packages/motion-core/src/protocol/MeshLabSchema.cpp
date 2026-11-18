#include "protocol/MeshLabSchema.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>

namespace motion_core::protocol {

namespace {
constexpr double PI = 3.14159265358979323846;

void normalizeVector(Vector3d& v) {
    double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len > 1e-12) {
        v.x /= len;
        v.y /= len;
        v.z /= len;
    }
}

Vector3d crossProduct(const Vector3d& a, const Vector3d& b) {
    return Vector3d{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}
} // namespace

CameraExtrinsics SphericalExtrinsicCalculator::calculate(
    double azimuth_deg,
    double elevation_deg,
    double radius_m,
    double focus_z_m) {

    double theta = azimuth_deg * (PI / 180.0);
    double phi = elevation_deg * (PI / 180.0);
    double r = radius_m + focus_z_m;
    if (r <= 0.001) r = 0.001;

    // Camera center in world coordinates
    Vector3d C;
    C.x = r * std::cos(phi) * std::cos(theta);
    C.y = r * std::cos(phi) * std::sin(theta);
    C.z = r * std::sin(phi);

    // Forward direction (Camera +Z pointing towards target at origin)
    Vector3d Z_cam{ -C.x, -C.y, -C.z };
    normalizeVector(Z_cam);

    // Up vector (world Z if not pointing straight along Z)
    Vector3d world_up{0.0, 0.0, 1.0};
    if (std::abs(Z_cam.z) > 0.99) {
        world_up = Vector3d{0.0, 1.0, 0.0};
    }

    // Camera Right (+X) = world_up x Z_cam
    Vector3d X_cam = crossProduct(world_up, Z_cam);
    normalizeVector(X_cam);

    // Camera Down (+Y) = Z_cam x X_cam (OpenCV / COLMAP camera coordinate convention)
    Vector3d Y_cam = crossProduct(Z_cam, X_cam);
    normalizeVector(Y_cam);

    // Rotation matrix R_cw (world to camera)
    // Row 0: X_cam
    // Row 1: Y_cam
    // Row 2: Z_cam
    double R[3][3] = {
        { X_cam.x, X_cam.y, X_cam.z },
        { Y_cam.x, Y_cam.y, Y_cam.z },
        { Z_cam.x, Z_cam.y, Z_cam.z }
    };

    // Translation T_cw = -R_cw * C
    Vector3d T;
    T.x = -(R[0][0] * C.x + R[0][1] * C.y + R[0][2] * C.z);
    T.y = -(R[1][0] * C.x + R[1][1] * C.y + R[1][2] * C.z);
    T.z = -(R[2][0] * C.x + R[2][1] * C.y + R[2][2] * C.z);

    // Convert R_cw to quaternion (w, x, y, z)
    QuaternionWXYZ q;
    double tr = R[0][0] + R[1][1] + R[2][2];

    if (tr > 0.0) {
        double s = 2.0 * std::sqrt(tr + 1.0);
        q.w = 0.25 * s;
        q.x = (R[2][1] - R[1][2]) / s;
        q.y = (R[0][2] - R[2][0]) / s;
        q.z = (R[1][0] - R[0][1]) / s;
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
        q.w = (R[2][1] - R[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (R[0][1] + R[1][0]) / s;
        q.z = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
        q.w = (R[0][2] - R[2][0]) / s;
        q.x = (R[0][1] + R[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (R[1][2] + R[2][1]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
        q.w = (R[1][0] - R[0][1]) / s;
        q.x = (R[0][2] + R[2][0]) / s;
        q.y = (R[1][2] + R[2][1]) / s;
        q.z = 0.25 * s;
    }

    CameraExtrinsics ext;
    ext.quaternion_wxyz = q;
    ext.translation_xyz_m = T;
    ext.camera_center_m = C;
    return ext;
}

JsonValue ScanPoseRecord::toJson() const {
    JsonValue root = JsonValue::object();
    root.set("frame_id", JsonValue(frame_id));
    root.set("image_file", JsonValue(image_file));
    root.set("timestamp_iso", JsonValue(timestamp_iso));

    // Spherical pose
    JsonValue pose_obj = JsonValue::object();
    pose_obj.set("azimuth_deg", JsonValue(azimuth_deg));
    pose_obj.set("elevation_deg", JsonValue(elevation_deg));
    pose_obj.set("focus_z_mm", JsonValue(focus_z_mm));
    root.set("pose", std::move(pose_obj));

    // Motor steps abs
    JsonValue steps_obj = JsonValue::object();
    for (const auto& [axis, steps] : motor_steps_abs) {
        steps_obj.set(axis, JsonValue(steps));
    }
    root.set("motor_steps_abs", std::move(steps_obj));

    // Motor pos calibrated
    JsonValue calib_obj = JsonValue::object();
    for (const auto& [axis, pos] : motor_pos_calibrated) {
        calib_obj.set(axis, JsonValue(pos));
    }
    root.set("motor_pos_calibrated", std::move(calib_obj));

    // Camera extrinsic
    JsonValue ext_obj = JsonValue::object();
    JsonValue quat_arr = JsonValue::array();
    quat_arr.push_back(JsonValue(extrinsics.quaternion_wxyz.w));
    quat_arr.push_back(JsonValue(extrinsics.quaternion_wxyz.x));
    quat_arr.push_back(JsonValue(extrinsics.quaternion_wxyz.y));
    quat_arr.push_back(JsonValue(extrinsics.quaternion_wxyz.z));
    ext_obj.set("quaternion_wxyz", std::move(quat_arr));

    JsonValue trans_arr = JsonValue::array();
    trans_arr.push_back(JsonValue(extrinsics.translation_xyz_m.x));
    trans_arr.push_back(JsonValue(extrinsics.translation_xyz_m.y));
    trans_arr.push_back(JsonValue(extrinsics.translation_xyz_m.z));
    ext_obj.set("translation_xyz_m", std::move(trans_arr));

    root.set("camera_extrinsic", std::move(ext_obj));

    return root;
}

std::string ScanPoseRecord::toJsonString(bool pretty) const {
    return toJson().serialize(pretty);
}

std::string ScanPoseRecord::toColmapImageLine(uint32_t camera_id) const {
    // Format: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << frame_id << " "
       << extrinsics.quaternion_wxyz.w << " "
       << extrinsics.quaternion_wxyz.x << " "
       << extrinsics.quaternion_wxyz.y << " "
       << extrinsics.quaternion_wxyz.z << " "
       << extrinsics.translation_xyz_m.x << " "
       << extrinsics.translation_xyz_m.y << " "
       << extrinsics.translation_xyz_m.z << " "
       << camera_id << " "
       << (image_file.empty() ? ("frame_" + std::to_string(frame_id) + ".jpg") : image_file);
    return ss.str();
}

std::string ScanPoseRecord::toRealityCaptureCsvLine() const {
    // Format: #name, x, y, z, heading, pitch, roll
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    std::string name = image_file.empty() ? ("frame_" + std::to_string(frame_id) + ".jpg") : image_file;
    ss << name << ", "
       << extrinsics.camera_center_m.x << ", "
       << extrinsics.camera_center_m.y << ", "
       << extrinsics.camera_center_m.z << ", "
       << azimuth_deg << ", "
       << -elevation_deg << ", "
       << 0.0;
    return ss.str();
}

std::string ScanPoseRecord::toMeshLabPlyVertex(uint8_t red, uint8_t green, uint8_t blue) const {
    // PLY vertex: x y z nx ny nz red green blue
    // Direction points from camera center to origin
    Vector3d dir{ -extrinsics.camera_center_m.x, -extrinsics.camera_center_m.y, -extrinsics.camera_center_m.z };
    normalizeVector(dir);

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << extrinsics.camera_center_m.x << " "
       << extrinsics.camera_center_m.y << " "
       << extrinsics.camera_center_m.z << " "
       << dir.x << " " << dir.y << " " << dir.z << " "
       << static_cast<int>(red) << " "
       << static_cast<int>(green) << " "
       << static_cast<int>(blue);
    return ss.str();
}

std::string MeshLabPlyBuilder::buildCameraCloudPly(const std::vector<ScanPoseRecord>& poses) {
    std::ostringstream ss;
    ss << "ply\n";
    ss << "format ascii 1.0\n";
    ss << "comment Generated by stepper-motion-core for MeshLab 3D Reconstruction\n";
    ss << "element vertex " << poses.size() << "\n";
    ss << "property float x\n";
    ss << "property float y\n";
    ss << "property float z\n";
    ss << "property float nx\n";
    ss << "property float ny\n";
    ss << "property float nz\n";
    ss << "property uchar red\n";
    ss << "property uchar green\n";
    ss << "property uchar blue\n";
    ss << "end_header\n";

    for (const auto& pose : poses) {
        ss << pose.toMeshLabPlyVertex() << "\n";
    }

    return ss.str();
}

} // namespace motion_core::protocol
