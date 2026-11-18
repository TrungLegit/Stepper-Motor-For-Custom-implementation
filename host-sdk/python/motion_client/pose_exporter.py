"""
3D Scanning Pose Exporter for MeshLab, COLMAP, and RealityCapture.
Transforms logged camera telemetry records into photogrammetry reconstruction metadata.
"""

import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class Vector3d:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class QuaternionWXYZ:
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class CameraExtrinsics:
    quaternion_wxyz: QuaternionWXYZ
    translation_xyz_m: Vector3d
    camera_center_m: Vector3d


@dataclass
class ScanPose:
    frame_id: int
    image_file: str
    timestamp_iso: str
    azimuth_deg: float
    elevation_deg: float
    focus_z_mm: float
    motor_steps_abs: Dict[str, int] = field(default_factory=dict)
    motor_pos_calibrated: Dict[str, float] = field(default_factory=dict)
    extrinsics: Optional[CameraExtrinsics] = None


def solve_spherical_extrinsics(azimuth_deg: float, elevation_deg: float, radius_m: float = 0.20, focus_z_m: float = 0.0) -> CameraExtrinsics:
    """
    Compute 6-DoF camera extrinsics assuming camera is mounted on a spherical gimbal
    at distance radius_m pointing directly toward target object at origin (0, 0, 0).
    """
    theta = math.radians(azimuth_deg)
    phi = math.radians(elevation_deg)
    r = max(0.001, radius_m + focus_z_m)

    # Camera center in world coordinates
    cx = r * math.cos(phi) * math.cos(theta)
    cy = r * math.cos(phi) * math.sin(theta)
    cz = r * math.sin(phi)
    cam_center = Vector3d(cx, cy, cz)

    # Optical axis (Camera +Z pointing towards target origin from camera)
    zx = -cx
    zy = -cy
    zz = -cz
    z_len = math.sqrt(zx * zx + zy * zy + zz * zz)
    if z_len > 1e-12:
        zx /= z_len
        zy /= z_len
        zz /= z_len

    # World up
    up_x, up_y, up_z = 0.0, 0.0, 1.0
    if abs(zz) > 0.99:
        up_x, up_y, up_z = 0.0, 1.0, 0.0

    # Camera right (+X) = up x Z_cam
    xx = up_y * zz - up_z * zy
    xy = up_z * zx - up_x * zz
    xz = up_x * zy - up_y * zx
    x_len = math.sqrt(xx * xx + xy * xy + xz * xz)
    if x_len > 1e-12:
        xx /= x_len
        xy /= x_len
        xz /= x_len

    # Camera down (+Y) = Z_cam x X_cam
    yx = zy * xz - zz * xy
    yy = zz * xx - zx * xz
    yz = zx * xy - zy * xx

    # World-to-camera rotation matrix R_cw
    # Rows: X_cam, Y_cam, Z_cam
    r00, r01, r02 = xx, xy, xz
    r10, r11, r12 = yx, yy, yz
    r20, r21, r22 = zx, zy, zz

    # Translation T_cw = -R_cw * C
    tx = -(r00 * cx + r01 * cy + r02 * cz)
    ty = -(r10 * cx + r11 * cy + r12 * cz)
    tz = -(r20 * cx + r21 * cy + r22 * cz)
    translation = Vector3d(tx, ty, tz)

    # Matrix to quaternion
    tr = r00 + r11 + r22
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif (r00 > r11) and (r00 > r22):
        s = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s

    return CameraExtrinsics(
        quaternion_wxyz=QuaternionWXYZ(qw, qx, qy, qz),
        translation_xyz_m=translation,
        camera_center_m=cam_center
    )


class PoseExporter:
    """
    Collects scan poses and exports to MeshLab .ply, COLMAP, and RealityCapture formats.
    """

    def __init__(self, default_radius_m: float = 0.20):
        self.default_radius_m = default_radius_m
        self.poses: List[ScanPose] = []

    def add_pose(self, pose_data: dict) -> ScanPose:
        """Add a pose record from scan telemetry or manual specification."""
        frame_id = pose_data.get("frame_id", len(self.poses) + 1)
        image_file = pose_data.get("image_file", f"frame_{frame_id:04d}.jpg")
        timestamp_iso = pose_data.get("timestamp_iso", "")

        pose_info = pose_data.get("pose", {})
        azimuth_deg = float(pose_info.get("azimuth_deg", pose_info.get("azimuth", 0.0)))
        elevation_deg = float(pose_info.get("elevation_deg", pose_info.get("elevation", 0.0)))
        focus_z_mm = float(pose_info.get("focus_z_mm", pose_info.get("focus_z", 0.0)))

        motor_steps_abs = pose_data.get("motor_steps_abs", {})
        motor_pos_calib = pose_data.get("motor_pos_calibrated", {})

        ext = solve_spherical_extrinsics(azimuth_deg, elevation_deg, self.default_radius_m, focus_z_mm / 1000.0)

        record = ScanPose(
            frame_id=frame_id,
            image_file=image_file,
            timestamp_iso=timestamp_iso,
            azimuth_deg=azimuth_deg,
            elevation_deg=elevation_deg,
            focus_z_mm=focus_z_mm,
            motor_steps_abs=motor_steps_abs,
            motor_pos_calibrated=motor_pos_calib,
            extrinsics=ext
        )
        self.poses.append(record)
        return record

    def export_json(self, file_path: str) -> None:
        """Export all poses as formatted native JSON metadata."""
        out = []
        for p in self.poses:
            ext = p.extrinsics
            out.append({
                "frame_id": p.frame_id,
                "image_file": p.image_file,
                "timestamp_iso": p.timestamp_iso,
                "pose": {
                    "azimuth_deg": p.azimuth_deg,
                    "elevation_deg": p.elevation_deg,
                    "focus_z_mm": p.focus_z_mm
                },
                "motor_steps_abs": p.motor_steps_abs,
                "motor_pos_calibrated": p.motor_pos_calibrated,
                "camera_extrinsic": {
                    "quaternion_wxyz": [ext.quaternion_wxyz.w, ext.quaternion_wxyz.x, ext.quaternion_wxyz.y, ext.quaternion_wxyz.z] if ext else [1, 0, 0, 0],
                    "translation_xyz_m": [ext.translation_xyz_m.x, ext.translation_xyz_m.y, ext.translation_xyz_m.z] if ext else [0, 0, 0]
                }
            })
        os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2)

    def export_meshlab_ply(self, file_path: str) -> None:
        """
        Export MeshLab ASCII .ply point cloud with camera center positions,
        view vectors as normals, and RGB color coded by elevation.
        """
        os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
        lines = [
            "ply",
            "format ascii 1.0",
            "comment Created by stepper-motion-core for MeshLab visualization",
            f"element vertex {len(self.poses)}",
            "property float x",
            "property float y",
            "property float z",
            "property float nx",
            "property float ny",
            "property float nz",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "end_header"
        ]

        for p in self.poses:
            c = p.extrinsics.camera_center_m
            # Normal pointing towards target origin (0, 0, 0)
            nx, ny, nz = -c.x, -c.y, -c.z
            n_len = math.sqrt(nx * nx + ny * ny + nz * nz)
            if n_len > 1e-12:
                nx /= n_len
                ny /= n_len
                nz /= n_len

            # Color gradient: green to cyan by elevation
            norm_el = max(0.0, min(1.0, p.elevation_deg / 90.0))
            r = int(50 * (1.0 - norm_el))
            g = int(200 + 55 * norm_el)
            b = int(255 * norm_el)

            lines.append(f"{c.x:.6f} {c.y:.6f} {c.z:.6f} {nx:.6f} {ny:.6f} {nz:.6f} {r} {g} {b}")

        with open(file_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    def export_colmap(self, cameras_file: str, images_file: str, width: int = 1920, height: int = 1080, focal_length: float = 1800.0) -> None:
        """Export cameras.txt and images.txt compatible with COLMAP and Gaussian Splatting."""
        os.makedirs(os.path.dirname(os.path.abspath(cameras_file)), exist_ok=True)
        os.makedirs(os.path.dirname(os.path.abspath(images_file)), exist_ok=True)

        # cameras.txt: CAMERA_ID, MODEL, WIDTH, HEIGHT, fx, fy, cx, cy
        cx = width / 2.0
        cy = height / 2.0
        with open(cameras_file, "w", encoding="utf-8") as f:
            f.write("# Camera list with one line of data per camera:\n")
            f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
            f.write(f"1 PINHOLE {width} {height} {focal_length:.4f} {focal_length:.4f} {cx:.4f} {cy:.4f}\n")

        # images.txt: IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        with open(images_file, "w", encoding="utf-8") as f:
            f.write("# Image list with two lines of data per image:\n")
            f.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
            f.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
            for p in self.poses:
                q = p.extrinsics.quaternion_wxyz
                t = p.extrinsics.translation_xyz_m
                f.write(f"{p.frame_id} {q.w:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {t.x:.6f} {t.y:.6f} {t.z:.6f} 1 {p.image_file}\n")
                f.write("\n")  # Empty 2D feature points line

    def export_reality_capture(self, csv_file: str) -> None:
        """Export rc_poses.csv compatible with RealityCapture photogrammetry pipeline."""
        os.makedirs(os.path.dirname(os.path.abspath(csv_file)), exist_ok=True)
        with open(csv_file, "w", encoding="utf-8") as f:
            f.write("#name, x, y, z, heading, pitch, roll\n")
            for p in self.poses:
                c = p.extrinsics.camera_center_m
                # Heading = azimuth, Pitch = -elevation, Roll = 0
                f.write(f"{p.image_file}, {c.x:.6f}, {c.y:.6f}, {c.z:.6f}, {p.azimuth_deg:.4f}, {-p.elevation_deg:.4f}, 0.0000\n")

    def export_all(self, output_dir: str) -> Dict[str, str]:
        """Convenience function to generate all supported photogrammetry formats in output_dir."""
        os.makedirs(output_dir, exist_ok=True)
        json_path = os.path.join(output_dir, "poses.json")
        ply_path = os.path.join(output_dir, "camera_frustums.ply")
        colmap_cams = os.path.join(output_dir, "cameras.txt")
        colmap_imgs = os.path.join(output_dir, "images.txt")
        rc_path = os.path.join(output_dir, "rc_poses.csv")

        self.export_json(json_path)
        self.export_meshlab_ply(ply_path)
        self.export_colmap(colmap_cams, colmap_imgs)
        self.export_reality_capture(rc_path)

        return {
            "json": json_path,
            "meshlab_ply": ply_path,
            "colmap_cameras": colmap_cams,
            "colmap_images": colmap_imgs,
            "reality_capture": rc_path
        }
