"""
motion_client: Host Client Python SDK for stepper-motion-core and 3D Scanner.
"""

from .connection import ConnectionTransport, JsonRpcError, MockTransport, MotionConnection, SerialTransport
from .pose_exporter import CameraExtrinsics, PoseExporter, QuaternionWXYZ, ScanPose, Vector3d, solve_spherical_extrinsics
from .scanner import ScannerController, generate_fibonacci_sphere_poses, generate_uniform_grid_poses

__version__ = "0.2.0"

__all__ = [
    "MotionConnection",
    "ConnectionTransport",
    "MockTransport",
    "SerialTransport",
    "JsonRpcError",
    "ScannerController",
    "generate_fibonacci_sphere_poses",
    "generate_uniform_grid_poses",
    "PoseExporter",
    "ScanPose",
    "CameraExtrinsics",
    "QuaternionWXYZ",
    "Vector3d",
    "solve_spherical_extrinsics",
]
