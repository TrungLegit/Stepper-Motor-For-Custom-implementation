"""
High-Level 3D Scanner Controller and Automated Scan Orchestrator.
Manages connection, homing, jog, Fibonacci sphere scan path execution, and pose logging.
"""

import logging
import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from .connection import ConnectionTransport, MockTransport, MotionConnection, SerialTransport
from .pose_exporter import PoseExporter, ScanPose

logger = logging.getLogger("motion_client.scanner")


def generate_fibonacci_sphere_poses(
    n_poses: int = 56,
    elevation_min_deg: float = 5.0,
    elevation_max_deg: float = 85.0,
    focus_z_mm: float = 0.0
) -> List[Dict[str, float]]:
    """
    Generate N uniformly distributed scanning poses over the upper hemisphere
    using a Fibonacci spiral / golden ratio lattice.
    """
    poses = []
    golden_ratio = (1.0 + math.sqrt(5.0)) / 2.0
    golden_angle_rad = 2.0 * math.pi * (1.0 - 1.0 / golden_ratio)

    el_min_rad = math.radians(elevation_min_deg)
    el_max_rad = math.radians(elevation_max_deg)
    y_min = math.sin(el_min_rad)
    y_max = math.sin(el_max_rad)

    for i in range(n_poses):
        # Uniform height in [y_min, y_max]
        fraction = (i + 0.5) / float(n_poses)
        y = y_min + fraction * (y_max - y_min)
        elevation = math.degrees(math.asin(y))

        # Golden angle azimuth rotation
        azimuth_rad = (i * golden_angle_rad) % (2.0 * math.pi)
        azimuth = math.degrees(azimuth_rad)

        poses.append({
            "azimuth_deg": round(azimuth, 2),
            "elevation_deg": round(elevation, 2),
            "focus_z_mm": round(focus_z_mm, 2)
        })

    return poses


def generate_uniform_grid_poses(
    azimuth_steps: int = 24,
    elevation_angles: Optional[List[float]] = None,
    focus_z_mm: float = 0.0
) -> List[Dict[str, float]]:
    """
    Generate scanning poses on a regular angular grid.
    """
    if elevation_angles is None:
        elevation_angles = [15.0, 30.0, 45.0, 60.0, 75.0]

    poses = []
    az_delta = 360.0 / float(azimuth_steps)

    for el in elevation_angles:
        for a_idx in range(azimuth_steps):
            az = a_idx * az_delta
            poses.append({
                "azimuth_deg": round(az, 2),
                "elevation_deg": round(el, 2),
                "focus_z_mm": round(focus_z_mm, 2)
            })
    return poses


class ScannerController:
    """
    High-level orchestrator for the 3D scanning rig.
    """

    def __init__(self, transport: Optional[ConnectionTransport] = None):
        self.conn = MotionConnection(transport or MockTransport())
        self.exporter = PoseExporter()
        self._is_scanning = False
        self._is_paused = False
        self._abort_requested = False
        self._scan_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Cached live telemetry
        self.last_status: dict = {}
        self.conn.add_event_listener("motion.status", self._on_motion_status)
        self.conn.add_event_listener("scan.pose_complete", self._on_pose_complete)

    def _on_motion_status(self, data: dict):
        self.last_status = data

    def _on_pose_complete(self, data: dict):
        self.exporter.add_pose(data)

    def connect(self, port: Optional[str] = None, baudrate: int = 115200, mock: bool = False) -> None:
        """Connect to scanner hardware via serial port or mock simulator."""
        if mock or port == "mock" or port is None:
            self.conn.transport = MockTransport()
        else:
            self.conn.transport = SerialTransport(port, baudrate)
        self.conn.connect()

    def disconnect(self) -> None:
        """Disconnect transport and stop any ongoing background scan."""
        self.abort_scan()
        self.conn.disconnect()

    def is_connected(self) -> bool:
        return self.conn.is_connected()

    # -------------------------------------------------------------------------
    # Core Motion Commands
    # -------------------------------------------------------------------------

    def get_info(self) -> dict:
        return self.conn.call("system.info")

    def get_status(self) -> dict:
        res = self.conn.call("axis.get_status")
        self.last_status = res
        return res

    def home(self, axes: Optional[List[str]] = None) -> dict:
        """Execute homing sequence on specified axes (or all if omitted)."""
        params = {"axes": axes} if axes else {}
        return self.conn.call("axis.home", params, timeout=30.0)

    def jog(self, axis: str, distance: float, feedrate: float = 0.0) -> dict:
        """Relative jog move on a single axis."""
        params = {"axis": axis, "distance": distance, "feedrate": feedrate}
        return self.conn.call("axis.jog", params)

    def move_to(self, axes: Dict[str, float], profile: str = "scurve", feedrate: float = 0.0) -> dict:
        """Absolute coordinated move across axes."""
        params = {"axes": axes, "profile": profile, "feedrate": feedrate}
        return self.conn.call("axis.move", params)

    def trigger_camera(self, settle_ms: int = 100, pulse_us: int = 50000) -> dict:
        """Fire camera shutter trigger pulse."""
        params = {"settle_ms": settle_ms, "pulse_us": pulse_us}
        return self.conn.call("trigger.capture", params)

    def emergency_stop(self, reason: str = "User triggered E-STOP") -> dict:
        """Immediate Emergency Stop: aborts motion and enters locked safety state."""
        self._abort_requested = True
        return self.conn.emergency_stop(reason)

    def reset(self) -> dict:
        """Clear E-STOP lock and restore system to IDLE."""
        self._abort_requested = False
        return self.conn.call("system.reset")

    # -------------------------------------------------------------------------
    # Automated 3D Scan Workflow
    # -------------------------------------------------------------------------

    def is_scanning(self) -> bool:
        return self._is_scanning

    def is_paused(self) -> bool:
        return self._is_paused

    def pause_scan(self) -> None:
        self._is_paused = True
        logger.info("Scan paused.")

    def resume_scan(self) -> None:
        self._is_paused = False
        logger.info("Scan resumed.")

    def abort_scan(self) -> None:
        self._abort_requested = True
        self._is_paused = False
        if self._scan_thread and self._scan_thread.is_alive():
            self._scan_thread.join(timeout=2.0)
        logger.info("Scan aborted.")

    def run_scan(
        self,
        poses: Optional[List[Dict[str, float]]] = None,
        n_poses: int = 56,
        settle_ms: int = 150,
        async_run: bool = False,
        progress_cb: Optional[Callable[[int, int, dict], None]] = None,
        camera_cb: Optional[Callable[[int, str], None]] = None
    ) -> List[ScanPose]:
        """
        Execute automated scan sequence through list of poses.
        If poses is None, generates an optimal Fibonacci sphere scan path.
        """
        if self._is_scanning:
            raise RuntimeError("Scan session already in progress")

        if poses is None:
            poses = generate_fibonacci_sphere_poses(n_poses=n_poses)

        self._is_scanning = True
        self._is_paused = False
        self._abort_requested = False
        self.exporter.poses.clear()

        def _worker():
            try:
                self._execute_scan_loop(poses, settle_ms, progress_cb, camera_cb)
            finally:
                self._is_scanning = False

        if async_run:
            self._scan_thread = threading.Thread(target=_worker, daemon=True, name="Scanner-Job")
            self._scan_thread.start()
            return self.exporter.poses
        else:
            _worker()
            return self.exporter.poses

    def _execute_scan_loop(
        self,
        poses: List[Dict[str, float]],
        settle_ms: int,
        progress_cb: Optional[Callable[[int, int, dict], None]],
        camera_cb: Optional[Callable[[int, str], None]]
    ) -> None:
        total = len(poses)
        logger.info("Starting automated scan with %d poses...", total)

        for idx, pose in enumerate(poses):
            if self._abort_requested:
                logger.warning("Scan loop interrupted by abort request.")
                break

            while self._is_paused and not self._abort_requested:
                time.sleep(0.1)

            frame_id = idx + 1
            image_name = f"frame_{frame_id:04d}.jpg"

            # 1. MOVE_TO_POSE
            az = pose["azimuth_deg"]
            el = pose["elevation_deg"]
            fz = pose.get("focus_z_mm", 0.0)
            self.move_to({"A": az, "B": el, "Z": fz})

            # Wait for motion to complete
            time.sleep(0.05)
            while True:
                if self._abort_requested:
                    return
                st = self.get_status()
                if not st.get("moving", False):
                    break
                time.sleep(0.05)

            # 2. SETTLING_DELAY (vibration suppression)
            time.sleep(settle_ms / 1000.0)

            # 3. TRIGGER_CAMERA
            self.trigger_camera(settle_ms=settle_ms)
            if camera_cb:
                try:
                    camera_cb(frame_id, image_name)
                except Exception as e:
                    logger.error("Camera hook callback error: %s", e)

            # 4. LOG_POSE
            record_data = {
                "frame_id": frame_id,
                "image_file": image_name,
                "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "pose": {
                    "azimuth_deg": az,
                    "elevation_deg": el,
                    "focus_z_mm": fz
                },
                "motor_steps_abs": {"A": int(az * 35.55), "B": int(el * 35.55), "Z": int(fz * 400)},
                "motor_pos_calibrated": {"A_deg": az, "B_deg": el, "Z_mm": fz}
            }
            rec = self.exporter.add_pose(record_data)

            if progress_cb:
                try:
                    progress_cb(frame_id, total, record_data)
                except Exception as e:
                    logger.error("Progress callback error: %s", e)

        logger.info("Scan session completed. Total logged poses: %d", len(self.exporter.poses))
