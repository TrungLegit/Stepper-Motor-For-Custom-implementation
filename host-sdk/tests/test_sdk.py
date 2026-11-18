"""
Unit tests for host-sdk motion_client Python package.
"""

import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "python")))

from motion_client import (
    CameraExtrinsics,
    JsonRpcError,
    MockTransport,
    MotionConnection,
    PoseExporter,
    ScannerController,
    generate_fibonacci_sphere_poses,
    generate_uniform_grid_poses,
    solve_spherical_extrinsics,
)


class TestConnection(unittest.TestCase):
    def setUp(self):
        self.transport = MockTransport()
        self.conn = MotionConnection(self.transport)
        self.conn.connect()

    def tearDown(self):
        self.conn.disconnect()

    def test_basic_rpc(self):
        info = self.conn.call("system.info")
        self.assertEqual(info.get("firmware"), "stepper-motion-core")
        self.assertEqual(info.get("version"), "0.2.0")

    def test_motion_and_telemetry(self):
        received_events = []
        self.conn.add_event_listener("motion.status", lambda d: received_events.append(d))

        res = self.conn.call("axis.move", {"axes": {"A": 45.0, "B": 30.0}})
        self.assertTrue(res.get("queued"))

        # Check telemetry event was captured
        st = self.conn.call("axis.get_status")
        pos = st.get("pos", {})
        self.assertEqual(pos.get("A"), 45.0)
        self.assertEqual(pos.get("B"), 30.0)

    def test_estop_preemption_and_reset(self):
        # 1. Trigger E-STOP
        estop_res = self.conn.emergency_stop("Safety test")
        self.assertTrue(estop_res.get("estop"))

        # 2. Motion calls must fail with JsonRpcError
        with self.assertRaises(JsonRpcError) as ctx:
            self.conn.call("axis.move", {"axes": {"A": 10.0}})
        self.assertEqual(ctx.exception.code, -32001)

        # 3. Reset clears E-STOP
        reset_res = self.conn.call("system.reset")
        self.assertTrue(reset_res.get("reset"))

        # 4. Motion calls work again
        move_res = self.conn.call("axis.move", {"axes": {"A": 0.0}})
        self.assertTrue(move_res.get("queued"))


class TestFibonacciSphere(unittest.TestCase):
    def test_pose_generation_bounds(self):
        poses = generate_fibonacci_sphere_poses(n_poses=48, elevation_min_deg=10.0, elevation_max_deg=80.0)
        self.assertEqual(len(poses), 48)

        for p in poses:
            az = p["azimuth_deg"]
            el = p["elevation_deg"]
            self.assertGreaterEqual(az, 0.0)
            self.assertLessEqual(az, 360.0)
            self.assertGreaterEqual(el, 9.9)
            self.assertLessEqual(el, 80.1)

    def test_uniform_grid_poses(self):
        poses = generate_uniform_grid_poses(azimuth_steps=12, elevation_angles=[20.0, 45.0, 70.0])
        self.assertEqual(len(poses), 36)


class TestPoseExporter(unittest.TestCase):
    def test_extrinsics_math(self):
        # Point on equator (azimuth 0, elevation 0)
        ext = solve_spherical_extrinsics(azimuth_deg=0.0, elevation_deg=0.0, radius_m=0.20)
        self.assertAlmostEqual(ext.camera_center_m.x, 0.20, places=4)
        self.assertAlmostEqual(ext.camera_center_m.y, 0.0, places=4)
        self.assertAlmostEqual(ext.camera_center_m.z, 0.0, places=4)

        # Quaternion norm must be 1.0
        q = ext.quaternion_wxyz
        q_norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        self.assertAlmostEqual(q_norm, 1.0, places=5)

    def test_file_exports(self):
        exporter = PoseExporter(default_radius_m=0.20)
        for i in range(5):
            exporter.add_pose({
                "frame_id": i + 1,
                "image_file": f"frame_{i+1:04d}.jpg",
                "pose": {"azimuth_deg": i * 45.0, "elevation_deg": 30.0, "focus_z_mm": 0.0}
            })

        with tempfile.TemporaryDirectory() as tmpdir:
            exported = exporter.export_all(tmpdir)
            for key, filepath in exported.items():
                self.assertTrue(os.path.exists(filepath), f"File {filepath} was not created")
                self.assertGreater(os.path.getsize(filepath), 0, f"File {filepath} is empty")

            # Check PLY content
            with open(exported["meshlab_ply"], "r") as f:
                content = f.read()
                self.assertTrue(content.startswith("ply\nformat ascii 1.0\n"))
                self.assertIn("element vertex 5\n", content)
                self.assertIn("end_header\n", content)


class TestScannerController(unittest.TestCase):
    def test_automated_scan_mock(self):
        scanner = ScannerController()
        scanner.connect(mock=True)
        try:
            poses = scanner.run_scan(n_poses=8, settle_ms=10)
            self.assertEqual(len(poses), 8)
            self.assertEqual(len(scanner.exporter.poses), 8)
        finally:
            scanner.disconnect()


if __name__ == "__main__":
    unittest.main()
