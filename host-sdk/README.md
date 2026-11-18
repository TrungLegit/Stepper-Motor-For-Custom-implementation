# host-sdk: Python Client SDK, CLI & Web Dashboard

Host controller tools for **stepper-motion-core** and the **High-Detail 3D Scanner**.

---

## 1. Quick Start

### Installation

```bash
cd host-sdk
pip install -e .
```

### CLI Tool (`mc-cli`)

```bash
# Query system status (hardware or mock simulation)
mc-cli status --mock

# Manual axis jog
mc-cli jog --axis A --deg 15.0 --mock
mc-cli jog --axis B --deg 5.0 --mock

# Home all axes
mc-cli home --mock

# Automated 3D scan with Fibonacci sphere lattice and export
mc-cli scan --poses 56 --settle-ms 150 --out-dir ./output --mock

# Emergency Stop & Reset
mc-cli estop --mock
mc-cli reset --mock
```

---

## 2. Python API

```python
from motion_client import ScannerController

scanner = ScannerController()
# Connect to physical hardware (e.g. COM3 or /dev/ttyUSB0) or Mock simulator
scanner.connect(mock=True)

# Query live status
status = scanner.get_status()
print("Position:", status["pos"])

# Execute automated 3D photogrammetry scan
poses = scanner.run_scan(n_poses=56, settle_ms=150)

# Export all reconstruction formats
scanner.exporter.export_all("./scan_output")
# Generates:
# - scan_output/poses.json
# - scan_output/camera_frustums.ply (MeshLab point cloud + normals)
# - scan_output/cameras.txt & images.txt (COLMAP / Gaussian Splatting)
# - scan_output/rc_poses.csv (RealityCapture)
```

---

## 3. Web Dashboard

Open `host-sdk/web-dashboard/index.html` in any modern web browser.
- **3D Viewport**: Real-time Three.js visualization of the Fibonacci scan path, target object, turntable plate, and camera frustum.
- **Live Telemetry**: Real-time position coordinates (Azimuth, Elevation, Focus) and machine state.
- **Manual Jog & Homing**: Clickable buttons with selectable degree increments.
- **3D Scan Runner**: Progress bar, settle delay tuning, and instant photogrammetry export.
- **Safety Control**: Prominent Emergency Stop (E-STOP) button and system reset.
