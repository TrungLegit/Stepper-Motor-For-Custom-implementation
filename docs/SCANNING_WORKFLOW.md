# SCANNING_WORKFLOW.md — High-Detail 3D Scanning to MeshLab & COLMAP Pipeline

This guide outlines the end-to-end automated scanning workflow, from capturing spherical viewpoints to reconstructing 3D textured models using **COLMAP** and **MeshLab**.

---

## 1. Physical Setup & Calibration

1. **Mount Target Object**: Center the mechanical part on the Azimuth turntable. Ensure matte, uniform lighting (diffused ring light or softboxes) to minimize harsh specular reflections.
2. **Camera Alignment**: Adjust the tilt gimbal so the optical axis points toward the center of rotation $(0, 0, 0)$ of the object.
3. **Home the Machine**:
   ```bash
   python host-sdk/python/cli/mc_cli.py --port COM3 home
   ```

---

## 2. Automated Spherical Scan Execution

### Option A: Using the Interactive Web Dashboard
1. Open `host-sdk/web-dashboard/index.html` in your browser.
2. Connect to the ESP32-S3 via Serial or WebSocket (`ws://192.168.4.1:8080`).
3. View the 3D Fibonacci sphere viewpoint visualization in real-time.
4. Set:
   * **Poses**: `56` (or `120` for ultra-fine detail)
   * **Settle Delay**: `150 ms` (allows structural vibrations to damp out)
5. Click **"Start Automated Scan"**.

### Option B: Using the Command Line CLI
```bash
python host-sdk/python/cli/mc_cli.py --port COM3 scan \
    --poses 56 \
    --elevation-min 5.0 \
    --elevation-max 85.0 \
    --settle-ms 150 \
    --out-dir ./scans/my_part_scan
```

During execution, the scanner executes the deterministic state machine:
$$\text{MOVE} \longrightarrow \text{SETTLE (150ms)} \longrightarrow \text{TRIGGER SHUTTER} \longrightarrow \text{WAIT ACK} \longrightarrow \text{LOG POSE}$$

---

## 3. Dataset Output Structure

After the scan finishes, the output directory contains synchronized images and pre-calculated 6-DoF camera poses:

```
scans/my_part_scan/
├── images/
│   ├── frame_0000.jpg
│   ├── frame_0001.jpg
│   └── ...
├── poses.ply             ← MeshLab-compatible Camera Frustum & Point Cloud
├── poses.json            ← Full JSON metadata
├── colmap/
│   ├── cameras.txt       ← COLMAP pinhole camera intrinsics
│   └── images.txt        ← Pre-calculated World-to-Camera quaternions & translations
└── reality_capture/
    └── rc_poses.csv      ← RealityCapture rigid pose priors
```

---

## 4. Reconstructing with COLMAP

Since camera poses are already calibrated and saved in `images.txt`, you can run COLMAP feature extraction and dense reconstruction with maximal stability:

```bash
# 1. Feature Extraction
colmap feature_extractor \
    --database_path colmap.db \
    --image_path scans/my_part_scan/images

# 2. Exhaustive Feature Matching
colmap exhaustive_matcher \
    --database_path colmap.db

# 3. Dense Multi-View Stereo (MVS)
colmap dense_stereo \
    --workspace_path ./dense \
    --workspace_format COLMAP

# 4. Dense Point Cloud Fusion
colmap dense_fuser \
    --workspace_path ./dense \
    --workspace_format COLMAP \
    --output_path ./dense/fused.ply
```

---

## 5. MeshLab Post-Processing & Poisson Surface Meshing

1. **Verify Camera Geometry**:
   * Open **MeshLab** → `File` → `Import Mesh` → select `scans/my_part_scan/poses.ply`.
   * You will immediately see the complete spherical distribution of camera poses and viewing frustums pointing toward the object.

2. **Import Dense Point Cloud**:
   * In MeshLab, import `dense/fused.ply`.
   * Apply statistical outlier removal: `Filters` → `Point Set` → `Clean outliers`.

3. **Compute Surface Mesh (Screened Poisson)**:
   * Select `Filters` → `Remeshing, Simplification and Reconstruction` → `Surface Reconstruction: Screened Poisson`.
   * Set `Reconstruction Depth`: `10` or `11`.
   * Click **Apply**. MeshLab will generate a watertight, high-fidelity 3D triangular surface mesh (`meshed-poisson.ply`).

4. **Export Clean OBJ**:
   * `File` → `Export Mesh As...` → `my_part_mesh.obj`.
