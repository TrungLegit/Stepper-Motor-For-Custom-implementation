# Stepper-Motion-Core & High-Detail 3D Scanner

A production-grade, dual-layer open-source mechatronics framework:
1. **`packages/motion-core`**: A modular, hardware-agnostic C++17 Stepper Motor Motion SDK featuring non-blocking 7-phase S-curve velocity profiling, micro-vibration suppression, backlash compensation, and flexible homing (Microswitch / Hall Effect Sensor / StallGuard4).
2. **`apps/high-detail-3d-scanner`**: A flagship reference application implementing an automated 3-axis spherical 3D photogrammetry scanner with direct export to **MeshLab** (`.ply` camera frustums & point clouds), **COLMAP**, and **RealityCapture**.

---

## 🌟 Key Features

* **Deterministic Step Generation**: Zero division (`/`), zero square root (`sqrt`), and zero dynamic memory allocation (`malloc/new`) inside the high-frequency Timer ISR.
* **Jerk-Limited 7-Phase S-Curve Profile**: Eliminates high-frequency mechanical resonance and motor buzz, preventing blur during macro camera capture.
* **ESP32-S3 Target & MockHAL PC Simulator**: First-class hardware support for ESP32-S3 (with `esp_timer` 64-bit microsecond timing and single-wire TMC2209 UART) plus an included PC `MockHAL` simulator allowing 100% unit-test execution without physical hardware.
* **Modular Limit Switch Support**: Seamless transition from mechanical microswitches (with configurable software debounce filtering) to Hall effect magnetic sensors (A3144, active LOW/HIGH).
* **MeshLab & COLMAP Photogrammetry Ready**: Automatic spherical Fibonacci pose generation producing shadow-free viewpoints and generating native `.ply`, `cameras.txt`, and `images.txt` metadata.
* **Dual Driver Architecture**: Full support for smart **TMC2209** (StealthChop2, CoolStep, UART) and legacy **A4988 / DRV8825** (pure STEP/DIR/EN mode).
* **Host Control Suite**: Python SDK, Command-Line CLI (`mc_cli.py`), and a modern Web Dashboard featuring a Three.js interactive 3D viewpoint preview.

---

## 📁 Repository Structure

```
.
├── packages/
│   └── motion-core/             # Reusable C++17 Motion Planning SDK
│       ├── include/             # Pure virtual HAL, Planner, Protocol headers
│       ├── src/                 # S-curve math, step generation, JSON-RPC
│       ├── hal_impl/            # ESP32-S3 hardware HAL & PC MockHAL simulator
│       └── tests/               # 6 unit test suites (100% CTest pass)
├── apps/
│   ├── high-detail-3d-scanner/  # Reference 3D Scanner Application (Firmware & Tests)
│   └── minimal-template/        # 15-minute cloner template for custom rigs
├── host-sdk/
│   ├── python/                  # Python SDK & CLI tool (`mc-cli`)
│   └── web-dashboard/           # Web UI with Three.js 3D viewpoint visualizer
└── docs/                        # Architecture, BOM, Cloner Guide, Workflow
```

---

## 🚀 Quick Start

### 1. Build and Run Tests on PC (No Hardware Needed)
```bash
# Configure and build with CMake
cmake -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Execute all 7 CTest suites (100% Pass)
ctest --test-dir build --output-on-failure
```

### 2. Run the Python Host SDK Tests
```bash
python host-sdk/tests/test_sdk.py
```

### 3. Launch the Web Dashboard
Open [`host-sdk/web-dashboard/index.html`](host-sdk/web-dashboard/index.html) in any modern web browser to interact with the 3D Fibonacci viewpoint preview, jog axes, and simulate scan routines.

---

## 📚 Documentation

* [Architecture & Blueprint](docs/ARCHITECTURE.md)
* [How to Fork & Build Your Own Rig in 15 Minutes](docs/CLONER_GUIDE.md)
* [Hardware BOM & Wiring Guide](docs/HARDWARE_BOM.md)
* [JSON-RPC 2.0 Command & Telemetry Specification](docs/PROTOCOL_SPEC.md)
* [3D Scanning to MeshLab & COLMAP Pipeline](docs/SCANNING_WORKFLOW.md)

---

## 📄 License
MIT License. Free for open-source and commercial mechatronic projects.
