# ARCHITECTURE.md — Stepper Motion Core & High-Detail 3D Scanner

> **Version:** 0.2.0 · **Phase:** 2 — Implementation Underway  
> **Target MCU:** ESP32-S3 (Primary)  
> **Reconstruction Target:** MeshLab (.ply / .obj point clouds & camera poses) + COLMAP / RealityCapture  
> **Endstops:** Configurable Physical Limit Switch -> Hall Effect Sensor (with software debounce)

---

## Table of Contents

1. [Project Overview & Dual-Layer Mission](#1-project-overview--dual-layer-mission)
2. [Repository Monorepo Structure](#2-repository-monorepo-structure)
3. [Hardware Stack Analysis & Recommendations](#3-hardware-stack-analysis--recommendations)
   - 3.1 [MCU Selection](#31-mcu-selection)
   - 3.2 [Stepper Driver Tier](#32-stepper-driver-tier)
   - 3.3 [Default Reference Hardware BOM](#33-default-reference-hardware-bom)
4. [3D Scanner Physics & Kinematics](#4-3d-scanner-physics--kinematics)
   - 4.1 [Multi-Axis Spherical Kinematic Model](#41-multi-axis-spherical-kinematic-model)
   - 4.2 [Micro-Vibration Suppression Strategy](#42-micro-vibration-suppression-strategy)
   - 4.3 [Capture State Machine](#43-capture-state-machine)
5. [Software Architecture — `packages/motion-core`](#5-software-architecture--packagesmotion-core)
   - 5.1 [Layer Diagram](#51-layer-diagram)
   - 5.2 [Hardware Abstraction Layer (HAL)](#52-hardware-abstraction-layer-hal)
   - 5.3 [Motion Planner Engine](#53-motion-planner-engine)
   - 5.4 [Host-Controller Protocol](#54-host-controller-protocol)
6. [Reference Application — `apps/3d-scanner`](#6-reference-application--apps3d-scanner)
7. [Extensibility & Cloner DX (Developer Experience)](#7-extensibility--cloner-dx-developer-experience)
8. [Implementation Phases & Milestones](#8-implementation-phases--milestones)
9. [Open Questions for Project Owner](#9-open-questions-for-project-owner)

---

## 1. Project Overview & Dual-Layer Mission

```
+---------------------------------------------------------------------+
|          LAYER 2 — Flagship Reference Application                    |
|          apps/high-detail-3d-scanner                                 |
|   (Spherical scan paths · Capture sync · Pose export · Host UI)      |
+---------------------------------------------------------------------+
|          LAYER 1 — Reusable Core SDK                                  |
|          packages/motion-core                                         |
|   (HAL · Motion Planner · Step ISR · Protocol · MockHAL)             |
+---------------------------------------------------------------------+
```

| Goal | Description |
|------|-------------|
| **Core SDK (`motion-core`)** | Hardware-agnostic, fully unit-testable stepper control framework. Any cloner replaces only the board pin-map and writes a custom Kinematics/App module. Zero changes to the motion engine. |
| **Reference App (`3d-scanner`)** | End-to-end high-precision multi-axis 3D scanner consuming exclusively public `motion-core` APIs. Demonstrates photogrammetry / structured-light / laser-triangulation workflows for small intricate mechanical parts. |
| **Template (`apps/minimal-template`)** | Stripped-down skeleton. "Fork & spin up your own rig in 15 minutes." |

---

## 2. Repository Monorepo Structure

```
stepper-motion-core/                   <- repo root
|
+-- packages/
|   +-- motion-core/                   <- CORE SDK (firmware + MockHAL)
|       +-- include/
|       |   +-- hal/                   <- Pure-virtual HAL interfaces
|       |   |   +-- IGpioHAL.hpp
|       |   |   +-- ITimerHAL.hpp
|       |   |   +-- IDriverHAL.hpp     <- Wraps STEP/DIR/EN + UART/SPI config
|       |   |   +-- IEndstopHAL.hpp
|       |   |   +-- IEncoderHAL.hpp
|       |   +-- planner/
|       |   |   +-- MotionPlanner.hpp
|       |   |   +-- StepGenerator.hpp
|       |   |   +-- VelocityProfile.hpp  <- Trapezoidal + S-Curve
|       |   |   +-- Axis.hpp
|       |   |   +-- KinematicsInterface.hpp
|       |   |   +-- BacklashCompensator.hpp
|       |   +-- protocol/
|       |   |   +-- CommandParser.hpp
|       |   |   +-- JsonRpcDispatcher.hpp
|       |   |   +-- TelemetryEmitter.hpp
|       |   +-- motion_core.hpp        <- Single-include convenience header
|       |
|       +-- src/
|       |   +-- hal/                   <- Shared HAL utilities (non-platform)
|       |   +-- planner/               <- Platform-independent motion math
|       |   +-- protocol/
|       |
|       +-- hal_impl/                  <- Platform-specific HAL implementations
|       |   +-- rp2040_pio/            <- RP2040 PIO step generator
|       |   +-- stm32_tim_dma/         <- STM32 Timer+DMA step generator
|       |   +-- esp32s3_mcpwm/         <- ESP32-S3 MCPWM step generator
|       |   +-- mock/                  <- PC MockHAL for unit tests & simulation
|       |
|       +-- tests/                     <- GoogleTest unit tests (runs on PC)
|       |   +-- test_scurve.cpp
|       |   +-- test_planner.cpp
|       |   +-- test_backlash.cpp
|       |   +-- test_protocol.cpp
|       |
|       +-- CMakeLists.txt
|
+-- apps/
|   +-- high-detail-3d-scanner/        <- REFERENCE APPLICATION (firmware side)
|   |   +-- include/
|   |   |   +-- ScanPathGenerator.hpp  <- Fibonacci sphere + uniform grid
|   |   |   +-- CaptureEngine.hpp      <- Move->Settle->Trigger->ACK state machine
|   |   |   +-- ScannerKinematics.hpp  <- Spherical axis mapping
|   |   |   +-- PoseExporter.hpp       <- COLMAP / RealityCapture / JSON output
|   |   +-- src/
|   |   +-- board/
|   |   |   +-- pinmap_rp2040.hpp      <- Default reference board
|   |   |   +-- pinmap_stm32.hpp
|   |   +-- CMakeLists.txt
|   |
|   +-- minimal-template/              <- CLONER STARTING POINT
|       +-- include/
|       |   +-- MyKinematics.hpp       <- Only file cloner needs to edit
|       |   +-- MyBoardPinmap.hpp
|       +-- src/
|       |   +-- main.cpp
|       +-- CMakeLists.txt
|
+-- host-sdk/                          <- Python SDK + Web Dashboard (runs on PC/RPi)
|   +-- python/
|   |   +-- motion_client/             <- Python package (pip-installable)
|   |   |   +-- __init__.py
|   |   |   +-- connection.py          <- USB-Serial + WebSocket transport
|   |   |   +-- axis.py
|   |   |   +-- scanner.py             <- High-level 3D scanner commands
|   |   |   +-- pose_exporter.py
|   |   +-- cli/
|   |       +-- mc_cli.py              <- Click-based CLI tool
|   +-- web-dashboard/
|       +-- src/                       <- Vite + React (or plain HTML/JS)
|       +-- package.json
|
+-- docs/
|   +-- ARCHITECTURE.md                <- This file
|   +-- CLONER_GUIDE.md                <- "Fork & build in 15 minutes"
|   +-- PROTOCOL_SPEC.md               <- JSON-RPC command reference
|   +-- HARDWARE_BOM.md                <- Reference hardware bill of materials
|   +-- SCANNING_WORKFLOW.md           <- End-to-end scan tutorial
|
+-- .github/
|   +-- workflows/
|       +-- ci-unit-tests.yml          <- Build & test MockHAL on PC (Linux/macOS/Windows)
|       +-- ci-firmware-build.yml      <- Cross-compile for each target MCU
|
+-- ARCHITECTURE.md                    <- (this file)
+-- LICENSE                            <- MIT
+-- README.md
```

---

## 3. Hardware Stack Analysis & Recommendations

### 3.1 MCU Selection

#### Candidates Evaluated

| Criterion | RP2040 (PIO) | STM32G4/F4 (TIM+DMA) | ESP32-S3 (MCPWM/RMT) |
|-----------|-------------|----------------------|----------------------|
| **Step pulse jitter** | Ultra-low (cycle-exact PIO SM) | Near-zero (pure hardware timer) | Moderate (FreeRTOS/WiFi contention) |
| **Multi-axis scaling** | 8 PIO SMs -> 8 axes free | Multiple TIM channels + DMA | 6 MCPWM channels (restrictive) |
| **CPU overhead for stepping** | Very low (CPU feeds FIFO only) | Negligible (DMA streams pre-calc array) | Medium-high (MCPWM config complex) |
| **Built-in Wi-Fi / BT** | No (needs add-on: Pico W) | No (needs add-on) | Yes native dual-core + BLE |
| **Cost (bare die / module)** | ~$1 (RP2040) / ~$4 (Pico) | ~$3-8 (Nucleo 32) | ~$3-6 (ESP32-S3-DevKitC) |
| **Ecosystem / toolchain** | C/C++ SDK, MicroPython, Arduino | STM32CubeIDE, Arduino via libopencm3 | ESP-IDF, Arduino |
| **Learning curve** | Medium (PIO assembly) | High (register-level timer/DMA) | Low-Medium (ESP-IDF well-documented) |
| **USB-Serial native** | Yes TinyUSB built-in | Yes via ST USB stack | Yes via UART-USB bridge |
| **Best for this project** | ** PRIMARY DEFAULT ** | Alternative (high-precision industrial) | Alternative (IoT-first builds) |

#### RECOMMENDED DEFAULT: RP2040 (Raspberry Pi Pico / Pico W)

**Rationale:**

1. **Deterministic PIO pulse generation** — Step pulses are generated entirely inside a PIO state machine running at the system clock (133 MHz), producing cycle-exact timing fully independent of the Cortex-M0+ CPU cores and FreeRTOS scheduler. This is non-negotiable for high-microstep precision.
2. **8 independent PIO state machines** -> easily drive 4 stepper axes (STEP + DIR each), leaving PIO bandwidth for encoder quadrature decode or additional I/O protocols.
3. **Cost and open-source alignment** — At $4 (Pico) / $6 (Pico W with Wi-Fi), the RP2040 is the ideal open-source community target.
4. **Pico W variant** provides ESP32-S3-level Wi-Fi connectivity without sacrificing PIO determinism, since the CYW43 radio runs on its own ARM core via SPI, never competing with PIO state machines.
5. **HAL abstraction** means the firmware can be recompiled for STM32 or ESP32-S3 without touching the motion planner — platform-specific HAL implementations live in `hal_impl/`.

> **Note:** The STM32 HAL implementation will be fully spec'd and usable for anyone who needs safety-certifiable, industrial-grade builds.

---

### 3.2 Stepper Driver Tier

#### Candidates Evaluated

| Feature | DRV8825 | TMC2209 | TMC5160 |
|---------|---------|---------|---------|
| **Max voltage** | 45V | 29V | 60V |
| **Max current (RMS)** | ~2A | 2A (2.8A peak) | 3A+ (external MOSFETs) |
| **Acoustic noise** | High (audible whine) | Silent (StealthChop2) | Silent (StealthChop2) |
| **Microstepping (native)** | 1/32 | 1/256 | 1/256 |
| **MicroPlyer interpolation** | No | Yes (to 256 from low-res input) | Yes |
| **Sensorless homing** | No | Yes StallGuard4 | Yes StallGuard2 |
| **Dynamic current scaling** | No | Yes CoolStep | Yes CoolStep + advanced |
| **Config interface** | Hardware pins only | UART (single-wire) | SPI or UART |
| **Thermal management** | Manual (pot trimmer) | Software (UART register) | Software (SPI register) |
| **Target application fit** | Obsolete for this project | ** PRIMARY DEFAULT ** | Industrial / high-torque |

#### RECOMMENDED DEFAULT: TMC2209 (UART mode)

**Rationale for 3D Scanner / Precision Mechatronics:**

1. **StealthChop2 silent operation** is critical for eliminating motor-induced vibration during the camera capture settle window.
2. **MicroPlyer 256-microstep interpolation** means the RP2040 PIO only needs to generate 1/16-step pulses (manageable FIFO rate), while the TMC2209 internally smooths to true 1/256 resolution.
3. **StallGuard4 sensorless homing** eliminates physical endstop microswitches, reducing the number of electrical connections and mechanical complexity on rotating/tilting axes.
4. **CoolStep dynamic current reduction** prevents thermal warping of 3D-printed mounts during long scan sessions.
5. **UART configurability** allows run-time adjustment of current, StealthChop/SpreadCycle switchover speed, and stall sensitivity from the host SDK.

#### TMC5160 Upgrade Path
For users running larger NEMA 23 motors at 36-48V (e.g., heavy CNC stages, large-format printers), the `IDriverHAL` interface accommodates a TMC5160 HAL implementation with minimal changes to application code.

---

### 3.3 Default Reference Hardware BOM

| Component | Specification | Qty (3-axis scanner) |
|-----------|--------------|---------------------|
| **MCU Board** | Raspberry Pi Pico W (RP2040 + CYW43439 Wi-Fi) | 1 |
| **Stepper Drivers** | TMC2209 breakout (BigTreeTech TMC2209 V1.2) | 3 |
| **Motors - Azimuth (Turntable)** | NEMA 17, 1.8 deg/step, 59 N*cm holding torque | 1 |
| **Motors - Elevation (Gimbal)** | NEMA 17, 1.8 deg/step, 40 N*cm | 1 |
| **Motors - Focus Z (Optional)** | NEMA 17, 1.8 deg/step, 40 N*cm | 1 |
| **Stepper PSU** | 24V / 5A regulated DC | 1 |
| **Logic PSU** | 5V USB or 3.3V LDO from 24V rail | 1 |
| **Camera Trigger Interface** | 3.3V GPIO -> PC817 optocoupler -> 2.5mm TRRS shutter jack | 1 |
| **Homing** | Sensorless via TMC2209 StallGuard4 (no physical switches) | - |
| **Communication** | USB-Serial (native TinyUSB) + Wi-Fi WebSocket (Pico W) | - |
| **Optional Encoder** | Magnetic AS5048A on turntable for closed-loop verification | 1 |

**Microstep Resolution Calculation (Azimuth axis, default config):**

```
Motor step angle     : 1.8 deg  ->  200 full steps/rev
TMC2209 MicroPlyer   : 1/16 input x 16 = 256 microsteps native equiv.
Effective resolution : 200 x 256 = 51,200 microsteps/rev
Angular resolution   : 360 / 51,200 = 0.00703 deg  ~  0.42 arcminutes per microstep
```

> **Practical scanner accuracy**: With a typical 150mm scan radius, one microstep of the azimuth axis produces a surface displacement of ~0.018mm — well within photogrammetry reprojection tolerances.

---

## 4. 3D Scanner Physics & Kinematics

### 4.1 Multi-Axis Spherical Kinematic Model

The scanner operates in a spherical coordinate frame centered on the target object:

```
         Z (Elevation axis - tilt gimbal)
         |
         |   * Camera head
         |  /
         | / r (scan radius, fixed)
         |/_______ Y (optional Focus-Stack Z axis)
        /|
       / |
      X  +--- Azimuth turntable (theta rotation around Z-world)
```

**Coordinate Convention:**

| Axis | Physical Component | Motor | Motion Type |
|------|--------------------|-------|-------------|
| theta (azimuth) | Turntable platform (object rotates) | Motor A | Continuous rotation, 0-360 deg |
| phi (elevation) | Tilt gimbal (camera tilts) | Motor B | Arc, configurable range (e.g., 0-90 deg) |
| z (focus) | Linear Z stage (focus stack) | Motor C | Linear, +/- range mm |

**Kinematic transforms from spherical to motor steps:**

```
motor_steps_azimuth   = theta_deg  x (steps_per_rev / 360.0)
motor_steps_elevation = phi_deg    x (steps_per_rev / 360.0) x (gearbox_ratio)
motor_steps_focus_z   = z_mm      x (steps_per_rev / lead_screw_pitch_mm)
```

**Scan Pose Generation Strategies:**

| Strategy | Description | Use Case |
|----------|-------------|----------|
| **Uniform Angular Grid** | Regular delta_theta x delta_phi grid on sphere | Simple objects, fast scans |
| **Fibonacci Sphere** | Sunflower/Fibonacci lattice distributes N poses uniformly | Complex geometries, shadow-free coverage |
| **Adaptive Density** | Higher density at equatorial band, sparse at poles | NEMA-geometry objects |
| **Custom Waypoint List** | User-specified CSV of (theta, phi, z) poses | Fine-tuned professional scans |

**Fibonacci Sphere Formula (N poses, pose index i = 0 to N-1):**

```
phi_golden = pi x (1 + sqrt(5))           // golden angle ~137.508 deg
y_i        = 1 - (2i / (N - 1))           // uniform y in [-1, +1]
r_i        = sqrt(1 - y_i^2)
theta_i    = i x phi_golden  (mod 2*pi)
azimuth    = atan2(sin(theta_i), cos(theta_i))   // -> motor A angle
elevation  = asin(y_i)                            // -> motor B angle
```

---

### 4.2 Micro-Vibration Suppression Strategy

#### 4.2.1 S-Curve (Jerk-Limited) Velocity Profile — 7-Phase Model

Standard trapezoidal profiles produce instantaneous acceleration changes (infinite jerk), exciting structural resonances in 3D-printed mounts. The core library implements the full 7-phase jerk-limited S-curve:

```
Velocity
  Vmax  ---------------------------------------------------------------- Phase 4 (cruise)
        /                                                               \
       /  P3 (neg jerk)                               P5 (neg jerk)    \
  P2  /                                                                  \ P6
(const accel)                                                        (const decel)
    /                                                                      \
P1 / (pos jerk)                                             (pos jerk) P7  \
  /                                                                          \---------
t0  t1    t2   t3                 t4                 t5   t6   t7               time
```

**Jerk phase duration tuning for vibration cancellation:**

The jerk ramp time T_j should equal (or be a multiple of) the structural resonant period T_n of the scanner platform:

```
T_j = T_n = 1 / f_natural
```

For 3D-printed PLA/PETG structures, typical f_natural is 20-80 Hz. Setting T_j = 15-50ms causes the residual oscillation from the start of the ramp to destructively cancel the oscillation from the end of the ramp (input shaping principle).

#### 4.2.2 Holding Current Thermal Management

```
Scenario                            Recommended current setting
-----------------------------------------------------------------
Moving (travel / slew)              100% rated current (via CoolStep auto)
Settling (post-move, pre-capture)   70% (reduces vibration from coil buzz)
Hold during long idle               30-40% (prevents PLA/PETG thermal deform)
```

TMC2209 UART register IHOLD_IRUN allows independent setting of:
- IRUN  (run current, 0-31 scale)
- IHOLD (hold current, 0-31 scale)
- IHOLDDELAY (ramp-down time after stop)

#### 4.2.3 Microstep Resolution vs. Resonance

| Microstep Mode | Pulse Frequency @ 1 rev/sec | Resonance Risk |
|---------------|-----------------------------|----------------|
| 1/8 | 25 Hz | High (matches structural f_n of many rigs) |
| 1/16 (input to TMC) | 50 Hz | Medium |
| 1/256 (via MicroPlyer) | 800 Hz | Very Low (well above structural modes) |

---

### 4.3 Capture State Machine

```
+---------------+
|     IDLE      |<---------------------------------------------------+
+-------+-------+                                                    |
        | StartScan command                                           |
        v                                                            |
+---------------+                                                    |
| MOVE_TO_NEXT  |  MotionPlanner.moveTo(azimuth, elevation, z)       |
|     POSE      |  (non-blocking, S-curve profile)                   |
+-------+-------+                                                    |
        | OnMotionComplete callback                                   |
        v                                                            |
+---------------+                                                    |
|   SETTLING    |  Wait settle_delay_ms (configurable, typ 100-500ms)|
|    TIMER      |  Timer ISR, NOT blocking delay!                     |
+-------+-------+                                                    |
        | OnSettleComplete                                            |
        v                                                            |
+---------------+                                                    |
|    TRIGGER    |  GPIO optocoupler pulse (shutter trigger)          |
|    CAMERA     |  OR HTTP/USB-PTP software trigger                   |
+-------+-------+                                                    |
        |                                                            |
        v                                                            |
+---------------+                                                    |
|   WAIT_ACK   |  Wait for capture acknowledgment:                   |
|              |  - GPIO "capture done" signal from camera           |
|              |  - Or HTTP response from camera API                 |
|              |  - Timeout -> RETRY or ABORT                        |
+-------+-------+                                                    |
        | ACK received                                                |
        v                                                            |
+---------------+                                                    |
|   LOG_POSE    |  Record: frame_id, theta, phi, z, timestamp,      |
|               |  motor_steps (absolute), calibrated_mm             |
+-------+-------+                                                    |
        | More poses?                                                 |
        +--- YES --------------------------------------------------->+
        |
        v NO
+---------------+
|   SCAN_DONE   |  Export poses.json / COLMAP / RealityCapture
+---------------+
```

**State Machine Properties:**
- Fully non-blocking — all waits use hardware timer callbacks, never `delay()`
- E-STOP can interrupt any state, triggering immediate motor disable + safe stop
- Each transition emits a telemetry event over the host protocol for real-time progress monitoring

---

## 5. Software Architecture — `packages/motion-core`

### 5.1 Layer Diagram

```
+------------------------------------------------------------------+
|  Application Layer  (apps/3d-scanner, apps/minimal-template)    |
|  Uses only: MotionPlanner API + Protocol API                     |
+------------------------------------------------------------------+
|  Protocol Layer     (CommandParser, JsonRpcDispatcher,           |
|                      TelemetryEmitter)                           |
+------------------------------------------------------------------+
|  Motion Planner     (MotionPlanner, StepGenerator,               |
|                      VelocityProfile, Axis, BacklashCompensator) |
+------------------------------------------------------------------+
|  HAL Interface      (IGpioHAL, ITimerHAL, IDriverHAL,           |
|                      IEndstopHAL, IEncoderHAL) -- pure virtual   |
+------------------+------------------+------------------+---------+
| RP2040 PIO HAL   | STM32 TIM+DMA    | ESP32-S3 MCPWM   | MockHAL |
+------------------+------------------+------------------+---------+
```

---

### 5.2 Hardware Abstraction Layer (HAL)

#### IGpioHAL — Digital I/O

```cpp
class IGpioHAL {
public:
    virtual ~IGpioHAL() = default;
    virtual void     pinMode(uint8_t pin, PinMode mode) = 0;
    virtual void     digitalWrite(uint8_t pin, PinState state) = 0;
    virtual PinState digitalRead(uint8_t pin) = 0;
    virtual void     attachInterrupt(uint8_t pin, IsrCallback cb, TriggerEdge edge) = 0;
};
```

#### ITimerHAL — Hardware Timers (non-blocking callbacks)

```cpp
class ITimerHAL {
public:
    virtual ~ITimerHAL() = default;
    // One-shot timer: fires callback once after delay_us microseconds
    virtual TimerHandle createOneShotTimer(uint32_t delay_us, TimerCallback cb) = 0;
    // Periodic timer: fires callback every period_us microseconds
    virtual TimerHandle createPeriodicTimer(uint32_t period_us, TimerCallback cb) = 0;
    virtual void cancelTimer(TimerHandle handle) = 0;
};
```

#### IDriverHAL — Stepper Driver (Step/Dir + UART config)

```cpp
class IDriverHAL {
public:
    virtual ~IDriverHAL() = default;

    // Low-level step generation (called by StepGenerator, NOT user code)
    virtual void setStepPin(uint8_t axis, PinState state) = 0;
    virtual void setDirPin(uint8_t axis, PinDirection dir) = 0;
    virtual void setEnablePin(uint8_t axis, bool enabled) = 0;

    // Smart driver UART/SPI config (optional — default = no-op for dumb drivers)
    virtual bool configureDriver(uint8_t axis, const DriverConfig& cfg) { return false; }
    virtual bool setRunCurrent(uint8_t axis, float amps)                { return false; }
    virtual bool setHoldCurrent(uint8_t axis, float amps)               { return false; }
    virtual bool readStallGuard(uint8_t axis, uint16_t& sg_value)       { return false; }
};
```

#### MockHAL — PC Simulation (for unit tests and kinematics preview)

The `MockHAL` implements all HAL interfaces in pure C++ for host PC. It:
- Simulates pin state changes with timestamps
- Counts step pulses and tracks simulated motor positions
- Fires timer callbacks synchronously in a simulation time-step loop
- Integrates with GoogleTest / Catch2 for assertion-based testing

```bash
# Build and run unit tests entirely on PC — no hardware required
cd packages/motion-core
cmake -B build -DMOTION_CORE_HAL=mock -DBUILD_TESTS=ON
cmake --build build
./build/tests/motion_core_tests
```

---

### 5.3 Motion Planner Engine

#### Core Data Structures

```cpp
struct AxisConfig {
    uint8_t  axis_id;
    float    steps_per_mm;        // or steps_per_degree for rotary axes
    float    max_speed_mm_s;
    float    max_accel_mm_s2;
    float    max_jerk_mm_s3;      // S-curve jerk limit
    float    backlash_mm;         // Backlash compensation distance
    float    travel_min_mm;
    float    travel_max_mm;
    bool     is_rotary;
};

struct MotionCommand {
    enum class Type { MOVE_ABS, MOVE_REL, HOME, JOG, SET_SPEED, WAIT };
    Type type;
    float targets[MAX_AXES];      // Target positions (mm or degrees)
    float feed_rate_mm_s;         // Override feed rate (0 = use axis max)
    ProfileType profile;          // TRAPEZOIDAL or SCURVE
};

struct MotionStatus {
    float   current_pos[MAX_AXES];
    float   current_vel[MAX_AXES];
    bool    is_moving;
    bool    is_homing;
    bool    estop_active;
    int32_t queue_depth;
    float   estimated_time_remaining_s;
};
```

#### Non-Blocking Step Generator Architecture

```
MotionPlanner::update() -- called from main loop (or Core 1 on RP2040)
|
+-- Dequeue next MotionCommand from command ring buffer
+-- Compute S-curve or trapezoidal profile segments
+-- Load step-interval array into StepGenerator

StepGenerator -- driven by ITimerHAL periodic ISR @ 100kHz to 1MHz
|
+-- Per ISR tick: decrement interval counter for each axis
+-- When counter reaches 0: fire STEP pulse via IDriverHAL
+-- Reload counter from pre-computed profile table
+-- On profile completion: invoke MotionPlanner.onStepComplete()
```

**Key invariant**: The step ISR never performs division, square root, or memory allocation. All profile math is pre-computed by `MotionPlanner` on the CPU core and stored as a fixed-point step-interval array consumed by the ISR.

#### S-Curve Phase Detection Logic

```
Input:  start_pos, end_pos, start_vel, end_vel, v_max, a_max, j_max
Output: array of step_interval_ticks[] for each of the 7 phases

Phase detection:
  1. Compute unconstrained peak velocity
  2. If peak_v > v_max: clamp and add constant-velocity phase (Phase 4)
  3. If distance too short: collapse Phase 2/6 (no constant accel plateau)
     -> "Triangular S-curve" (5-phase)
  4. If very short: collapse all to pure-jerk S (3-phase)
  5. Output: flat array of (step_count, isr_ticks_per_step) pairs
```

#### Backlash Compensator

```cpp
class BacklashCompensator {
    // Tracks last direction per axis.
    // On direction reversal: inserts a backlash_steps pre-move segment
    // at reduced speed before the actual commanded move.
    // Transparent to the application layer.
};
```

#### Homing Sequence (Sensorless with TMC2209 StallGuard)

```
1. Move axis at slow homing feed rate toward negative limit
2. Poll IDriverHAL::readStallGuard() on each step ISR
3. StallGuard value drops below threshold -> motor stalled against hard stop
4. Set software current position = 0 (home)
5. Move to safe park position (back-off distance)
6. Optional: re-home slowly for precision refinement
```

---

### 5.4 Host-Controller Protocol

#### Transport

| Interface | Use Case | Implementation |
|-----------|----------|----------------|
| **USB-Serial (CDC)** | Primary development & wired operation | TinyUSB CDC on RP2040 |
| **WebSocket over Wi-Fi** | Wireless dashboard operation | lwIP + Pico W CYW43 |

#### Command Format — JSON-RPC 2.0

```json
// Request
{ "jsonrpc": "2.0", "id": 1, "method": "axis.move",
  "params": { "axes": {"A": 90.0, "B": 45.0}, "profile": "scurve" } }

// Success Response
{ "jsonrpc": "2.0", "id": 1, "result": { "queued": true, "queue_depth": 1 } }

// Error Response
{ "jsonrpc": "2.0", "id": 1, "error": { "code": -32001, "message": "ESTOP active" } }
```

#### Command Set

| Method | Description |
|--------|-------------|
| `system.info` | MCU info, firmware version, axis count |
| `system.estop` | Immediate emergency stop (highest priority) |
| `system.reset` | Clear E-STOP, re-enable motors |
| `axis.home` | Execute homing sequence (one or all axes) |
| `axis.move` | Absolute move to target position(s) |
| `axis.jog` | Relative move (incremental jog) |
| `axis.set_config` | Update axis parameters (speed, accel, jerk, backlash) |
| `axis.get_status` | Poll current positions and motion status |
| `trigger.capture` | Fire camera trigger GPIO + await ACK |
| `scan.start` | Begin automated scan job with pose list |
| `scan.pause` | Pause scan at next safe pose |
| `scan.abort` | Abort scan, return to home |
| `scan.get_progress` | Query completed/total poses, ETA |

#### Telemetry — Async Push Events

```json
{ "event": "motion.status",
  "data": { "pos": {"A": 45.2, "B": 30.0, "Z": 0.0}, "moving": true } }

{ "event": "scan.pose_complete",
  "data": { "frame_id": 42, "pose": {"azimuth": 45.0, "elevation": 30.0},
             "timestamp_ms": 123456, "steps_abs": {"A": 2560, "B": 1706} } }

{ "event": "system.estop", "data": { "reason": "user_requested" } }
```

---

## 6. Reference Application — `apps/3d-scanner`

### Scan Path Generation

```cpp
ScanPathGenerator::fibonacci(n_poses=120, elevation_min=5, elevation_max=85)
  -> std::vector<ScanPose> {azimuth_deg, elevation_deg, focus_z_mm}

ScanPathGenerator::uniform_grid(az_steps=36, el_steps=12)
  -> std::vector<ScanPose>

ScanPathGenerator::from_csv(path)
  -> std::vector<ScanPose>  // User-defined waypoints
```

### Pose Export — Dataset Metadata Format

```json
{
  "frame_id": 42,
  "image_file": "frame_0042.jpg",
  "timestamp_iso": "2026-11-18T11:30:22.456Z",
  "pose": {
    "azimuth_deg": 128.57,
    "elevation_deg": 34.21,
    "focus_z_mm": 0.0
  },
  "motor_steps_abs": { "A": 18234, "B": 9821, "C": 0 },
  "motor_pos_calibrated": { "A_deg": 128.57, "B_deg": 34.21, "C_mm": 0.0 },
  "camera_extrinsic": {
    "quaternion_wxyz": [0.9239, 0.0, 0.3827, 0.0],
    "translation_xyz_m": [0.15, 0.0, 0.0]
  }
}
```

**Export Formats Supported:**

| Format | File | Use With |
|--------|------|----------|
| Native JSON | `poses.json` | Custom pipelines, host-sdk |
| COLMAP text | `cameras.txt` + `images.txt` | COLMAP MVS, Gaussian Splatting |
| RealityCapture | `rc_poses.csv` | RealityCapture reconstruction |
| Meshroom | `sfm_data.json` | AliceVision/Meshroom |

---

## 7. Extensibility & Cloner DX (Developer Experience)

**Goal**: Someone cloning for a completely different rig (CNC stage, syringe pump, camera slider) touches only 2 files and writes ~50 lines of code.

### What a Cloner Modifies

```
apps/minimal-template/
+-- include/
|   +-- MyBoardPinmap.hpp    <- 1. Map MCU pins to axis IDs
|   +-- MyKinematics.hpp     <- 2. Define axis configs (steps/mm, limits, etc.)
+-- src/
    +-- main.cpp             <- 3. Create MotionPlanner, attach HAL, start protocol
```

### What a Cloner Never Touches

```
packages/motion-core/        <- Entire motion engine: untouched
  +-- planner/               <- S-curve math: untouched
  +-- hal/                   <- Interfaces: untouched
  +-- protocol/              <- JSON-RPC: untouched
```

### Plugin/Adapter Boundaries

```cpp
// MyKinematics.hpp -- All the cloner needs to define
struct MyMachineConfig {
    static constexpr uint8_t NUM_AXES = 2;
    static constexpr AxisConfig AXES[] = {
        { .axis_id=0, .steps_per_mm=80.0f, .max_speed_mm_s=100.0f,
          .max_accel_mm_s2=500.0f, .max_jerk_mm_s3=2000.0f,
          .backlash_mm=0.05f, .travel_min_mm=0.0f, .travel_max_mm=200.0f },
        { .axis_id=1, .steps_per_mm=80.0f, /* ... */ }
    };
};
```

---

## 8. Implementation Phases & Milestones

```
Phase 1 -- DONE (this document) ----------------------------------------
  [x] Hardware stack research & recommendation
  [x] Repository structure design
  [x] HAL interface specification
  [x] Motion planner architecture
  [x] Protocol specification
  [x] 3D scanner kinematics & state machine

Phase 2 -- Core SDK (packages/motion-core) ------------------------------
  [ ] HAL interface headers (all platforms)
  [ ] MockHAL implementation (PC, unit-testable)
  [ ] S-curve + trapezoidal velocity profile math
  [ ] Non-blocking StepGenerator (ISR-driven)
  [ ] MotionPlanner (command queue, axis management)
  [ ] BacklashCompensator
  [ ] Homing sequence engine
  [ ] JSON-RPC command parser + dispatcher
  [ ] Telemetry emitter
  [ ] RP2040 PIO HAL implementation
  [ ] Unit test suite (GoogleTest, runs on PC via MockHAL)

Phase 3 -- 3D Scanner App (apps/3d-scanner) -----------------------------
  [ ] ScanPathGenerator (Fibonacci + uniform grid + CSV)
  [ ] CaptureEngine state machine
  [ ] PoseExporter (JSON + COLMAP + RealityCapture + Meshroom)
  [ ] Camera trigger GPIO + optocoupler circuit schematic
  [ ] Board pinmap for reference hardware

Phase 4 -- Host SDK & Repository Polish ---------------------------------
  [ ] Python SDK (host-sdk/python/motion_client/)
  [ ] CLI tool (mc_cli.py -- jog, home, scan, export)
  [ ] Web Dashboard (React / plain JS over WebSocket)
  [ ] CI: unit tests (GitHub Actions, all platforms)
  [ ] CI: firmware cross-compile (RP2040, STM32, ESP32-S3)
  [ ] CLONER_GUIDE.md ("Fork & build in 15 minutes")
  [ ] PROTOCOL_SPEC.md (full JSON-RPC reference)
  [ ] HARDWARE_BOM.md + wiring schematics (KiCad / Fritzing)
```

---

## 9. Open Questions for Project Owner

> **Please confirm or override these defaults before Phase 2 coding begins.**

| # | Question | Default Proposal |
|---|----------|-----------------|
| **Q1** | **Primary MCU target?** | RP2040 (Pico W) |
| **Q2** | **Secondary MCU support?** | STM32G4 + ESP32-S3 as HAL alternates |
| **Q3** | **Stepper driver default?** | TMC2209 (UART mode) |
| **Q4** | **Velocity profile default?** | S-Curve (7-phase jerk-limited) |
| **Q5** | **Number of scanner axes?** | 3 axes (Azimuth + Elevation + Focus Z) |
| **Q6** | **Homing method?** | Sensorless (TMC2209 StallGuard4) |
| **Q7** | **Host protocol?** | JSON-RPC 2.0 over USB-Serial + WebSocket |
| **Q8** | **Camera trigger method?** | GPIO optocoupler (hardware) + HTTP hook (software) |
| **Q9** | **Primary scan path algorithm?** | Fibonacci sphere (default) + uniform grid (option) |
| **Q10** | **Host OS target for Python SDK?** | Windows + Linux + macOS + Raspberry Pi OS |
| **Q11** | **C++ standard?** | C++17 |
| **Q12** | **Build system?** | CMake (with presets for each platform target) |
| **Q13** | **Testing framework?** | GoogleTest (via MockHAL, runs on PC) |
| **Q14** | **Do you have specific motor specs?** | NEMA 17, 1.8 deg/step (default) |
| **Q15** | **Physical scan radius / object size target?** | 50-200mm radius, objects up to 100mm across |

---

*Document generated by Phase 1 architectural research -- Stepper Motion Core Project*  
*Next step: Confirm Q1-Q15 above, then proceed to Phase 2 implementation.*
