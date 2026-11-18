# CLONER_GUIDE.md — How to Fork & Build Your Own Mechanical Rig in 15 Minutes

The `stepper-motion-core` framework is designed from the ground up for clean mechatronic reusability. Whether you are building an automated syringe pump, a 2-axis camera slider, a custom CNC stage, or an automated test jig, you **never** need to touch the motion planner, S-curve jerk math, or step interrupt timing engine.

---

## 1. The 3-File Cloner Contract

To create a brand-new mechanical machine, you only need to create or edit **three files** inside your application folder (starting from `apps/minimal-template/`):

```
apps/my-custom-rig/
├── include/
│   ├── MyBoardPinmap.hpp      ← 1. Hardware pin assignment for your MCU
│   └── MyKinematics.hpp       ← 2. Steps/mm, speeds, acceleration, backlash & limits
└── src/
    └── main.cpp               ← 3. Firmware entry point: initialize HAL, bind planner & run
```

---

## 2. Step-by-Step 15-Minute Tutorial

### Step 1: Copy the Template (1 Minute)
```bash
cp -r apps/minimal-template apps/my-syringe-pump
```

### Step 2: Define Your Board Pinmap (4 Minutes)
Edit `apps/my-syringe-pump/include/MyBoardPinmap.hpp`:
```cpp
#pragma once
#include <cstdint>

namespace my_rig {
    // Motor 0 (Lead screw drive)
    constexpr uint8_t STEP_PIN_PUMP     = 4;
    constexpr uint8_t DIR_PIN_PUMP      = 5;
    constexpr uint8_t EN_PIN_PUMP       = 6;

    // Endstops (Mechanical Microswitch or A3144 Hall Effect Sensor)
    constexpr uint8_t ENDSTOP_PIN_PUMP  = 7;

    // Optional Safety E-STOP Button
    constexpr uint8_t ESTOP_PIN         = 1;
}
```

### Step 3: Configure Your Axis Kinematics (5 Minutes)
Edit `apps/my-syringe-pump/include/MyKinematics.hpp`:
```cpp
#pragma once
#include "planner/Axis.hpp"

namespace my_rig {
    using namespace motion_core;

    inline AxisConfig createPumpAxisConfig() {
        AxisConfig cfg{};
        cfg.axis_id         = 0;
        cfg.name            = "PUMP";
        // Example: Lead screw pitch = 1.0mm, 200 full steps, 1/16 microstepping
        cfg.steps_per_mm    = 3200.0f;  // 200 * 16 / 1.0
        cfg.max_speed_mm_s  = 10.0f;    // 10 mm/s infusion rate limit
        cfg.max_accel_mm_s2 = 50.0f;    // Smooth acceleration
        cfg.max_jerk_mm_s3  = 200.0f;   // S-curve jerk limit for smooth fluid delivery
        cfg.backlash_mm     = 0.02f;    // Measured lead screw backlash
        cfg.travel_min_mm   = 0.0f;
        cfg.travel_max_mm   = 60.0f;    // Syringe barrel length
        cfg.is_rotary       = false;
        return cfg;
    }
}
```

### Step 4: Build & Run (5 Minutes)
Configure and build with CMake:
```bash
# Build on PC with MockHAL simulation for testing:
cmake -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Or flash directly to ESP32-S3 via ESP-IDF / PlatformIO
idf.py -p COM3 flash monitor
```

---

## 3. Host Control via Python SDK

Once your firmware is running, control it immediately via the included Python SDK without writing custom communication protocols:

```python
from motion_client import ScannerController

# Connect over USB-Serial or Wi-Fi WebSocket
client = ScannerController(port="COM3", baudrate=115200)
client.connect()

# Home the machine
client.home()

# Non-blocking move with S-curve acceleration profile
client.move_to({"PUMP": 25.0}, feedrate=5.0)

# Instant emergency stop
client.emergency_stop()
```

Or control via the command line:
```bash
python host-sdk/python/cli/mc_cli.py --port COM3 home
python host-sdk/python/cli/mc_cli.py --port COM3 move --axis PUMP --pos 25.0
```
