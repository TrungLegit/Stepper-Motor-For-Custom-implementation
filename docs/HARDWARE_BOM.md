# HARDWARE_BOM.md — Bill of Materials & Wiring Guide

Reference hardware configuration for the **High-Detail 3D Scanner** powered by `ESP32-S3` and `TMC2209` (or `A4988`).

---

## 1. Bill of Materials (BOM)

| Category | Component | Specification | Qty | Approx Cost | Notes |
|---|---|---|:---:|---|---|
| **MCU** | ESP32-S3-DevKitC-1 | ESP32-S3-WROOM-1 (N16R8: 16MB Flash, 8MB PSRAM) | 1 | ~220,000 VND | Dual-Core 240MHz, USB-CDC + Wi-Fi |
| **Motors** | NEMA 17 Stepper Motors | 1.8° step angle, 40-59 N·cm holding torque | 3 | ~120,000 VND / ea | Axis 0 (Turntable), Axis 1 (Gimbal), Axis 2 (Z) |
| **Drivers** | BigTreeTech TMC2209 V1.2 (or A4988) | StealthChop2, UART single-wire, 2.0A RMS | 3 | ~85,000 VND / ea | Silent microstepping, CoolStep thermal control |
| **Power Supply**| 24V / 5A DC Regulated PSU | 120W switching power supply | 1 | ~180,000 VND | Stepper motor drive power rail |
| **Logic Step-down**| MP1584EN or LM2596 Buck | 24V to 5V / 3A step-down converter | 1 | ~25,000 VND | Powers ESP32-S3 and logic rail |
| **Sensors** | A3144 Hall Effect Sensors (or microswitches) | Unipolar Hall magnetic switch, Open-Drain output | 3 | ~8,000 VND / ea | Position homing (South pole magnetic trigger) |
| **Magnets** | Neodymium Disc Magnets | 4mm x 2mm or 5mm x 2mm N35/N52 | 6 | ~15,000 VND | Glued to turntable plate & gimbal arm |
| **Bearings** | 608ZZ Ball Bearings | 8mm ID x 22mm OD x 7mm Width | 4 | ~10,000 VND / ea | Smooth turntable & elevation pivot |
| **Optocoupler** | PC817 Phototransistor Module | 4-channel or 1-channel optoisolator | 1 | ~15,000 VND | Shutter trigger galvanic isolation |
| **Jack** | 2.5mm / 3.5mm Stereo Audio Jack | TRS / TRRS camera shutter remote connector | 1 | ~10,000 VND | Connects to DSLR / Mirrorless remote port |

---

## 2. Complete ESP32-S3 Pinout Mapping

```
                               ESP32-S3 Pinout
                       ┌─────────────────────────┐
                       │          [USB]          │
         E-STOP Button │ GPIO 1           GPIO 4 │ Motor 0 (Azimuth) STEP
   Motor 0 (Azim) DIR  │ GPIO 5           GPIO 6 │ Motor 0 (Azim) ENABLE
  Motor 0 Endstop/Hall │ GPIO 7           GPIO 8 │ Motor 1 (Elevation) STEP
   Motor 1 (Elev) DIR  │ GPIO 9          GPIO 10 │ Motor 1 (Elev) ENABLE
  Motor 1 Endstop/Hall │ GPIO 11         GPIO 12 │ Motor 2 (Focus Z) STEP
   Motor 2 (Z) DIR     │ GPIO 13         GPIO 14 │ Motor 2 (Z) ENABLE
  TMC2209 UART1 TX     │ GPIO 15         GPIO 16 │ TMC2209 UART1 RX (1k resistor)
  Motor 2 Endstop/Hall │ GPIO 17         GPIO 18 │ Camera Shutter (Optocoupler)
  Camera Pre-Focus     │ GPIO 19         GPIO 21 │ Camera Flash Sync / ACK
  Status LED           │ GPIO 38             GND │ Ground
                       └─────────────────────────┘
```

---

## 3. Wiring Guidelines

### A. Stepper Driver Wiring
* **TMC2209 Single-Wire UART Bus**: Connect GPIO 15 (TX) and GPIO 16 (RX) with a $1\text{k}\Omega$ series resistor between TX and RX. Connect RX directly to the `PDN_UART` pin of all 3 TMC2209 drivers. Set MS1/MS2 jumpers for slave addresses:
  * Motor 0 (Azimuth): Address `0` (MS1=GND, MS2=GND)
  * Motor 1 (Elevation): Address `1` (MS1=VIO, MS2=GND)
  * Motor 2 (Focus Z): Address `2` (MS1=GND, MS2=VIO)
* **A4988 Generic Mode**: If using A4988, connect STEP, DIR, and ENABLE pins directly. Leave UART pins unconnected.

### B. Endstop & Hall Effect Sensor Wiring
* **A3144 Hall Sensors**:
  * Pin 1 (VCC): Connect to 3.3V
  * Pin 2 (GND): Connect to GND
  * Pin 3 (OUT): Connect to GPIO (7, 11, or 17). Add a $10\text{k}\Omega$ pull-up resistor to 3.3V if internal pull-up is not used.
  * *Trigger logic*: Active LOW when a magnet's South pole approaches within 2–5mm.
* **Mechanical Microswitches**:
  * Wire Common (C) to GND, Normally Closed (NC) to the GPIO pin.

### C. Camera Shutter Optocoupler (PC817)
* Anode: Connect to GPIO 18 via a $330\Omega$ current-limiting resistor.
* Cathode: Connect to ESP32 GND.
* Collector & Emitter: Connect to Camera Shutter Pin and Camera GND on the 2.5mm remote cable. Galvanic isolation guarantees zero risk to your camera circuitry.
