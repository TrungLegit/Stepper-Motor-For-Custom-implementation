#pragma once

#include <cstdint>
#include "hal/HalTypes.hpp"
#include "../../packages/motion-core/hal_impl/esp32s3/ESP32S3_HAL.hpp"

/**
 * @file pinmap_esp32s3.hpp
 * @brief Reference Hardware Board Pinout & Wiring Specifications for High-Detail 3D Scanner
 *        Target MCU: ESP32-S3 (Xtensa Dual-Core 240MHz, DevKitC-1 / custom scanner board)
 *
 * =========================================================================================
 * Hardware Overview & Pin Assignments
 * =========================================================================================
 *
 * Axis 0: Azimuth (Turntable continuous / index rotation)
 *   - STEP:     GPIO 4   (High-speed digital output to STEP on Driver A)
 *   - DIR:      GPIO 5   (Digital output to DIR on Driver A)
 *   - ENABLE:   GPIO 6   (Active LOW driver enable to EN on Driver A)
 *   - ENDSTOP:  GPIO 7   (A3144 Hall sensor / microswitch for 0 deg index reference)
 *   - TMC ADDR: 0        (MS1=GND, MS2=GND for UART slave address 0)
 *
 * Axis 1: Elevation (Tilt gimbal camera swing 0 - 90 deg)
 *   - STEP:     GPIO 8   (High-speed digital output to STEP on Driver B)
 *   - DIR:      GPIO 9   (Digital output to DIR on Driver B)
 *   - ENABLE:   GPIO 10  (Active LOW driver enable to EN on Driver B)
 *   - ENDSTOP:  GPIO 11  (A3144 Hall sensor / microswitch for 0 deg horizon reference)
 *   - TMC ADDR: 1        (MS1=VCC_IO, MS2=GND for UART slave address 1)
 *
 * Axis 2: Focus Z (Macro focus rail / camera focal distance stage)
 *   - STEP:     GPIO 12  (High-speed digital output to STEP on Driver C)
 *   - DIR:      GPIO 13  (Digital output to DIR on Driver C)
 *   - ENABLE:   GPIO 14  (Active LOW driver enable to EN on Driver C)
 *   - ENDSTOP:  GPIO 17  (A3144 Hall sensor / microswitch for Z minimum limit)
 *   - TMC ADDR: 2        (MS1=GND, MS2=VCC_IO for UART slave address 2)
 *
 * TMC2209 Smart Driver Single-Wire Half-Duplex UART Bus:
 *   - TX:       GPIO 15  (ESP32-S3 UART1 TX)
 *   - RX:       GPIO 16  (ESP32-S3 UART1 RX)
 *   - Baud:     115200 bps
 *   - Bus Note: Connect a 1k Ohm series resistor between TX and RX, with RX directly
 *               connected to the shared PDN_UART line of all three TMC2209 drivers.
 *
 * Camera Shutter & Flash Triggering Interface:
 *   - SHUTTER:  GPIO 18  (Drives PC817 optocoupler through a 330 Ohm resistor to trigger
 *                         camera shutter via 2.5mm / 3.5mm remote terminal)
 *   - FOCUS:    GPIO 19  (Optional camera focus pre-wake trigger)
 *   - FLASH/ACK:GPIO 21  (Hot-shoe sync or camera exposure status input ACK)
 *
 * Safety & Diagnostics:
 *   - ESTOP_PIN:GPIO 1   (Hardware Emergency Stop button, active LOW, internal/external pull-up.
 *                         Instantly cuts stepper motor enable and halts all motion)
 *   - STATUS_LED:GPIO 38 (Onboard status / diagnostic indicator LED)
 *
 * =========================================================================================
 * Wiring & Compatibility Notes:
 * =========================================================================================
 * 1. TMC2209 UART vs. Generic Step/Dir Drivers (A4988 / DRV8825):
 *    - TMC2209: Configure MS1/MS2 jumpers for addresses 0, 1, 2. Enable single-wire UART.
 *    - A4988 / DRV8825: Set uart_pins.enabled = false. Step, dir, and enable operate directly
 *      without software changes. Set microstepping jumpers (MS1, MS2, MS3) on the board.
 * 2. Hall Effect Sensor (A3144) Wiring:
 *    - Pin 1 (VCC) -> 3.3V or 5V (A3144 operates 4.5-24V; for 3.3V logic use AH3362 or 3.3V-rated Hall).
 *    - Pin 2 (GND) -> System Ground.
 *    - Pin 3 (OUT) -> Connect to Endstop GPIO (7, 11, or 17). Open-collector output requires
 *                     a 10k Ohm pull-up resistor to 3.3V (or enable ESP32 internal pullup).
 *                     When magnet South pole is proximate, OUT goes LOW.
 * 3. Normally Closed (NC) Physical Switch Wiring:
 *    - COM -> GND.
 *    - NC -> Endstop GPIO with internal pull-up enabled.
 *    - Wire disconnection or cut opens the circuit, immediately triggering limit state (fail-safe).
 */

namespace scanner {
namespace board {

// -----------------------------------------------------------------------------
// Axis 0: Azimuth Turntable
// -----------------------------------------------------------------------------
constexpr int8_t AZIMUTH_STEP_PIN     = 4;
constexpr int8_t AZIMUTH_DIR_PIN      = 5;
constexpr int8_t AZIMUTH_ENABLE_PIN   = 6;
constexpr int8_t AZIMUTH_ENDSTOP_PIN  = 7;
constexpr uint8_t AZIMUTH_TMC_ADDR    = 0;

// -----------------------------------------------------------------------------
// Axis 1: Elevation Gimbal
// -----------------------------------------------------------------------------
constexpr int8_t ELEVATION_STEP_PIN   = 8;
constexpr int8_t ELEVATION_DIR_PIN    = 9;
constexpr int8_t ELEVATION_ENABLE_PIN = 10;
constexpr int8_t ELEVATION_ENDSTOP_PIN= 11;
constexpr uint8_t ELEVATION_TMC_ADDR  = 1;

// -----------------------------------------------------------------------------
// Axis 2: Focus Z Rail
// -----------------------------------------------------------------------------
constexpr int8_t FOCUS_Z_STEP_PIN     = 12;
constexpr int8_t FOCUS_Z_DIR_PIN      = 13;
constexpr int8_t FOCUS_Z_ENABLE_PIN   = 14;
constexpr int8_t FOCUS_Z_ENDSTOP_PIN  = 17;
constexpr uint8_t FOCUS_Z_TMC_ADDR    = 2;

// -----------------------------------------------------------------------------
// TMC2209 Single-Wire UART Bus
// -----------------------------------------------------------------------------
constexpr uint8_t TMC_UART_NUM        = 1;
constexpr int8_t  TMC_UART_TX_PIN     = 15;
constexpr int8_t  TMC_UART_RX_PIN     = 16;
constexpr uint32_t TMC_UART_BAUD_RATE = 115200;

// -----------------------------------------------------------------------------
// Camera Trigger & Feedback
// -----------------------------------------------------------------------------
constexpr int8_t CAMERA_SHUTTER_PIN   = 18; // Shutter release (PC817 optocoupler)
constexpr int8_t CAMERA_FOCUS_PIN     = 19; // Half-press focus release
constexpr int8_t CAMERA_ACK_INPUT_PIN = 21; // Exposure active / flash sync input

// -----------------------------------------------------------------------------
// Safety & Status
// -----------------------------------------------------------------------------
constexpr int8_t ESTOP_HARDWARE_PIN   = 1;  // Normally closed E-Stop switch to GND
constexpr int8_t STATUS_LED_PIN       = 38; // System status indicator LED

/**
 * @brief Populate ESP32S3 AxisPins array with scanner reference board pinout
 */
inline void populateScannerAxisPins(motion_core::esp32s3::AxisPins pins[motion_core::MAX_AXES]) {
    // Axis 0: Azimuth
    pins[0].step_pin = AZIMUTH_STEP_PIN;
    pins[0].dir_pin = AZIMUTH_DIR_PIN;
    pins[0].enable_pin = AZIMUTH_ENABLE_PIN;
    pins[0].endstop_pin = AZIMUTH_ENDSTOP_PIN;
    pins[0].tmc_slave_addr = AZIMUTH_TMC_ADDR;

    // Axis 1: Elevation
    pins[1].step_pin = ELEVATION_STEP_PIN;
    pins[1].dir_pin = ELEVATION_DIR_PIN;
    pins[1].enable_pin = ELEVATION_ENABLE_PIN;
    pins[1].endstop_pin = ELEVATION_ENDSTOP_PIN;
    pins[1].tmc_slave_addr = ELEVATION_TMC_ADDR;

    // Axis 2: Focus Z
    pins[2].step_pin = FOCUS_Z_STEP_PIN;
    pins[2].dir_pin = FOCUS_Z_DIR_PIN;
    pins[2].enable_pin = FOCUS_Z_ENABLE_PIN;
    pins[2].endstop_pin = FOCUS_Z_ENDSTOP_PIN;
    pins[2].tmc_slave_addr = FOCUS_Z_TMC_ADDR;

    // Axis 3: Unused / Aux
    pins[3].step_pin = -1;
    pins[3].dir_pin = -1;
    pins[3].enable_pin = -1;
    pins[3].endstop_pin = -1;
    pins[3].tmc_slave_addr = 3;
}

/**
 * @brief Get scanner UART configuration (or generic mode if use_tmc_uart is false)
 */
inline motion_core::esp32s3::UartPins getScannerUartConfig(bool use_tmc_uart = true) {
    motion_core::esp32s3::UartPins uart;
    uart.uart_num = TMC_UART_NUM;
    uart.tx_pin = TMC_UART_TX_PIN;
    uart.rx_pin = TMC_UART_RX_PIN;
    uart.baud_rate = TMC_UART_BAUD_RATE;
    uart.enabled = use_tmc_uart;
    return uart;
}

} // namespace board
} // namespace scanner
