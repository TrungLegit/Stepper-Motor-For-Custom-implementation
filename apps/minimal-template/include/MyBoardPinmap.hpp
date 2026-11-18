#pragma once

#include <cstdint>

/**
 * @file MyBoardPinmap.hpp
 * @brief Default reference pin mapping for ESP32-S3 DevKit with 3x TMC2209 stepper drivers
 *        and camera optocoupler shutter trigger.
 */
namespace board_pinmap {

// -------------------------------------------------------------------------
// Axis 0: Azimuth (Turntable)
// -------------------------------------------------------------------------
constexpr uint8_t AXIS_0_STEP_PIN      = 4;
constexpr uint8_t AXIS_0_DIR_PIN       = 5;
constexpr uint8_t AXIS_0_ENABLE_PIN    = 6;
constexpr uint8_t AXIS_0_ENDSTOP_PIN   = 7;  // Hall effect or microswitch

// -------------------------------------------------------------------------
// Axis 1: Elevation (Gimbal)
// -------------------------------------------------------------------------
constexpr uint8_t AXIS_1_STEP_PIN      = 8;
constexpr uint8_t AXIS_1_DIR_PIN       = 9;
constexpr uint8_t AXIS_1_ENABLE_PIN    = 10;
constexpr uint8_t AXIS_1_ENDSTOP_PIN   = 11; // Hall effect or microswitch

// -------------------------------------------------------------------------
// Axis 2: Focus Z (Linear stage)
// -------------------------------------------------------------------------
constexpr uint8_t AXIS_2_STEP_PIN      = 12;
constexpr uint8_t AXIS_2_DIR_PIN       = 13;
constexpr uint8_t AXIS_2_ENABLE_PIN    = 14;
constexpr uint8_t AXIS_2_ENDSTOP_PIN   = 17; // Hall effect or microswitch

// -------------------------------------------------------------------------
// TMC2209 Single-Wire UART Bus (Shared address bus with addressing 0, 1, 2)
// -------------------------------------------------------------------------
constexpr uint8_t TMC_UART_TX_PIN      = 15;
constexpr uint8_t TMC_UART_RX_PIN      = 16;

// -------------------------------------------------------------------------
// Camera & Auxiliary I/O
// -------------------------------------------------------------------------
constexpr uint8_t CAMERA_SHUTTER_PIN   = 18; // Optocoupler trigger for camera shutter
constexpr uint8_t STATUS_LED_PIN       = 38; // Onboard diagnostic LED

} // namespace board_pinmap
