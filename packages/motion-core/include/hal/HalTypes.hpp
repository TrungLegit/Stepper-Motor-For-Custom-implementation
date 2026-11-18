#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace motion_core {

// Logical pin output/input state
enum class PinState : uint8_t {
    LOW = 0,
    HIGH = 1
};

// Axis motion direction
enum class Direction : uint8_t {
    POSITIVE = 0,
    NEGATIVE = 1
};

// Direction aliases
constexpr Direction DIR_CW = Direction::POSITIVE;
constexpr Direction DIR_CCW = Direction::NEGATIVE;
constexpr Direction FORWARD = Direction::POSITIVE;
constexpr Direction REVERSE = Direction::NEGATIVE;

// GPIO mode configuration
enum class PinMode : uint8_t {
    INPUT = 0,
    OUTPUT = 1,
    INPUT_PULLUP = 2,
    INPUT_PULLDOWN = 3,
    OUTPUT_OPEN_DRAIN = 4
};

// GPIO interrupt trigger edge
enum class TriggerEdge : uint8_t {
    NONE = 0,
    RISING = 1,
    FALLING = 2,
    BOTH = 3
};

// Resistor pull mode
enum class PullMode : uint8_t {
    NONE = 0,
    PULL_UP = 1,
    PULL_DOWN = 2
};

// Endstop sensor technology and logic type
enum class EndstopType : uint8_t {
    SWITCH_NO = 0,        // Physical mechanical switch - Normally Open
    SWITCH_NC = 1,        // Physical mechanical switch - Normally Closed (fail-safe)
    HALL_ACTIVE_LOW = 2,  // Hall effect sensor (e.g. A3144), logic LOW when magnet detected
    HALL_ACTIVE_HIGH = 3  // Hall effect sensor, logic HIGH when magnet detected
};

// Maximum supported motion axes (default 4: X/Y/Z/E or Turntable/Elevation/Focus/Aux)
constexpr uint8_t MAX_AXES = 4;

// High-resolution hardware timer definitions
using TimerHandle = uint32_t;
constexpr TimerHandle INVALID_TIMER_HANDLE = 0;
using TimerCallback = std::function<void()>;

// GPIO and Endstop event callbacks
using GpioInterruptCallback = std::function<void(uint8_t pin, PinState state)>;
using EndstopCallback = std::function<void(uint8_t axis, bool triggered)>;

// Driver configuration parameters (TMC2209 and generic step/dir drivers)
struct DriverConfig {
    uint16_t microsteps = 16;           // 1, 2, 4, 8, 16, 32, 64, 128, 256
    bool     interpolate = true;         // TMC MicroPlyer 256-microstep interpolation
    float    run_current_amps = 0.8f;    // RMS run current in Amperes
    float    hold_current_amps = 0.4f;   // RMS hold current in Amperes
    uint8_t  hold_delay = 10;            // Delay before reducing to hold current (0-15)
    bool     stealthchop_enabled = true; // True = StealthChop2 (silent), False = SpreadCycle
    uint8_t  stallguard_threshold = 50;  // SGTHRS (0-255: 0=least sensitive, 255=most sensitive)
    bool     coolstep_enabled = false;   // TMC CoolStep dynamic current scaling
    uint32_t tcoolthrs = 0xFFFFF;        // CoolStep upper velocity threshold
    uint32_t thigh = 0;                  // Velocity threshold for StealthChop -> SpreadCycle switch
    bool     invert_dir = false;         // Invert DIR pin logic
    float    r_sense = 0.11f;            // Sense resistor in Ohms (0.11 typical for BTT TMC2209)
};

// TMC2209 Driver Diagnostic / Status feedback
struct DriverStatus {
    bool     ot = false;          // Overtemperature flag
    bool     otpw = false;        // Overtemperature pre-warning flag
    bool     s2ga = false;        // Short to ground coil A
    bool     s2gb = false;        // Short to ground coil B
    bool     ola = false;         // Open load coil A
    bool     olb = false;         // Open load coil B
    bool     standstill = false;  // Motor at standstill
    uint16_t sg_result = 0;       // StallGuard4 current measurement value
    bool     driver_error = false;// Generic driver fault flag
};

// Backwards-compatibility and sub-namespace alias
namespace hal {
    using PinState = motion_core::PinState;
    using Direction = motion_core::Direction;
    using PinMode = motion_core::PinMode;
    using TriggerEdge = motion_core::TriggerEdge;
    using PullMode = motion_core::PullMode;
    using EndstopType = motion_core::EndstopType;
    using TimerHandle = motion_core::TimerHandle;
    constexpr TimerHandle INVALID_TIMER_HANDLE = motion_core::INVALID_TIMER_HANDLE;
    using TimerCallback = motion_core::TimerCallback;
    using GpioInterruptCallback = motion_core::GpioInterruptCallback;
    using EndstopCallback = motion_core::EndstopCallback;
    using DriverConfig = motion_core::DriverConfig;
    using DriverStatus = motion_core::DriverStatus;
    constexpr uint8_t MAX_AXES = motion_core::MAX_AXES;
}

} // namespace motion_core
