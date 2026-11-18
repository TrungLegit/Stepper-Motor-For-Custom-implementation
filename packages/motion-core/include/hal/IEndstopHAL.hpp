#pragma once

#include "HalTypes.hpp"
#include <cstdint>
#include <functional>

namespace motion_core {

// Configuration parameters for an endstop/limit sensor
struct EndstopConfig {
    uint8_t     pin = 0;
    EndstopType type = EndstopType::SWITCH_NC;
    PullMode    pull = PullMode::PULL_UP;
    uint32_t    debounce_us = 5000; // Software debounce filter in microseconds (e.g. 5ms for mechanical, 0-100us for Hall)
};

/**
 * @brief Pure virtual Hardware Abstraction Layer for Endstop and Limit sensors.
 * Supports:
 * - Physical switches: Normally Open (NO) and Normally Closed (NC) with configurable pull-up/down.
 * - Hall effect sensors: Active LOW (e.g. A3144 open-drain with pull-up) and Active HIGH.
 * - Configurable software debounce filter (debounce_us) per pin/axis.
 * - Edge-triggered event callbacks and non-blocking polling queries.
 */
class IEndstopHAL {
public:
    virtual ~IEndstopHAL() = default;

    /**
     * @brief Configure endstop sensor parameters for an axis
     * @param axis Axis index (0 to MAX_AXES-1)
     * @param type Sensor technology and logic (SWITCH_NO, SWITCH_NC, HALL_ACTIVE_LOW, HALL_ACTIVE_HIGH)
     * @param debounce_us Software debounce filter window in microseconds
     * @param pull Resistor pull mode (PULL_UP, PULL_DOWN, NONE)
     * @return true if configuration was successfully applied
     */
    virtual bool configureEndstop(uint8_t axis, EndstopType type, uint32_t debounce_us = 5000, PullMode pull = PullMode::PULL_UP) = 0;

    /**
     * @brief Configure endstop using an EndstopConfig struct
     */
    virtual bool configureEndstop(uint8_t axis, const EndstopConfig& config) = 0;

    /**
     * @brief Query whether the endstop is currently triggered (logical hit), filtered through software debounce
     * @param axis Axis index
     * @return true if limit / home position is triggered
     */
    virtual bool isTriggered(uint8_t axis) const = 0;

    /**
     * @brief Read the instantaneous unfiltered raw physical pin state
     */
    virtual PinState rawPinState(uint8_t axis) const = 0;

    /**
     * @brief Attach an interrupt or event callback triggered on change of debounced limit state
     * @param axis Axis index
     * @param cb Callback function receiving (uint8_t axis, bool triggered)
     */
    virtual void attachTriggerCallback(uint8_t axis, EndstopCallback cb) = 0;

    /**
     * @brief Detach the trigger callback from the specified axis
     */
    virtual void detachTriggerCallback(uint8_t axis) = 0;

    /**
     * @brief Dynamically adjust the software debounce filter duration
     */
    virtual void setDebounceUs(uint8_t axis, uint32_t debounce_us) = 0;

    /**
     * @brief Periodic or step-update call to advance the debounce state machine
     * @param current_time_us Current system time in microseconds
     */
    virtual void update(uint64_t current_time_us) = 0;
};

namespace hal {
    using IEndstopHAL = motion_core::IEndstopHAL;
    using EndstopConfig = motion_core::EndstopConfig;
}

} // namespace motion_core
