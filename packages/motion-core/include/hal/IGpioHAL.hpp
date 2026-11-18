#pragma once

#include "HalTypes.hpp"
#include <cstdint>

namespace motion_core {

/**
 * @brief Pure virtual Hardware Abstraction Layer for general-purpose digital I/O.
 * Provides zero-overhead pin mode configuration, direct pin manipulation, and interrupt attachment.
 */
class IGpioHAL {
public:
    virtual ~IGpioHAL() = default;

    /**
     * @brief Configure mode of specified GPIO pin (INPUT, OUTPUT, PULLUP, PULLDOWN, OPEN_DRAIN)
     */
    virtual void pinMode(uint8_t pin, PinMode mode) = 0;

    /**
     * @brief Write logical level (HIGH or LOW) to output pin
     */
    virtual void digitalWrite(uint8_t pin, PinState state) = 0;

    /**
     * @brief Read current logical level from input pin
     */
    virtual PinState digitalRead(uint8_t pin) = 0;

    /**
     * @brief Toggle state of specified output pin
     */
    virtual void togglePin(uint8_t pin) = 0;

    /**
     * @brief Attach edge-triggered hardware interrupt callback
     */
    virtual void attachInterrupt(uint8_t pin, GpioInterruptCallback cb, TriggerEdge edge) = 0;

    /**
     * @brief Detach hardware interrupt from pin
     */
    virtual void detachInterrupt(uint8_t pin) = 0;
};

namespace hal {
    using IGpioHAL = motion_core::IGpioHAL;
}

} // namespace motion_core
