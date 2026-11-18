#pragma once

#include "HalTypes.hpp"
#include <cstdint>

namespace motion_core {

/**
 * @brief Pure virtual Hardware Abstraction Layer for Rotary and Linear Encoders.
 * Supports incremental quadrature encoders and absolute magnetic encoders (e.g. AS5048A)
 * for closed-loop verification and turntable calibration.
 */
class IEncoderHAL {
public:
    virtual ~IEncoderHAL() = default;

    /**
     * @brief Initialize encoder hardware for the specified axis
     */
    virtual bool initEncoder(uint8_t axis) = 0;

    /**
     * @brief Get cumulative encoder counts (ticks)
     */
    virtual int64_t getCount(uint8_t axis) const = 0;

    /**
     * @brief Reset encoder count to a specified value (default 0)
     */
    virtual void resetCount(uint8_t axis, int64_t count = 0) = 0;

    /**
     * @brief Get absolute or cumulative angular position in degrees
     */
    virtual float getAngleDegrees(uint8_t axis) const = 0;

    /**
     * @brief Get instantaneous angular velocity in degrees per second
     */
    virtual float getVelocity(uint8_t axis) const = 0;

    /**
     * @brief Check whether index/Z pulse has triggered
     */
    virtual bool hasIndexTriggered(uint8_t axis) const = 0;

    /**
     * @brief Reset the index pulse trigger flag
     */
    virtual void resetIndexTrigger(uint8_t axis) = 0;

    /**
     * @brief Query whether encoder is connected and communicating
     */
    virtual bool isConnected(uint8_t axis) const = 0;
};

namespace hal {
    using IEncoderHAL = motion_core::IEncoderHAL;
}

} // namespace motion_core
