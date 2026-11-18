#pragma once

#include "HalTypes.hpp"
#include "TMC2209Registers.hpp"
#include <cstdint>

namespace motion_core {

/**
 * @brief Pure virtual Hardware Abstraction Layer for stepper motor drivers.
 * Provides high-speed step/dir pin toggling for the step generator ISR,
 * as well as smart TMC2209 UART register configuration and StallGuard4 diagnostics.
 */
class IDriverHAL {
public:
    virtual ~IDriverHAL() = default;

    // --- High-Speed Step / Dir / Enable (Invoked in Step ISR) ---

    /**
     * @brief Set logical state of STEP pin for specified axis
     */
    virtual void setStepPin(uint8_t axis, PinState state) = 0;

    /**
     * @brief Set motion direction of DIR pin for specified axis
     */
    virtual void setDirPin(uint8_t axis, Direction dir) = 0;

    /**
     * @brief Enable or disable power to motor driver stage (typically active LOW on TMC)
     */
    virtual void setEnablePin(uint8_t axis, bool enabled) = 0;

    /**
     * @brief Generates a fast single step pulse on STEP pin
     */
    virtual void step(uint8_t axis) = 0;

    // --- TMC2209 Smart Configuration & Current Control ---

    /**
     * @brief Applies full DriverConfig parameters to driver
     */
    virtual bool configureDriver(uint8_t axis, const DriverConfig& cfg) = 0;

    /**
     * @brief Configures microstepping resolution (1, 2, 4, 8, 16, 32, 64, 128, 256)
     */
    virtual bool setMicrosteps(uint8_t axis, uint16_t microsteps) = 0;

    /**
     * @brief Set RMS run current in Amperes
     */
    virtual bool setRunCurrent(uint8_t axis, float current_amps) = 0;

    /**
     * @brief Set RMS hold current in Amperes
     */
    virtual bool setHoldCurrent(uint8_t axis, float current_amps) = 0;

    /**
     * @brief Enable/disable StealthChop2 (silent mode vs SpreadCycle torque mode)
     */
    virtual bool setStealthChop(uint8_t axis, bool enable) = 0;

    /**
     * @brief Set StallGuard4 threshold (SGTHRS: 0-255) for sensorless homing
     */
    virtual bool setStallGuardThreshold(uint8_t axis, uint8_t threshold) = 0;

    /**
     * @brief Read current StallGuard measurement value (SG_RESULT) from TMC2209
     * @param axis Target motor axis
     * @param sg_val Out parameter containing 10-bit stall measurement (0-1023)
     * @return true if UART communication succeeded
     */
    virtual bool readStallGuard(uint8_t axis, uint16_t& sg_val) = 0;

    // --- Low-Level TMC2209 Register Access ---

    /**
     * @brief Write 32-bit register value over TMC UART
     */
    virtual bool writeRegister(uint8_t axis, uint8_t reg_addr, uint32_t value) = 0;

    /**
     * @brief Read 32-bit register value over TMC UART
     */
    virtual bool readRegister(uint8_t axis, uint8_t reg_addr, uint32_t& value) = 0;

    /**
     * @brief Query driver status and diagnostic flags
     */
    virtual bool getDriverStatus(uint8_t axis, DriverStatus& status) = 0;
};

namespace hal {
    using IDriverHAL = motion_core::IDriverHAL;
}

} // namespace motion_core
