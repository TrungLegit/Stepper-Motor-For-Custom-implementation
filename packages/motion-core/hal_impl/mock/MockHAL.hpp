#pragma once

#include "hal/IGpioHAL.hpp"
#include "hal/ITimerHAL.hpp"
#include "hal/IDriverHAL.hpp"
#include "hal/IEndstopHAL.hpp"
#include "hal/IEncoderHAL.hpp"

#include "MockGpioHAL.hpp"
#include "MockTimerHAL.hpp"
#include "MockDriverHAL.hpp"
#include "MockEndstopHAL.hpp"
#include "MockEncoderHAL.hpp"

#include <memory>
#include <mutex>

namespace motion_core {
namespace mock {

/**
 * @brief Unified Mock Hardware Abstraction Layer for Host PC Testing & Simulation.
 * Implements all HAL interfaces simultaneously:
 * - IGpioHAL
 * - ITimerHAL
 * - IDriverHAL
 * - IEndstopHAL
 * - IEncoderHAL
 *
 * Coordinates time advancement, timer callbacks, step counting, endstop triggers,
 * StallGuard simulation, and debouncing without hardware dependencies.
 */
class MockHAL : public IGpioHAL,
                public ITimerHAL,
                public IDriverHAL,
                public IEndstopHAL,
                public IEncoderHAL {
public:
    MockHAL();
    virtual ~MockHAL() = default;

    // =========================================================================
    // Time Simulation Control
    // =========================================================================

    /**
     * @brief Advance simulation clock by delta_us microseconds.
     * Fires scheduled timer callbacks, updates software debouncers, and synchronizes components.
     */
    void advanceTime(uint64_t delta_us);

    /**
     * @brief Alias for advanceTime
     */
    void stepTimeUs(uint64_t delta_us) {
        advanceTime(delta_us);
    }

    /**
     * @brief Reset all simulated hardware, registers, positions, and timers to initial state
     */
    void reset();

    // =========================================================================
    // Component Sub-HAL Accessors
    // =========================================================================
    MockGpioHAL&    gpio()    { return gpio_; }
    MockTimerHAL&   timer()   { return timer_; }
    MockDriverHAL&  driver()  { return driver_; }
    MockEndstopHAL& endstop() { return endstop_; }
    MockEncoderHAL& encoder() { return encoder_; }

    // =========================================================================
    // IGpioHAL Implementation
    // =========================================================================
    void pinMode(uint8_t pin, PinMode mode) override {
        gpio_.pinMode(pin, mode);
    }
    void digitalWrite(uint8_t pin, PinState state) override {
        gpio_.digitalWrite(pin, state);
    }
    PinState digitalRead(uint8_t pin) override {
        return gpio_.digitalRead(pin);
    }
    void togglePin(uint8_t pin) override {
        gpio_.togglePin(pin);
    }
    void attachInterrupt(uint8_t pin, GpioInterruptCallback cb, TriggerEdge edge) override {
        gpio_.attachInterrupt(pin, cb, edge);
    }
    void detachInterrupt(uint8_t pin) override {
        gpio_.detachInterrupt(pin);
    }

    // =========================================================================
    // ITimerHAL Implementation
    // =========================================================================
    uint64_t getMicros() const override {
        return timer_.getMicros();
    }
    uint32_t getMillis() const override {
        return timer_.getMillis();
    }
    TimerHandle createPeriodicTimer(uint32_t period_us, TimerCallback cb) override {
        return timer_.createPeriodicTimer(period_us, cb);
    }
    TimerHandle createOneShotTimer(uint32_t delay_us, TimerCallback cb) override {
        return timer_.createOneShotTimer(delay_us, cb);
    }
    bool cancelTimer(TimerHandle handle) override {
        return timer_.cancelTimer(handle);
    }
    bool setTimerPeriod(TimerHandle handle, uint32_t period_us) override {
        return timer_.setTimerPeriod(handle, period_us);
    }
    bool isTimerActive(TimerHandle handle) const override {
        return timer_.isTimerActive(handle);
    }
    void delayMicros(uint32_t us) override {
        advanceTime(us);
    }

    // =========================================================================
    // IDriverHAL Implementation
    // =========================================================================
    void setStepPin(uint8_t axis, PinState state) override {
        driver_.setStepPin(axis, state);
    }
    void setDirPin(uint8_t axis, Direction dir) override {
        driver_.setDirPin(axis, dir);
    }
    void setEnablePin(uint8_t axis, bool enabled) override {
        driver_.setEnablePin(axis, enabled);
    }
    void step(uint8_t axis) override {
        driver_.step(axis);
    }
    bool configureDriver(uint8_t axis, const DriverConfig& cfg) override {
        return driver_.configureDriver(axis, cfg);
    }
    bool setMicrosteps(uint8_t axis, uint16_t microsteps) override {
        return driver_.setMicrosteps(axis, microsteps);
    }
    bool setRunCurrent(uint8_t axis, float current_amps) override {
        return driver_.setRunCurrent(axis, current_amps);
    }
    bool setHoldCurrent(uint8_t axis, float current_amps) override {
        return driver_.setHoldCurrent(axis, current_amps);
    }
    bool setStealthChop(uint8_t axis, bool enable) override {
        return driver_.setStealthChop(axis, enable);
    }
    bool setStallGuardThreshold(uint8_t axis, uint8_t threshold) override {
        return driver_.setStallGuardThreshold(axis, threshold);
    }
    bool readStallGuard(uint8_t axis, uint16_t& sg_val) override {
        return driver_.readStallGuard(axis, sg_val);
    }
    bool writeRegister(uint8_t axis, uint8_t reg_addr, uint32_t value) override {
        return driver_.writeRegister(axis, reg_addr, value);
    }
    bool readRegister(uint8_t axis, uint8_t reg_addr, uint32_t& value) override {
        return driver_.readRegister(axis, reg_addr, value);
    }
    bool getDriverStatus(uint8_t axis, DriverStatus& status) override {
        return driver_.getDriverStatus(axis, status);
    }

    // =========================================================================
    // IEndstopHAL Implementation
    // =========================================================================
    bool configureEndstop(uint8_t axis, EndstopType type, uint32_t debounce_us = 5000, PullMode pull = PullMode::PULL_UP) override {
        return endstop_.configureEndstop(axis, type, debounce_us, pull);
    }
    bool configureEndstop(uint8_t axis, const EndstopConfig& config) override {
        return endstop_.configureEndstop(axis, config);
    }
    bool isTriggered(uint8_t axis) const override {
        return endstop_.isTriggered(axis);
    }
    PinState rawPinState(uint8_t axis) const override {
        return endstop_.rawPinState(axis);
    }
    void attachTriggerCallback(uint8_t axis, EndstopCallback cb) override {
        endstop_.attachTriggerCallback(axis, cb);
    }
    void detachTriggerCallback(uint8_t axis) override {
        endstop_.detachTriggerCallback(axis);
    }
    void setDebounceUs(uint8_t axis, uint32_t debounce_us) override {
        endstop_.setDebounceUs(axis, debounce_us);
    }
    void update(uint64_t current_time_us) override {
        endstop_.update(current_time_us);
    }

    // =========================================================================
    // IEncoderHAL Implementation
    // =========================================================================
    bool initEncoder(uint8_t axis) override {
        return encoder_.initEncoder(axis);
    }
    int64_t getCount(uint8_t axis) const override {
        return encoder_.getCount(axis);
    }
    void resetCount(uint8_t axis, int64_t count = 0) override {
        encoder_.resetCount(axis, count);
    }
    float getAngleDegrees(uint8_t axis) const override {
        return encoder_.getAngleDegrees(axis);
    }
    float getVelocity(uint8_t axis) const override {
        return encoder_.getVelocity(axis);
    }
    bool hasIndexTriggered(uint8_t axis) const override {
        return encoder_.hasIndexTriggered(axis);
    }
    void resetIndexTrigger(uint8_t axis) override {
        encoder_.resetIndexTrigger(axis);
    }
    bool isConnected(uint8_t axis) const override {
        return encoder_.isConnected(axis);
    }

private:
    MockGpioHAL    gpio_;
    MockTimerHAL   timer_;
    MockDriverHAL  driver_;
    MockEndstopHAL endstop_;
    MockEncoderHAL encoder_;
};

} // namespace mock
} // namespace motion_core
