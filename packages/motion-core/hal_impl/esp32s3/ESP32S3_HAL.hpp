#pragma once

#include "hal/IGpioHAL.hpp"
#include "hal/ITimerHAL.hpp"
#include "hal/IDriverHAL.hpp"
#include "hal/IEndstopHAL.hpp"
#include "hal/TMC2209Registers.hpp"
#include "ESP32S3_Timer.hpp"
#include "../../src/hal/EndstopDebouncer.hpp"

#include <vector>
#include <mutex>
#include <functional>

namespace motion_core {
namespace esp32s3 {

// Pin mapping definitions for an axis on ESP32-S3
struct AxisPins {
    int8_t step_pin = -1;
    int8_t dir_pin = -1;
    int8_t enable_pin = -1;
    int8_t endstop_pin = -1;
    uint8_t tmc_slave_addr = 0; // 0 to 3
};

// UART configuration for TMC2209 bus on ESP32-S3
struct UartPins {
    uint8_t  uart_num = 1;     // UART1 or UART2
    int8_t   tx_pin = 17;
    int8_t   rx_pin = 18;
    uint32_t baud_rate = 115200;
};

/**
 * @brief Concrete Hardware Abstraction Layer for ESP32-S3 (Xtensa Dual-Core 240MHz).
 * Features:
 * - High-speed GPIO with optional single-cycle register direct access.
 * - Hardware timer powered by ESP-IDF esp_timer.
 * - Hardware UART with half-duplex single-wire TMC2209 communication and CRC8 validation.
 * - Configurable endstop debounce filter for mechanical switches (NO/NC) and Hall sensors (A3144).
 */
class ESP32S3_HAL : public IGpioHAL,
                    public ITimerHAL,
                    public IDriverHAL,
                    public IEndstopHAL {
public:
    ESP32S3_HAL(const AxisPins pins[MAX_AXES], const UartPins& uart_cfg = UartPins{});
    virtual ~ESP32S3_HAL();

    // Initialize all GPIOs, UART, and timer peripheral
    bool init();

    // =========================================================================
    // IGpioHAL
    // =========================================================================
    void pinMode(uint8_t pin, PinMode mode) override;
    void digitalWrite(uint8_t pin, PinState state) override;
    PinState digitalRead(uint8_t pin) override;
    void togglePin(uint8_t pin) override;
    void attachInterrupt(uint8_t pin, GpioInterruptCallback cb, TriggerEdge edge) override;
    void detachInterrupt(uint8_t pin) override;

    // =========================================================================
    // ITimerHAL
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
        timer_.delayMicros(us);
    }

    // =========================================================================
    // IDriverHAL
    // =========================================================================
    void setStepPin(uint8_t axis, PinState state) override;
    void setDirPin(uint8_t axis, Direction dir) override;
    void setEnablePin(uint8_t axis, bool enabled) override;
    void step(uint8_t axis) override;

    bool configureDriver(uint8_t axis, const DriverConfig& cfg) override;
    bool setMicrosteps(uint8_t axis, uint16_t microsteps) override;
    bool setRunCurrent(uint8_t axis, float current_amps) override;
    bool setHoldCurrent(uint8_t axis, float current_amps) override;
    bool setStealthChop(uint8_t axis, bool enable) override;
    bool setStallGuardThreshold(uint8_t axis, uint8_t threshold) override;
    bool readStallGuard(uint8_t axis, uint16_t& sg_val) override;

    bool writeRegister(uint8_t axis, uint8_t reg_addr, uint32_t value) override;
    bool readRegister(uint8_t axis, uint8_t reg_addr, uint32_t& value) override;
    bool getDriverStatus(uint8_t axis, DriverStatus& status) override;

    // =========================================================================
    // IEndstopHAL
    // =========================================================================
    bool configureEndstop(uint8_t axis, EndstopType type, uint32_t debounce_us = 5000, PullMode pull = PullMode::PULL_UP) override;
    bool configureEndstop(uint8_t axis, const EndstopConfig& config) override;
    bool isTriggered(uint8_t axis) const override;
    PinState rawPinState(uint8_t axis) const override;
    void attachTriggerCallback(uint8_t axis, EndstopCallback cb) override;
    void detachTriggerCallback(uint8_t axis) override;
    void setDebounceUs(uint8_t axis, uint32_t debounce_us) override;
    void update(uint64_t current_time_us) override;

private:
    bool initUart();
    bool tmcWrite(uint8_t slave_addr, uint8_t reg, uint32_t value);
    bool tmcRead(uint8_t slave_addr, uint8_t reg, uint32_t& value);

    AxisPins axis_pins_[MAX_AXES];
    UartPins uart_pins_;
    ESP32S3_Timer timer_;
    EndstopDebouncer debouncers_[MAX_AXES];
    EndstopCallback endstop_callbacks_[MAX_AXES];
    DriverConfig driver_configs_[MAX_AXES];

    mutable std::mutex mutex_;
    bool initialized_ = false;
};

} // namespace esp32s3
} // namespace motion_core
