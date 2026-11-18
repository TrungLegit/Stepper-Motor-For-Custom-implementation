#pragma once

#include "hal/IGpioHAL.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>

namespace motion_core {
namespace mock {

struct GpioPinEvent {
    uint64_t time_us;
    uint8_t  pin;
    PinState state;
};

class MockGpioHAL : public IGpioHAL {
public:
    MockGpioHAL() = default;
    virtual ~MockGpioHAL() = default;

    void pinMode(uint8_t pin, PinMode mode) override {
        std::lock_guard<std::mutex> lock(mutex_);
        pin_modes_[pin] = mode;
    }

    void digitalWrite(uint8_t pin, PinState state) override {
        std::lock_guard<std::mutex> lock(mutex_);
        PinState old_state = pin_states_[pin];
        pin_states_[pin] = state;
        history_.push_back({current_time_us_, pin, state});

        if (old_state != state && isr_callbacks_.count(pin)) {
            TriggerEdge edge = isr_edges_[pin];
            bool fire = (edge == TriggerEdge::BOTH) ||
                        (edge == TriggerEdge::RISING && state == PinState::HIGH) ||
                        (edge == TriggerEdge::FALLING && state == PinState::LOW);
            if (fire && isr_callbacks_[pin]) {
                isr_callbacks_[pin](pin, state);
            }
        }
    }

    PinState digitalRead(uint8_t pin) override {
        std::lock_guard<std::mutex> lock(mutex_);
        return pin_states_[pin];
    }

    void togglePin(uint8_t pin) override {
        PinState current = digitalRead(pin);
        digitalWrite(pin, current == PinState::HIGH ? PinState::LOW : PinState::HIGH);
    }

    void attachInterrupt(uint8_t pin, GpioInterruptCallback cb, TriggerEdge edge) override {
        std::lock_guard<std::mutex> lock(mutex_);
        isr_callbacks_[pin] = cb;
        isr_edges_[pin] = edge;
    }

    void detachInterrupt(uint8_t pin) override {
        std::lock_guard<std::mutex> lock(mutex_);
        isr_callbacks_.erase(pin);
        isr_edges_.erase(pin);
    }

    // --- Simulation Helpers ---
    void setSimulationTime(uint64_t time_us) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_time_us_ = time_us;
    }

    void setPinInput(uint8_t pin, PinState state) {
        digitalWrite(pin, state);
    }

    PinMode getPinMode(uint8_t pin) {
        std::lock_guard<std::mutex> lock(mutex_);
        return pin_modes_[pin];
    }

    const std::vector<GpioPinEvent>& getHistory() const {
        return history_;
    }

    void clearHistory() {
        std::lock_guard<std::mutex> lock(mutex_);
        history_.clear();
    }

private:
    mutable std::mutex mutex_;
    uint64_t current_time_us_ = 0;
    std::unordered_map<uint8_t, PinMode> pin_modes_;
    std::unordered_map<uint8_t, PinState> pin_states_;
    std::unordered_map<uint8_t, GpioInterruptCallback> isr_callbacks_;
    std::unordered_map<uint8_t, TriggerEdge> isr_edges_;
    std::vector<GpioPinEvent> history_;
};

} // namespace mock
} // namespace motion_core
