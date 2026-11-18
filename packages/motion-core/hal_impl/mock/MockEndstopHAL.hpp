#pragma once

#include "hal/IEndstopHAL.hpp"
#include "../../src/hal/EndstopDebouncer.hpp"
#include <vector>
#include <mutex>
#include <functional>

namespace motion_core {
namespace mock {

class MockEndstopHAL : public IEndstopHAL {
public:
    MockEndstopHAL() {
        for (uint8_t a = 0; a < MAX_AXES; ++a) {
            debouncers_[a].configure(EndstopType::SWITCH_NC, 5000, PullMode::PULL_UP);
            position_trigger_enabled_[a] = false;
            trigger_step_[a] = 0;
            trigger_less_than_[a] = true;
        }
    }
    virtual ~MockEndstopHAL() = default;

    bool configureEndstop(uint8_t axis, EndstopType type, uint32_t debounce_us = 5000, PullMode pull = PullMode::PULL_UP) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        debouncers_[axis].configure(type, debounce_us, pull);
        return true;
    }

    bool configureEndstop(uint8_t axis, const EndstopConfig& config) override {
        return configureEndstop(axis, config.type, config.debounce_us, config.pull);
    }

    bool isTriggered(uint8_t axis) const override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return debouncers_[axis].isTriggered();
    }

    PinState rawPinState(uint8_t axis) const override {
        if (axis >= MAX_AXES) return PinState::LOW;
        std::lock_guard<std::mutex> lock(mutex_);
        return debouncers_[axis].rawPinState();
    }

    void attachTriggerCallback(uint8_t axis, EndstopCallback cb) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        callbacks_[axis] = cb;
    }

    void detachTriggerCallback(uint8_t axis) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        callbacks_[axis] = nullptr;
    }

    void setDebounceUs(uint8_t axis, uint32_t debounce_us) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        debouncers_[axis].setDebounceUs(debounce_us);
    }

    void update(uint64_t current_time_us) override {
        std::vector<std::pair<uint8_t, bool>> triggered_events;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_time_us_ = current_time_us;
            for (uint8_t a = 0; a < MAX_AXES; ++a) {
                if (debouncers_[a].updateTime(current_time_us)) {
                    if (callbacks_[a]) {
                        triggered_events.push_back({a, debouncers_[a].isTriggered()});
                    }
                }
            }
        }

        // Fire callbacks outside the mutex
        for (const auto& ev : triggered_events) {
            EndstopCallback cb;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cb = callbacks_[ev.first];
            }
            if (cb) {
                cb(ev.first, ev.second);
            }
        }
    }

    // --- Simulation Helpers ---

    void setSimulationTime(uint64_t current_time_us) {
        update(current_time_us);
    }

    void setRawPinState(uint8_t axis, PinState state, uint64_t timestamp_us = 0) {
        if (axis >= MAX_AXES) return;
        bool state_changed = false;
        bool new_triggered = false;
        EndstopCallback cb = nullptr;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            uint64_t t = (timestamp_us > 0) ? timestamp_us : current_time_us_;
            state_changed = debouncers_[axis].updateRaw(state, t);
            new_triggered = debouncers_[axis].isTriggered();
            cb = callbacks_[axis];
        }

        if (state_changed && cb) {
            cb(axis, new_triggered);
        }
    }

    void setSimulatedTrigger(uint8_t axis, bool triggered, uint64_t timestamp_us = 0) {
        if (axis >= MAX_AXES) return;
        EndstopCallback cb = nullptr;
        bool changed = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            uint64_t t = (timestamp_us > 0) ? timestamp_us : current_time_us_;
            bool was_triggered = debouncers_[axis].isTriggered();
            debouncers_[axis].forceTriggered(triggered, t);
            changed = (was_triggered != triggered);
            cb = callbacks_[axis];
        }

        if (changed && cb) {
            cb(axis, triggered);
        }
    }

    void setSimulatedTriggerAtStep(uint8_t axis, int64_t step_pos, bool trigger_less_than = true) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        position_trigger_enabled_[axis] = true;
        trigger_step_[axis] = step_pos;
        trigger_less_than_[axis] = trigger_less_than;
    }

    void clearSimulatedTriggerAtStep(uint8_t axis) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        position_trigger_enabled_[axis] = false;
    }

    void checkPositionTrigger(uint8_t axis, int64_t current_step, uint64_t time_us) {
        if (axis >= MAX_AXES) return;
        bool should_trigger = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!position_trigger_enabled_[axis]) return;
            if (trigger_less_than_[axis]) {
                should_trigger = (current_step <= trigger_step_[axis]);
            } else {
                should_trigger = (current_step >= trigger_step_[axis]);
            }
        }
        setSimulatedTrigger(axis, should_trigger, time_us);
    }

private:
    mutable std::mutex mutex_;
    uint64_t current_time_us_ = 0;
    EndstopDebouncer debouncers_[MAX_AXES];
    EndstopCallback callbacks_[MAX_AXES];

    bool position_trigger_enabled_[MAX_AXES]{};
    int64_t trigger_step_[MAX_AXES]{};
    bool trigger_less_than_[MAX_AXES]{};
};

} // namespace mock
} // namespace motion_core
