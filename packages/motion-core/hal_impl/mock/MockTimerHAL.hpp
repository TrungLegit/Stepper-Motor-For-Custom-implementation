#pragma once

#include "hal/ITimerHAL.hpp"
#include <unordered_map>
#include <vector>
#include <mutex>
#include <algorithm>

namespace motion_core {
namespace mock {

struct SimulatedTimer {
    TimerHandle   handle = INVALID_TIMER_HANDLE;
    bool          is_periodic = false;
    uint32_t      period_us = 0;
    uint64_t      next_fire_time_us = 0;
    TimerCallback callback;
    bool          active = false;
};

class MockTimerHAL : public ITimerHAL {
public:
    MockTimerHAL() = default;
    virtual ~MockTimerHAL() = default;

    uint64_t getMicros() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_time_us_;
    }

    uint32_t getMillis() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint32_t>(current_time_us_ / 1000);
    }

    TimerHandle createPeriodicTimer(uint32_t period_us, TimerCallback cb) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (period_us == 0 || !cb) return INVALID_TIMER_HANDLE;

        TimerHandle handle = next_handle_++;
        SimulatedTimer timer;
        timer.handle = handle;
        timer.is_periodic = true;
        timer.period_us = period_us;
        timer.next_fire_time_us = current_time_us_ + period_us;
        timer.callback = cb;
        timer.active = true;

        timers_[handle] = timer;
        return handle;
    }

    TimerHandle createOneShotTimer(uint32_t delay_us, TimerCallback cb) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!cb) return INVALID_TIMER_HANDLE;

        TimerHandle handle = next_handle_++;
        SimulatedTimer timer;
        timer.handle = handle;
        timer.is_periodic = false;
        timer.period_us = delay_us;
        timer.next_fire_time_us = current_time_us_ + delay_us;
        timer.callback = cb;
        timer.active = true;

        timers_[handle] = timer;
        return handle;
    }

    bool cancelTimer(TimerHandle handle) override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = timers_.find(handle);
        if (it != timers_.end()) {
            timers_.erase(it);
            return true;
        }
        return false;
    }

    bool setTimerPeriod(TimerHandle handle, uint32_t period_us) override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = timers_.find(handle);
        if (it != timers_.end() && it->second.is_periodic && period_us > 0) {
            it->second.period_us = period_us;
            it->second.next_fire_time_us = current_time_us_ + period_us;
            return true;
        }
        return false;
    }

    bool isTimerActive(TimerHandle handle) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = timers_.find(handle);
        return (it != timers_.end() && it->second.active);
    }

    void delayMicros(uint32_t us) override {
        advanceTime(us);
    }

    // --- Time Simulation Control ---

    /**
     * @brief Advance virtual simulation time by delta_us microseconds, executing scheduled timer callbacks in chronological order.
     */
    void advanceTime(uint64_t delta_us) {
        uint64_t target_time = current_time_us_ + delta_us;

        while (true) {
            TimerCallback callback_to_fire = nullptr;
            TimerHandle handle_to_fire = INVALID_TIMER_HANDLE;
            uint64_t earliest_fire = target_time + 1;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (auto& [h, t] : timers_) {
                    if (t.active && t.next_fire_time_us <= target_time) {
                        if (t.next_fire_time_us < earliest_fire) {
                            earliest_fire = t.next_fire_time_us;
                            handle_to_fire = h;
                        }
                    }
                }

                if (handle_to_fire == INVALID_TIMER_HANDLE) {
                    current_time_us_ = target_time;
                    break;
                }

                // Advance to the timer's scheduled timestamp
                current_time_us_ = earliest_fire;
                auto& timer = timers_[handle_to_fire];
                callback_to_fire = timer.callback;

                if (timer.is_periodic) {
                    timer.next_fire_time_us += timer.period_us;
                } else {
                    timer.active = false;
                    timers_.erase(handle_to_fire);
                }
            }

            // Invoke callback outside mutex lock
            if (callback_to_fire) {
                callback_to_fire();
            }
        }
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        timers_.clear();
        current_time_us_ = 0;
        next_handle_ = 1;
    }

private:
    mutable std::mutex mutex_;
    uint64_t current_time_us_ = 0;
    TimerHandle next_handle_ = 1;
    std::unordered_map<TimerHandle, SimulatedTimer> timers_;
};

} // namespace mock
} // namespace motion_core
