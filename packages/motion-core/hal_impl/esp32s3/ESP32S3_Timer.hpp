#pragma once

#include "hal/ITimerHAL.hpp"

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "rom/ets_sys.h"
#include <unordered_map>
#include <mutex>
#include <memory>

namespace motion_core {
namespace esp32s3 {

struct EspTimerEntry {
    TimerHandle handle;
    esp_timer_handle_t esp_timer;
    TimerCallback callback;
    bool is_periodic;
};

/**
 * @brief High-resolution timer HAL for ESP32-S3 (Xtensa Dual-Core 240MHz).
 * Backed by ESP-IDF esp_timer (64-bit microsecond hardware-backed timer).
 */
class ESP32S3_Timer : public ITimerHAL {
public:
    ESP32S3_Timer() = default;
    virtual ~ESP32S3_Timer() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [h, entry] : timers_) {
            if (entry.esp_timer) {
                esp_timer_stop(entry.esp_timer);
                esp_timer_delete(entry.esp_timer);
            }
        }
        timers_.clear();
    }

    uint64_t getMicros() const override {
        return static_cast<uint64_t>(esp_timer_get_time());
    }

    uint32_t getMillis() const override {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000);
    }

    TimerHandle createPeriodicTimer(uint32_t period_us, TimerCallback cb) override {
        if (period_us == 0 || !cb) return INVALID_TIMER_HANDLE;
        return allocateTimer(period_us, cb, true);
    }

    TimerHandle createOneShotTimer(uint32_t delay_us, TimerCallback cb) override {
        if (!cb) return INVALID_TIMER_HANDLE;
        return allocateTimer(delay_us, cb, false);
    }

    bool cancelTimer(TimerHandle handle) override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = timers_.find(handle);
        if (it != timers_.end()) {
            if (it->second.esp_timer) {
                esp_timer_stop(it->second.esp_timer);
                esp_timer_delete(it->second.esp_timer);
            }
            timers_.erase(it);
            return true;
        }
        return false;
    }

    bool setTimerPeriod(TimerHandle handle, uint32_t period_us) override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = timers_.find(handle);
        if (it != timers_.end() && it->second.is_periodic && it->second.esp_timer) {
            esp_timer_stop(it->second.esp_timer);
            esp_err_t err = esp_timer_start_periodic(it->second.esp_timer, period_us);
            return (err == ESP_OK);
        }
        return false;
    }

    bool isTimerActive(TimerHandle handle) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = timers_.find(handle);
        if (it != timers_.end() && it->second.esp_timer) {
            return esp_timer_is_active(it->second.esp_timer);
        }
        return false;
    }

    void delayMicros(uint32_t us) override {
        esp_rom_delay_us(us);
    }

private:
    static void espTimerCallbackThunk(void* arg) {
        auto* entry = static_cast<EspTimerEntry*>(arg);
        if (entry && entry->callback) {
            entry->callback();
        }
    }

    TimerHandle allocateTimer(uint32_t time_us, TimerCallback cb, bool periodic) {
        std::lock_guard<std::mutex> lock(mutex_);
        TimerHandle handle = next_handle_++;

        auto& entry = timers_[handle];
        entry.handle = handle;
        entry.callback = cb;
        entry.is_periodic = periodic;

        esp_timer_create_args_t args = {};
        args.callback = &ESP32S3_Timer::espTimerCallbackThunk;
        args.arg = &entry;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "motion_timer";

        esp_err_t err = esp_timer_create(&args, &entry.esp_timer);
        if (err != ESP_OK) {
            timers_.erase(handle);
            return INVALID_TIMER_HANDLE;
        }

        if (periodic) {
            err = esp_timer_start_periodic(entry.esp_timer, time_us);
        } else {
            err = esp_timer_start_once(entry.esp_timer, time_us);
        }

        if (err != ESP_OK) {
            esp_timer_delete(entry.esp_timer);
            timers_.erase(handle);
            return INVALID_TIMER_HANDLE;
        }

        return handle;
    }

    mutable std::mutex mutex_;
    TimerHandle next_handle_ = 1;
    std::unordered_map<TimerHandle, EspTimerEntry> timers_;
};

} // namespace esp32s3
} // namespace motion_core

#else

// Host/Stub fallback for PC compilation when ESP-IDF is not present
#include <chrono>
#include <thread>

namespace motion_core {
namespace esp32s3 {

class ESP32S3_Timer : public ITimerHAL {
public:
    ESP32S3_Timer() : start_time_(std::chrono::steady_clock::now()) {}
    virtual ~ESP32S3_Timer() = default;

    uint64_t getMicros() const override {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_).count();
    }

    uint32_t getMillis() const override {
        return static_cast<uint32_t>(getMicros() / 1000);
    }

    TimerHandle createPeriodicTimer(uint32_t /*period_us*/, TimerCallback /*cb*/) override {
        return 1;
    }

    TimerHandle createOneShotTimer(uint32_t /*delay_us*/, TimerCallback /*cb*/) override {
        return 2;
    }

    bool cancelTimer(TimerHandle /*handle*/) override {
        return true;
    }

    bool setTimerPeriod(TimerHandle /*handle*/, uint32_t /*period_us*/) override {
        return true;
    }

    bool isTimerActive(TimerHandle /*handle*/) const override {
        return false;
    }

    void delayMicros(uint32_t us) override {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }

private:
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace esp32s3
} // namespace motion_core

#endif
