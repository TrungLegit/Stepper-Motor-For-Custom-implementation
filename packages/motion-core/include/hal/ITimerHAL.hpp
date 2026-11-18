#pragma once

#include "HalTypes.hpp"
#include <cstdint>

namespace motion_core {

/**
 * @brief Pure virtual Hardware Abstraction Layer for high-resolution timers.
 * Powers the step generator ISR, settle delays, and non-blocking state machines.
 */
class ITimerHAL {
public:
    virtual ~ITimerHAL() = default;

    /**
     * @brief Get elapsed time in microseconds from hardware high-resolution counter
     */
    virtual uint64_t getMicros() const = 0;

    /**
     * @brief Get elapsed time in milliseconds
     */
    virtual uint32_t getMillis() const = 0;

    /**
     * @brief Create and start a periodic timer firing cb every period_us microseconds
     * @param period_us Interval in microseconds between callbacks
     * @param cb Callback function invoked without blocking
     * @return Valid TimerHandle if started, or INVALID_TIMER_HANDLE on failure
     */
    virtual TimerHandle createPeriodicTimer(uint32_t period_us, TimerCallback cb) = 0;

    /**
     * @brief Create and start a one-shot timer firing cb once after delay_us microseconds
     * @param delay_us Microseconds to wait before firing callback
     * @param cb Callback function
     * @return Valid TimerHandle if scheduled, or INVALID_TIMER_HANDLE on failure
     */
    virtual TimerHandle createOneShotTimer(uint32_t delay_us, TimerCallback cb) = 0;

    /**
     * @brief Cancel an active periodic or one-shot timer
     * @return true if successfully cancelled
     */
    virtual bool cancelTimer(TimerHandle handle) = 0;

    /**
     * @brief Update the period of an existing periodic timer without reallocation
     */
    virtual bool setTimerPeriod(TimerHandle handle, uint32_t period_us) = 0;

    /**
     * @brief Check if a timer handle is currently active
     */
    virtual bool isTimerActive(TimerHandle handle) const = 0;

    /**
     * @brief Non-blocking busy-delay for sub-microsecond pulse widths (used for step pin timing)
     */
    virtual void delayMicros(uint32_t us) = 0;
};

namespace hal {
    using ITimerHAL = motion_core::ITimerHAL;
}

} // namespace motion_core
