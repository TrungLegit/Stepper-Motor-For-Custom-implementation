#pragma once

#include "hal/HalTypes.hpp"
#include "hal/IEndstopHAL.hpp"
#include <cstdint>

namespace motion_core {

/**
 * @brief Software debounce and polarity evaluation filter for endstop/limit sensors.
 */
class EndstopDebouncer {
public:
    EndstopDebouncer() = default;

    void configure(EndstopType type, uint32_t debounce_us, PullMode pull = PullMode::PULL_UP) {
        type_ = type;
        debounce_us_ = debounce_us;
        pull_ = pull;
        // Set initial raw state corresponding to untriggered
        raw_state_ = isUntriggeredRawState();
        debounced_state_ = raw_state_;
        is_triggered_ = false;
        last_change_time_us_ = 0;
        in_debounce_ = false;
    }

    void setDebounceUs(uint32_t debounce_us) {
        debounce_us_ = debounce_us;
    }

    uint32_t getDebounceUs() const {
        return debounce_us_;
    }

    EndstopType getType() const {
        return type_;
    }

    PullMode getPullMode() const {
        return pull_;
    }

    /**
     * @brief Feed a raw pin sample into the filter
     * @return true if the debounced triggered state changed during this call
     */
    bool updateRaw(PinState raw, uint64_t current_time_us) {
        if (raw != raw_state_) {
            raw_state_ = raw;
            last_change_time_us_ = current_time_us;
            in_debounce_ = true;
        }

        if (in_debounce_) {
            if (debounce_us_ == 0 || (current_time_us - last_change_time_us_ >= debounce_us_)) {
                in_debounce_ = false;
                if (debounced_state_ != raw_state_) {
                    debounced_state_ = raw_state_;
                    bool new_triggered = evaluateTriggered(debounced_state_);
                    if (new_triggered != is_triggered_) {
                        is_triggered_ = new_triggered;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * @brief Periodic update with time check
     */
    bool updateTime(uint64_t current_time_us) {
        return updateRaw(raw_state_, current_time_us);
    }

    bool isTriggered() const {
        return is_triggered_;
    }

    PinState rawPinState() const {
        return raw_state_;
    }

    void forceTriggered(bool triggered, uint64_t current_time_us) {
        is_triggered_ = triggered;
        PinState target_raw = triggered ? getTriggeredRawState() : isUntriggeredRawState();
        raw_state_ = target_raw;
        debounced_state_ = target_raw;
        in_debounce_ = false;
        last_change_time_us_ = current_time_us;
    }

private:
    PinState isUntriggeredRawState() const {
        switch (type_) {
            case EndstopType::SWITCH_NC:
                return (pull_ == PullMode::PULL_DOWN) ? PinState::HIGH : PinState::LOW;
            case EndstopType::SWITCH_NO:
                return (pull_ == PullMode::PULL_DOWN) ? PinState::LOW : PinState::HIGH;
            case EndstopType::HALL_ACTIVE_LOW:
                return PinState::HIGH;
            case EndstopType::HALL_ACTIVE_HIGH:
                return PinState::LOW;
        }
        return PinState::LOW;
    }

    PinState getTriggeredRawState() const {
        return (isUntriggeredRawState() == PinState::HIGH) ? PinState::LOW : PinState::HIGH;
    }

    bool evaluateTriggered(PinState state) const {
        return (state == getTriggeredRawState());
    }

    EndstopType type_ = EndstopType::SWITCH_NC;
    PullMode    pull_ = PullMode::PULL_UP;
    uint32_t    debounce_us_ = 5000;
    PinState    raw_state_ = PinState::LOW;
    PinState    debounced_state_ = PinState::LOW;
    bool        is_triggered_ = false;
    uint64_t    last_change_time_us_ = 0;
    bool        in_debounce_ = false;
};

} // namespace motion_core
