#include "MockHAL.hpp"

namespace motion_core {
namespace mock {

MockHAL::MockHAL() {
    // Wire driver step pulse event to endstop position trigger checks
    driver_.setStepCallback([this](uint8_t axis, int64_t pos) {
        uint64_t t = timer_.getMicros();
        endstop_.checkPositionTrigger(axis, pos, t);
        // Track encoder ticks with motor steps (1:1 default simulation)
        encoder_.setCounts(axis, pos);
    });
}

void MockHAL::advanceTime(uint64_t delta_us) {
    uint64_t start_time = timer_.getMicros();
    uint64_t end_time = start_time + delta_us;

    // Advance time on timer HAL (fires scheduled timer callbacks in exact chronological order)
    timer_.advanceTime(delta_us);

    // Synchronize GPIO and Endstop timestamps
    gpio_.setSimulationTime(end_time);
    driver_.setSimulationTime(end_time);
    endstop_.update(end_time);
}

void MockHAL::reset() {
    timer_.reset();
    gpio_.clearHistory();
    driver_.clearStepHistory();
    for (uint8_t a = 0; a < MAX_AXES; ++a) {
        driver_.setPositionSteps(a, 0);
        driver_.clearSimulatedStall(a);
        endstop_.clearSimulatedTriggerAtStep(a);
        endstop_.setSimulatedTrigger(a, false);
        encoder_.resetCount(a, 0);
    }
}

} // namespace mock
} // namespace motion_core
