#include "planner/HomingEngine.hpp"
#include "hal_impl/mock/MockHAL.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

using namespace motion_core;
using namespace motion_core::mock;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

// Helper simulation context for driving HomingEngine moves
struct HomingSimContext {
    MockHAL hal;
    HomingEngine engine;
    bool is_moving{false};
    uint8_t move_axis{0};
    Direction move_dir{Direction::NEGATIVE};
    float move_dist{0.0f};
    float move_speed{0.0f};
    float current_pos[MAX_AXES]{0.0f, 0.0f, 0.0f, 0.0f};
    uint64_t sim_time_us{1000};
    bool finish_called{false};
    bool finish_success{false};

    HomingSimContext() {
        engine.init(&hal.endstop(), &hal.driver());

        engine.setMoveCallbacks(
            [this](uint8_t axis, Direction dir, float dist, float speed) -> bool {
                is_moving = true;
                move_axis = axis;
                move_dir = dir;
                move_dist = dist;
                move_speed = speed;
                return true;
            },
            [this]() {
                is_moving = false;
            },
            [this](uint8_t axis, float offset) {
                current_pos[axis] = offset;
            }
        );

        engine.setOnFinishedCallback([this](uint8_t /*axis*/, bool success) {
            finish_called = true;
            finish_success = success;
        });
    }

    void advance(uint64_t delta_us) {
        sim_time_us += delta_us;
        hal.advanceTime(delta_us);
        engine.update(sim_time_us, is_moving);
    }
};

// 1. Physical Limit Switch Normally Closed (NC) full double-tap homing
bool test_homing_physical_switch_nc() {
    std::cout << "[TEST] Homing with Physical Switch (NC fail-safe)...\n";

    HomingSimContext sim;
    const uint8_t axis = 0;

    HomingConfig cfg;
    cfg.sensor_type = HomingSensorType::LIMIT_SWITCH_NC;
    cfg.homing_dir = Direction::NEGATIVE;
    cfg.fast_speed_mm_s = 20.0f;
    cfg.slow_speed_mm_s = 4.0f;
    cfg.backoff_distance_mm = 5.0f;
    cfg.max_travel_mm = 100.0f;
    cfg.home_offset_mm = 0.0f;
    cfg.debounce_us = 1000; // 1ms debounce

    sim.engine.setConfig(axis, cfg);

    // Initial state: axis starts at position 50, switch untriggered
    sim.current_pos[axis] = 50.0f;
    sim.hal.endstop().setSimulatedTrigger(axis, false, sim.sim_time_us);

    // Start homing
    TEST_ASSERT(sim.engine.startHoming(axis), "Start homing returns true");
    TEST_ASSERT(sim.engine.isHoming(), "Engine is in homing state");
    TEST_ASSERT(sim.is_moving, "Fast approach move started");
    TEST_ASSERT(sim.move_dir == Direction::NEGATIVE, "Fast approach in negative direction");

    sim.advance(5000); // 5ms travel

    // 1. Fast approach reaches limit: trigger switch
    sim.hal.endstop().setSimulatedTrigger(axis, true, sim.sim_time_us);
    sim.advance(cfg.debounce_us + 100);

    // Engine should detect fast trigger and switch to BACKOFF
    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_BACKOFF_DONE, "State transitioned to wait backoff");
    TEST_ASSERT(sim.move_dir == Direction::POSITIVE, "Backoff moves in positive direction");
    TEST_ASSERT(sim.move_dist == cfg.backoff_distance_mm, "Backoff distance matches config");

    // 2. Backoff completes and switch releases
    sim.hal.endstop().setSimulatedTrigger(axis, false, sim.sim_time_us);
    sim.is_moving = false; // Move finished
    sim.advance(cfg.debounce_us + 100);

    // Engine should start slow precision approach
    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_SLOW_TRIGGER, "State transitioned to wait slow trigger");
    TEST_ASSERT(sim.is_moving, "Slow approach move started");
    TEST_ASSERT(sim.move_dir == Direction::NEGATIVE, "Slow approach moving negative");
    TEST_ASSERT(sim.move_speed == cfg.slow_speed_mm_s, "Slow approach speed matches slow_speed_mm_s");

    // 3. Slow approach reaches limit: trigger switch
    sim.hal.endstop().setSimulatedTrigger(axis, true, sim.sim_time_us);
    sim.advance(cfg.debounce_us + 100);

    // Engine should finalize, zero axis, and mark homed
    TEST_ASSERT(sim.engine.getState() == HomingState::COMPLETE, "Homing completed");
    TEST_ASSERT(sim.engine.isAxisHomed(axis), "Axis marked homed");
    TEST_ASSERT(sim.current_pos[axis] == cfg.home_offset_mm, "Axis position zeroed");
    TEST_ASSERT(sim.finish_called && sim.finish_success, "Finished callback called with success");

    return true;
}

// 2. Physical Limit Switch Normally Open (NO) with Debounce Filter verification
bool test_homing_physical_switch_no_debounce() {
    std::cout << "[TEST] Homing with Physical Switch (NO) & Debounce filter rejection...\n";

    HomingSimContext sim;
    const uint8_t axis = 1;

    HomingConfig cfg;
    cfg.sensor_type = HomingSensorType::LIMIT_SWITCH_NO;
    cfg.homing_dir = Direction::NEGATIVE;
    cfg.debounce_us = 5000; // 5ms debounce filter

    sim.engine.setConfig(axis, cfg);
    sim.engine.startHoming(axis);
    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_FAST_TRIGGER, "Waiting fast trigger");

    // Introduce a quick noise spike of 1ms (shorter than 5ms debounce)
    sim.hal.endstop().setRawPinState(axis, PinState::LOW, sim.sim_time_us); // NO triggered raw state
    sim.advance(1000); // 1ms
    sim.hal.endstop().setRawPinState(axis, PinState::HIGH, sim.sim_time_us); // return to normal
    sim.advance(500);

    // Verify debounce filter rejected the noise spike: still waiting fast trigger!
    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_FAST_TRIGGER, "Noise spike was rejected by debounce");
    TEST_ASSERT(!sim.engine.isTriggered(axis), "Sensor is not considered triggered");

    // Genuine contact held for > debounce duration (6ms)
    sim.hal.endstop().setRawPinState(axis, PinState::LOW, sim.sim_time_us);
    sim.advance(6000);

    // Verified trigger confirmed
    TEST_ASSERT(sim.engine.isTriggered(axis), "Debounced contact confirmed");
    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_BACKOFF_DONE, "Backoff triggered after genuine hit");

    return true;
}

// 3. Hall Effect Sensor Active LOW (e.g. A3144)
bool test_homing_hall_sensor_active_low() {
    std::cout << "[TEST] Homing with Hall Effect Sensor (Active LOW - A3144)...\n";

    HomingSimContext sim;
    const uint8_t axis = 2;

    HomingConfig cfg;
    cfg.sensor_type = HomingSensorType::HALL_ACTIVE_LOW;
    cfg.homing_dir = Direction::NEGATIVE;
    cfg.fast_speed_mm_s = 15.0f;
    cfg.slow_speed_mm_s = 2.0f;
    cfg.backoff_distance_mm = 3.0f;
    cfg.debounce_us = 100; // Small debounce for magnetic sensor

    sim.engine.setConfig(axis, cfg);
    sim.engine.startHoming(axis);

    // Initial state: No magnet present -> pin is HIGH (untriggered)
    TEST_ASSERT(!sim.engine.isTriggered(axis), "Hall sensor untriggered initially");

    // Fast approach: Magnet detected (Active LOW brings pin to LOW)
    sim.hal.endstop().setRawPinState(axis, PinState::LOW, sim.sim_time_us);
    sim.advance(200);

    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_BACKOFF_DONE, "Magnet detected during fast approach");

    // Backoff: Magnet moves away -> pin goes HIGH (untriggered)
    sim.hal.endstop().setRawPinState(axis, PinState::HIGH, sim.sim_time_us);
    sim.is_moving = false;
    sim.advance(200);

    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_SLOW_TRIGGER, "Slow precision approach started");

    // Slow approach: Magnet detected again
    sim.hal.endstop().setRawPinState(axis, PinState::LOW, sim.sim_time_us);
    sim.advance(200);

    TEST_ASSERT(sim.engine.getState() == HomingState::COMPLETE, "Hall sensor homing complete");
    TEST_ASSERT(sim.engine.isAxisHomed(axis), "Axis 2 marked homed");

    return true;
}

// 4. Hall Effect Sensor Active HIGH
bool test_homing_hall_sensor_active_high() {
    std::cout << "[TEST] Homing with Hall Effect Sensor (Active HIGH)...\n";

    HomingSimContext sim;
    const uint8_t axis = 3;

    HomingConfig cfg;
    cfg.sensor_type = HomingSensorType::HALL_ACTIVE_HIGH;
    cfg.homing_dir = Direction::POSITIVE; // positive homing direction
    cfg.debounce_us = 100;

    sim.engine.setConfig(axis, cfg);
    sim.engine.startHoming(axis);

    // Untriggered state is LOW
    TEST_ASSERT(!sim.engine.isTriggered(axis), "Hall sensor initially untriggered");

    // Fast approach: Magnet detected -> pin goes HIGH
    sim.hal.endstop().setRawPinState(axis, PinState::HIGH, sim.sim_time_us);
    sim.advance(200);
    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_BACKOFF_DONE, "Active HIGH magnet trigger detected");

    // Backoff finishes and magnet released
    sim.hal.endstop().setRawPinState(axis, PinState::LOW, sim.sim_time_us);
    sim.is_moving = false;
    sim.advance(200);

    // Slow approach hits magnet
    sim.hal.endstop().setRawPinState(axis, PinState::HIGH, sim.sim_time_us);
    sim.advance(200);
    TEST_ASSERT(sim.engine.isAxisHomed(axis), "Axis 3 successfully homed with Active HIGH Hall sensor");

    return true;
}

// 5. TMC2209 Sensorless StallGuard4 homing
bool test_homing_stallguard_sensorless() {
    std::cout << "[TEST] Homing with TMC2209 StallGuard4 sensorless detection...\n";

    HomingSimContext sim;
    const uint8_t axis = 0;

    HomingConfig cfg;
    cfg.sensor_type = HomingSensorType::STALLGUARD;
    cfg.homing_dir = Direction::NEGATIVE;
    cfg.stallguard_threshold = 50; // SGTHRS

    sim.engine.setConfig(axis, cfg);
    sim.engine.startHoming(axis);

    // Initial normal operation: SG_RESULT = 450 (well above threshold of 50)
    TEST_ASSERT(!sim.engine.isTriggered(axis), "Motor running freely, no stall");

    // Fast approach: Motor contacts hard stop -> stall occurs (SG_RESULT drops to 0)
    sim.hal.driver().writeRegister(axis, tmc2209::REG_SG_RESULT, 0);
    sim.advance(100);
    TEST_ASSERT(sim.engine.isTriggered(axis), "StallGuard stall detected");
    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_BACKOFF_DONE, "Backoff started after stall");

    // Backoff: motor moves away, SG_RESULT returns to 400
    sim.hal.driver().writeRegister(axis, tmc2209::REG_SG_RESULT, 400);
    sim.is_moving = false;
    sim.advance(100);
    TEST_ASSERT(sim.engine.getState() == HomingState::WAIT_SLOW_TRIGGER, "Precision slow approach started");

    // Slow approach: gentle contact with hard stop -> stall occurs
    sim.hal.driver().writeRegister(axis, tmc2209::REG_SG_RESULT, 10);
    sim.advance(100);
    TEST_ASSERT(sim.engine.isAxisHomed(axis), "Axis 0 successfully homed sensorless with StallGuard4");

    return true;
}

// 6. Multi-axis sequential homing
bool test_homing_multi_axis_sequence() {
    std::cout << "[TEST] Multi-axis sequential homing (Axis 0 -> Axis 1)...\n";

    HomingSimContext sim;
    HomingConfig cfg;
    cfg.debounce_us = 100;
    sim.engine.setConfig(0, cfg);
    sim.engine.setConfig(1, cfg);

    std::vector<uint8_t> order = {0, 1};
    TEST_ASSERT(sim.engine.startHomingSequence(order), "Sequence started");
    TEST_ASSERT(sim.engine.getCurrentAxis() == 0, "Axis 0 is first in sequence");

    // Complete Axis 0 homing
    sim.hal.endstop().setSimulatedTrigger(0, true, sim.sim_time_us);
    sim.advance(200);
    sim.hal.endstop().setSimulatedTrigger(0, false, sim.sim_time_us);
    sim.is_moving = false;
    sim.advance(200);
    sim.hal.endstop().setSimulatedTrigger(0, true, sim.sim_time_us);
    sim.advance(200);

    TEST_ASSERT(sim.engine.isAxisHomed(0), "Axis 0 homed");

    // Verify engine auto-advanced to Axis 1
    TEST_ASSERT(sim.engine.getCurrentAxis() == 1, "Engine advanced to Axis 1 in sequence");

    // Complete Axis 1 homing
    sim.hal.endstop().setSimulatedTrigger(1, true, sim.sim_time_us);
    sim.advance(200);
    sim.hal.endstop().setSimulatedTrigger(1, false, sim.sim_time_us);
    sim.is_moving = false;
    sim.advance(200);
    sim.hal.endstop().setSimulatedTrigger(1, true, sim.sim_time_us);
    sim.advance(200);

    TEST_ASSERT(sim.engine.isAxisHomed(1), "Axis 1 homed");
    TEST_ASSERT(sim.engine.getState() == HomingState::COMPLETE, "Full sequence completed");

    return true;
}

// 7. Abort / safety during homing
bool test_homing_abort() {
    std::cout << "[TEST] Homing abort halts motion...\n";

    HomingSimContext sim;
    HomingConfig cfg;
    sim.engine.setConfig(0, cfg);
    sim.engine.startHoming(0);

    TEST_ASSERT(sim.engine.isHoming(), "Homing active");
    TEST_ASSERT(sim.is_moving, "Motion active");

    sim.engine.abort();

    TEST_ASSERT(!sim.engine.isHoming(), "Homing aborted");
    TEST_ASSERT(!sim.is_moving, "Motion halted");
    TEST_ASSERT(sim.engine.getState() == HomingState::FAILED, "State marked FAILED");

    return true;
}

bool run_all_homing_tests() {
    std::cout << "========================================\n";
    std::cout << " RUNNING HOMING ENGINE UNIT TESTS       \n";
    std::cout << "========================================\n";

    if (!test_homing_physical_switch_nc()) return false;
    if (!test_homing_physical_switch_no_debounce()) return false;
    if (!test_homing_hall_sensor_active_low()) return false;
    if (!test_homing_hall_sensor_active_high()) return false;
    if (!test_homing_stallguard_sensorless()) return false;
    if (!test_homing_multi_axis_sequence()) return false;
    if (!test_homing_abort()) return false;

    std::cout << "========================================\n";
    std::cout << " ALL HOMING ENGINE TESTS PASSED!        \n";
    std::cout << "========================================\n";
    return true;
}

#ifndef TEST_COMBINED_RUNNER
int main() {
    return run_all_homing_tests() ? 0 : 1;
}
#endif
