#pragma once

#include "hal/IDriverHAL.hpp"
#include "hal/TMC2209Registers.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>
#include <functional>

namespace motion_core {
namespace mock {

struct StepEvent {
    uint64_t  time_us;
    uint8_t   axis;
    Direction dir;
    int64_t   position_steps;
};

class MockDriverHAL : public IDriverHAL {
public:
    MockDriverHAL() {
        for (uint8_t a = 0; a < MAX_AXES; ++a) {
            enabled_[a] = false;
            dir_[a] = Direction::POSITIVE;
            step_state_[a] = PinState::LOW;
            position_steps_[a] = 0;
            total_pulses_[a] = 0;
            sg_result_[a] = 450; // normal operating SG_RESULT without stall
            stall_at_step_[a] = 0;
            stall_simulation_enabled_[a] = false;
            stall_dir_[a] = Direction::NEGATIVE;

            // Initialize default TMC registers
            registers_[a][tmc2209::REG_GCONF] = 0x00000000;
            registers_[a][tmc2209::REG_IHOLD_IRUN] = tmc2209::encodeIHoldIRun(8, 16, 10);
            registers_[a][tmc2209::REG_CHOPCONF] = 0x10000053; // 1/16 with interpolation
            registers_[a][tmc2209::REG_SGTHRS] = 50;
            registers_[a][tmc2209::REG_SG_RESULT] = 450;
            registers_[a][tmc2209::REG_DRV_STATUS] = 0;
        }
    }
    virtual ~MockDriverHAL() = default;

    void setStepPin(uint8_t axis, PinState state) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        PinState prev = step_state_[axis];
        step_state_[axis] = state;

        // Rising edge generates a step
        if (prev == PinState::LOW && state == PinState::HIGH) {
            handleStepPulse(axis);
        }
    }

    void setDirPin(uint8_t axis, Direction dir) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        dir_[axis] = dir;
    }

    void setEnablePin(uint8_t axis, bool enabled) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_[axis] = enabled;
    }

    void step(uint8_t axis) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        step_state_[axis] = PinState::HIGH;
        handleStepPulse(axis);
        step_state_[axis] = PinState::LOW;
    }

    bool configureDriver(uint8_t axis, const DriverConfig& cfg) override {
        if (axis >= MAX_AXES) return false;
        configs_[axis] = cfg;
        setMicrosteps(axis, cfg.microsteps);
        setRunCurrent(axis, cfg.run_current_amps);
        setHoldCurrent(axis, cfg.hold_current_amps);
        setStealthChop(axis, cfg.stealthchop_enabled);
        setStallGuardThreshold(axis, cfg.stallguard_threshold);
        return true;
    }

    bool setMicrosteps(uint8_t axis, uint16_t microsteps) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        uint8_t mres = tmc2209::microstepsToMres(microsteps);
        uint32_t chopconf = registers_[axis][tmc2209::REG_CHOPCONF];
        chopconf = (chopconf & ~(0x0Fu << 24)) | (static_cast<uint32_t>(mres & 0x0F) << 24);
        registers_[axis][tmc2209::REG_CHOPCONF] = chopconf;
        configs_[axis].microsteps = microsteps;
        return true;
    }

    bool setRunCurrent(uint8_t axis, float current_amps) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        uint8_t cs = tmc2209::currentToCS(current_amps, configs_[axis].r_sense);
        uint32_t val = registers_[axis][tmc2209::REG_IHOLD_IRUN];
        val = (val & ~(0x1Fu << 8)) | (static_cast<uint32_t>(cs & 0x1F) << 8);
        registers_[axis][tmc2209::REG_IHOLD_IRUN] = val;
        configs_[axis].run_current_amps = current_amps;
        return true;
    }

    bool setHoldCurrent(uint8_t axis, float current_amps) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        uint8_t cs = tmc2209::currentToCS(current_amps, configs_[axis].r_sense);
        uint32_t val = registers_[axis][tmc2209::REG_IHOLD_IRUN];
        val = (val & ~0x1Fu) | static_cast<uint32_t>(cs & 0x1F);
        registers_[axis][tmc2209::REG_IHOLD_IRUN] = val;
        configs_[axis].hold_current_amps = current_amps;
        return true;
    }

    bool setStealthChop(uint8_t axis, bool enable) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t gconf = registers_[axis][tmc2209::REG_GCONF];
        if (enable) {
            gconf &= ~tmc2209::GCONF_EN_SPREADCYCLE;
        } else {
            gconf |= tmc2209::GCONF_EN_SPREADCYCLE;
        }
        registers_[axis][tmc2209::REG_GCONF] = gconf;
        configs_[axis].stealthchop_enabled = enable;
        return true;
    }

    bool setStallGuardThreshold(uint8_t axis, uint8_t threshold) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        registers_[axis][tmc2209::REG_SGTHRS] = threshold;
        configs_[axis].stallguard_threshold = threshold;
        return true;
    }

    bool readStallGuard(uint8_t axis, uint16_t& sg_val) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);

        if (stall_simulation_enabled_[axis]) {
            bool is_stalled = false;
            if (stall_dir_[axis] == Direction::NEGATIVE && position_steps_[axis] <= stall_at_step_[axis]) {
                is_stalled = true;
            } else if (stall_dir_[axis] == Direction::POSITIVE && position_steps_[axis] >= stall_at_step_[axis]) {
                is_stalled = true;
            }
            if (is_stalled) {
                sg_val = stalled_sg_val_[axis];
                registers_[axis][tmc2209::REG_SG_RESULT] = sg_val;
                return true;
            }
        }

        sg_val = sg_result_[axis];
        registers_[axis][tmc2209::REG_SG_RESULT] = sg_val;
        return true;
    }

    bool writeRegister(uint8_t axis, uint8_t reg_addr, uint32_t value) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        registers_[axis][reg_addr] = value;
        if (reg_addr == tmc2209::REG_SG_RESULT) {
            sg_result_[axis] = static_cast<uint16_t>(value);
        }
        return true;
    }

    bool readRegister(uint8_t axis, uint8_t reg_addr, uint32_t& value) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = registers_[axis].find(reg_addr);
        if (it != registers_[axis].end()) {
            value = it->second;
            return true;
        }
        value = 0;
        return true;
    }

    bool getDriverStatus(uint8_t axis, DriverStatus& status) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        status = driver_statuses_[axis];
        status.standstill = !enabled_[axis];
        status.sg_result = sg_result_[axis];
        return true;
    }

    // --- Simulation Inspection & Control Helpers ---

    void setSimulationTime(uint64_t time_us) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_time_us_ = time_us;
    }

    int64_t getPositionSteps(uint8_t axis) const {
        if (axis >= MAX_AXES) return 0;
        std::lock_guard<std::mutex> lock(mutex_);
        return position_steps_[axis];
    }

    void setPositionSteps(uint8_t axis, int64_t pos) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        position_steps_[axis] = pos;
    }

    uint32_t getTotalPulses(uint8_t axis) const {
        if (axis >= MAX_AXES) return 0;
        std::lock_guard<std::mutex> lock(mutex_);
        return total_pulses_[axis];
    }

    bool isEnabled(uint8_t axis) const {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return enabled_[axis];
    }

    Direction getDirection(uint8_t axis) const {
        if (axis >= MAX_AXES) return Direction::POSITIVE;
        std::lock_guard<std::mutex> lock(mutex_);
        return dir_[axis];
    }

    const std::vector<StepEvent>& getStepHistory() const {
        return history_;
    }

    void clearStepHistory() {
        std::lock_guard<std::mutex> lock(mutex_);
        history_.clear();
    }

    void setSimulatedStallAtStep(uint8_t axis, int64_t step_pos, Direction approach_dir = Direction::NEGATIVE, uint16_t stalled_sg_val = 0) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        stall_simulation_enabled_[axis] = true;
        stall_at_step_[axis] = step_pos;
        stall_dir_[axis] = approach_dir;
        stalled_sg_val_[axis] = stalled_sg_val;
    }

    void clearSimulatedStall(uint8_t axis) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        stall_simulation_enabled_[axis] = false;
    }

    void setStepCallback(std::function<void(uint8_t axis, int64_t pos)> cb) {
        std::lock_guard<std::mutex> lock(mutex_);
        step_callback_ = cb;
    }

private:
    void handleStepPulse(uint8_t axis) {
        total_pulses_[axis]++;
        if (dir_[axis] == Direction::POSITIVE) {
            position_steps_[axis]++;
        } else {
            position_steps_[axis]--;
        }

        history_.push_back({current_time_us_, axis, dir_[axis], position_steps_[axis]});

        if (step_callback_) {
            step_callback_(axis, position_steps_[axis]);
        }
    }

    mutable std::mutex mutex_;
    uint64_t current_time_us_ = 0;
    bool enabled_[MAX_AXES]{};
    Direction dir_[MAX_AXES]{};
    PinState step_state_[MAX_AXES]{};
    int64_t position_steps_[MAX_AXES]{};
    uint32_t total_pulses_[MAX_AXES]{};
    uint16_t sg_result_[MAX_AXES]{};
    bool stall_simulation_enabled_[MAX_AXES]{};
    int64_t stall_at_step_[MAX_AXES]{};
    Direction stall_dir_[MAX_AXES]{};
    uint16_t stalled_sg_val_[MAX_AXES]{};

    DriverConfig configs_[MAX_AXES]{};
    DriverStatus driver_statuses_[MAX_AXES]{};
    std::unordered_map<uint8_t, uint32_t> registers_[MAX_AXES];
    std::vector<StepEvent> history_;
    std::function<void(uint8_t axis, int64_t pos)> step_callback_;
};

} // namespace mock
} // namespace motion_core
