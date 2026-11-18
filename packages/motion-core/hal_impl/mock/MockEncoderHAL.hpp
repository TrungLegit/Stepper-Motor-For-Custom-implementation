#pragma once

#include "hal/IEncoderHAL.hpp"
#include <mutex>

namespace motion_core {
namespace mock {

class MockEncoderHAL : public IEncoderHAL {
public:
    MockEncoderHAL() {
        for (uint8_t a = 0; a < MAX_AXES; ++a) {
            counts_[a] = 0;
            angles_[a] = 0.0f;
            velocities_[a] = 0.0f;
            index_triggered_[a] = false;
            connected_[a] = true;
            cpr_[a] = 4096.0f; // 12-bit / 14-bit default
        }
    }
    virtual ~MockEncoderHAL() = default;

    bool initEncoder(uint8_t axis) override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        connected_[axis] = true;
        return true;
    }

    int64_t getCount(uint8_t axis) const override {
        if (axis >= MAX_AXES) return 0;
        std::lock_guard<std::mutex> lock(mutex_);
        return counts_[axis];
    }

    void resetCount(uint8_t axis, int64_t count = 0) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        counts_[axis] = count;
        angles_[axis] = (cpr_[axis] > 0) ? (static_cast<float>(count) / cpr_[axis] * 360.0f) : 0.0f;
    }

    float getAngleDegrees(uint8_t axis) const override {
        if (axis >= MAX_AXES) return 0.0f;
        std::lock_guard<std::mutex> lock(mutex_);
        return angles_[axis];
    }

    float getVelocity(uint8_t axis) const override {
        if (axis >= MAX_AXES) return 0.0f;
        std::lock_guard<std::mutex> lock(mutex_);
        return velocities_[axis];
    }

    bool hasIndexTriggered(uint8_t axis) const override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return index_triggered_[axis];
    }

    void resetIndexTrigger(uint8_t axis) override {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        index_triggered_[axis] = false;
    }

    bool isConnected(uint8_t axis) const override {
        if (axis >= MAX_AXES) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return connected_[axis];
    }

    // --- Simulation Helpers ---

    void setCounts(uint8_t axis, int64_t count) {
        resetCount(axis, count);
    }

    void setAngleDegrees(uint8_t axis, float deg) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        angles_[axis] = deg;
        counts_[axis] = static_cast<int64_t>(deg / 360.0f * cpr_[axis]);
    }

    void setVelocity(uint8_t axis, float vel_deg_s) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        velocities_[axis] = vel_deg_s;
    }

    void setIndexTriggered(uint8_t axis, bool val) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        index_triggered_[axis] = val;
    }

    void setConnected(uint8_t axis, bool conn) {
        if (axis >= MAX_AXES) return;
        std::lock_guard<std::mutex> lock(mutex_);
        connected_[axis] = conn;
    }

private:
    mutable std::mutex mutex_;
    int64_t counts_[MAX_AXES]{};
    float angles_[MAX_AXES]{};
    float velocities_[MAX_AXES]{};
    bool index_triggered_[MAX_AXES]{};
    bool connected_[MAX_AXES]{};
    float cpr_[MAX_AXES]{};
};

} // namespace mock
} // namespace motion_core
