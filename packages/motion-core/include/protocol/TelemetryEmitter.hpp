#pragma once

#include "protocol/Json.hpp"
#include "protocol/MeshLabSchema.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <functional>
#include <utility>

namespace motion_core::protocol {

struct MotionTelemetryData {
    std::vector<std::pair<std::string, float>> pos;
    std::vector<std::pair<std::string, float>> vel;
    bool moving{false};
    bool homing{false};
    bool estop_active{false};
    int32_t queue_depth{0};
    float estimated_time_remaining_s{0.0f};
};

/**
 * @brief Non-blocking asynchronous telemetry emitter for real-time status push events.
 */
class TelemetryEmitter {
public:
    using TelemetrySink = std::function<void(const std::string& event_json)>;

    TelemetryEmitter() = default;

    /**
     * @brief Register a sink callback (e.g., UART writer, WebSocket broadcaster, or test hook).
     */
    void addSink(TelemetrySink sink);

    /**
     * @brief Remove all sinks.
     */
    void clearSinks() noexcept;

    /**
     * @brief Emit real-time motion status (positions, velocities, moving state).
     */
    void emitMotionStatus(const MotionTelemetryData& data);

    /**
     * @brief Emit command queue depth update.
     */
    void emitQueueDepth(int32_t depth, int32_t capacity);

    /**
     * @brief Emit homing state machine progress update.
     */
    void emitHomingProgress(const std::string& axis,
                            const std::string& phase,
                            float position_mm,
                            bool completed);

    /**
     * @brief Emit hardware endstop or Hall sensor trigger alert.
     */
    void emitEndstopAlert(const std::string& axis,
                          uint8_t pin,
                          const std::string& trigger_type,
                          bool active);

    /**
     * @brief Emit 3D scan pose capture completion event.
     */
    void emitScanPoseComplete(const ScanPoseRecord& pose);

    /**
     * @brief Emit emergency stop activation notification.
     */
    void emitEstop(const std::string& reason);

    /**
     * @brief Emit system reset notification.
     */
    void emitSystemReset();

    /**
     * @brief Emit generic custom telemetry event.
     */
    void emitEvent(std::string_view event_name, const JsonValue& data);

private:
    void broadcast(const std::string& json);

    std::vector<TelemetrySink> m_sinks;
};

} // namespace motion_core::protocol
