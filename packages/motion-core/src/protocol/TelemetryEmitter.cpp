#include "protocol/TelemetryEmitter.hpp"

namespace motion_core::protocol {

void TelemetryEmitter::addSink(TelemetrySink sink) {
    if (sink) {
        m_sinks.push_back(std::move(sink));
    }
}

void TelemetryEmitter::clearSinks() noexcept {
    m_sinks.clear();
}

void TelemetryEmitter::broadcast(const std::string& json) {
    for (auto& sink : m_sinks) {
        if (sink) {
            sink(json);
        }
    }
}

void TelemetryEmitter::emitEvent(std::string_view event_name, const JsonValue& data) {
    JsonValue root = JsonValue::object();
    root.set("event", JsonValue(event_name));
    root.set("data", data);
    broadcast(root.serialize());
}

void TelemetryEmitter::emitMotionStatus(const MotionTelemetryData& data) {
    JsonValue d = JsonValue::object();

    JsonValue pos_obj = JsonValue::object();
    for (const auto& [axis, val] : data.pos) {
        pos_obj.set(axis, JsonValue(val));
    }
    d.set("pos", std::move(pos_obj));

    JsonValue vel_obj = JsonValue::object();
    for (const auto& [axis, val] : data.vel) {
        vel_obj.set(axis, JsonValue(val));
    }
    d.set("vel", std::move(vel_obj));

    d.set("moving", JsonValue(data.moving));
    d.set("homing", JsonValue(data.homing));
    d.set("estop_active", JsonValue(data.estop_active));
    d.set("queue_depth", JsonValue(data.queue_depth));
    d.set("estimated_time_remaining_s", JsonValue(data.estimated_time_remaining_s));

    emitEvent("motion.status", d);
}

void TelemetryEmitter::emitQueueDepth(int32_t depth, int32_t capacity) {
    JsonValue d = JsonValue::object();
    d.set("depth", JsonValue(depth));
    d.set("capacity", JsonValue(capacity));
    emitEvent("motion.queue_depth", d);
}

void TelemetryEmitter::emitHomingProgress(const std::string& axis,
                                          const std::string& phase,
                                          float position_mm,
                                          bool completed) {
    JsonValue d = JsonValue::object();
    d.set("axis", JsonValue(axis));
    d.set("phase", JsonValue(phase));
    d.set("position_mm", JsonValue(position_mm));
    d.set("completed", JsonValue(completed));
    emitEvent("homing.progress", d);
}

void TelemetryEmitter::emitEndstopAlert(const std::string& axis,
                                        uint8_t pin,
                                        const std::string& trigger_type,
                                        bool active) {
    JsonValue d = JsonValue::object();
    d.set("axis", JsonValue(axis));
    d.set("pin", JsonValue(pin));
    d.set("trigger_type", JsonValue(trigger_type));
    d.set("active", JsonValue(active));
    emitEvent("endstop.alert", d);
}

void TelemetryEmitter::emitScanPoseComplete(const ScanPoseRecord& pose) {
    JsonValue d = JsonValue::object();
    d.set("frame_id", JsonValue(pose.frame_id));

    JsonValue pose_obj = JsonValue::object();
    pose_obj.set("azimuth", JsonValue(pose.azimuth_deg));
    pose_obj.set("elevation", JsonValue(pose.elevation_deg));
    pose_obj.set("focus_z", JsonValue(pose.focus_z_mm));
    d.set("pose", std::move(pose_obj));

    JsonValue steps_obj = JsonValue::object();
    for (const auto& [axis, steps] : pose.motor_steps_abs) {
        steps_obj.set(axis, JsonValue(steps));
    }
    d.set("steps_abs", std::move(steps_obj));

    emitEvent("scan.pose_complete", d);
}

void TelemetryEmitter::emitEstop(const std::string& reason) {
    JsonValue d = JsonValue::object();
    d.set("reason", JsonValue(reason));
    emitEvent("system.estop", d);
}

void TelemetryEmitter::emitSystemReset() {
    JsonValue d = JsonValue::object();
    d.set("status", JsonValue("idle"));
    emitEvent("system.reset", d);
}

} // namespace motion_core::protocol
