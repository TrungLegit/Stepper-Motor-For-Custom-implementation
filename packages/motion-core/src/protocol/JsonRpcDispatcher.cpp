#include "protocol/JsonRpcDispatcher.hpp"
#include <algorithm>

namespace motion_core::protocol {

JsonRpcDispatcher::JsonRpcDispatcher() {
    m_state = SystemState::IDLE;
}

void JsonRpcDispatcher::setState(SystemState new_state) noexcept {
    // If we are locked in ESTOP, only system.reset can exit it
    if (m_state == SystemState::ESTOP_LOCKED && new_state != SystemState::ESTOP_LOCKED && new_state != SystemState::IDLE) {
        return;
    }
    m_state = new_state;
}

void JsonRpcDispatcher::triggerEstop(const std::string& reason) {
    m_state = SystemState::ESTOP_LOCKED;
    m_estop_reason = reason.empty() ? "Manual E-STOP triggered" : reason;

    // Immediately execute hardware safety disable
    if (m_estop_action) {
        m_estop_action(m_estop_reason);
    }

    // Emit asynchronous telemetry event
    m_telemetry.emitEstop(m_estop_reason);
}

bool JsonRpcDispatcher::resetSystem() {
    if (m_reset_action) {
        if (!m_reset_action()) {
            return false;
        }
    }

    m_state = SystemState::IDLE;
    m_estop_reason.clear();

    // Emit asynchronous reset telemetry
    m_telemetry.emitSystemReset();
    return true;
}

void JsonRpcDispatcher::setResponseSink(ResponseOutputSink sink) {
    m_response_sink = std::move(sink);
}

void JsonRpcDispatcher::feedStream(const char* data, size_t length) {
    m_packet_buffer.push(data, length);

    // Immediate out-of-band preemption check for E-STOP in stream
    if (m_packet_buffer.peekEmergencyStop()) {
        triggerEstop("E-STOP preemption: emergency stop detected in input stream");
    }

    // Process all ready lines
    while (m_packet_buffer.hasLine()) {
        auto line_opt = m_packet_buffer.popLine();
        if (line_opt && !line_opt->empty()) {
            std::string resp = dispatch(*line_opt);
            if (!resp.empty() && m_response_sink) {
                m_response_sink(resp + "\n");
            }
        }
    }
}

void JsonRpcDispatcher::feedStream(std::string_view chunk) {
    feedStream(chunk.data(), chunk.size());
}

std::string JsonRpcDispatcher::dispatch(std::string_view raw_json) {
    // Skip leading whitespace
    size_t i = 0;
    while (i < raw_json.size() && (raw_json[i] == ' ' || raw_json[i] == '\t' || raw_json[i] == '\r' || raw_json[i] == '\n')) {
        i++;
    }

    if (i >= raw_json.size()) {
        return CommandParser::formatError(JsonValue(nullptr), JsonRpcErrorCode::InvalidRequest, "Empty request");
    }

    // Batch request handling
    if (raw_json[i] == '[') {
        std::string parse_err;
        JsonValue batch = JsonValue::parse(raw_json, &parse_err);
        if (batch.is_null() && !parse_err.empty()) {
            return CommandParser::formatError(JsonValue(nullptr), JsonRpcErrorCode::ParseError, parse_err);
        }
        if (!batch.is_array()) {
            return CommandParser::formatError(JsonValue(nullptr), JsonRpcErrorCode::InvalidRequest, "Expected array for batch request");
        }
        if (batch.empty()) {
            return CommandParser::formatError(JsonValue(nullptr), JsonRpcErrorCode::InvalidRequest, "Batch cannot be empty");
        }

        std::vector<std::string> responses;
        for (size_t idx = 0; idx < batch.size(); ++idx) {
            std::string resp = dispatchSingle(batch[idx]);
            if (!resp.empty()) {
                responses.push_back(std::move(resp));
            }
        }

        if (responses.empty()) {
            return ""; // All were notifications
        }

        std::string out = "[";
        for (size_t r = 0; r < responses.size(); ++r) {
            if (r > 0) out += ",";
            out += responses[r];
        }
        out += "]";
        return out;
    }

    // Single request
    std::string parse_err;
    JsonValue single = JsonValue::parse(raw_json, &parse_err);
    if (single.is_null() && !parse_err.empty()) {
        return CommandParser::formatError(JsonValue(nullptr), JsonRpcErrorCode::ParseError, parse_err);
    }

    return dispatchSingle(single);
}

std::string JsonRpcDispatcher::dispatchSingle(const JsonValue& req_val) {
    if (!req_val.is_object()) {
        return CommandParser::formatError(JsonValue(nullptr), JsonRpcErrorCode::InvalidRequest, "Request must be a JSON object");
    }

    JsonRpcRequest req;
    JsonRpcResponse err_resp;
    std::string serialized = req_val.serialize();
    if (!CommandParser::parseRequest(serialized, req, err_resp)) {
        return err_resp.serialize();
    }

    // Immediate E-STOP preemption handling
    if (req.method == "system.estop") {
        std::string resp = executeMethod(req);
        if (req.is_notification) return "";
        return resp;
    }

    // Safety lock enforcement: In ESTOP_LOCKED, block all motion commands
    if (m_state == SystemState::ESTOP_LOCKED) {
        if (req.method == "axis.move" ||
            req.method == "axis.jog" ||
            req.method == "axis.home" ||
            req.method == "axis.set_config" ||
            req.method == "trigger.capture" ||
            req.method == "scan.start") {
            if (req.is_notification) return "";
            return CommandParser::formatError(
                req.id,
                JsonRpcErrorCode::EstopActive,
                "E-STOP active: motion commands locked until system.reset"
            );
        }
    }

    std::string resp = executeMethod(req);
    if (req.is_notification) {
        return "";
    }
    return resp;
}

std::string JsonRpcDispatcher::executeMethod(const JsonRpcRequest& req) {
    JsonRpcErrorCode err_code = JsonRpcErrorCode::InternalError;
    std::string err_msg;
    JsonValue result;

    if (req.method == "system.info") {
        result = handleSystemInfo(req, err_code, err_msg);
    } else if (req.method == "system.estop") {
        result = handleSystemEstop(req, err_code, err_msg);
    } else if (req.method == "system.reset") {
        result = handleSystemReset(req, err_code, err_msg);
    } else if (req.method == "axis.home") {
        result = handleAxisHome(req, err_code, err_msg);
    } else if (req.method == "axis.move") {
        result = handleAxisMove(req, err_code, err_msg);
    } else if (req.method == "axis.jog") {
        result = handleAxisJog(req, err_code, err_msg);
    } else if (req.method == "axis.set_config") {
        result = handleAxisSetConfig(req, err_code, err_msg);
    } else if (req.method == "axis.get_status") {
        result = handleAxisGetStatus(req, err_code, err_msg);
    } else if (req.method == "trigger.capture") {
        result = handleTriggerCapture(req, err_code, err_msg);
    } else {
        // Check registered custom methods
        auto it = m_custom_methods.find(req.method);
        if (it != m_custom_methods.end()) {
            bool ok = it->second(req.params, result, err_code, err_msg);
            if (!ok) {
                return CommandParser::formatError(req.id, err_code, err_msg);
            }
            return CommandParser::formatSuccess(req.id, result);
        } else {
            return CommandParser::formatError(
                req.id,
                JsonRpcErrorCode::MethodNotFound,
                "Method '" + req.method + "' not found"
            );
        }
    }

    if (!err_msg.empty()) {
        return CommandParser::formatError(req.id, err_code, err_msg);
    }

    return CommandParser::formatSuccess(req.id, result);
}

JsonValue JsonRpcDispatcher::handleSystemInfo(const JsonRpcRequest& /*req*/, JsonRpcErrorCode& /*err_code*/, std::string& /*err_msg*/) {
    if (m_info_provider) {
        return m_info_provider();
    }

    JsonValue info = JsonValue::object();
    info.set("firmware", JsonValue("stepper-motion-core"));
    info.set("version", JsonValue("0.2.0"));
    info.set("protocol", JsonValue("JSON-RPC 2.0"));
    info.set("state", JsonValue(systemStateToString(m_state)));

    JsonValue axes = JsonValue::array();
    axes.push_back(JsonValue("A"));
    axes.push_back(JsonValue("B"));
    axes.push_back(JsonValue("Z"));
    info.set("axes", std::move(axes));

    return info;
}

JsonValue JsonRpcDispatcher::handleSystemEstop(const JsonRpcRequest& req, JsonRpcErrorCode& /*err_code*/, std::string& /*err_msg*/) {
    std::string reason = "system.estop RPC request";
    if (req.params.is_object() && req.params.contains("reason")) {
        reason = req.params.get("reason").as_string(reason);
    }

    triggerEstop(reason);

    JsonValue res = JsonValue::object();
    res.set("estop", JsonValue(true));
    res.set("state", JsonValue(systemStateToString(m_state)));
    res.set("reason", JsonValue(m_estop_reason));
    return res;
}

JsonValue JsonRpcDispatcher::handleSystemReset(const JsonRpcRequest& /*req*/, JsonRpcErrorCode& err_code, std::string& err_msg) {
    if (!resetSystem()) {
        err_code = JsonRpcErrorCode::InternalError;
        err_msg = "Failed to reset system hardware or driver fault active";
        return JsonValue(nullptr);
    }

    JsonValue res = JsonValue::object();
    res.set("reset", JsonValue(true));
    res.set("state", JsonValue(systemStateToString(m_state)));
    return res;
}

JsonValue JsonRpcDispatcher::handleAxisMove(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg) {
    CommandParser::MoveCommandParams params;
    if (!CommandParser::parseMoveParams(req.params, params, err_msg)) {
        err_code = JsonRpcErrorCode::InvalidParams;
        return JsonValue(nullptr);
    }

    if (m_move_handler) {
        if (!m_move_handler(params, err_msg)) {
            err_code = JsonRpcErrorCode::MotionError;
            return JsonValue(nullptr);
        }
    }

    JsonValue res = JsonValue::object();
    res.set("queued", JsonValue(true));
    res.set("profile", JsonValue(params.profile));
    res.set("queue_depth", JsonValue(1));
    return res;
}

JsonValue JsonRpcDispatcher::handleAxisJog(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg) {
    CommandParser::JogCommandParams params;
    if (!CommandParser::parseJogParams(req.params, params, err_msg)) {
        err_code = JsonRpcErrorCode::InvalidParams;
        return JsonValue(nullptr);
    }

    if (m_jog_handler) {
        if (!m_jog_handler(params, err_msg)) {
            err_code = JsonRpcErrorCode::MotionError;
            return JsonValue(nullptr);
        }
    }

    JsonValue res = JsonValue::object();
    res.set("jogging", JsonValue(true));
    res.set("axis", JsonValue(params.axis));
    res.set("distance", JsonValue(params.distance));
    return res;
}

JsonValue JsonRpcDispatcher::handleAxisHome(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg) {
    CommandParser::HomingCommandParams params;
    if (!CommandParser::parseHomingParams(req.params, params, err_msg)) {
        err_code = JsonRpcErrorCode::InvalidParams;
        return JsonValue(nullptr);
    }

    if (m_home_handler) {
        if (!m_home_handler(params, err_msg)) {
            err_code = JsonRpcErrorCode::HomingFailed;
            return JsonValue(nullptr);
        }
    }

    JsonValue res = JsonValue::object();
    res.set("homing", JsonValue(true));
    JsonValue axes_arr = JsonValue::array();
    for (const auto& axis : params.axes) {
        axes_arr.push_back(JsonValue(axis));
    }
    res.set("axes", std::move(axes_arr));
    return res;
}

JsonValue JsonRpcDispatcher::handleAxisSetConfig(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg) {
    CommandParser::AxisConfigParams params;
    if (!CommandParser::parseConfigParams(req.params, params, err_msg)) {
        err_code = JsonRpcErrorCode::InvalidParams;
        return JsonValue(nullptr);
    }

    if (m_config_handler) {
        if (!m_config_handler(params, err_msg)) {
            err_code = JsonRpcErrorCode::InvalidConfig;
            return JsonValue(nullptr);
        }
    }

    JsonValue res = JsonValue::object();
    res.set("configured", JsonValue(true));
    res.set("axis", JsonValue(params.axis));
    return res;
}

JsonValue JsonRpcDispatcher::handleAxisGetStatus(const JsonRpcRequest& req, JsonRpcErrorCode& /*err_code*/, std::string& /*err_msg*/) {
    std::string axis_filter;
    if (req.params.is_object() && req.params.contains("axis")) {
        axis_filter = req.params.get("axis").as_string();
    }

    if (m_status_provider) {
        return m_status_provider(axis_filter);
    }

    JsonValue status = JsonValue::object();
    status.set("state", JsonValue(systemStateToString(m_state)));
    status.set("estop_active", JsonValue(isEstopActive()));

    JsonValue pos = JsonValue::object();
    pos.set("A", JsonValue(0.0));
    pos.set("B", JsonValue(0.0));
    pos.set("Z", JsonValue(0.0));
    status.set("pos", std::move(pos));

    status.set("moving", JsonValue(m_state == SystemState::MOVING));
    status.set("queue_depth", JsonValue(0));
    return status;
}

JsonValue JsonRpcDispatcher::handleTriggerCapture(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg) {
    CommandParser::TriggerCaptureParams params;
    if (!CommandParser::parseTriggerCaptureParams(req.params, params, err_msg)) {
        err_code = JsonRpcErrorCode::InvalidParams;
        return JsonValue(nullptr);
    }

    JsonValue res = JsonValue::object();
    if (m_trigger_handler) {
        if (!m_trigger_handler(params, res, err_msg)) {
            err_code = JsonRpcErrorCode::TriggerTimeout;
            return JsonValue(nullptr);
        }
    } else {
        res.set("triggered", JsonValue(true));
        res.set("settle_ms", JsonValue(params.settle_ms));
        res.set("status", JsonValue("ack"));
    }

    return res;
}

void JsonRpcDispatcher::setInfoProvider(InfoProvider provider) {
    m_info_provider = std::move(provider);
}

void JsonRpcDispatcher::setEstopAction(EstopAction action) {
    m_estop_action = std::move(action);
}

void JsonRpcDispatcher::setResetAction(ResetAction action) {
    m_reset_action = std::move(action);
}

void JsonRpcDispatcher::setAxisMoveHandler(AxisMoveHandler handler) {
    m_move_handler = std::move(handler);
}

void JsonRpcDispatcher::setAxisJogHandler(AxisJogHandler handler) {
    m_jog_handler = std::move(handler);
}

void JsonRpcDispatcher::setAxisHomeHandler(AxisHomeHandler handler) {
    m_home_handler = std::move(handler);
}

void JsonRpcDispatcher::setAxisConfigHandler(AxisConfigHandler handler) {
    m_config_handler = std::move(handler);
}

void JsonRpcDispatcher::setAxisStatusProvider(AxisStatusProvider provider) {
    m_status_provider = std::move(provider);
}

void JsonRpcDispatcher::setTriggerCaptureHandler(TriggerCaptureHandler handler) {
    m_trigger_handler = std::move(handler);
}

void JsonRpcDispatcher::registerCustomMethod(std::string method, CustomMethodHandler handler) {
    m_custom_methods[std::move(method)] = std::move(handler);
}

} // namespace motion_core::protocol
