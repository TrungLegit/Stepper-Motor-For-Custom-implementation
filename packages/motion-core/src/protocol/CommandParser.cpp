#include "protocol/CommandParser.hpp"

namespace motion_core::protocol {

std::string JsonRpcResponse::serialize() const {
    JsonValue root = JsonValue::object();
    root.set("jsonrpc", JsonValue("2.0"));
    root.set("id", id);

    if (error.has_value()) {
        JsonValue err_obj = JsonValue::object();
        err_obj.set("code", JsonValue(error->code));
        err_obj.set("message", JsonValue(error->message));
        if (!error->data.is_null()) {
            err_obj.set("data", error->data);
        }
        root.set("error", std::move(err_obj));
    } else if (result.has_value()) {
        root.set("result", *result);
    } else {
        root.set("result", JsonValue(nullptr));
    }

    return root.serialize();
}

bool CommandParser::parseRequest(std::string_view raw_json, JsonRpcRequest& req, JsonRpcResponse& err_resp) {
    std::string parse_err;
    JsonValue root = JsonValue::parse(raw_json, &parse_err);

    if (root.is_null() && !parse_err.empty()) {
        err_resp.id = JsonValue(nullptr);
        err_resp.error = JsonRpcResponse::ErrorPayload{
            static_cast<int32_t>(JsonRpcErrorCode::ParseError),
            std::string(getErrorMessage(JsonRpcErrorCode::ParseError)) + ": " + parse_err,
            JsonValue(nullptr)
        };
        return false;
    }

    if (!root.is_object()) {
        err_resp.id = JsonValue(nullptr);
        err_resp.error = JsonRpcResponse::ErrorPayload{
            static_cast<int32_t>(JsonRpcErrorCode::InvalidRequest),
            "Request must be a JSON object",
            JsonValue(nullptr)
        };
        return false;
    }

    // Check "jsonrpc" version member
    const JsonValue* v_rpc = root.find("jsonrpc");
    if (!v_rpc || !v_rpc->is_string() || v_rpc->as_string() != "2.0") {
        err_resp.id = root.contains("id") ? root["id"] : JsonValue(nullptr);
        err_resp.error = JsonRpcResponse::ErrorPayload{
            static_cast<int32_t>(JsonRpcErrorCode::InvalidRequest),
            "Member 'jsonrpc' must be exactly '2.0'",
            JsonValue(nullptr)
        };
        return false;
    }
    req.jsonrpc = "2.0";

    // Check "method" member
    const JsonValue* v_method = root.find("method");
    if (!v_method || !v_method->is_string() || v_method->as_string().empty()) {
        err_resp.id = root.contains("id") ? root["id"] : JsonValue(nullptr);
        err_resp.error = JsonRpcResponse::ErrorPayload{
            static_cast<int32_t>(JsonRpcErrorCode::InvalidRequest),
            "Member 'method' must be a non-empty string",
            JsonValue(nullptr)
        };
        return false;
    }
    req.method = v_method->as_string();

    // Check "id" member
    if (root.contains("id")) {
        req.id = root["id"];
        req.is_notification = false;
    } else {
        req.id = JsonValue(nullptr);
        req.is_notification = true;
    }

    // Check "params" member (optional)
    if (root.contains("params")) {
        req.params = root["params"];
    } else {
        req.params = JsonValue::object();
    }

    return true;
}

bool CommandParser::parseMoveParams(const JsonValue& params, MoveCommandParams& out, std::string& err_msg) {
    if (!params.is_object()) {
        err_msg = "'params' for axis.move must be an object";
        return false;
    }

    out.axes.clear();
    out.profile = "scurve";
    out.feedrate_mm_s = 0.0f;
    out.relative = false;

    // Check "profile"
    if (params.contains("profile")) {
        out.profile = params.get("profile").as_string("scurve");
    }

    // Check "feedrate"
    if (params.contains("feedrate")) {
        out.feedrate_mm_s = params.get("feedrate").as_float(0.0f);
    }

    // Check "relative"
    if (params.contains("relative")) {
        out.relative = params.get("relative").as_bool(false);
    }

    // Check "axes": either nested under "axes": { "A": 90.0, ... } or directly in params
    const JsonValue* axes_val = params.find("axes");
    if (axes_val && axes_val->is_object()) {
        for (const auto& [axis_name, val] : axes_val->as_object()) {
            if (val.is_number()) {
                out.axes.emplace_back(axis_name, val.as_float());
            }
        }
    } else {
        // Direct keys in params that are numbers (e.g. "A": 90.0, "B": 45.0)
        for (const auto& [axis_name, val] : params.as_object()) {
            if (axis_name != "profile" && axis_name != "feedrate" && axis_name != "relative") {
                if (val.is_number()) {
                    out.axes.emplace_back(axis_name, val.as_float());
                }
            }
        }
    }

    if (out.axes.empty()) {
        err_msg = "No target axis coordinates specified in move command";
        return false;
    }

    return true;
}

bool CommandParser::parseJogParams(const JsonValue& params, JogCommandParams& out, std::string& err_msg) {
    if (!params.is_object()) {
        err_msg = "'params' for axis.jog must be an object";
        return false;
    }

    if (!params.contains("axis") || !params.get("axis").is_string()) {
        err_msg = "Missing 'axis' string in jog parameters";
        return false;
    }
    out.axis = params.get("axis").as_string();

    if (!params.contains("distance") || !params.get("distance").is_number()) {
        err_msg = "Missing numeric 'distance' in jog parameters";
        return false;
    }
    out.distance = params.get("distance").as_float();

    if (params.contains("feedrate") && params.get("feedrate").is_number()) {
        out.feedrate_mm_s = params.get("feedrate").as_float(0.0f);
    } else {
        out.feedrate_mm_s = 0.0f;
    }

    return true;
}

bool CommandParser::parseHomingParams(const JsonValue& params, HomingCommandParams& out, std::string& /*err_msg*/) {
    out.axes.clear();
    if (params.is_object()) {
        if (params.contains("axes")) {
            const auto& axes_val = params.get("axes");
            if (axes_val.is_array()) {
                for (const auto& item : axes_val.as_array()) {
                    if (item.is_string()) {
                        out.axes.push_back(item.as_string());
                    }
                }
            } else if (axes_val.is_string()) {
                out.axes.push_back(axes_val.as_string());
            }
        } else if (params.contains("axis")) {
            const auto& axis_val = params.get("axis");
            if (axis_val.is_string()) {
                out.axes.push_back(axis_val.as_string());
            }
        }
    } else if (params.is_array()) {
        for (const auto& item : params.as_array()) {
            if (item.is_string()) {
                out.axes.push_back(item.as_string());
            }
        }
    }
    return true;
}

bool CommandParser::parseConfigParams(const JsonValue& params, AxisConfigParams& out, std::string& err_msg) {
    if (!params.is_object()) {
        err_msg = "'params' for axis.set_config must be an object";
        return false;
    }

    if (!params.contains("axis") || !params.get("axis").is_string()) {
        err_msg = "Missing 'axis' string in set_config parameters";
        return false;
    }
    out.axis = params.get("axis").as_string();

    if (params.contains("steps_per_mm") && params.get("steps_per_mm").is_number()) {
        out.steps_per_mm = params.get("steps_per_mm").as_float();
    }
    if (params.contains("max_speed_mm_s") && params.get("max_speed_mm_s").is_number()) {
        out.max_speed_mm_s = params.get("max_speed_mm_s").as_float();
    }
    if (params.contains("max_accel_mm_s2") && params.get("max_accel_mm_s2").is_number()) {
        out.max_accel_mm_s2 = params.get("max_accel_mm_s2").as_float();
    }
    if (params.contains("max_jerk_mm_s3") && params.get("max_jerk_mm_s3").is_number()) {
        out.max_jerk_mm_s3 = params.get("max_jerk_mm_s3").as_float();
    }
    if (params.contains("backlash_mm") && params.get("backlash_mm").is_number()) {
        out.backlash_mm = params.get("backlash_mm").as_float();
    }

    return true;
}

bool CommandParser::parseTriggerCaptureParams(const JsonValue& params, TriggerCaptureParams& out, std::string& /*err_msg*/) {
    out.settle_ms = 100;
    out.pulse_us = 50000;
    out.timeout_ms = 5000;

    if (params.is_object()) {
        if (params.contains("settle_ms") && params.get("settle_ms").is_number()) {
            out.settle_ms = static_cast<uint32_t>(params.get("settle_ms").as_int(100));
        }
        if (params.contains("pulse_us") && params.get("pulse_us").is_number()) {
            out.pulse_us = static_cast<uint32_t>(params.get("pulse_us").as_int(50000));
        }
        if (params.contains("timeout_ms") && params.get("timeout_ms").is_number()) {
            out.timeout_ms = static_cast<uint32_t>(params.get("timeout_ms").as_int(5000));
        }
    }
    return true;
}

std::string CommandParser::formatSuccess(const JsonValue& id, const JsonValue& result) {
    JsonRpcResponse resp;
    resp.id = id;
    resp.result = result;
    return resp.serialize();
}

std::string CommandParser::formatError(const JsonValue& id,
                                         JsonRpcErrorCode code,
                                         std::string_view custom_message,
                                         const JsonValue& data) {
    JsonRpcResponse resp;
    resp.id = id;
    std::string msg = custom_message.empty() ? std::string(getErrorMessage(code)) : std::string(custom_message);
    resp.error = JsonRpcResponse::ErrorPayload{
        static_cast<int32_t>(code),
        std::move(msg),
        data
    };
    return resp.serialize();
}

} // namespace motion_core::protocol
