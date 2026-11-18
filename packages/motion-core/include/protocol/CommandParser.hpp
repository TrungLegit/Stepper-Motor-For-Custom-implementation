#pragma once

#include "protocol/Json.hpp"
#include "protocol/ErrorCodes.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <optional>
#include <utility>

namespace motion_core::protocol {

/**
 * @brief Represents a parsed JSON-RPC 2.0 Request.
 */
struct JsonRpcRequest {
    std::string jsonrpc{"2.0"};
    JsonValue id;
    std::string method;
    JsonValue params;
    bool is_notification{false};
};

/**
 * @brief Represents a JSON-RPC 2.0 Response.
 */
struct JsonRpcResponse {
    struct ErrorPayload {
        int32_t code{0};
        std::string message;
        JsonValue data;
    };

    std::string jsonrpc{"2.0"};
    JsonValue id;
    std::optional<JsonValue> result;
    std::optional<ErrorPayload> error;

    std::string serialize() const;
};

/**
 * @brief Parser and serializer for JSON-RPC 2.0 command frames and typed parameter models.
 */
class CommandParser {
public:
    struct MoveCommandParams {
        std::vector<std::pair<std::string, float>> axes; // Map of axis name -> target
        std::string profile{"scurve"};                  // "scurve" or "trapezoidal"
        float feedrate_mm_s{0.0f};                      // 0 = default max
        bool relative{false};                           // true for relative move
    };

    struct JogCommandParams {
        std::string axis;
        float distance{0.0f};                           // relative step/distance
        float feedrate_mm_s{0.0f};                      // 0 = default jog speed
    };

    struct HomingCommandParams {
        std::vector<std::string> axes;                  // empty = all axes
    };

    struct AxisConfigParams {
        std::string axis;
        std::optional<float> steps_per_mm;
        std::optional<float> max_speed_mm_s;
        std::optional<float> max_accel_mm_s2;
        std::optional<float> max_jerk_mm_s3;
        std::optional<float> backlash_mm;
    };

    struct TriggerCaptureParams {
        uint32_t settle_ms{100};
        uint32_t pulse_us{50000};
        uint32_t timeout_ms{5000};
    };

    /**
     * @brief Parse and validate a raw JSON string into a JSON-RPC 2.0 Request.
     * @param raw_json Incoming payload.
     * @param req Output parsed request object.
     * @param err_resp Output error response if parsing/validation fails.
     * @return true if valid request, false if error response generated.
     */
    static bool parseRequest(std::string_view raw_json, JsonRpcRequest& req, JsonRpcResponse& err_resp);

    /**
     * @brief Parse parameters for "axis.move" method.
     */
    static bool parseMoveParams(const JsonValue& params, MoveCommandParams& out, std::string& err_msg);

    /**
     * @brief Parse parameters for "axis.jog" method.
     */
    static bool parseJogParams(const JsonValue& params, JogCommandParams& out, std::string& err_msg);

    /**
     * @brief Parse parameters for "axis.home" method.
     */
    static bool parseHomingParams(const JsonValue& params, HomingCommandParams& out, std::string& err_msg);

    /**
     * @brief Parse parameters for "axis.set_config" method.
     */
    static bool parseConfigParams(const JsonValue& params, AxisConfigParams& out, std::string& err_msg);

    /**
     * @brief Parse parameters for "trigger.capture" method.
     */
    static bool parseTriggerCaptureParams(const JsonValue& params, TriggerCaptureParams& out, std::string& err_msg);

    /**
     * @brief Format a standard JSON-RPC 2.0 success response.
     */
    static std::string formatSuccess(const JsonValue& id, const JsonValue& result);

    /**
     * @brief Format a standard JSON-RPC 2.0 error response.
     */
    static std::string formatError(const JsonValue& id,
                                   JsonRpcErrorCode code,
                                   std::string_view custom_message = "",
                                   const JsonValue& data = JsonValue());
};

} // namespace motion_core::protocol
