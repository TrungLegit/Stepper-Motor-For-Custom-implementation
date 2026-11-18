#pragma once

#include "protocol/Json.hpp"
#include "protocol/ErrorCodes.hpp"
#include "protocol/CommandParser.hpp"
#include "protocol/TelemetryEmitter.hpp"
#include "protocol/PacketBuffer.hpp"
#include <string>
#include <string_view>
#include <functional>
#include <unordered_map>
#include <memory>
#include <atomic>

namespace motion_core::protocol {

enum class SystemState {
    IDLE,
    MOVING,
    HOMING,
    ESTOP_LOCKED,
    ERROR
};

inline const char* systemStateToString(SystemState s) noexcept {
    switch (s) {
        case SystemState::IDLE: return "idle";
        case SystemState::MOVING: return "moving";
        case SystemState::HOMING: return "homing";
        case SystemState::ESTOP_LOCKED: return "estop_locked";
        case SystemState::ERROR: return "error";
    }
    return "unknown";
}

/**
 * @brief High-performance JSON-RPC 2.0 Dispatcher and Safety Controller.
 *        Guarantees E-STOP preemption, locked safety state, and clean delegation
 *        to MotionPlanner and HAL.
 */
class JsonRpcDispatcher {
public:
    // Delegate callbacks for HAL / MotionPlanner injection
    using InfoProvider = std::function<JsonValue()>;
    using EstopAction = std::function<void(const std::string& reason)>;
    using ResetAction = std::function<bool()>;
    using AxisMoveHandler = std::function<bool(const CommandParser::MoveCommandParams& params, std::string& error_msg)>;
    using AxisJogHandler = std::function<bool(const CommandParser::JogCommandParams& params, std::string& error_msg)>;
    using AxisHomeHandler = std::function<bool(const CommandParser::HomingCommandParams& params, std::string& error_msg)>;
    using AxisConfigHandler = std::function<bool(const CommandParser::AxisConfigParams& params, std::string& error_msg)>;
    using AxisStatusProvider = std::function<JsonValue(const std::string& axis)>;
    using TriggerCaptureHandler = std::function<bool(const CommandParser::TriggerCaptureParams& params, JsonValue& result, std::string& error_msg)>;
    using CustomMethodHandler = std::function<bool(const JsonValue& params, JsonValue& result, JsonRpcErrorCode& error_code, std::string& error_msg)>;
    using ResponseOutputSink = std::function<void(const std::string& response_json)>;

    JsonRpcDispatcher();

    // -------------------------------------------------------------------------
    // Execution & Dispatching
    // -------------------------------------------------------------------------

    /**
     * @brief Dispatch a single JSON-RPC 2.0 request or batch request string.
     * @param raw_json Raw JSON payload.
     * @return Formatted JSON-RPC 2.0 response string, or empty string if notification.
     */
    std::string dispatch(std::string_view raw_json);

    /**
     * @brief Feed streaming bytes from UART / USB CDC / TCP socket.
     *        Preemptively detects E-STOP, extracts lines, dispatches commands,
     *        and emits responses via registered ResponseOutputSink.
     */
    void feedStream(const char* data, size_t length);
    void feedStream(std::string_view chunk);

    /**
     * @brief Set response output sink for stream feed.
     */
    void setResponseSink(ResponseOutputSink sink);

    // -------------------------------------------------------------------------
    // Safety & E-STOP Control
    // -------------------------------------------------------------------------

    /**
     * @brief Trigger Emergency Stop immediately.
     *        Bypasses command queue, calls registered EstopAction (driver disable),
     *        transitions to ESTOP_LOCKED state, and emits telemetry.
     */
    void triggerEstop(const std::string& reason = "Manual E-STOP triggered");

    /**
     * @brief Reset system from ESTOP_LOCKED or ERROR state to IDLE.
     *        Invokes registered ResetAction (driver re-enable) and emits telemetry.
     * @return true if successfully reset, false if reset rejected.
     */
    bool resetSystem();

    /**
     * @brief Query current system state.
     */
    SystemState getState() const noexcept { return m_state; }

    /**
     * @brief Check if E-STOP is active.
     */
    bool isEstopActive() const noexcept { return m_state == SystemState::ESTOP_LOCKED; }

    /**
     * @brief Set current system state (e.g. MOVING or IDLE from MotionPlanner).
     */
    void setState(SystemState new_state) noexcept;

    // -------------------------------------------------------------------------
    // Telemetry & Buffer Access
    // -------------------------------------------------------------------------

    TelemetryEmitter& telemetry() noexcept { return m_telemetry; }
    const TelemetryEmitter& telemetry() const noexcept { return m_telemetry; }

    PacketBuffer& packetBuffer() noexcept { return m_packet_buffer; }
    const PacketBuffer& packetBuffer() const noexcept { return m_packet_buffer; }

    // -------------------------------------------------------------------------
    // Delegation / Integration Hooks
    // -------------------------------------------------------------------------

    void setInfoProvider(InfoProvider provider);
    void setEstopAction(EstopAction action);
    void setResetAction(ResetAction action);
    void setAxisMoveHandler(AxisMoveHandler handler);
    void setAxisJogHandler(AxisJogHandler handler);
    void setAxisHomeHandler(AxisHomeHandler handler);
    void setAxisConfigHandler(AxisConfigHandler handler);
    void setAxisStatusProvider(AxisStatusProvider provider);
    void setTriggerCaptureHandler(TriggerCaptureHandler handler);
    void registerCustomMethod(std::string method, CustomMethodHandler handler);

private:
    std::string dispatchSingle(const JsonValue& req_val);
    std::string executeMethod(const JsonRpcRequest& req);

    // Built-in core handlers
    JsonValue handleSystemInfo(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);
    JsonValue handleSystemEstop(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);
    JsonValue handleSystemReset(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);
    JsonValue handleAxisMove(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);
    JsonValue handleAxisJog(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);
    JsonValue handleAxisHome(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);
    JsonValue handleAxisSetConfig(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);
    JsonValue handleAxisGetStatus(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);
    JsonValue handleTriggerCapture(const JsonRpcRequest& req, JsonRpcErrorCode& err_code, std::string& err_msg);

    SystemState m_state{SystemState::IDLE};
    std::string m_estop_reason;

    TelemetryEmitter m_telemetry;
    PacketBuffer m_packet_buffer;
    ResponseOutputSink m_response_sink;

    // Delegation hooks
    InfoProvider m_info_provider;
    EstopAction m_estop_action;
    ResetAction m_reset_action;
    AxisMoveHandler m_move_handler;
    AxisJogHandler m_jog_handler;
    AxisHomeHandler m_home_handler;
    AxisConfigHandler m_config_handler;
    AxisStatusProvider m_status_provider;
    TriggerCaptureHandler m_trigger_handler;
    std::unordered_map<std::string, CustomMethodHandler> m_custom_methods;
};

} // namespace motion_core::protocol
