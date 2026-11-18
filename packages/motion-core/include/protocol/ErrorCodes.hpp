#pragma once

#include <cstdint>
#include <string_view>

namespace motion_core::protocol {

/**
 * @brief Standard JSON-RPC 2.0 error codes and motion-core domain-specific error codes.
 */
enum class JsonRpcErrorCode : int32_t {
    // Standard JSON-RPC 2.0 specifications
    ParseError     = -32700, // Invalid JSON was received by the server
    InvalidRequest = -32600, // The JSON sent is not a valid Request object
    MethodNotFound = -32601, // The method does not exist / is not available
    InvalidParams  = -32602, // Invalid method parameter(s)
    InternalError  = -32603, // Internal JSON-RPC error

    // Server-defined motion core domain errors (-32000 to -32099)
    EstopActive    = -32001, // Emergency Stop is active. System locked until system.reset
    EstopTriggered = -32002, // Emergency Stop triggered by safety input or command
    AxisNotFound   = -32010, // Axis identifier not found or invalid
    AxisBusy       = -32011, // Axis or motion planner is busy
    QueueFull      = -32012, // Motion command queue is full
    LimitSwitchHit = -32013, // Hardware limit switch or endstop triggered
    MotionError    = -32014, // Motion planning or kinematics execution error
    HomingFailed   = -32015, // Homing sequence failed or timed out
    InvalidConfig  = -32016, // Invalid axis configuration parameter
    HardwareFault  = -32017, // Driver or hardware fault detected (e.g., TMC overtemp/short)
    TriggerTimeout = -32018  // Camera trigger ACK timeout
};

/**
 * @brief Get human-readable description for an error code.
 */
inline constexpr std::string_view getErrorMessage(JsonRpcErrorCode code) noexcept {
    switch (code) {
        case JsonRpcErrorCode::ParseError:
            return "Parse error";
        case JsonRpcErrorCode::InvalidRequest:
            return "Invalid Request";
        case JsonRpcErrorCode::MethodNotFound:
            return "Method not found";
        case JsonRpcErrorCode::InvalidParams:
            return "Invalid params";
        case JsonRpcErrorCode::InternalError:
            return "Internal error";
        case JsonRpcErrorCode::EstopActive:
            return "E-STOP active: motion commands locked until system.reset";
        case JsonRpcErrorCode::EstopTriggered:
            return "Emergency stop triggered";
        case JsonRpcErrorCode::AxisNotFound:
            return "Axis not found";
        case JsonRpcErrorCode::AxisBusy:
            return "Axis busy";
        case JsonRpcErrorCode::QueueFull:
            return "Command queue full";
        case JsonRpcErrorCode::LimitSwitchHit:
            return "Limit switch triggered";
        case JsonRpcErrorCode::MotionError:
            return "Motion error";
        case JsonRpcErrorCode::HomingFailed:
            return "Homing failed";
        case JsonRpcErrorCode::InvalidConfig:
            return "Invalid configuration";
        case JsonRpcErrorCode::HardwareFault:
            return "Hardware fault";
        case JsonRpcErrorCode::TriggerTimeout:
            return "Camera trigger timeout";
        default:
            return "Unknown error";
    }
}

} // namespace motion_core::protocol
