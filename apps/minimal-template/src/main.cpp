#include "motion_core.hpp"
#include "MyBoardPinmap.hpp"
#include "MyKinematics.hpp"
#include <iostream>
#include <string>
#include <memory>
#include <array>

using namespace motion_core;
using namespace motion_core::protocol;
using namespace my_rig;

/**
 * @brief Global motion and protocol controller instances.
 */
static std::unique_ptr<MotionPlanner> g_planner;
static std::unique_ptr<JsonRpcDispatcher> g_dispatcher;

/**
 * @brief Map axis name ("A", "B", "Z") to numeric axis ID (0, 1, 2).
 */
static int8_t getAxisIdByName(const std::string& name) {
    if (name == "A" || name == "a" || name == "0") return 0;
    if (name == "B" || name == "b" || name == "1") return 1;
    if (name == "Z" || name == "z" || name == "2") return 2;
    return -1;
}

/**
 * @brief Wire JsonRpcDispatcher callbacks to MotionPlanner instance.
 */
static void wireProtocolToPlanner(JsonRpcDispatcher& dispatcher, MotionPlanner& planner) {
    // 1. Move command handler
    dispatcher.setAxisMoveHandler([&planner](const CommandParser::MoveCommandParams& params, std::string& err) {
        std::array<float, MAX_AXES> targets;
        for (uint8_t a = 0; a < MAX_AXES; ++a) {
            targets[a] = planner.getPosition(a);
        }

        for (const auto& [axis_name, coord] : params.axes) {
            int8_t aid = getAxisIdByName(axis_name);
            if (aid < 0 || aid >= MyMachineConfig::NUM_AXES) {
                err = "Invalid axis name: " + axis_name;
                return false;
            }
            targets[aid] = coord;
        }

        ProfileType prof = (params.profile == "trapezoidal") ? ProfileType::TRAPEZOIDAL : ProfileType::SCURVE;
        if (params.relative) {
            std::array<float, MAX_AXES> deltas{};
            for (const auto& [axis_name, delta] : params.axes) {
                int8_t aid = getAxisIdByName(axis_name);
                if (aid >= 0 && aid < MyMachineConfig::NUM_AXES) {
                    deltas[aid] = delta;
                }
            }
            return planner.moveRelative(deltas, params.feedrate_mm_s, prof);
        } else {
            return planner.moveTo(targets, params.feedrate_mm_s, prof);
        }
    });

    // 2. Jog command handler
    dispatcher.setAxisJogHandler([&planner](const CommandParser::JogCommandParams& params, std::string& err) {
        int8_t aid = getAxisIdByName(params.axis);
        if (aid < 0 || aid >= MyMachineConfig::NUM_AXES) {
            err = "Invalid jog axis: " + params.axis;
            return false;
        }
        return planner.jog(static_cast<uint8_t>(aid), params.distance, params.feedrate_mm_s);
    });

    // 3. Homing handler
    dispatcher.setAxisHomeHandler([&planner](const CommandParser::HomingCommandParams& params, std::string& err) {
        if (params.axes.empty()) {
            return planner.homeAll();
        }
        for (const auto& axis_name : params.axes) {
            int8_t aid = getAxisIdByName(axis_name);
            if (aid < 0 || aid >= MyMachineConfig::NUM_AXES) {
                err = "Invalid homing axis: " + axis_name;
                return false;
            }
            if (!planner.home(static_cast<uint8_t>(aid))) {
                err = "Homing failed on axis " + axis_name;
                return false;
            }
        }
        return true;
    });

    // 4. Safety E-STOP preemption
    dispatcher.setEstopAction([&planner](const std::string& /*reason*/) {
        planner.emergencyStop();
    });

    // 5. System Reset
    dispatcher.setResetAction([&planner]() {
        planner.resetEstop();
        return true;
    });

    // 6. Live Status Provider
    dispatcher.setAxisStatusProvider([&planner, &dispatcher](const std::string& /*axis_filter*/) {
        MotionStatus ms = planner.getStatus();
        JsonValue status = JsonValue::object();
        status.set("state", JsonValue(systemStateToString(dispatcher.getState())));
        status.set("estop_active", JsonValue(dispatcher.isEstopActive()));
        status.set("moving", JsonValue(planner.isMoving()));
        status.set("homing", JsonValue(planner.isHoming()));
        status.set("queue_depth", JsonValue(ms.queue_depth));

        JsonValue pos = JsonValue::object();
        pos.set("A", JsonValue(planner.getPosition(0)));
        pos.set("B", JsonValue(planner.getPosition(1)));
        pos.set("Z", JsonValue(planner.getPosition(2)));
        status.set("pos", std::move(pos));

        return status;
    });
}

/**
 * @brief Initialize all scanner subsystems, HAL, MotionPlanner, and JSON-RPC.
 */
void app_init() {
    std::cout << "[INFO] Initializing stepper-motion-core on ESP32-S3...\n";

    g_planner = std::make_unique<MotionPlanner>();
    g_dispatcher = std::make_unique<JsonRpcDispatcher>();

    // Configure 3-axis scanning rig
    for (const auto& cfg : MyMachineConfig::getAxesConfig()) {
        g_planner->configureAxis(cfg);
    }

    // Connect JSON-RPC 2.0 dispatcher to MotionPlanner
    wireProtocolToPlanner(*g_dispatcher, *g_planner);

    // Setup telemetry output stream
    g_dispatcher->telemetry().addSink([](const std::string& event_json) {
        // Output to USB-Serial (stdout) or WebSocket
        std::cout << event_json << "\n";
    });

    // Setup JSON-RPC response sink
    g_dispatcher->setResponseSink([](const std::string& response_json) {
        std::cout << response_json;
    });

    std::cout << "[INFO] System initialized. Ready for JSON-RPC 2.0 commands.\n";
}

/**
 * @brief Main execution loop (USB-Serial + Wi-Fi WebSocket stream processing).
 */
void app_loop() {
    // 1. Advance motion planner state machine
    if (g_planner) {
        g_planner->update();
    }

    // 2. Feed stream data from UART / USB-CDC / WebSocket to dispatcher
    // Example: g_dispatcher->feedStream(rx_buffer, rx_len);
}

#ifdef ESP_PLATFORM
extern "C" void app_main() {
    app_init();
    while (true) {
        app_loop();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#else
int main() {
    app_init();
    app_loop();
    std::cout << "[INFO] Template sanity check passed.\n";
    return 0;
}
#endif
