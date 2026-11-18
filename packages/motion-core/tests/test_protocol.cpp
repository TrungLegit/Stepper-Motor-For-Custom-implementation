#include "protocol/Json.hpp"
#include "protocol/ErrorCodes.hpp"
#include "protocol/PacketBuffer.hpp"
#include "protocol/CommandParser.hpp"
#include "protocol/MeshLabSchema.hpp"
#include "protocol/TelemetryEmitter.hpp"
#include "protocol/JsonRpcDispatcher.hpp"

#include <iostream>
#include <cassert>
#include <string>
#include <vector>
#include <cmath>

using namespace motion_core::protocol;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

bool test_json_parsing_and_serialization() {
    std::cout << "[TEST] JSON parser & serializer...\n";

    // 1. Primitives
    JsonValue num(42);
    TEST_ASSERT(num.is_number(), "Number check");
    TEST_ASSERT(num.as_int() == 42, "Integer value check");

    JsonValue f(3.14159);
    TEST_ASSERT(f.is_number(), "Float check");
    TEST_ASSERT(std::abs(f.as_double() - 3.14159) < 1e-4, "Float value check");

    JsonValue str("hello world");
    TEST_ASSERT(str.is_string(), "String check");
    TEST_ASSERT(str.as_string() == "hello world", "String value check");

    JsonValue b(true);
    TEST_ASSERT(b.is_bool(), "Bool check");
    TEST_ASSERT(b.as_bool() == true, "Bool value check");

    JsonValue n(nullptr);
    TEST_ASSERT(n.is_null(), "Null check");

    // 2. Objects and Arrays
    JsonValue obj = JsonValue::object();
    obj.set("name", JsonValue("scanner"));
    obj.set("axes_count", JsonValue(3));
    obj.set("active", JsonValue(true));

    TEST_ASSERT(obj.is_object(), "Object check");
    TEST_ASSERT(obj.contains("name"), "Contains check");
    TEST_ASSERT(obj.get("axes_count").as_int() == 3, "Object value check");

    std::string serialized = obj.serialize();
    std::string err;
    JsonValue parsed = JsonValue::parse(serialized, &err);
    TEST_ASSERT(err.empty(), "Round-trip parse error: " + err);
    TEST_ASSERT(parsed.is_object(), "Parsed object check");
    TEST_ASSERT(parsed.get("name").as_string() == "scanner", "Parsed string check");
    TEST_ASSERT(parsed.get("axes_count").as_int() == 3, "Parsed int check");
    TEST_ASSERT(parsed.get("active").as_bool() == true, "Parsed bool check");

    // 3. String escaping round-trip
    std::string special = "line1\nline2\ttab \"quotes\" \\ backslash / slash";
    JsonValue str_special(special);
    std::string special_ser = str_special.serialize();
    JsonValue special_parsed = JsonValue::parse(special_ser, &err);
    TEST_ASSERT(err.empty(), "Special characters parse error: " + err);
    TEST_ASSERT(special_parsed.as_string() == special, "Special characters round-trip mismatch");

    return true;
}

bool test_packet_buffer() {
    std::cout << "[TEST] PacketBuffer & streaming E-STOP preemption...\n";

    PacketBuffer pb(256);
    TEST_ASSERT(!pb.hasLine(), "Empty buffer has no line");

    // Push fragmented chunks
    pb.push("{\"jsonrpc\": \"2.0\", \"id\": 1");
    TEST_ASSERT(!pb.hasLine(), "Incomplete line");

    pb.push(", \"method\": \"system.info\"}\n{\"jsonrpc\": \"2.0\"");
    TEST_ASSERT(pb.hasLine(), "Complete line available");

    auto line1 = pb.popLine();
    TEST_ASSERT(line1.has_value(), "Line 1 popped");
    TEST_ASSERT(*line1 == "{\"jsonrpc\": \"2.0\", \"id\": 1, \"method\": \"system.info\"}", "Line 1 content");

    TEST_ASSERT(!pb.hasLine(), "Line 2 not complete yet");
    pb.push(", \"method\": \"system.estop\"}\r\n");
    TEST_ASSERT(pb.hasLine(), "Line 2 complete with CRLF");

    // Fast E-STOP peek
    TEST_ASSERT(pb.peekEmergencyStop(), "peekEmergencyStop detects system.estop");

    auto line2 = pb.popLine();
    TEST_ASSERT(line2.has_value(), "Line 2 popped");
    TEST_ASSERT(*line2 == "{\"jsonrpc\": \"2.0\", \"method\": \"system.estop\"}", "Line 2 content stripped CRLF");

    return true;
}

bool test_json_rpc_compliance() {
    std::cout << "[TEST] JSON-RPC 2.0 Compliance (methods, errors, notifications, batch)...\n";

    JsonRpcDispatcher dispatcher;

    // 1. system.info
    std::string resp_info = dispatcher.dispatch(R"({"jsonrpc": "2.0", "id": 100, "method": "system.info"})");
    JsonValue v_info = JsonValue::parse(resp_info);
    TEST_ASSERT(v_info.get("jsonrpc").as_string() == "2.0", "JSON-RPC 2.0 tag in response");
    TEST_ASSERT(v_info.get("id").as_int() == 100, "Response ID matches");
    TEST_ASSERT(v_info.contains("result"), "Result object present");
    TEST_ASSERT(v_info.get("result").get("firmware").as_string() == "stepper-motion-core", "Firmware name");

    // 2. Notification (no id) -> Server must not reply
    std::string resp_notif = dispatcher.dispatch(R"({"jsonrpc": "2.0", "method": "system.info"})");
    TEST_ASSERT(resp_notif.empty(), "Notification must produce empty response");

    // 3. Parse error (-32700)
    std::string resp_bad_json = dispatcher.dispatch(R"({"jsonrpc": "2.0", "id": 1, "method": )");
    JsonValue v_bad_json = JsonValue::parse(resp_bad_json);
    TEST_ASSERT(v_bad_json.get("error").get("code").as_int() == -32700, "Parse error code");

    // 4. Invalid Request (-32600)
    std::string resp_invalid = dispatcher.dispatch(R"({"jsonrpc": "1.0", "id": 2, "method": "test"})");
    JsonValue v_invalid = JsonValue::parse(resp_invalid);
    TEST_ASSERT(v_invalid.get("error").get("code").as_int() == -32600, "Invalid Request code");

    // 5. Method not found (-32601)
    std::string resp_no_method = dispatcher.dispatch(R"({"jsonrpc": "2.0", "id": 3, "method": "non.existent"})");
    JsonValue v_no_method = JsonValue::parse(resp_no_method);
    TEST_ASSERT(v_no_method.get("error").get("code").as_int() == -32601, "Method not found code");

    // 6. Invalid params (-32602)
    std::string resp_bad_params = dispatcher.dispatch(R"({"jsonrpc": "2.0", "id": 4, "method": "axis.jog", "params": {}})");
    JsonValue v_bad_params = JsonValue::parse(resp_bad_params);
    TEST_ASSERT(v_bad_params.get("error").get("code").as_int() == -32602, "Invalid params code");

    // 7. Batch request
    std::string batch_req = R"([
        {"jsonrpc": "2.0", "id": 1, "method": "system.info"},
        {"jsonrpc": "2.0", "method": "system.info"},
        {"jsonrpc": "2.0", "id": 2, "method": "non.existent"}
    ])";
    std::string batch_resp = dispatcher.dispatch(batch_req);
    JsonValue v_batch = JsonValue::parse(batch_resp);
    TEST_ASSERT(v_batch.is_array(), "Batch response is array");
    TEST_ASSERT(v_batch.size() == 2, "Batch response contains exactly 2 replies (notification omitted)");
    TEST_ASSERT(v_batch[0].get("id").as_int() == 1, "Batch item 1 ID");
    TEST_ASSERT(v_batch[1].get("id").as_int() == 2, "Batch item 2 ID");

    // 8. Empty batch -> Invalid Request error
    std::string empty_batch_resp = dispatcher.dispatch("[]");
    JsonValue v_empty = JsonValue::parse(empty_batch_resp);
    TEST_ASSERT(v_empty.get("error").get("code").as_int() == -32600, "Empty batch error");

    return true;
}

bool test_safety_and_estop() {
    std::cout << "[TEST] Safety State Machine & E-STOP preemption...\n";

    JsonRpcDispatcher dispatcher;
    bool estop_hardware_callback_called = false;
    bool reset_hardware_callback_called = false;

    dispatcher.setEstopAction([&](const std::string& reason) {
        estop_hardware_callback_called = true;
        (void)reason;
    });

    dispatcher.setResetAction([&]() {
        reset_hardware_callback_called = true;
        return true;
    });

    // 1. Initial State: IDLE
    TEST_ASSERT(dispatcher.getState() == SystemState::IDLE, "Initial state IDLE");
    TEST_ASSERT(!dispatcher.isEstopActive(), "Initial ESTOP inactive");

    // 2. Normal move succeeds
    std::string move_resp = dispatcher.dispatch(R"({
        "jsonrpc": "2.0",
        "id": 10,
        "method": "axis.move",
        "params": {
            "axes": {"A": 45.0, "B": 30.0},
            "profile": "scurve",
            "feedrate": 50.0
        }
    })");
    JsonValue v_move = JsonValue::parse(move_resp);
    TEST_ASSERT(v_move.get("result").get("queued").as_bool() == true, "Move queued");

    // 3. Trigger E-STOP via JSON-RPC
    std::string estop_resp = dispatcher.dispatch(R"({
        "jsonrpc": "2.0",
        "id": 11,
        "method": "system.estop",
        "params": {"reason": "Test safety stop"}
    })");
    JsonValue v_estop = JsonValue::parse(estop_resp);
    TEST_ASSERT(v_estop.get("result").get("estop").as_bool() == true, "E-STOP result true");
    TEST_ASSERT(dispatcher.getState() == SystemState::ESTOP_LOCKED, "System in ESTOP_LOCKED state");
    TEST_ASSERT(dispatcher.isEstopActive(), "isEstopActive true");
    TEST_ASSERT(estop_hardware_callback_called, "Driver disable callback called");

    // 4. In ESTOP_LOCKED state: ALL motion commands MUST be rejected with EstopActive (-32001)
    std::string move_locked_resp = dispatcher.dispatch(R"({
        "jsonrpc": "2.0",
        "id": 12,
        "method": "axis.move",
        "params": {"axes": {"A": 10.0}}
    })");
    JsonValue v_locked_move = JsonValue::parse(move_locked_resp);
    TEST_ASSERT(v_locked_move.get("error").get("code").as_int() == -32001, "Move rejected during E-STOP with -32001");

    std::string jog_locked_resp = dispatcher.dispatch(R"({
        "jsonrpc": "2.0",
        "id": 13,
        "method": "axis.jog",
        "params": {"axis": "A", "distance": 5.0}
    })");
    JsonValue v_locked_jog = JsonValue::parse(jog_locked_resp);
    TEST_ASSERT(v_locked_jog.get("error").get("code").as_int() == -32001, "Jog rejected during E-STOP with -32001");

    std::string home_locked_resp = dispatcher.dispatch(R"({
        "jsonrpc": "2.0",
        "id": 14,
        "method": "axis.home",
        "params": {}
    })");
    JsonValue v_locked_home = JsonValue::parse(home_locked_resp);
    TEST_ASSERT(v_locked_home.get("error").get("code").as_int() == -32001, "Homing rejected during E-STOP with -32001");

    std::string trigger_locked_resp = dispatcher.dispatch(R"({
        "jsonrpc": "2.0",
        "id": 15,
        "method": "trigger.capture",
        "params": {}
    })");
    JsonValue v_locked_trigger = JsonValue::parse(trigger_locked_resp);
    TEST_ASSERT(v_locked_trigger.get("error").get("code").as_int() == -32001, "Trigger capture rejected during E-STOP");

    // 5. Query status during E-STOP is still allowed
    std::string status_resp = dispatcher.dispatch(R"({"jsonrpc": "2.0", "id": 16, "method": "axis.get_status"})");
    JsonValue v_status = JsonValue::parse(status_resp);
    TEST_ASSERT(v_status.contains("result"), "Status query allowed during E-STOP");
    TEST_ASSERT(v_status.get("result").get("estop_active").as_bool() == true, "Status reports estop_active true");

    // 6. Explicit system.reset clears E-STOP
    std::string reset_resp = dispatcher.dispatch(R"({"jsonrpc": "2.0", "id": 17, "method": "system.reset"})");
    JsonValue v_reset = JsonValue::parse(reset_resp);
    TEST_ASSERT(v_reset.get("result").get("reset").as_bool() == true, "System reset acknowledged");
    TEST_ASSERT(dispatcher.getState() == SystemState::IDLE, "State back to IDLE");
    TEST_ASSERT(!dispatcher.isEstopActive(), "ESTOP inactive");
    TEST_ASSERT(reset_hardware_callback_called, "Driver re-enable callback called");

    // 7. Motion commands operational again
    std::string move_post_reset = dispatcher.dispatch(R"({
        "jsonrpc": "2.0",
        "id": 18,
        "method": "axis.move",
        "params": {"axes": {"A": 0.0}}
    })");
    JsonValue v_post_reset = JsonValue::parse(move_post_reset);
    TEST_ASSERT(v_post_reset.get("result").get("queued").as_bool() == true, "Move accepted after reset");

    return true;
}

bool test_telemetry_emitter() {
    std::cout << "[TEST] Asynchronous Telemetry Emitter...\n";

    TelemetryEmitter emitter;
    std::vector<std::string> received_events;

    emitter.addSink([&](const std::string& json) {
        received_events.push_back(json);
    });

    // 1. Motion status
    MotionTelemetryData mtd;
    mtd.pos = {{"A", 45.2f}, {"B", 30.0f}, {"Z", 0.0f}};
    mtd.vel = {{"A", 10.0f}, {"B", 5.0f}, {"Z", 0.0f}};
    mtd.moving = true;
    mtd.homing = false;
    mtd.estop_active = false;
    mtd.queue_depth = 2;
    emitter.emitMotionStatus(mtd);

    TEST_ASSERT(received_events.size() == 1, "Event count 1");
    JsonValue v1 = JsonValue::parse(received_events.back());
    TEST_ASSERT(v1.get("event").as_string() == "motion.status", "motion.status event");
    TEST_ASSERT(v1.get("data").get("moving").as_bool() == true, "moving true");
    TEST_ASSERT(std::abs(v1.get("data").get("pos").get("A").as_float() - 45.2f) < 1e-3, "Axis A position");

    // 2. Queue depth
    emitter.emitQueueDepth(3, 16);
    TEST_ASSERT(received_events.size() == 2, "Event count 2");
    JsonValue v2 = JsonValue::parse(received_events.back());
    TEST_ASSERT(v2.get("event").as_string() == "motion.queue_depth", "queue_depth event");
    TEST_ASSERT(v2.get("data").get("depth").as_int() == 3, "depth value");

    // 3. Homing progress
    emitter.emitHomingProgress("A", "slow_approach", -0.5f, false);
    TEST_ASSERT(received_events.size() == 3, "Event count 3");
    JsonValue v3 = JsonValue::parse(received_events.back());
    TEST_ASSERT(v3.get("event").as_string() == "homing.progress", "homing.progress event");
    TEST_ASSERT(v3.get("data").get("phase").as_string() == "slow_approach", "homing phase");

    // 4. Endstop alert (Hall sensor trigger)
    emitter.emitEndstopAlert("A", 4, "hall_sensor", true);
    TEST_ASSERT(received_events.size() == 4, "Event count 4");
    JsonValue v4 = JsonValue::parse(received_events.back());
    TEST_ASSERT(v4.get("event").as_string() == "endstop.alert", "endstop.alert event");
    TEST_ASSERT(v4.get("data").get("trigger_type").as_string() == "hall_sensor", "sensor type");

    // 5. E-STOP alert
    emitter.emitEstop("Limit hit");
    TEST_ASSERT(received_events.size() == 5, "Event count 5");
    JsonValue v5 = JsonValue::parse(received_events.back());
    TEST_ASSERT(v5.get("event").as_string() == "system.estop", "system.estop event");

    return true;
}

bool test_meshlab_and_3d_scanning_schema() {
    std::cout << "[TEST] MeshLab & 3D Scanning Schema (PLY, COLMAP, RealityCapture)...\n";

    ScanPoseRecord pose;
    pose.frame_id = 42;
    pose.image_file = "frame_0042.jpg";
    pose.timestamp_iso = "2026-11-18T11:30:22.456Z";
    pose.azimuth_deg = 128.57;
    pose.elevation_deg = 34.21;
    pose.focus_z_mm = 0.0;
    pose.motor_steps_abs = {{"A", 18234}, {"B", 9821}, {"C", 0}};
    pose.motor_pos_calibrated = {{"A_deg", 128.57}, {"B_deg", 34.21}, {"C_mm", 0.0}};

    // Calculate 6-DoF extrinsics
    pose.extrinsics = SphericalExtrinsicCalculator::calculate(pose.azimuth_deg, pose.elevation_deg, 0.20);

    // 1. JSON representation
    JsonValue json = pose.toJson();
    TEST_ASSERT(json.get("frame_id").as_int() == 42, "frame_id in JSON");
    TEST_ASSERT(json.get("pose").get("azimuth_deg").as_double() == 128.57, "azimuth in JSON");
    TEST_ASSERT(json.get("camera_extrinsic").contains("quaternion_wxyz"), "quaternion in JSON");
    TEST_ASSERT(json.get("camera_extrinsic").contains("translation_xyz_m"), "translation in JSON");

    // 2. COLMAP images.txt line
    std::string colmap_line = pose.toColmapImageLine(1);
    TEST_ASSERT(colmap_line.find("42 ") == 0, "COLMAP image id header");
    TEST_ASSERT(colmap_line.find("frame_0042.jpg") != std::string::npos, "COLMAP image filename");

    // 3. RealityCapture CSV line
    std::string rc_line = pose.toRealityCaptureCsvLine();
    TEST_ASSERT(rc_line.find("frame_0042.jpg,") != std::string::npos, "RealityCapture filename");

    // 4. MeshLab .ply export
    std::vector<ScanPoseRecord> poses = {pose};
    std::string ply_content = MeshLabPlyBuilder::buildCameraCloudPly(poses);
    TEST_ASSERT(ply_content.find("ply\nformat ascii 1.0\n") == 0, "PLY header valid");
    TEST_ASSERT(ply_content.find("element vertex 1\n") != std::string::npos, "PLY vertex count 1");
    TEST_ASSERT(ply_content.find("end_header\n") != std::string::npos, "PLY end_header valid");

    return true;
}

int main() {
    std::cout << "========================================\n";
    std::cout << " RUNNING PROTOCOL & SAFETY UNIT TESTS   \n";
    std::cout << "========================================\n";

    if (!test_json_parsing_and_serialization()) return 1;
    if (!test_packet_buffer()) return 1;
    if (!test_json_rpc_compliance()) return 1;
    if (!test_safety_and_estop()) return 1;
    if (!test_telemetry_emitter()) return 1;
    if (!test_meshlab_and_3d_scanning_schema()) return 1;

    std::cout << "========================================\n";
    std::cout << " ALL PROTOCOL & SAFETY TESTS PASSED!    \n";
    std::cout << "========================================\n";
    return 0;
}
