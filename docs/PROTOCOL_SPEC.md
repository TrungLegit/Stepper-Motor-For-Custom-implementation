# PROTOCOL_SPEC.md — Embedded JSON-RPC 2.0 Command & Telemetry Specification

The `stepper-motion-core` framework uses a lightweight, structured **JSON-RPC 2.0** protocol over USB-CDC Serial and Wi-Fi WebSockets.

---

## 1. Frame Framing & Safety Preemption

* **Framing**: Messages are terminated by newline (`\n`).
* **High-Priority Emergency Stop (`E-STOP`)**: The low-level transport parser (`PacketBuffer`) scans every incoming chunk for `"system.estop"` before JSON parsing. It preempts all queued commands, halts the step generator timer, and turns off motor power in $< 10\,\mu\text{s}$.

---

## 2. Command Reference

### `system.info`
Query device capability and firmware version.
* **Request**:
  ```json
  {"jsonrpc": "2.0", "id": 1, "method": "system.info", "params": {}}
  ```
* **Response**:
  ```json
  {"jsonrpc": "2.0", "id": 1, "result": {"firmware": "motion-core-1.0.0", "target": "ESP32-S3", "axes": 3}}
  ```

### `system.estop`
Emergency Stop. Halts all motion immediately and locks the machine.
* **Request**:
  ```json
  {"jsonrpc": "2.0", "id": 2, "method": "system.estop", "params": {}}
  ```
* **Response**:
  ```json
  {"jsonrpc": "2.0", "id": 2, "result": {"estop": true, "status": "LOCKED"}}
  ```

### `system.reset`
Clears the E-STOP lock and re-enables motor drivers.
* **Request**:
  ```json
  {"jsonrpc": "2.0", "id": 3, "method": "system.reset", "params": {}}
  ```
* **Response**:
  ```json
  {"jsonrpc": "2.0", "id": 3, "result": {"reset": true, "status": "IDLE"}}
  ```

### `axis.home`
Run homing sequence (physical switch, Hall sensor, or StallGuard4).
* **Request**:
  ```json
  {"jsonrpc": "2.0", "id": 4, "method": "axis.home", "params": {"axis": 0}}
  ```
* **Response**:
  ```json
  {"jsonrpc": "2.0", "id": 4, "result": {"homing_started": true, "axis": 0}}
  ```

### `axis.move`
Absolute multi-axis coordinated move with S-curve acceleration.
* **Request**:
  ```json
  {
    "jsonrpc": "2.0",
    "id": 5,
    "method": "axis.move",
    "params": {
      "targets": {"0": 180.0, "1": 45.0, "2": 0.0},
      "feedrate": 25.0,
      "profile": "scurve"
    }
  }
  ```
* **Response**:
  ```json
  {"jsonrpc": "2.0", "id": 5, "result": {"queued": true, "queue_depth": 1}}
  ```

### `axis.jog`
Incremental jog command.
* **Request**:
  ```json
  {"jsonrpc": "2.0", "id": 6, "method": "axis.jog", "params": {"axis": 0, "delta": 10.0, "feedrate": 15.0}}
  ```

### `axis.get_status`
Query current machine positions and state.
* **Request**:
  ```json
  {"jsonrpc": "2.0", "id": 7, "method": "axis.get_status", "params": {}}
  ```
* **Response**:
  ```json
  {
    "jsonrpc": "2.0",
    "id": 7,
    "result": {
      "positions": {"0": 180.0, "1": 45.0, "2": 0.0},
      "moving": false,
      "homed": {"0": true, "1": true, "2": true},
      "estop": false
    }
  }
  ```

---

## 3. Asynchronous Telemetry Events

The firmware pushes unsolicited status events:
* **Motion Status**:
  ```json
  {"event": "motion.status", "data": {"positions": {"0": 90.0, "1": 30.0}, "moving": true}}
  ```
* **Pose Captured**:
  ```json
  {"event": "scan.pose_complete", "data": {"pose_idx": 42, "azimuth": 128.5, "elevation": 34.2, "frame": "frame_0042.jpg"}}
  ```
* **Emergency Stop Alert**:
  ```json
  {"event": "system.estop", "data": {"reason": "hardware_button_triggered"}}
  ```
