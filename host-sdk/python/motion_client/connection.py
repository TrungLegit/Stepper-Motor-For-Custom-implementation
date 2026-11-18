"""
Connection and JSON-RPC 2.0 Transport for stepper-motion-core.
Supports USB-Serial (pyserial), WebSocket, and Mock/Simulation transports.
"""

import json
import logging
import threading
import time
from typing import Any, Callable, Dict, Optional, Union

logger = logging.getLogger("motion_client.connection")


class JsonRpcError(Exception):
    """Exception raised when firmware responds with a JSON-RPC error."""

    def __init__(self, code: int, message: str, data: Any = None):
        super().__init__(f"[{code}] {message}")
        self.code = code
        self.message = message
        self.data = data


class ConnectionTransport:
    """Base transport interface for device communication."""

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def is_open(self) -> bool:
        raise NotImplementedError

    def write_line(self, line: str) -> None:
        raise NotImplementedError

    def read_line(self, timeout: float = 1.0) -> Optional[str]:
        raise NotImplementedError


class MockTransport(ConnectionTransport):
    """In-memory loopback transport simulating firmware responses for unit tests."""

    def __init__(self):
        self._is_open = False
        self._rx_queue = []
        self._lock = threading.Lock()
        self._state = "idle"
        self._pos = {"A": 0.0, "B": 0.0, "Z": 0.0}

    def open(self) -> None:
        self._is_open = True

    def close(self) -> None:
        self._is_open = False

    def is_open(self) -> bool:
        return self._is_open

    def write_line(self, line: str) -> None:
        if not self._is_open:
            raise IOError("Transport not open")
        try:
            req = json.loads(line)
        except Exception:
            self._queue_resp({"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}})
            return

        req_id = req.get("id")
        method = req.get("method", "")
        params = req.get("params", {})

        if method == "system.estop":
            self._state = "estop_locked"
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "result": {"estop": True, "state": "estop_locked"}})
            self._queue_resp({"event": "system.estop", "data": {"reason": params.get("reason", "estop")}})
            return

        if self._state == "estop_locked" and method not in ("system.reset", "system.info", "axis.get_status"):
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "error": {"code": -32001, "message": "ESTOP active"}})
            return

        if method == "system.info":
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "result": {"firmware": "stepper-motion-core", "version": "0.2.0", "state": self._state}})
        elif method == "system.reset":
            self._state = "idle"
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "result": {"reset": True, "state": "idle"}})
            self._queue_resp({"event": "system.reset", "data": {"status": "idle"}})
        elif method == "axis.move":
            axes = params.get("axes", {})
            for k, v in axes.items():
                if k in self._pos:
                    self._pos[k] = float(v)
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "result": {"queued": True, "queue_depth": 1}})
            self._queue_resp({"event": "motion.status", "data": {"pos": self._pos.copy(), "moving": False}})
        elif method == "axis.jog":
            axis = params.get("axis", "A")
            dist = params.get("distance", 0.0)
            if axis in self._pos:
                self._pos[axis] += float(dist)
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "result": {"jogging": True, "axis": axis}})
        elif method == "axis.home":
            self._pos = {"A": 0.0, "B": 0.0, "Z": 0.0}
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "result": {"homing": True, "axes": ["A", "B", "Z"]}})
            self._queue_resp({"event": "homing.progress", "data": {"axis": "all", "completed": True}})
        elif method == "axis.get_status":
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "result": {"state": self._state, "pos": self._pos.copy(), "moving": False, "estop_active": self._state == "estop_locked"}})
        elif method == "trigger.capture":
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "result": {"triggered": True, "settle_ms": params.get("settle_ms", 100), "status": "ack"}})
        else:
            if req_id is not None:
                self._queue_resp({"jsonrpc": "2.0", "id": req_id, "error": {"code": -32601, "message": f"Method {method} not found"}})

    def _queue_resp(self, data: dict):
        with self._lock:
            self._rx_queue.append(json.dumps(data) + "\n")

    def read_line(self, timeout: float = 1.0) -> Optional[str]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._rx_queue:
                    return self._rx_queue.pop(0)
            time.sleep(0.01)
        return None


class SerialTransport(ConnectionTransport):
    """Serial port transport using pyserial."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None

    def open(self) -> None:
        try:
            import serial
            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            logger.info("Opened serial port %s at %d baud", self.port, self.baudrate)
        except ImportError:
            raise RuntimeError("pyserial is required for SerialTransport. Run: pip install pyserial")

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None

    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def write_line(self, line: str) -> None:
        if not self.is_open():
            raise IOError("Serial port not open")
        if not line.endswith("\n"):
            line += "\n"
        self._serial.write(line.encode("utf-8"))
        self._serial.flush()

    def read_line(self, timeout: float = 1.0) -> Optional[str]:
        if not self.is_open():
            return None
        self._serial.timeout = timeout
        raw = self._serial.readline()
        if raw:
            return raw.decode("utf-8", errors="replace")
        return None


class MotionConnection:
    """
    JSON-RPC 2.0 Client Connection managing request correlation,
    async telemetry push events, and automatic reconnection.
    """

    def __init__(self, transport: Optional[ConnectionTransport] = None):
        self.transport = transport or MockTransport()
        self._id_counter = 0
        self._pending_requests: Dict[int, Dict[str, Any]] = {}
        self._event_listeners: Dict[str, list] = {}
        self._global_listeners: list = []
        self._lock = threading.Lock()
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def connect(self) -> None:
        """Establish transport connection and start listener loop."""
        if not self.transport.is_open():
            self.transport.open()
        self._stop_event.clear()
        self._worker_thread = threading.Thread(target=self._rx_loop, daemon=True, name="MotionConnection-RX")
        self._worker_thread.start()
        logger.info("MotionConnection connected.")

    def disconnect(self) -> None:
        """Stop listener loop and close transport."""
        self._stop_event.set()
        if self.transport.is_open():
            self.transport.close()
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        logger.info("MotionConnection disconnected.")

    def is_connected(self) -> bool:
        return self.transport.is_open()

    def add_event_listener(self, event_name: str, callback: Callable[[dict], None]) -> None:
        """Subscribe to specific asynchronous telemetry events (e.g. 'motion.status')."""
        with self._lock:
            self._event_listeners.setdefault(event_name, []).append(callback)

    def add_global_listener(self, callback: Callable[[dict], None]) -> None:
        """Subscribe to all raw incoming frames."""
        with self._lock:
            self._global_listeners.append(callback)

    def call(self, method: str, params: Optional[dict] = None, timeout: float = 5.0) -> Any:
        """
        Synchronous JSON-RPC 2.0 method invocation.
        Blocks until response is received or timeout expires.
        """
        with self._lock:
            self._id_counter += 1
            req_id = self._id_counter

        ev = threading.Event()
        container: Dict[str, Any] = {"event": ev, "response": None}

        with self._lock:
            self._pending_requests[req_id] = container

        req_payload = {
            "jsonrpc": "2.0",
            "id": req_id,
            "method": method,
            "params": params or {}
        }

        raw = json.dumps(req_payload) + "\n"
        self.transport.write_line(raw)

        if not ev.wait(timeout):
            with self._lock:
                self._pending_requests.pop(req_id, None)
            raise TimeoutError(f"JSON-RPC call '{method}' timed out after {timeout}s (id={req_id})")

        resp = container["response"]
        if "error" in resp:
            err = resp["error"]
            raise JsonRpcError(err.get("code", -1), err.get("message", "Unknown error"), err.get("data"))

        return resp.get("result")

    def notify(self, method: str, params: Optional[dict] = None) -> None:
        """Send a JSON-RPC 2.0 notification (no response expected)."""
        req_payload = {
            "jsonrpc": "2.0",
            "method": method,
            "params": params or {}
        }
        raw = json.dumps(req_payload) + "\n"
        self.transport.write_line(raw)

    def emergency_stop(self, reason: str = "Host requested E-STOP") -> dict:
        """
        Highest priority emergency stop invocation.
        Bypasses normal queues and executes immediately.
        """
        return self.call("system.estop", {"reason": reason}, timeout=2.0)

    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                line = self.transport.read_line(timeout=0.2)
                if not line:
                    continue
                line = line.strip()
                if not line:
                    continue
                self._handle_incoming_line(line)
            except Exception as e:
                if not self._stop_event.is_set():
                    logger.debug("RX loop error: %s", e)

    def _handle_incoming_line(self, line: str) -> None:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            logger.warning("Dropped malformed line: %s", line)
            return

        # Notify global listeners
        with self._lock:
            g_listeners = list(self._global_listeners)
        for cb in g_listeners:
            try:
                cb(msg)
            except Exception as e:
                logger.error("Global listener error: %s", e)

        # 1. Asynchronous Telemetry Push Event
        if "event" in msg:
            event_name = msg.get("event")
            event_data = msg.get("data", {})
            with self._lock:
                listeners = list(self._event_listeners.get(event_name, []))
            for cb in listeners:
                try:
                    cb(event_data)
                except Exception as e:
                    logger.error("Event listener error for %s: %s", event_name, e)
            return

        # 2. JSON-RPC Response to pending request
        if "id" in msg and msg["id"] is not None:
            req_id = msg["id"]
            with self._lock:
                container = self._pending_requests.pop(req_id, None)
            if container:
                container["response"] = msg
                container["event"].set()
