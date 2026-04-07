"""
lumio/web.py
------------
Flask + SocketIO web control interface for Lumio V2.

Routes
──────
GET  /                  → main robot control dashboard
GET  /gpio              → RP2040 GPIO programming panel (user sensor mapping)
POST /api/gpio/config   → save pin config to RP2040
GET  /api/status        → JSON robot + sensor snapshot
WS   command            → movement / action commands
WS   get_sensor         → real-time ADC / distance readings
"""

import logging
import threading
from queue import Queue, Empty

from flask import Flask, render_template_string, jsonify, request
from flask_socketio import SocketIO, emit

from lumio.robot import RobotController
from lumio.adc import MCP3008
from lumio.coprocessor import RP2040

log = logging.getLogger(__name__)

app = Flask(__name__, template_folder="../templates")
app.config["SECRET_KEY"] = "lumio-v2-secret"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

# Globals set by create_app()
_robot: RobotController | None = None
_adc:   MCP3008 | None = None
_rp:    RP2040  | None = None
_action_queue: Queue = Queue()
_busy = threading.Lock()   # prevents concurrent movement commands

# ── RP2040 event forwarding ───────────────────────────────────────────────────

def _rp_event_handler(frame: dict):
    """Forward RP2040 async events to all connected web clients."""
    socketio.emit("rp2040_event", frame)


# ── Flask routes ──────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template_string(open("templates/control.html").read())


@app.route("/gpio")
def gpio_page():
    return render_template_string(open("templates/gpio.html").read())


@app.route("/api/status")
def api_status():
    data: dict = {"connected": True}
    if _robot:
        data["front_cm"] = _robot.front_distance_cm
        data["back_cm"]  = _robot.back_distance_cm
        data["stopped"]  = _robot.stopped
    if _adc:
        data["adc"] = _adc.read_all_voltage()
    if _rp:
        data["rp2040_online"] = _rp.ping()
    return jsonify(data)


@app.route("/api/gpio/config", methods=["POST"])
def api_gpio_config():
    """
    Accepts JSON: {"configs": [{"pin": 3, "mode": "input", "label": "IR sensor"}, ...]}
    Forwards each entry to the RP2040 via UART.
    """
    if not _rp:
        return jsonify({"ok": False, "error": "RP2040 not connected"}), 503
    payload = request.get_json(force=True)
    configs = payload.get("configs", [])
    results = []
    for cfg in configs:
        pin   = cfg.get("pin")
        mode  = cfg.get("mode", "input")   # 'input' | 'output'
        value = 1 if mode == "input" else 0
        ok    = _rp.gpio_write(pin, value)  # firmware interprets value as mode-set command
        results.append({"pin": pin, "ok": ok})
    return jsonify({"ok": True, "results": results})


# ── SocketIO events ───────────────────────────────────────────────────────────

@socketio.on("connect")
def on_connect():
    log.info("WebSocket client connected")
    emit("status", {"connected": True})


@socketio.on("disconnect")
def on_disconnect():
    log.info("WebSocket client disconnected")


@socketio.on("command")
def on_command(data: dict):
    """
    Execute a movement / action command.

    Expected keys:
        type  – 'forward' | 'backward' | 'left' | 'right' | 'dance' | 'hi' | 'stop'
        value – distance (m) or angle (°), float (optional, default 1.0)
        duty  – PWM duty cycle 0-100 (optional, default 50)
    """
    cmd   = data.get("type", "")
    value = float(data.get("value", 1.0))
    duty  = int(data.get("duty", 50))

    if not _robot:
        emit("command_result", {"ok": False, "error": "Robot not initialised"})
        return

    if not _busy.acquire(blocking=False):
        emit("command_result", {"ok": False, "error": "Robot busy"})
        return

    def _execute():
        try:
            ok = False
            if cmd == "forward":
                ok = _robot.move_forward(value, duty)
            elif cmd == "backward":
                ok = _robot.move_backward(value, duty)
            elif cmd == "left":
                ok = _robot.turn(-value, duty)
            elif cmd == "right":
                ok = _robot.turn(value, duty)
            elif cmd == "dance":
                ok = _robot.dance()
            elif cmd == "hi":
                ok = _robot.say_hi()
            elif cmd == "stop":
                _robot.stop()
                ok = True
            else:
                log.warning("Unknown command: %s", cmd)
            socketio.emit("command_result", {"ok": ok, "cmd": cmd})
        finally:
            _busy.release()

    t = threading.Thread(target=_execute, daemon=True)
    t.start()


@socketio.on("get_sensor")
def on_get_sensor(data: dict):
    """
    Read a sensor on demand.
    data: {"type": "front"} | {"type": "back"} | {"type": "adc", "ch": 0}
          | {"type": "rp_gpio", "pin": 3}
    """
    stype = data.get("type", "")
    result: dict = {"type": stype}

    if stype == "front" and _robot:
        result["cm"] = _robot.front_distance_cm
    elif stype == "back" and _robot:
        result["cm"] = _robot.back_distance_cm
    elif stype == "adc" and _adc:
        ch = int(data.get("ch", 0))
        result["raw"]     = _adc.read_raw(ch)
        result["voltage"] = _adc.read_voltage(ch)
    elif stype == "rp_gpio" and _rp:
        pin = int(data.get("pin", 0))
        result["value"] = _rp.gpio_read(pin)
    else:
        result["error"] = "Unknown sensor type or device not available"

    emit("sensor_data", result)


# ── Factory ───────────────────────────────────────────────────────────────────

def create_app(robot: RobotController, adc: MCP3008, rp: RP2040) -> tuple[Flask, SocketIO]:
    global _robot, _adc, _rp
    _robot = robot
    _adc   = adc
    _rp    = rp
    if rp:
        rp.on_event(_rp_event_handler)
    return app, socketio
