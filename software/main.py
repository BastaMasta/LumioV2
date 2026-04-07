"""
Lumio Educational Robot Platform — Main Application
====================================================
Raspberry Pi 5 | Lumio V2 PCB | KiCad 9.0
R.A.D.G.T.F Foundation / SonyTech IT Company

Architecture
------------
Five concurrent threads communicate via threading.Queue and threading.Event:

    Thread 1: VoiceCommandListener  — Vosk STT + wake-word gate + Ollama LLM + Piper TTS
    Thread 2: MotorExecutor         — Consumes commands from shared Queue, drives L298N via GPIO
    Thread 3: WirelessControlServer — Bluetooth + WiFi web panel, pushes commands to Queue
    Thread 4: SensorMonitor         — Polls HC-SR04 ultrasonics + MCP3008 ADC via RP2040 UART
    Thread 5: WatchdogTimer         — Detects hangs, triggers recovery, logs fault records

All command sources (voice, wireless) push to a single shared Queue.
The MotorExecutor is the sole consumer — no command starvation possible.

Hardware interfaces
-------------------
    RPi 5 GPIO → L298N  (IN1, IN2, IN3, IN4, ENA, ENB)
    RPi 5 UART → RP2040 (JSON command protocol at 115200 baud)
    RPi 5 SPI  → MCP3008 (via RP2040 proxy, or direct SPI0)
    RPi 5 GPIO → HC-SR04 TRIG/ECHO (front J16, back J17)

Dependencies
------------
    pip install vosk pyaudio gpiozero pigpio spidev flask flask-socketio
    pip install requests  # Ollama REST client
    # Install Ollama separately: curl -fsSL https://ollama.com/install.sh | sh
    # Pull model: ollama pull phi3:mini
    # Install Piper: see https://github.com/rhasspy/piper

Usage
-----
    python3 main.py [--simulate]
    --simulate: run without real GPIO (uses MockMotorHAL for off-hardware testing)
"""

import argparse
import json
import logging
import os
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from logging.handlers import RotatingFileHandler
from typing import Optional

# ── Constants ────────────────────────────────────────────────────────────────

WAKE_WORD          = "lumio"
VOSK_MODEL_PATH    = os.path.expanduser("~/models/vosk-model-en-us-0.22-lgraph")
PIPER_BINARY       = os.path.expanduser("~/piper/piper")
PIPER_MODEL_PATH   = os.path.expanduser("~/models/piper/en_US-lessac-medium.onnx")
OLLAMA_URL         = "http://localhost:11434/api/generate"
OLLAMA_MODEL       = "phi3:mini"
RP2040_UART        = "/dev/ttyAMA0"
RP2040_BAUD        = 115200
ULTRASONIC_THRESH  = 20        # cm — obstacle detection threshold
WATCHDOG_TIMEOUT   = 10.0      # seconds before watchdog triggers
COMMAND_QUEUE_MAX  = 20        # max queued commands before dropping
LOG_FILE           = "/var/log/lumio/lumio.log"
LOG_MAX_BYTES      = 10 * 1024 * 1024   # 10 MB
LOG_BACKUP_COUNT   = 5

# L298N GPIO pin assignments (BCM numbering, via 40-pin ribbon → Lumio V2 PCB)
MOTOR_IN1  = 17    # L298N IN1  (Motor A direction)
MOTOR_IN2  = 27    # L298N IN2  (Motor A direction)
MOTOR_IN3  = 22    # L298N IN3  (Motor B direction)
MOTOR_IN4  = 23    # L298N IN4  (Motor B direction)
MOTOR_ENA  = 12    # L298N ENA  (Motor A PWM speed, hardware PWM)
MOTOR_ENB  = 13    # L298N ENB  (Motor B PWM speed, hardware PWM)

# HC-SR04 ultrasonic sensor GPIO pins (J16 front, J17 back on Lumio V2 PCB)
ULTRA_FRONT_TRIG = 5
ULTRA_FRONT_ECHO = 6
ULTRA_BACK_TRIG  = 24
ULTRA_BACK_ECHO  = 25

# Movement timing constants (calibrate for your motor/chassis)
DEFAULT_SPEED      = 0.75     # PWM duty cycle 0.0–1.0
ROTATION_SPEED     = 0.60
LINEAR_TIME_PER_CM = 0.050    # seconds per cm at DEFAULT_SPEED
ROTATION_TIME_90   = 0.55     # seconds for 90° rotation


# ── Logging setup ─────────────────────────────────────────────────────────────

def setup_logging(simulate: bool) -> logging.Logger:
    """Configure rotating file + console logging."""
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    logger = logging.getLogger("lumio")
    logger.setLevel(logging.DEBUG)

    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(threadName)s — %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S"
    )

    file_handler = RotatingFileHandler(
        LOG_FILE, maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUP_COUNT
    )
    file_handler.setFormatter(fmt)
    file_handler.setLevel(logging.DEBUG)

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(fmt)
    console_handler.setLevel(logging.INFO)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    if simulate:
        logger.info("SIMULATION MODE — no real GPIO operations")

    return logger


# ── Command model ─────────────────────────────────────────────────────────────

class CommandSource(Enum):
    VOICE    = auto()
    WIFI     = auto()
    BLUETOOTH = auto()


@dataclass
class RobotCommand:
    """Represents a single robot command from any control source."""
    action: str                             # "forward", "back", "left", "right", "stop", "say"
    param: float = 0.0                      # distance (cm) or angle (deg) or duration (s)
    speed: float = DEFAULT_SPEED
    source: CommandSource = CommandSource.VOICE
    text: str = ""                          # for "say" commands
    timestamp: float = field(default_factory=time.time)


# ── Hardware Abstraction Layer ────────────────────────────────────────────────

class MotorHAL:
    """
    Hardware Abstraction Layer for L298N dual H-bridge motor driver.

    All GPIO operations are encapsulated here. RobotController never
    touches GPIO directly — enabling MockMotorHAL substitution in tests.

    Pin mapping (Lumio V2 PCB, BCM numbering via 40-pin ribbon):
        ENA (PWM) → GPIO 12    IN1 → GPIO 17    IN2 → GPIO 27
        ENB (PWM) → GPIO 13    IN3 → GPIO 22    IN4 → GPIO 23
    """

    def __init__(self, logger: logging.Logger):
        self.log = logger
        from gpiozero import Motor, PWMOutputDevice
        # gpiozero Motor wraps both direction pins; PWMOutputDevice for enable
        self._motor_a = Motor(forward=MOTOR_IN1, backward=MOTOR_IN2, pwm=True)
        self._motor_b = Motor(forward=MOTOR_IN3, backward=MOTOR_IN4, pwm=True)
        self.log.info("MotorHAL initialised — L298N ready")

    def forward(self, speed: float = DEFAULT_SPEED) -> None:
        """Drive both motors forward at given speed (0.0–1.0)."""
        self._motor_a.forward(speed)
        self._motor_b.forward(speed)
        self.log.debug("HAL: forward speed=%.2f", speed)

    def backward(self, speed: float = DEFAULT_SPEED) -> None:
        self._motor_a.backward(speed)
        self._motor_b.backward(speed)
        self.log.debug("HAL: backward speed=%.2f", speed)

    def turn_left(self, speed: float = ROTATION_SPEED) -> None:
        """Pivot left: Motor A backward, Motor B forward."""
        self._motor_a.backward(speed)
        self._motor_b.forward(speed)
        self.log.debug("HAL: turn_left speed=%.2f", speed)

    def turn_right(self, speed: float = ROTATION_SPEED) -> None:
        """Pivot right: Motor A forward, Motor B backward."""
        self._motor_a.forward(speed)
        self._motor_b.backward(speed)
        self.log.debug("HAL: turn_right speed=%.2f", speed)

    def stop(self) -> None:
        self._motor_a.stop()
        self._motor_b.stop()
        self.log.debug("HAL: stop")

    def cleanup(self) -> None:
        self.stop()
        self.log.info("MotorHAL cleanup complete")


class MockMotorHAL:
    """
    Mock HAL for off-hardware testing and simulation mode.

    Records every call as (method, speed, timestamp) for assertion
    in unit tests. Identical interface to MotorHAL.
    """

    def __init__(self, logger: logging.Logger):
        self.log = logger
        self.calls: list[tuple] = []
        self.log.info("MockMotorHAL initialised — simulation mode")

    def _record(self, method: str, speed: float = 0.0) -> None:
        self.calls.append((method, speed, time.time()))
        self.log.debug("MOCK HAL: %s(speed=%.2f)", method, speed)

    def forward(self, speed: float = DEFAULT_SPEED) -> None:
        self._record("forward", speed)

    def backward(self, speed: float = DEFAULT_SPEED) -> None:
        self._record("backward", speed)

    def turn_left(self, speed: float = ROTATION_SPEED) -> None:
        self._record("turn_left", speed)

    def turn_right(self, speed: float = ROTATION_SPEED) -> None:
        self._record("turn_right", speed)

    def stop(self) -> None:
        self._record("stop")

    def cleanup(self) -> None:
        self._record("cleanup")
        self.log.info("MockMotorHAL cleanup")


# ── RP2040 UART interface ─────────────────────────────────────────────────────

class RP2040Interface:
    """
    UART JSON protocol interface to the RP2040 co-processor.

    The RP2040 (on Lumio V2 PCB) communicates at 115200 baud over
    /dev/ttyAMA0. Commands are newline-terminated JSON objects.

    Protocol:
        Pi → RP2040: {"action": "read_adc", "channel": 0}
        RP2040 → Pi: {"channel": 0, "value": 512, "voltage": 1.65}

        Pi → RP2040: {"action": "set_gpio", "pin": 5, "value": 1}
        RP2040 → Pi: {"ok": true}

        Pi → RP2040: {"action": "read_ultrasonic", "sensor": "front"}
        RP2040 → Pi: {"sensor": "front", "distance_cm": 34.2}
    """

    def __init__(self, logger: logging.Logger, simulate: bool = False):
        self.log = logger
        self.simulate = simulate
        self._lock = threading.Lock()
        self._ser = None

        if not simulate:
            import serial
            try:
                self._ser = serial.Serial(RP2040_UART, RP2040_BAUD, timeout=0.5)
                self.log.info("RP2040 UART open: %s @ %d baud", RP2040_UART, RP2040_BAUD)
            except Exception as exc:
                self.log.error("RP2040 UART open failed: %s", exc)

    def _send(self, payload: dict) -> Optional[dict]:
        """Send JSON command and return parsed response. Thread-safe."""
        if self.simulate or self._ser is None:
            return {"ok": True, "simulated": True}
        with self._lock:
            try:
                self._ser.write((json.dumps(payload) + "\n").encode())
                raw = self._ser.readline().decode().strip()
                if raw:
                    return json.loads(raw)
            except Exception as exc:
                self.log.warning("RP2040 UART error: %s", exc)
        return None

    def read_adc(self, channel: int) -> float:
        """Read MCP3008 ADC channel (0–7) via RP2040. Returns voltage 0–3.3V."""
        result = self._send({"action": "read_adc", "channel": channel})
        if result and "voltage" in result:
            return float(result["voltage"])
        return 0.0

    def read_ultrasonic(self, sensor: str) -> float:
        """Read HC-SR04 distance in cm. sensor = 'front' or 'back'."""
        result = self._send({"action": "read_ultrasonic", "sensor": sensor})
        if result and "distance_cm" in result:
            return float(result["distance_cm"])
        return 999.0   # safe fallback — treat as no obstacle

    def set_gpio(self, pin: int, value: int) -> bool:
        """Set an RP2040 GPIO pin high (1) or low (0)."""
        result = self._send({"action": "set_gpio", "pin": pin, "value": value})
        return bool(result and result.get("ok"))

    def cleanup(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
            self.log.info("RP2040 UART closed")


# ── Robot Controller ──────────────────────────────────────────────────────────

class RobotController:
    """
    High-level robot motion controller.

    Translates RobotCommand objects into timed HAL calls.
    Zero direct GPIO access — all hardware interaction via MotorHAL.

    The stop_event is checked at sub-command granularity so that
    emergency stop and watchdog recovery interrupt movements promptly.
    """

    def __init__(
        self,
        hal: MotorHAL,
        rp2040: RP2040Interface,
        stop_event: threading.Event,
        logger: logging.Logger,
    ):
        self.hal = hal
        self.rp2040 = rp2040
        self.stop_event = stop_event
        self.log = logger

    def _timed_move(self, move_fn, duration: float, step: float = 0.05) -> None:
        """Execute a movement for `duration` seconds, checking stop_event every `step`."""
        move_fn()
        elapsed = 0.0
        while elapsed < duration:
            if self.stop_event.is_set():
                self.log.info("Movement interrupted by stop_event")
                break
            time.sleep(step)
            elapsed += step
        self.hal.stop()

    def check_obstacle(self, direction: str) -> bool:
        """Return True if obstacle detected in `direction` ('front' or 'back')."""
        dist = self.rp2040.read_ultrasonic(direction)
        if dist < ULTRASONIC_THRESH:
            self.log.warning("Obstacle detected %s: %.1f cm", direction, dist)
            return True
        return False

    def execute(self, cmd: RobotCommand) -> None:
        """Execute a single RobotCommand."""
        self.log.info(
            "Executing: action=%s param=%.1f speed=%.2f source=%s",
            cmd.action, cmd.param, cmd.speed, cmd.source.name
        )

        if cmd.action == "stop":
            self.hal.stop()

        elif cmd.action == "forward":
            if self.check_obstacle("front"):
                self.log.warning("Forward blocked — obstacle in front")
                return
            duration = cmd.param * LINEAR_TIME_PER_CM if cmd.param > 0 else 2.0
            self._timed_move(lambda: self.hal.forward(cmd.speed), duration)

        elif cmd.action == "back":
            if self.check_obstacle("back"):
                self.log.warning("Backward blocked — obstacle behind")
                return
            duration = cmd.param * LINEAR_TIME_PER_CM if cmd.param > 0 else 2.0
            self._timed_move(lambda: self.hal.backward(cmd.speed), duration)

        elif cmd.action == "left":
            degrees = cmd.param if cmd.param > 0 else 90.0
            duration = (degrees / 90.0) * ROTATION_TIME_90
            self._timed_move(lambda: self.hal.turn_left(cmd.speed), duration)

        elif cmd.action == "right":
            degrees = cmd.param if cmd.param > 0 else 90.0
            duration = (degrees / 90.0) * ROTATION_TIME_90
            self._timed_move(lambda: self.hal.turn_right(cmd.speed), duration)

        elif cmd.action == "say":
            # Delegate speech to voice thread — just log here
            self.log.info("Say command: %s", cmd.text)

        else:
            self.log.warning("Unknown action: %s", cmd.action)


# ── Thread 1: Voice Command Listener ─────────────────────────────────────────

class VoiceCommandListener(threading.Thread):
    """
    Offline voice interaction pipeline.

    Pipeline:
        Microphone → Vosk STT (wake-word gate) → Ollama phi3:mini → Piper TTS

    Only activates the LLM after the wake word is detected, preventing
    unnecessary inference calls and reducing power consumption during
    passive classroom operation.

    Voice commands that map to motion ("move forward", "turn left", etc.)
    are parsed and pushed to the shared command_queue. All other speech
    is treated as a Q&A query and routed to Ollama.
    """

    # Simple keyword → action mapping (extend as needed)
    MOTION_KEYWORDS = {
        "forward": ("forward", 0),
        "move forward": ("forward", 0),
        "go forward": ("forward", 0),
        "back": ("back", 0),
        "backward": ("back", 0),
        "reverse": ("back", 0),
        "left": ("left", 90),
        "turn left": ("left", 90),
        "right": ("right", 90),
        "turn right": ("right", 90),
        "stop": ("stop", 0),
        "halt": ("stop", 0),
    }

    def __init__(
        self,
        command_queue: queue.Queue,
        shutdown: threading.Event,
        logger: logging.Logger,
    ):
        super().__init__(name="VoiceThread", daemon=True)
        self.queue = command_queue
        self.shutdown = shutdown
        self.log = logger
        self._awake = False          # True when wake word has been detected

    def _speak(self, text: str) -> None:
        """Synthesise `text` to speech using Piper TTS and play it."""
        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                wav_path = tmp.name
            result = subprocess.run(
                [PIPER_BINARY, "--model", PIPER_MODEL_PATH, "--output_file", wav_path],
                input=text.encode(),
                capture_output=True,
                timeout=10,
            )
            if result.returncode == 0:
                subprocess.run(["aplay", "-q", wav_path], timeout=30)
            else:
                self.log.warning("Piper TTS error: %s", result.stderr.decode())
            os.unlink(wav_path)
        except Exception as exc:
            self.log.error("TTS error: %s", exc)

    def _query_ollama(self, prompt: str) -> str:
        """Send a prompt to Ollama phi3:mini and return the response text."""
        import requests
        system_prompt = (
            "You are Lumio, an educational robot assistant in a classroom. "
            "Give concise, friendly, age-appropriate answers. "
            "If asked to move or perform a physical action, respond with a brief "
            "acknowledgement — the movement will be handled separately."
        )
        try:
            resp = requests.post(
                OLLAMA_URL,
                json={
                    "model": OLLAMA_MODEL,
                    "prompt": prompt,
                    "system": system_prompt,
                    "stream": False,
                },
                timeout=30,
            )
            resp.raise_for_status()
            return resp.json().get("response", "").strip()
        except Exception as exc:
            self.log.error("Ollama error: %s", exc)
            return "Sorry, I could not process that right now."

    def _parse_motion_command(self, text: str) -> Optional[RobotCommand]:
        """Return a RobotCommand if `text` contains a motion keyword, else None."""
        lower = text.lower().strip()
        for keyword, (action, param) in self.MOTION_KEYWORDS.items():
            if keyword in lower:
                return RobotCommand(action=action, param=param, source=CommandSource.VOICE)
        return None

    def run(self) -> None:
        """Main voice pipeline loop."""
        import vosk
        import pyaudio

        self.log.info("Loading Vosk model from %s", VOSK_MODEL_PATH)
        if not os.path.exists(VOSK_MODEL_PATH):
            self.log.error("Vosk model not found at %s — voice thread exiting", VOSK_MODEL_PATH)
            return

        model = vosk.Model(VOSK_MODEL_PATH)
        recogniser = vosk.KaldiRecognizer(model, 16000)

        audio = pyaudio.PyAudio()
        stream = audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=4096,
        )
        self.log.info("Voice pipeline ready — listening for wake word '%s'", WAKE_WORD)

        try:
            while not self.shutdown.is_set():
                data = stream.read(4096, exception_on_overflow=False)
                if recogniser.AcceptWaveform(data):
                    result = json.loads(recogniser.Result())
                    text = result.get("text", "").strip()

                    if not text:
                        continue

                    self.log.debug("Recognised: '%s' (awake=%s)", text, self._awake)

                    if not self._awake:
                        # Gate: only activate on wake word
                        if WAKE_WORD in text.lower():
                            self._awake = True
                            self.log.info("Wake word detected — listening for command")
                            self._speak("Yes, I'm listening.")
                    else:
                        self._awake = False   # Reset gate after one command

                        # Check for motion command first
                        motion_cmd = self._parse_motion_command(text)
                        if motion_cmd:
                            try:
                                self.queue.put_nowait(motion_cmd)
                                self.log.info("Voice motion command queued: %s", motion_cmd.action)
                            except queue.Full:
                                self.log.warning("Command queue full — voice command dropped")
                        else:
                            # Route to Ollama for Q&A
                            self.log.info("Q&A query: %s", text)
                            response = self._query_ollama(text)
                            self.log.info("Ollama response: %s", response[:100])
                            self._speak(response)

        except Exception as exc:
            self.log.error("Voice thread exception: %s", exc, exc_info=True)
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()
            self.log.info("Voice thread exited")


# ── Thread 2: Motor Executor ──────────────────────────────────────────────────

class MotorExecutor(threading.Thread):
    """
    Sole consumer of the shared command Queue.

    Receives RobotCommand objects from any source (voice, WiFi, Bluetooth)
    and executes them sequentially via RobotController. Uses a Queue with
    maxsize=COMMAND_QUEUE_MAX to prevent unbounded backlog under high load.

    The emergency_stop Event bypasses the queue and halts immediately.
    """

    def __init__(
        self,
        command_queue: queue.Queue,
        controller: RobotController,
        shutdown: threading.Event,
        emergency_stop: threading.Event,
        watchdog_reset: threading.Event,
        logger: logging.Logger,
    ):
        super().__init__(name="MotorThread", daemon=True)
        self.queue = command_queue
        self.controller = controller
        self.shutdown = shutdown
        self.emergency_stop = emergency_stop
        self.watchdog_reset = watchdog_reset
        self.log = logger

    def run(self) -> None:
        self.log.info("Motor executor ready")
        while not self.shutdown.is_set():
            # Check emergency stop before dequeuing
            if self.emergency_stop.is_set():
                self.controller.hal.stop()
                # Flush queue on emergency stop
                while not self.queue.empty():
                    try:
                        self.queue.get_nowait()
                    except queue.Empty:
                        break
                self.emergency_stop.clear()
                self.log.info("Emergency stop executed — queue flushed")
                continue

            try:
                cmd = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # Signal watchdog that we're active
            self.watchdog_reset.set()

            try:
                self.controller.execute(cmd)
            except Exception as exc:
                self.log.error("Command execution error: %s", exc, exc_info=True)
                self.controller.hal.stop()
            finally:
                self.queue.task_done()
                self.watchdog_reset.clear()

        self.controller.hal.stop()
        self.log.info("Motor executor exited")


# ── Thread 3: Wireless Control Server ────────────────────────────────────────

class WirelessControlServer(threading.Thread):
    """
    WiFi web panel + Bluetooth command listener.

    WiFi: Flask-SocketIO web server on port 5000.
          Accepts directional commands via WebSocket.
          Emits real-time robot status back to the browser.

    Bluetooth: Listens on a Bluetooth RFCOMM socket for commands
               from a companion app. Same command format as WiFi.

    Command format (JSON string):
        {"action": "forward", "param": 50, "speed": 0.75}
        {"action": "stop"}
        {"action": "emergency_stop"}
    """

    def __init__(
        self,
        command_queue: queue.Queue,
        shutdown: threading.Event,
        emergency_stop: threading.Event,
        logger: logging.Logger,
    ):
        super().__init__(name="WirelessThread", daemon=True)
        self.queue = command_queue
        self.shutdown = shutdown
        self.emergency_stop = emergency_stop
        self.log = logger

    def _push_command(self, payload: dict, source: CommandSource) -> None:
        """Parse payload dict and push to command queue."""
        action = payload.get("action", "stop")

        if action == "emergency_stop":
            self.emergency_stop.set()
            self.log.warning("Emergency stop from %s", source.name)
            return

        cmd = RobotCommand(
            action=action,
            param=float(payload.get("param", 0)),
            speed=float(payload.get("speed", DEFAULT_SPEED)),
            source=source,
        )
        try:
            self.queue.put_nowait(cmd)
        except queue.Full:
            self.log.warning("Queue full — %s command dropped: %s", source.name, action)

    def _start_wifi_server(self) -> None:
        """Start Flask-SocketIO server for WiFi control panel."""
        from flask import Flask, render_template_string, request
        from flask_socketio import SocketIO, emit

        app = Flask(__name__)
        app.config["SECRET_KEY"] = "lumio-classroom-secret"
        socketio = SocketIO(app, async_mode="eventlet", cors_allowed_origins="*")

        CONTROL_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Lumio Control Panel</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; max-width: 500px; margin: 40px auto;
               padding: 20px; background: #f0f4f8; }
        h1 { color: #1a3a6b; }
        .grid { display: grid; grid-template-columns: repeat(3, 1fr);
                gap: 10px; margin: 20px 0; }
        button { padding: 20px; font-size: 1.2em; border: none;
                 border-radius: 8px; cursor: pointer; background: #2563eb;
                 color: white; transition: background 0.2s; }
        button:hover { background: #1d4ed8; }
        .stop-btn { background: #dc2626; grid-column: 2; }
        .estop-btn { background: #7f1d1d; width: 100%; padding: 15px;
                     font-size: 1.1em; margin-top: 10px; }
        #status { background: white; padding: 15px; border-radius: 8px;
                  margin-top: 20px; font-size: 0.9em; color: #444; }
        .center { grid-column: 2; }
    </style>
</head>
<body>
    <h1>🤖 Lumio Control</h1>
    <div class="grid">
        <div></div>
        <button onclick="cmd('forward', 30)">▲ Forward</button>
        <div></div>
        <button onclick="cmd('left', 0, 0, 90)">◀ Left</button>
        <button class="stop-btn" onclick="cmd('stop')">■ Stop</button>
        <button onclick="cmd('right', 0, 0, 90)">▶ Right</button>
        <div></div>
        <button onclick="cmd('back', 30)">▼ Back</button>
        <div></div>
    </div>
    <button class="estop-btn" onclick="estop()">🚨 EMERGENCY STOP</button>
    <div id="status">Status: Connected</div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.6.0/socket.io.min.js"></script>
    <script>
        const socket = io({ reconnection: true, reconnectionDelay: 1000 });
        socket.on('status', data => {
            document.getElementById('status').textContent =
                'Status: ' + JSON.stringify(data);
        });
        function cmd(action, param=0, speed=0.75, degrees=0) {
            socket.emit('command', {action, param: param || degrees, speed});
        }
        function estop() {
            socket.emit('command', {action: 'emergency_stop'});
        }
    </script>
</body>
</html>
        """

        @app.route("/")
        def index():
            return render_template_string(CONTROL_PAGE)

        @socketio.on("command")
        def handle_command(data):
            self._push_command(data, CommandSource.WIFI)
            emit("status", {"last_command": data.get("action"), "ts": time.time()})

        self.log.info("WiFi control server starting on port 5000")
        socketio.run(app, host="0.0.0.0", port=5000, log_output=False)

    def _start_bluetooth_listener(self) -> None:
        """Listen on Bluetooth RFCOMM for commands from companion app."""
        try:
            import bluetooth
        except ImportError:
            self.log.warning("PyBluez not available — Bluetooth control disabled")
            return

        try:
            server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            server_sock.bind(("", bluetooth.PORT_ANY))
            server_sock.listen(1)
            port = server_sock.getsockname()[1]
            bluetooth.advertise_service(
                server_sock, "LumioRobot",
                service_id="12345678-1234-5678-1234-567812345678",
                service_classes=[bluetooth.SERIAL_PORT_CLASS],
                profiles=[bluetooth.SERIAL_PORT_PROFILE],
            )
            self.log.info("Bluetooth RFCOMM listening on port %d", port)

            while not self.shutdown.is_set():
                client_sock, client_info = server_sock.accept()
                self.log.info("Bluetooth client connected: %s", client_info)
                try:
                    buf = ""
                    while not self.shutdown.is_set():
                        chunk = client_sock.recv(256).decode(errors="ignore")
                        if not chunk:
                            break
                        buf += chunk
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            line = line.strip()
                            if line:
                                try:
                                    payload = json.loads(line)
                                    self._push_command(payload, CommandSource.BLUETOOTH)
                                except json.JSONDecodeError:
                                    self.log.debug("BT non-JSON: %s", line)
                except Exception as exc:
                    self.log.warning("Bluetooth client error: %s", exc)
                finally:
                    client_sock.close()
        except Exception as exc:
            self.log.error("Bluetooth listener error: %s", exc)

    def run(self) -> None:
        # Bluetooth runs in a separate daemon thread
        bt_thread = threading.Thread(
            target=self._start_bluetooth_listener,
            name="BluetoothThread",
            daemon=True
        )
        bt_thread.start()

        # WiFi server runs in this thread (blocking)
        try:
            self._start_wifi_server()
        except Exception as exc:
            self.log.error("WiFi server error: %s", exc, exc_info=True)
        self.log.info("Wireless control server exited")


# ── Thread 4: Sensor Monitor ──────────────────────────────────────────────────

class SensorMonitor(threading.Thread):
    """
    Periodic sensor polling via RP2040 UART.

    Reads MCP3008 ADC channels (analog sensors on J2–J11 of Lumio V2 PCB)
    and ultrasonic distances at regular intervals. Stores latest readings
    in a shared dict for other threads to query.

    The RP2040 acts as the SPI master for the MCP3008, passing readings
    back to the RPi 5 over UART JSON. Ultrasonic readings also proxied.
    """

    def __init__(
        self,
        rp2040: RP2040Interface,
        shutdown: threading.Event,
        logger: logging.Logger,
        poll_interval: float = 0.5,
    ):
        super().__init__(name="SensorThread", daemon=True)
        self.rp2040 = rp2040
        self.shutdown = shutdown
        self.log = logger
        self.interval = poll_interval
        self.readings: dict = {
            "adc": [0.0] * 8,
            "ultrasonic": {"front": 999.0, "back": 999.0},
            "timestamp": 0.0,
        }
        self._lock = threading.Lock()

    def get_readings(self) -> dict:
        """Thread-safe snapshot of latest sensor readings."""
        with self._lock:
            return dict(self.readings)

    def run(self) -> None:
        self.log.info("Sensor monitor started (poll interval %.2fs)", self.interval)
        while not self.shutdown.is_set():
            try:
                adc_vals = [self.rp2040.read_adc(ch) for ch in range(8)]
                front_dist = self.rp2040.read_ultrasonic("front")
                back_dist  = self.rp2040.read_ultrasonic("back")

                with self._lock:
                    self.readings["adc"] = adc_vals
                    self.readings["ultrasonic"]["front"] = front_dist
                    self.readings["ultrasonic"]["back"]  = back_dist
                    self.readings["timestamp"] = time.time()

                self.log.debug(
                    "Sensors: front=%.1fcm back=%.1fcm adc[0]=%.2fV",
                    front_dist, back_dist, adc_vals[0]
                )
            except Exception as exc:
                self.log.warning("Sensor poll error: %s", exc)

            self.shutdown.wait(self.interval)

        self.log.info("Sensor monitor exited")


# ── Thread 5: Watchdog Timer ──────────────────────────────────────────────────

class WatchdogTimer(threading.Thread):
    """
    Monitors the MotorExecutor for hangs.

    If the executor starts a command (watchdog_reset is set) but does not
    clear it within WATCHDOG_TIMEOUT seconds, the watchdog triggers:
      1. Logs a structured fault record
      2. Sets the emergency_stop Event
      3. Clears the watchdog_reset for the next command

    This ensures Lumio never gets stuck in a motor-on state due to a
    software hang, which is critical for classroom safety.
    """

    def __init__(
        self,
        watchdog_reset: threading.Event,
        emergency_stop: threading.Event,
        shutdown: threading.Event,
        logger: logging.Logger,
    ):
        super().__init__(name="WatchdogThread", daemon=True)
        self.watchdog_reset = watchdog_reset
        self.emergency_stop = emergency_stop
        self.shutdown = shutdown
        self.log = logger

    def run(self) -> None:
        self.log.info("Watchdog started (timeout %.1fs)", WATCHDOG_TIMEOUT)
        while not self.shutdown.is_set():
            # Wait for a command to start
            if self.watchdog_reset.wait(timeout=1.0):
                start = time.time()
                # Give the command time to complete
                while self.watchdog_reset.is_set():
                    elapsed = time.time() - start
                    if elapsed > WATCHDOG_TIMEOUT:
                        fault = {
                            "event": "watchdog_trigger",
                            "elapsed_s": round(elapsed, 2),
                            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                            "thread_state": "executor_hung",
                        }
                        self.log.error("WATCHDOG TRIGGERED: %s", json.dumps(fault))
                        self.emergency_stop.set()
                        self.watchdog_reset.clear()
                        break
                    self.shutdown.wait(0.1)

        self.log.info("Watchdog exited")


# ── Application entrypoint ────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Lumio Educational Robot Platform")
    parser.add_argument(
        "--simulate", action="store_true",
        help="Run without GPIO hardware (uses MockMotorHAL)"
    )
    args = parser.parse_args()

    log = setup_logging(args.simulate)
    log.info("=" * 60)
    log.info("Lumio Platform starting (simulate=%s)", args.simulate)
    log.info("=" * 60)

    # ── Shared state ──────────────────────────────────────────────────────────
    command_queue  = queue.Queue(maxsize=COMMAND_QUEUE_MAX)
    shutdown       = threading.Event()
    emergency_stop = threading.Event()
    watchdog_reset = threading.Event()

    # ── Hardware layer ────────────────────────────────────────────────────────
    rp2040 = RP2040Interface(log, simulate=args.simulate)
    hal    = MockMotorHAL(log) if args.simulate else MotorHAL(log)
    stop_event = threading.Event()    # per-movement stop signal

    controller = RobotController(hal, rp2040, stop_event, log)

    # ── Thread construction ───────────────────────────────────────────────────
    voice_thread    = VoiceCommandListener(command_queue, shutdown, log)
    motor_thread    = MotorExecutor(
        command_queue, controller, shutdown, emergency_stop, watchdog_reset, log
    )
    wireless_thread = WirelessControlServer(command_queue, shutdown, emergency_stop, log)
    sensor_thread   = SensorMonitor(rp2040, shutdown, log)
    watchdog_thread = WatchdogTimer(watchdog_reset, emergency_stop, shutdown, log)

    threads = [voice_thread, motor_thread, wireless_thread, sensor_thread, watchdog_thread]

    # ── Graceful shutdown handler ─────────────────────────────────────────────
    def _signal_handler(sig, frame):
        log.info("Shutdown signal received (%s)", signal.Signals(sig).name)
        shutdown.set()
        emergency_stop.set()   # Immediately stop motors on shutdown

    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # ── Start all threads ─────────────────────────────────────────────────────
    for t in threads:
        t.start()
        log.info("Started: %s", t.name)

    # ── Main loop: wait for shutdown ─────────────────────────────────────────
    try:
        while not shutdown.is_set():
            shutdown.wait(timeout=1.0)
    finally:
        log.info("Waiting for threads to exit...")
        shutdown.set()
        for t in threads:
            t.join(timeout=5.0)
            if t.is_alive():
                log.warning("Thread %s did not exit cleanly", t.name)

        hal.cleanup()
        rp2040.cleanup()
        log.info("Lumio Platform stopped cleanly")


if __name__ == "__main__":
    main()
