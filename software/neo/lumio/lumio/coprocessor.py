"""
lumio/coprocessor.py
--------------------
JSON-over-UART communication with the RP2040 co-processor.

Physical wiring (BCM GPIO numbering):
    RP2040 GPIO1  →  RPi5 GPIO15  (RXD / ttyAMA0)
    RP2040 GPIO2  →  RPi5 GPIO14  (TXD / ttyAMA0)

The RP2040 firmware speaks a simple newline-delimited JSON protocol:

  Host → RP2040   {"cmd": "gpio_write", "pin": 3, "value": 1}
  Host → RP2040   {"cmd": "gpio_read",  "pin": 4}
  Host → RP2040   {"cmd": "adc_read",   "ch": 0}
  Host → RP2040   {"cmd": "ping"}

  RP2040 → Host   {"ok": true,  "value": <int|float>}
  RP2040 → Host   {"ok": false, "error": "..."}
  RP2040 → Host   {"event": "sensor", "pin": 4, "value": 1}   (async push)
"""

import json
import logging
import threading
import time
from typing import Any, Callable

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from lumio.config import RP2040_UART_PORT, RP2040_UART_BAUD

log = logging.getLogger(__name__)


class RP2040:
    """
    Non-blocking UART interface to the RP2040 co-processor.

    A background reader thread pushes unsolicited 'event' frames to any
    registered callbacks; synchronous commands use a simple request-reply
    pattern with a configurable timeout.
    """

    def __init__(self, timeout: float = 1.0):
        self._timeout = timeout
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()
        self._reply_event = threading.Event()
        self._last_reply: dict | None = None
        self._event_callbacks: list[Callable[[dict], None]] = []
        self._running = False
        self._reader: threading.Thread | None = None
        self._connect()

    # ── Connection ────────────────────────────────────────────────────────────

    def _connect(self):
        if not SERIAL_AVAILABLE:
            log.warning("pyserial not available – RP2040 interface in stub mode")
            return
        try:
            self._serial = serial.Serial(
                RP2040_UART_PORT,
                RP2040_UART_BAUD,
                timeout=0.05,
            )
            self._running = True
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()
            log.info("RP2040 UART open: %s @ %d baud", RP2040_UART_PORT, RP2040_UART_BAUD)
        except Exception as exc:
            log.error("Cannot open RP2040 UART: %s", exc)
            self._serial = None

    # ── Background reader ─────────────────────────────────────────────────────

    def _read_loop(self):
        buf = b""
        while self._running:
            try:
                chunk = self._serial.read(64)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._dispatch(line.decode(errors="replace").strip())
            except Exception as exc:
                log.error("RP2040 read error: %s", exc)
                time.sleep(0.1)

    def _dispatch(self, line: str):
        if not line:
            return
        try:
            frame = json.loads(line)
        except json.JSONDecodeError:
            log.debug("RP2040 non-JSON: %r", line)
            return

        if "event" in frame:
            for cb in self._event_callbacks:
                try:
                    cb(frame)
                except Exception as exc:
                    log.error("Event callback error: %s", exc)
        else:
            # It's a reply to a synchronous command
            self._last_reply = frame
            self._reply_event.set()

    # ── Public API ────────────────────────────────────────────────────────────

    def on_event(self, callback: Callable[[dict], None]):
        """Register a callable that receives async event frames from RP2040."""
        self._event_callbacks.append(callback)

    def send(self, payload: dict) -> dict | None:
        """
        Send a JSON command and wait for a reply.
        Returns the reply dict, or None on timeout / connection error.
        """
        if self._serial is None:
            log.warning("RP2040 not connected – command dropped: %s", payload)
            return None
        with self._lock:
            self._reply_event.clear()
            self._last_reply = None
            try:
                line = json.dumps(payload) + "\n"
                self._serial.write(line.encode())
            except Exception as exc:
                log.error("RP2040 write error: %s", exc)
                return None
            if self._reply_event.wait(self._timeout):
                return self._last_reply
            log.warning("RP2040 reply timeout for: %s", payload)
            return None

    # ── Convenience wrappers ──────────────────────────────────────────────────

    def ping(self) -> bool:
        reply = self.send({"cmd": "ping"})
        return bool(reply and reply.get("ok"))

    def gpio_write(self, pin: int, value: int) -> bool:
        reply = self.send({"cmd": "gpio_write", "pin": pin, "value": value})
        return bool(reply and reply.get("ok"))

    def gpio_read(self, pin: int) -> int | None:
        reply = self.send({"cmd": "gpio_read", "pin": pin})
        if reply and reply.get("ok"):
            return reply.get("value")
        return None

    def adc_read(self, channel: int) -> float | None:
        reply = self.send({"cmd": "adc_read", "ch": channel})
        if reply and reply.get("ok"):
            return reply.get("value")
        return None

    def close(self):
        self._running = False
        if self._serial:
            self._serial.close()
        log.info("RP2040 UART closed")
