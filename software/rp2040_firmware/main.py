"""
Lumio V2 PCB — RP2040 Co-Processor Firmware
============================================
MicroPython | Lumio V2 PCB | KiCad 9.0

Runs on the RP2040 (U5) on the Lumio V2 PCB.
Listens on UART0 at 115200 baud for JSON commands from the Raspberry Pi 5.
Responds with JSON results.

Hardware connections (Lumio V2 PCB):
    UART0 TX → Raspberry Pi 5 RX (via ribbon cable)
    UART0 RX → Raspberry Pi 5 TX (via ribbon cable)
    QSPI     → W25Q64 SPI flash (U4) — used for program storage only
    XIN/XOUT → 12MHz HC-49SMD crystal (Y1) with 33pF load caps

Supported commands
------------------
    {"action": "read_adc", "channel": 0}
        → {"channel": 0, "raw": 512, "voltage": 1.65}

    {"action": "set_gpio", "pin": 5, "value": 1}
        → {"ok": true}

    {"action": "read_gpio", "pin": 5}
        → {"pin": 5, "value": 1}

    {"action": "read_ultrasonic", "sensor": "front"}
        → {"sensor": "front", "distance_cm": 34.2}

    {"action": "ping"}
        → {"pong": true, "uptime_s": 12.4}

Flash with Thonny IDE or via BOOTSEL USB mass storage:
    1. Hold BOOTSEL (SW1), press RESET (SW2), release RESET
    2. RP2040 appears as USB drive
    3. Copy this file as main.py to the drive
"""

import json
import time
import machine
from machine import UART, Pin, ADC

# ── UART ─────────────────────────────────────────────────────────────────────
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# ── ADC ──────────────────────────────────────────────────────────────────────
# MCP3008 is on SPI, but the RP2040 also has 3 onboard ADC channels (GP26-28)
# The RP2040 proxies the MCP3008 readings over SPI and returns them via UART.
# For the onboard ADC channels (if used for voltage monitoring):
_ADC_PINS = {
    26: ADC(Pin(26)),
    27: ADC(Pin(27)),
    28: ADC(Pin(28)),
}

# MCP3008 SPI interface (connected to RP2040 SPI1 on Lumio V2 PCB)
from machine import SPI
_spi = SPI(
    1,
    baudrate=1_000_000,
    polarity=0,
    phase=0,
    sck=Pin(10),
    mosi=Pin(11),
    miso=Pin(12),
)
_cs = Pin(13, Pin.OUT, value=1)   # MCP3008 CS — active low


def read_mcp3008(channel: int) -> int:
    """
    Read a single-ended channel (0–7) from the MCP3008 via SPI.
    Returns raw 10-bit value (0–1023).

    MCP3008 SPI protocol:
        Start bit: 1
        Mode: 1 (single-ended)
        Channel select: D2 D1 D0
        Two bytes sent, result in lower 10 bits of response.
    """
    if not 0 <= channel <= 7:
        return 0
    cmd = bytearray(3)
    cmd[0] = 0x01                       # start bit
    cmd[1] = (0x08 | channel) << 4     # single-ended + channel
    cmd[2] = 0x00
    result = bytearray(3)
    _cs.value(0)
    _spi.write_readinto(cmd, result)
    _cs.value(1)
    raw = ((result[1] & 0x03) << 8) | result[2]
    return raw


# ── HC-SR04 ultrasonic sensors ────────────────────────────────────────────────
# Front sensor (J16 on Lumio V2 PCB)
_ULTRA_FRONT_TRIG = Pin(2, Pin.OUT, value=0)
_ULTRA_FRONT_ECHO = Pin(3, Pin.IN)
# Back sensor (J17 on Lumio V2 PCB)
_ULTRA_BACK_TRIG  = Pin(4, Pin.OUT, value=0)
_ULTRA_BACK_ECHO  = Pin(5, Pin.IN)

SPEED_OF_SOUND_CM_US = 0.0343   # cm per microsecond at ~20°C
ULTRA_TIMEOUT_US = 30_000       # 30ms = max ~5m range


def measure_distance(trig: Pin, echo: Pin) -> float:
    """
    Measure distance in cm using HC-SR04 pulse timing.
    Returns 999.0 on timeout (no echo / out of range).
    """
    # 10µs trigger pulse
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    # Wait for echo to go high
    start = time.ticks_us()
    while echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), start) > ULTRA_TIMEOUT_US:
            return 999.0

    # Measure echo pulse width
    pulse_start = time.ticks_us()
    while echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), pulse_start) > ULTRA_TIMEOUT_US:
            return 999.0
    pulse_end = time.ticks_us()

    duration_us = time.ticks_diff(pulse_end, pulse_start)
    distance_cm = (duration_us * SPEED_OF_SOUND_CM_US) / 2.0
    return round(distance_cm, 1)


# ── GPIO management ───────────────────────────────────────────────────────────
_gpio_cache: dict = {}

def get_gpio(pin_num: int) -> Pin:
    """Get or create a GPIO Pin object, cached by pin number."""
    if pin_num not in _gpio_cache:
        _gpio_cache[pin_num] = Pin(pin_num, Pin.OUT)
    return _gpio_cache[pin_num]


# ── Command dispatcher ────────────────────────────────────────────────────────

_start_time = time.time()


def dispatch(payload: dict) -> dict:
    """Route a parsed command dict to the appropriate handler."""
    action = payload.get("action", "")

    if action == "ping":
        return {"pong": True, "uptime_s": round(time.time() - _start_time, 1)}

    elif action == "read_adc":
        channel = int(payload.get("channel", 0))
        raw = read_mcp3008(channel)
        voltage = round((raw / 1023.0) * 3.3, 3)
        return {"channel": channel, "raw": raw, "voltage": voltage}

    elif action == "set_gpio":
        pin_num = int(payload.get("pin", 0))
        value   = int(payload.get("value", 0))
        get_gpio(pin_num).value(value)
        return {"ok": True, "pin": pin_num, "value": value}

    elif action == "read_gpio":
        pin_num = int(payload.get("pin", 0))
        p = Pin(pin_num, Pin.IN)
        return {"pin": pin_num, "value": p.value()}

    elif action == "read_ultrasonic":
        sensor = payload.get("sensor", "front")
        if sensor == "front":
            dist = measure_distance(_ULTRA_FRONT_TRIG, _ULTRA_FRONT_ECHO)
        elif sensor == "back":
            dist = measure_distance(_ULTRA_BACK_TRIG, _ULTRA_BACK_ECHO)
        else:
            return {"error": f"unknown sensor: {sensor}"}
        return {"sensor": sensor, "distance_cm": dist}

    else:
        return {"error": f"unknown action: {action}"}


# ── Main loop ─────────────────────────────────────────────────────────────────

def main() -> None:
    buf = ""
    print("Lumio RP2040 firmware ready")

    while True:
        # Read available bytes from UART
        if uart.any():
            chunk = uart.read(256)
            if chunk:
                buf += chunk.decode("utf-8", "ignore")

        # Process complete newline-terminated commands
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            try:
                payload = json.loads(line)
                response = dispatch(payload)
            except Exception as exc:
                response = {"error": str(exc)}

            # Send response
            uart.write(json.dumps(response) + "\n")

        # Small yield to prevent busy-spin at 100% CPU
        time.sleep_ms(1)


main()
