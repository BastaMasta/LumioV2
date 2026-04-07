"""
lumio/adc.py
------------
Bit-banged SPI driver for the MCP3008 8-channel 10-bit ADC.

Wired to SPI1 GPIO pins (BCM):
    CS   = 16
    MOSI = 20
    MISO = 35
    CLK  = 21
"""

import time
import logging

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

from lumio.config import ADC_CS, ADC_MOSI, ADC_MISO, ADC_CLK

log = logging.getLogger(__name__)

_VREF = 3.3          # Reference voltage (Pi 5 GPIO is 3.3 V)
_MAX_CODE = 1023     # 10-bit ADC


class MCP3008:
    """
    Software-SPI interface to the MCP3008.

    Usage::

        adc = MCP3008()
        raw   = adc.read_raw(channel=0)      # 0-1023
        volts = adc.read_voltage(channel=0)  # 0.0 – 3.3 V
    """

    def __init__(self):
        if not GPIO_AVAILABLE:
            log.warning("GPIO unavailable – MCP3008 in simulation mode")
            return
        # Pins should already be in BCM mode (set by RobotController or main)
        GPIO.setup(ADC_CS,   GPIO.OUT)
        GPIO.setup(ADC_CLK,  GPIO.OUT)
        GPIO.setup(ADC_MOSI, GPIO.OUT)
        GPIO.setup(ADC_MISO, GPIO.IN)
        GPIO.output(ADC_CS,  GPIO.HIGH)
        GPIO.output(ADC_CLK, GPIO.LOW)
        log.info("MCP3008 ADC initialised (bit-bang SPI)")

    # ── Internal bit-bang read ────────────────────────────────────────────────

    def _read_raw(self, channel: int) -> int:
        if not (0 <= channel <= 7):
            raise ValueError(f"Channel must be 0-7, got {channel}")

        GPIO.output(ADC_CS, GPIO.LOW)

        # Start bit + single-ended + channel select (4 bits)
        cmd = 0b11000 | channel   # start=1, SGL=1, D2..D0=channel
        for bit in range(4, -1, -1):
            GPIO.output(ADC_MOSI, GPIO.HIGH if (cmd >> bit) & 1 else GPIO.LOW)
            GPIO.output(ADC_CLK, GPIO.HIGH)
            GPIO.output(ADC_CLK, GPIO.LOW)

        # One null bit then 10 data bits
        result = 0
        for _ in range(11):
            GPIO.output(ADC_CLK, GPIO.HIGH)
            GPIO.output(ADC_CLK, GPIO.LOW)
            result = (result << 1) | GPIO.input(ADC_MISO)

        GPIO.output(ADC_CS, GPIO.HIGH)
        return result & 0x3FF   # mask to 10 bits

    # ── Public interface ──────────────────────────────────────────────────────

    def read_raw(self, channel: int) -> int:
        """Return raw ADC code 0-1023."""
        if not GPIO_AVAILABLE:
            return 0
        try:
            return self._read_raw(channel)
        except Exception as exc:
            log.error("ADC read error on ch%d: %s", channel, exc)
            return 0

    def read_voltage(self, channel: int) -> float:
        """Return voltage in volts (0.0 – 3.3 V)."""
        return round(self.read_raw(channel) * _VREF / _MAX_CODE, 4)

    def read_all_raw(self) -> dict[int, int]:
        """Read all 8 channels; returns {ch: raw_value}."""
        return {ch: self.read_raw(ch) for ch in range(8)}

    def read_all_voltage(self) -> dict[int, float]:
        """Read all 8 channels; returns {ch: voltage}."""
        return {ch: self.read_voltage(ch) for ch in range(8)}
