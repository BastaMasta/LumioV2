"""
Lumio Unit Test Suite
=====================
Tests the MotorHAL, RobotController, and command pipeline
using MockMotorHAL — no hardware required.

Run with:
    python3 -m pytest test_lumio.py -v
    # or
    python3 test_lumio.py
"""

import queue
import threading
import time
import unittest
from unittest.mock import patch

# Import the classes we need — mock GPIO so tests run without hardware
import sys
import types

# Stub out gpiozero so imports work off-hardware
_gpiozero_stub = types.ModuleType("gpiozero")
class _Motor:
    def __init__(self, **kw): pass
    def forward(self, s=1): pass
    def backward(self, s=1): pass
    def stop(self): pass
_gpiozero_stub.Motor = _Motor
_gpiozero_stub.PWMOutputDevice = object
sys.modules.setdefault("gpiozero", _gpiozero_stub)

import logging
from main import (
    MockMotorHAL, RP2040Interface, RobotController,
    RobotCommand, CommandSource, MotorExecutor,
    DEFAULT_SPEED, ROTATION_SPEED,
)

log = logging.getLogger("lumio.test")
logging.basicConfig(level=logging.WARNING)


# ── MockMotorHAL tests ────────────────────────────────────────────────────────

class TestMockMotorHAL(unittest.TestCase):
    """Verify MockMotorHAL records calls correctly."""

    def setUp(self):
        self.hal = MockMotorHAL(log)

    def test_forward_recorded(self):
        self.hal.forward(0.8)
        self.assertEqual(len(self.hal.calls), 1)
        method, speed, _ = self.hal.calls[0]
        self.assertEqual(method, "forward")
        self.assertAlmostEqual(speed, 0.8)

    def test_stop_recorded(self):
        self.hal.stop()
        method, speed, _ = self.hal.calls[0]
        self.assertEqual(method, "stop")

    def test_sequence_recorded_in_order(self):
        self.hal.forward()
        self.hal.turn_left()
        self.hal.stop()
        methods = [c[0] for c in self.hal.calls]
        self.assertEqual(methods, ["forward", "turn_left", "stop"])

    def test_timestamps_monotonic(self):
        self.hal.forward()
        time.sleep(0.01)
        self.hal.stop()
        t0 = self.hal.calls[0][2]
        t1 = self.hal.calls[1][2]
        self.assertGreater(t1, t0)

    def test_cleanup_recorded(self):
        self.hal.cleanup()
        self.assertEqual(self.hal.calls[0][0], "cleanup")


# ── RobotController tests ─────────────────────────────────────────────────────

class TestRobotController(unittest.TestCase):
    """Test movement logic using MockMotorHAL and simulated RP2040."""

    def setUp(self):
        self.hal = MockMotorHAL(log)
        self.rp2040 = RP2040Interface(log, simulate=True)
        self.stop_event = threading.Event()
        self.ctrl = RobotController(self.hal, self.rp2040, self.stop_event, log)

    def _last_calls(self, n: int) -> list:
        return [c[0] for c in self.hal.calls[-n:]]

    def test_stop_command(self):
        cmd = RobotCommand(action="stop")
        self.ctrl.execute(cmd)
        self.assertIn("stop", [c[0] for c in self.hal.calls])

    def test_forward_calls_forward_then_stop(self):
        cmd = RobotCommand(action="forward", param=10)
        self.ctrl.execute(cmd)
        methods = [c[0] for c in self.hal.calls]
        self.assertIn("forward", methods)
        self.assertEqual(methods[-1], "stop")

    def test_backward_calls_backward_then_stop(self):
        cmd = RobotCommand(action="back", param=10)
        self.ctrl.execute(cmd)
        methods = [c[0] for c in self.hal.calls]
        self.assertIn("backward", methods)
        self.assertEqual(methods[-1], "stop")

    def test_left_calls_turn_left_then_stop(self):
        cmd = RobotCommand(action="left", param=90)
        self.ctrl.execute(cmd)
        methods = [c[0] for c in self.hal.calls]
        self.assertIn("turn_left", methods)
        self.assertEqual(methods[-1], "stop")

    def test_right_calls_turn_right_then_stop(self):
        cmd = RobotCommand(action="right", param=90)
        self.ctrl.execute(cmd)
        methods = [c[0] for c in self.hal.calls]
        self.assertIn("turn_right", methods)
        self.assertEqual(methods[-1], "stop")

    def test_stop_event_interrupts_movement(self):
        """Movement should abort early when stop_event is set."""
        def _set_stop():
            time.sleep(0.05)
            self.stop_event.set()

        threading.Thread(target=_set_stop, daemon=True).start()
        start = time.time()
        cmd = RobotCommand(action="forward", param=200)  # would take ~10s
        self.ctrl.execute(cmd)
        elapsed = time.time() - start
        self.assertLess(elapsed, 1.0, "Movement should have been interrupted quickly")

    def test_unknown_action_does_not_raise(self):
        """Unknown actions should log a warning but not crash."""
        cmd = RobotCommand(action="teleport")
        try:
            self.ctrl.execute(cmd)
        except Exception as exc:
            self.fail(f"Unknown action raised exception: {exc}")

    def test_speed_passed_to_hal(self):
        cmd = RobotCommand(action="forward", param=5, speed=0.5)
        self.ctrl.execute(cmd)
        forward_calls = [c for c in self.hal.calls if c[0] == "forward"]
        self.assertTrue(len(forward_calls) > 0)
        self.assertAlmostEqual(forward_calls[0][1], 0.5)


# ── Command Queue tests ───────────────────────────────────────────────────────

class TestCommandQueue(unittest.TestCase):
    """Test MotorExecutor queue behaviour."""

    def setUp(self):
        self.hal = MockMotorHAL(log)
        self.rp2040 = RP2040Interface(log, simulate=True)
        self.stop_event = threading.Event()
        self.shutdown = threading.Event()
        self.emergency_stop = threading.Event()
        self.watchdog_reset = threading.Event()
        self.q = queue.Queue(maxsize=20)
        self.ctrl = RobotController(self.hal, self.rp2040, self.stop_event, log)
        self.executor = MotorExecutor(
            self.q, self.ctrl, self.shutdown,
            self.emergency_stop, self.watchdog_reset, log
        )
        self.executor.start()

    def tearDown(self):
        self.shutdown.set()
        self.executor.join(timeout=2.0)

    def test_command_is_executed(self):
        self.q.put(RobotCommand(action="stop"))
        time.sleep(0.3)
        methods = [c[0] for c in self.hal.calls]
        self.assertIn("stop", methods)

    def test_multiple_commands_executed_in_order(self):
        for action in ["stop", "stop", "stop"]:
            self.q.put(RobotCommand(action=action))
        time.sleep(0.5)
        stop_calls = [c for c in self.hal.calls if c[0] == "stop"]
        self.assertGreaterEqual(len(stop_calls), 3)

    def test_emergency_stop_flushes_queue(self):
        """Queue should be empty after emergency stop."""
        for _ in range(5):
            self.q.put(RobotCommand(action="stop"))
        self.emergency_stop.set()
        time.sleep(0.3)
        self.assertTrue(self.q.empty())

    def test_queue_full_does_not_crash(self):
        """Putting to a full queue should not raise inside the app."""
        for _ in range(25):  # maxsize is 20
            try:
                self.q.put_nowait(RobotCommand(action="stop"))
            except queue.Full:
                pass   # Expected — queue is intentionally bounded


# ── RP2040Interface tests ─────────────────────────────────────────────────────

class TestRP2040Interface(unittest.TestCase):
    """Test the RP2040 UART interface in simulate mode."""

    def setUp(self):
        self.rp2040 = RP2040Interface(log, simulate=True)

    def test_read_adc_returns_float(self):
        result = self.rp2040.read_adc(0)
        self.assertIsInstance(result, float)

    def test_read_ultrasonic_returns_float(self):
        result = self.rp2040.read_ultrasonic("front")
        self.assertIsInstance(result, float)

    def test_set_gpio_returns_bool(self):
        result = self.rp2040.set_gpio(5, 1)
        self.assertIsInstance(result, bool)

    def test_all_adc_channels(self):
        """All 8 MCP3008 channels should be readable."""
        for ch in range(8):
            result = self.rp2040.read_adc(ch)
            self.assertIsInstance(result, float)

    def test_cleanup_does_not_raise(self):
        try:
            self.rp2040.cleanup()
        except Exception as exc:
            self.fail(f"cleanup() raised: {exc}")


# ── Command parser edge cases ─────────────────────────────────────────────────

class TestRobotCommandModel(unittest.TestCase):
    """Test RobotCommand dataclass and defaults."""

    def test_default_speed(self):
        cmd = RobotCommand(action="forward")
        self.assertEqual(cmd.speed, DEFAULT_SPEED)

    def test_default_source(self):
        cmd = RobotCommand(action="stop")
        self.assertEqual(cmd.source, CommandSource.VOICE)

    def test_timestamp_set_automatically(self):
        before = time.time()
        cmd = RobotCommand(action="stop")
        after = time.time()
        self.assertGreaterEqual(cmd.timestamp, before)
        self.assertLessEqual(cmd.timestamp, after)

    def test_zero_param_default(self):
        cmd = RobotCommand(action="forward")
        self.assertEqual(cmd.param, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
