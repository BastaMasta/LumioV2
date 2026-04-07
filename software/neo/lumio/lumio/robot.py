"""
lumio/robot.py
--------------
Low-level robot hardware controller.
Handles motor control and ultrasonic obstacle detection directly on RPi5 GPIO.
"""

import time
import logging
import threading

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    logging.warning("RPi.GPIO not available – running in simulation mode")

from lumio.config import (
    MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENA,
    MOTOR_B_IN3, MOTOR_B_IN4, MOTOR_B_ENB,
    FRONT_TRIG, FRONT_ECHO, BACK_TRIG, BACK_ECHO,
    LINEAR_SPEED_MPS, ROTATION_SPEED_DPS,
    OBSTACLE_THRESHOLD_CM, PWM_FREQUENCY_HZ,
)

log = logging.getLogger(__name__)


class _FakeGPIO:
    """Minimal stub so the module can load on non-Pi hardware for dev/testing."""
    BCM = OUT = IN = HIGH = LOW = 0

    def setwarnings(self, *a): pass
    def setmode(self, *a): pass
    def setup(self, *a, **kw): pass
    def output(self, *a): pass
    def input(self, *a): return 0
    def cleanup(self, *a): pass

    class PWM:
        def __init__(self, *a): pass
        def start(self, *a): pass
        def stop(self): pass
        def ChangeDutyCycle(self, *a): pass


if not GPIO_AVAILABLE:
    GPIO = _FakeGPIO()


class RobotController:
    """
    Drives the L298N dual H-bridge and reads the two HC-SR04 ultrasonic sensors.

    Motor A (left)  : IN1=17, IN2=27, ENA=22
    Motor B (right) : IN3=18, IN4=23, ENB=12
    Front US        : TRIG=5,  ECHO=6
    Back  US        : TRIG=13, ECHO=26
    """

    def __init__(self):
        self.speed       = LINEAR_SPEED_MPS
        self.rot_speed   = ROTATION_SPEED_DPS
        self.threshold   = OBSTACLE_THRESHOLD_CM
        self.stopped     = False
        self._pwm_a: GPIO.PWM | None = None
        self._pwm_b: GPIO.PWM | None = None
        self._lock = threading.Lock()
        self._setup_gpio()

    # ── GPIO setup ────────────────────────────────────────────────────────────

    def _setup_gpio(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        motor_pins = [MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENA,
                      MOTOR_B_IN3, MOTOR_B_IN4, MOTOR_B_ENB]
        GPIO.setup(motor_pins, GPIO.OUT)
        GPIO.output(motor_pins, GPIO.LOW)
        GPIO.setup(FRONT_TRIG, GPIO.OUT)
        GPIO.setup(FRONT_ECHO, GPIO.IN)
        GPIO.setup(BACK_TRIG,  GPIO.OUT)
        GPIO.setup(BACK_ECHO,  GPIO.IN)
        log.info("GPIO configured (BCM mode)")

    # ── Ultrasonic sensing ────────────────────────────────────────────────────

    def measure_distance_cm(self, trig_pin: int, echo_pin: int) -> float:
        """Return distance in cm; returns 999 on timeout/error."""
        try:
            GPIO.output(trig_pin, GPIO.HIGH)
            time.sleep(1e-5)
            GPIO.output(trig_pin, GPIO.LOW)

            deadline = time.monotonic() + 0.1
            while GPIO.input(echo_pin) == 0:
                if time.monotonic() > deadline:
                    return 999.0
                t_start = time.monotonic()

            deadline = time.monotonic() + 0.1
            while GPIO.input(echo_pin) == 1:
                if time.monotonic() > deadline:
                    return 999.0
                t_end = time.monotonic()

            return round((t_end - t_start) * 17150, 2)
        except Exception as exc:
            log.error("Distance measurement error: %s", exc)
            return 999.0

    @property
    def front_distance_cm(self) -> float:
        return self.measure_distance_cm(FRONT_TRIG, FRONT_ECHO)

    @property
    def back_distance_cm(self) -> float:
        return self.measure_distance_cm(BACK_TRIG, BACK_ECHO)

    def obstacle_front(self) -> bool:
        d = self.front_distance_cm
        log.debug("Front distance: %.1f cm", d)
        return d < self.threshold

    def obstacle_back(self) -> bool:
        d = self.back_distance_cm
        log.debug("Back distance: %.1f cm", d)
        return d < self.threshold

    # ── Internal PWM helpers ──────────────────────────────────────────────────

    def _start_pwm(self, duty: int = 50):
        self._pwm_a = GPIO.PWM(MOTOR_A_ENA, PWM_FREQUENCY_HZ)
        self._pwm_b = GPIO.PWM(MOTOR_B_ENB, PWM_FREQUENCY_HZ)
        self._pwm_a.start(duty)
        self._pwm_b.start(duty)

    def _set_duty(self, duty: int):
        if self._pwm_a:
            self._pwm_a.ChangeDutyCycle(duty)
        if self._pwm_b:
            self._pwm_b.ChangeDutyCycle(duty)

    def _stop_pwm(self):
        if self._pwm_a:
            self._pwm_a.stop()
            self._pwm_a = None
        if self._pwm_b:
            self._pwm_b.stop()
            self._pwm_b = None

    def _all_motor_low(self):
        GPIO.output([MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN3, MOTOR_B_IN4], GPIO.LOW)
        self._set_duty(0)

    # ── Emergency stops ───────────────────────────────────────────────────────

    def _nudge(self, forward: bool, duty: int = 30, duration: float = 0.5):
        """Move briefly to clear obstacle."""
        if forward:
            GPIO.output([MOTOR_A_IN1, MOTOR_B_IN3], GPIO.HIGH)
            GPIO.output([MOTOR_A_IN2, MOTOR_B_IN4], GPIO.LOW)
        else:
            GPIO.output([MOTOR_A_IN2, MOTOR_B_IN4], GPIO.HIGH)
            GPIO.output([MOTOR_A_IN1, MOTOR_B_IN3], GPIO.LOW)
        self._set_duty(duty)
        time.sleep(duration)
        self._all_motor_low()

    def emergency_stop(self, obstacle_front: bool = True):
        log.warning("OBSTACLE – emergency stop (%s)", "front" if obstacle_front else "back")
        self.stopped = True
        self._all_motor_low()
        # Nudge away from obstacle
        self._nudge(forward=not obstacle_front)

    # ── Movement primitives ───────────────────────────────────────────────────

    def move_forward(self, distance_m: float = 1.0, duty: int = 50) -> bool:
        """Move forward <distance_m> metres; returns False if blocked."""
        with self._lock:
            self.stopped = False
            log.info("Forward %.2f m", distance_m)
            self._start_pwm(duty)
            GPIO.output([MOTOR_A_IN1, MOTOR_B_IN3], GPIO.HIGH)
            GPIO.output([MOTOR_A_IN2, MOTOR_B_IN4], GPIO.LOW)
            t_end = time.monotonic() + distance_m / self.speed
            while time.monotonic() < t_end:
                if self.obstacle_front():
                    self.emergency_stop(obstacle_front=True)
                    self._stop_pwm()
                    return False
                time.sleep(0.05)
            self._all_motor_low()
            self._stop_pwm()
            return True

    def move_backward(self, distance_m: float = 1.0, duty: int = 50) -> bool:
        """Move backward <distance_m> metres; returns False if blocked."""
        with self._lock:
            self.stopped = False
            log.info("Backward %.2f m", distance_m)
            self._start_pwm(duty)
            GPIO.output([MOTOR_A_IN2, MOTOR_B_IN4], GPIO.HIGH)
            GPIO.output([MOTOR_A_IN1, MOTOR_B_IN3], GPIO.LOW)
            t_end = time.monotonic() + distance_m / self.speed
            while time.monotonic() < t_end:
                if self.obstacle_back():
                    self.emergency_stop(obstacle_front=False)
                    self._stop_pwm()
                    return False
                time.sleep(0.05)
            self._all_motor_low()
            self._stop_pwm()
            return True

    def turn(self, angle_deg: float, duty: int = 50) -> bool:
        """
        Turn in place.  Positive  = right (clockwise).
                        Negative  = left  (counter-clockwise).
        """
        if angle_deg == 0:
            return True
        with self._lock:
            log.info("Turn %.1f°", angle_deg)
            pwm_a = GPIO.PWM(MOTOR_A_ENA, 90)
            pwm_b = GPIO.PWM(MOTOR_B_ENB, 90)
            pwm_a.start(duty)
            pwm_b.start(duty)
            if angle_deg > 0:   # right: A forward, B backward
                GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
                GPIO.output(MOTOR_A_IN2, GPIO.LOW)
                GPIO.output(MOTOR_B_IN4, GPIO.HIGH)
                GPIO.output(MOTOR_B_IN3, GPIO.LOW)
            else:               # left: A backward, B forward
                GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
                GPIO.output(MOTOR_A_IN1, GPIO.LOW)
                GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
                GPIO.output(MOTOR_B_IN4, GPIO.LOW)
            time.sleep(abs(angle_deg) / self.rot_speed)
            GPIO.output([MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN3, MOTOR_B_IN4], GPIO.LOW)
            pwm_a.stop()
            pwm_b.stop()
            return True

    def stop(self):
        """Immediate full stop (no nudge)."""
        self._all_motor_low()
        self._stop_pwm()
        self.stopped = True
        log.info("Full stop")

    # ── Routines ──────────────────────────────────────────────────────────────

    def dance(self) -> bool:
        ok = self.move_forward(0.5)
        if not ok:
            return False
        time.sleep(0.2)
        ok = self.move_backward(0.5)
        if not ok:
            return False
        time.sleep(0.2)
        self.turn(-45)
        time.sleep(0.15)
        self.turn(90)
        time.sleep(0.15)
        self.turn(-45)
        return True

    def say_hi(self) -> bool:
        self.turn(-30)
        time.sleep(0.2)
        self.turn(60)
        time.sleep(0.2)
        self.turn(-30)
        return True

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self):
        self._stop_pwm()
        GPIO.cleanup()
        log.info("GPIO cleaned up")
