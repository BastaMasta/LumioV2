"""
lumio/config.py
---------------
Hardware pin definitions for Lumio V2 (BCM GPIO numbering).
All values match the V2 schematic and user-confirmed wiring.
"""

# ── Motor driver (L298N) ──────────────────────────────────────────────────────
# Motor A  =  Left  wheel
MOTOR_A_IN1  = 17   # Forward
MOTOR_A_IN2  = 27   # Backward
MOTOR_A_ENA  = 22   # PWM speed (Hardware PWM capable on Pi 5)

# Motor B  =  Right wheel
MOTOR_B_IN3  = 18   # Forward
MOTOR_B_IN4  = 23   # Backward
MOTOR_B_ENB  = 12   # PWM speed (Hardware PWM capable on Pi 5)

# ── Ultrasonic distance sensors (direct RPi5 GPIO for low latency) ────────────
FRONT_TRIG   = 5
FRONT_ECHO   = 6
BACK_TRIG    = 13
BACK_ECHO    = 26   # Changed from old REAR_ECHO=19 per V2 schematic

# ── MCP3008 ADC (SPI1 — software-mapped pins) ────────────────────────────────
ADC_CS       = 16   # SPI1 CE2
ADC_MOSI     = 20   # SPI1 MOSI
ADC_MISO     = 35   # SPI1 MISO  (BCM 35 = physical pin 19 on Pi 5 40-pin header)
ADC_CLK      = 21   # SPI1 CLK

# ── RP2040 co-processor UART ──────────────────────────────────────────────────
# rp2040 GPIO1 → RPi5 GPIO15 (RXD)
# rp2040 GPIO2 → RPi5 GPIO14 (TXD)
RP2040_UART_PORT = "/dev/ttyAMA0"   # UART0 on Pi 5 (GPIO14/15)
RP2040_UART_BAUD = 115200

# ── I2C bus ───────────────────────────────────────────────────────────────────
I2C_SCL      = 3    # I2C1 SCL
I2C_SDA      = 2    # I2C1 SDA
I2C_BUS      = 1    # /dev/i2c-1

# ── Motion parameters ─────────────────────────────────────────────────────────
LINEAR_SPEED_MPS     = 0.7    # metres per second at 50 % duty
ROTATION_SPEED_DPS   = 270    # degrees per second at 50 % duty
OBSTACLE_THRESHOLD_CM = 20    # emergency-stop distance (cm)
PWM_FREQUENCY_HZ     = 200    # motor PWM frequency

# ── Hotspot / WiFi provisioning ───────────────────────────────────────────────
HOTSPOT_SSID     = "Lumio-Setup"
HOTSPOT_PASSWORD = "lumio1234"
HOTSPOT_IP       = "192.168.50.1"
PROVISION_PORT   = 8080
