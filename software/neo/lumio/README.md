# Lumio V2 – Software Stack

Intelligent educational classroom robot running on **Raspberry Pi 5**.

---

## Hardware Pin Map (BCM GPIO)

| Peripheral | Signal | BCM GPIO |
|---|---|---|
| **L298N – Motor A (left)** | IN1 (forward) | 17 |
| | IN2 (backward) | 27 |
| | ENA (PWM) | 22 |
| **L298N – Motor B (right)** | IN3 (forward) | 18 |
| | IN4 (backward) | 23 |
| | ENB (PWM) | 12 |
| **Front ultrasonic** | TRIG | 5 |
| | ECHO | 6 |
| **Back ultrasonic** | TRIG | 13 |
| | ECHO | 26 |
| **MCP3008 ADC** | CS (SPI1 CE2) | 16 |
| | MOSI (SPI1) | 20 |
| | MISO (SPI1) | 35 |
| | CLK (SPI1) | 21 |
| **RP2040 UART** | TXD (RPi RX) | 15 |
| | RXD (RPi TX) | 14 |
| **I2C** | SCL | 3 |
| | SDA | 2 |

---

## Project Structure

```
lumio/
├── pyproject.toml        ← uv project manifest & dependencies
├── lumio.service         ← systemd unit file
├── lumio/
│   ├── main.py           ← entry point & boot sequence
│   ├── config.py         ← ALL pin numbers & constants
│   ├── robot.py          ← motor control + ultrasonic sensing
│   ├── adc.py            ← MCP3008 bit-bang SPI driver
│   ├── coprocessor.py    ← RP2040 JSON-UART interface
│   ├── wifi_provision.py ← hotspot + captive-portal WiFi setup
│   └── web.py            ← Flask/SocketIO control server
└── templates/
    ├── control.html      ← robot control dashboard
    └── gpio.html         ← RP2040 GPIO programming panel
```

---

## Prerequisites

```bash
# Raspberry Pi OS (Bookworm) with NetworkManager enabled
sudo apt update
sudo apt install -y python3 python3-dev gcc libffi-dev \
     network-manager nmcli avahi-daemon

# Install uv (universal Python package manager)
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.cargo/env
```

Enable UART for RP2040 (`/boot/firmware/config.txt`):
```
enable_uart=1
```
Disable the serial console so ttyAMA0 is free:
```bash
sudo raspi-config  # Interface Options → Serial Port → No console, Yes hardware
```

Enable I2C and SPI if using those buses:
```bash
sudo raspi-config  # Interface Options → I2C → Yes, SPI → Yes
```

---

## Installation

```bash
sudo mkdir -p /opt/lumio
sudo cp -r . /opt/lumio/
cd /opt/lumio

# Create the uv virtual environment and install all deps
uv sync

# Test run (Ctrl-C to exit)
sudo uv run python -m lumio.main
```

---

## Systemd Service

```bash
sudo cp lumio.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable lumio
sudo systemctl start lumio

# View live logs
sudo journalctl -u lumio -f
```

---

## First-Boot WiFi Provisioning

On first boot (or when no known WiFi is found):

1. Lumio creates a hotspot: **`Lumio-Setup`** / password **`lumio1234`**
2. Connect your phone or laptop to that network
3. Open **`http://192.168.50.1:8080`** (or any URL – it will redirect)
4. Enter your WiFi SSID and password → submit
5. Lumio connects, tears down the hotspot, and resumes normal operation
6. Credentials are saved to `/etc/lumio/wifi.conf` and survive reboots

On subsequent boots, Lumio auto-connects using saved credentials.

---

## Web Interface

Once on WiFi:

| URL | Purpose |
|---|---|
| `http://lumio.local` | Robot control dashboard |
| `http://lumio.local/gpio` | RP2040 GPIO programming panel |
| `http://lumio.local/api/status` | JSON system status snapshot |

---

## RP2040 UART Protocol

The RP2040 firmware must respond to newline-delimited JSON commands:

```json
{"cmd": "ping"}                        → {"ok": true}
{"cmd": "gpio_write", "pin": 5, "value": 1}  → {"ok": true}
{"cmd": "gpio_read",  "pin": 5}        → {"ok": true, "value": 0}
{"cmd": "adc_read",   "ch": 0}        → {"ok": true, "value": 1.65}
```

Async push from RP2040 (e.g. sensor threshold crossed):
```json
{"event": "sensor", "pin": 5, "value": 1}
```

---

## Development (non-Pi machine)

```bash
uv sync
uv run python -m lumio.main
# GPIO calls are silently stubbed – web interface still functional
```

---

## Changing Pin Assignments

Edit **`lumio/config.py`** only – everything else reads from there.

---

## License

MIT – see `LICENSE`.
