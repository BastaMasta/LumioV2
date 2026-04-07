# Lumio Educational Robot Platform — Software Setup

## Hardware

- Raspberry Pi 5 (primary compute)
- DFRobot UPS Module for Raspberry Pi 5 (on-system power)
- Lumio V2 PCB (connected via 40-pin GPIO extension ribbon cable)
- RP2040 co-processor (on Lumio V2 PCB, connected via UART /dev/ttyAMA0)

## File structure

```
lumio/
├── main.py                  # Main application (run this on RPi 5)
├── requirements.txt         # Python dependencies
├── test_lumio.py            # Unit test suite (runs off-hardware)
└── rp2040_firmware/
    └── main.py              # MicroPython firmware for the RP2040 on Lumio V2 PCB
```

## Raspberry Pi 5 setup

```bash
# 1. Enable interfaces
sudo raspi-config
# → Interface Options → SPI: Enable
# → Interface Options → Serial Port: Enable (disable login shell, enable serial hardware)
# → Interface Options → I2C: Enable

# 2. Install system packages
sudo apt update
sudo apt install -y python3-pip python3-venv portaudio19-dev aplay

# 3. Install Ollama
curl -fsSL https://ollama.com/install.sh | sh
ollama pull phi3:mini

# 4. Install Piper TTS
mkdir -p ~/piper ~/models/piper
wget https://github.com/rhasspy/piper/releases/download/v1.2.0/piper_arm64.tar.gz
tar -xzf piper_arm64.tar.gz -C ~/piper/
wget -P ~/models/piper/ \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json

# 5. Install Vosk model
mkdir -p ~/models
wget https://alphacephei.com/vosk/models/vosk-model-en-us-0.22-lgraph.zip
unzip vosk-model-en-us-0.22-lgraph.zip -d ~/models/

# 6. Install Python dependencies
cd ~/lumio
pip install -r requirements.txt
```

## RP2040 firmware flashing

1. Hold **BOOTSEL** (SW1 on Lumio V2 PCB), press **RESET** (SW2), release RESET
2. RP2040 appears as USB mass storage drive
3. Open Thonny IDE → connect to RP2040 → open `rp2040_firmware/main.py` → Run
   (or copy the file directly to the drive as `main.py`)

## Running

```bash
# Test mode (no GPIO required — runs on any machine)
python3 main.py --simulate

# Full hardware mode (on Raspberry Pi 5 with Lumio V2 PCB)
python3 main.py

# Run unit tests (no hardware required)
python3 -m pytest test_lumio.py -v
```

## WiFi control panel

Once running, open a browser on any device on the same network:
```
http://<raspberry-pi-ip>:5000
```

## Voice control

Say **"Lumio"** (wake word) then your command:
- "Lumio, move forward"
- "Lumio, turn left"
- "Lumio, stop"
- "Lumio, what is photosynthesis?" (Q&A — answered by Ollama)

## Systemd service (auto-start on boot)

```bash
sudo nano /etc/systemd/system/lumio.service
```

```ini
[Unit]
Description=Lumio Educational Robot Platform
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/lumio
ExecStart=/usr/bin/python3 /home/pi/lumio/main.py
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
EnvironmentFile=/home/pi/lumio/.env

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable lumio
sudo systemctl start lumio
sudo journalctl -u lumio -f    # follow logs
```
