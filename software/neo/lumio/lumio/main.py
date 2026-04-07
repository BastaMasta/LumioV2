"""
lumio/main.py
-------------
Lumio V2 entry point.

Boot sequence
─────────────
1. Logging setup
2. GPIO + hardware init
3. WiFi provisioning (hotspot → captive portal → connect)
4. Start Flask/SocketIO web server
"""

import logging
import os
import sys
import threading

from lumio.config import HOTSPOT_IP, PROVISION_PORT
from lumio.robot import RobotController
from lumio.adc import MCP3008
from lumio.coprocessor import RP2040
from lumio.wifi_provision import ensure_wifi
from lumio.web import create_app

# ── Logging ───────────────────────────────────────────────────────────────────

LOG_PATH = os.environ.get("LUMIO_LOG", "/var/log/lumio/lumio.log")
os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.FileHandler(LOG_PATH),
        logging.StreamHandler(sys.stdout),
    ],
)
log = logging.getLogger("lumio")


# ── Bootstrap ─────────────────────────────────────────────────────────────────

def main():
    log.info("═══════════════════════════════════════════")
    log.info("  Lumio V2 – Educational Robot Platform")
    log.info("═══════════════════════════════════════════")

    # ── 1. Hardware init ──────────────────────────────────────────────────────
    log.info("Initialising hardware…")
    robot = RobotController()
    adc   = MCP3008()
    rp    = RP2040()

    if rp.ping():
        log.info("RP2040 co-processor online ✓")
    else:
        log.warning("RP2040 co-processor not responding – GPIO expansion unavailable")

    # ── 2. WiFi provisioning ──────────────────────────────────────────────────
    log.info("Checking WiFi…")
    wifi_ok = ensure_wifi()
    if wifi_ok:
        log.info("Network ready ✓")
    else:
        log.warning("Running without network connectivity")

    # ── 3. Web server ─────────────────────────────────────────────────────────
    app, socketio = create_app(robot, adc, rp)

    host = "0.0.0.0"
    port = int(os.environ.get("LUMIO_PORT", 80))

    log.info("Starting web server on http://%s:%d", host, port)
    log.info("Access at:  http://lumio.local  or  http://<device-ip>")

    try:
        socketio.run(app, host=host, port=port, debug=False)
    except KeyboardInterrupt:
        log.info("Shutdown requested")
    finally:
        log.info("Cleaning up GPIO…")
        robot.cleanup()
        rp.close()
        log.info("Goodbye.")


if __name__ == "__main__":
    main()
