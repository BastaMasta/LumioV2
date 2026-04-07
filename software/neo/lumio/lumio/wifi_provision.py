"""
lumio/wifi_provision.py
-----------------------
Boot-time WiFi provisioning flow.

Boot sequence
─────────────
1. Check if a known WiFi network is configured and reachable.
2. If not → start a hotspot (lumio/config.py: HOTSPOT_SSID / HOTSPOT_PASSWORD).
3. Serve a captive-portal page on HOTSPOT_IP:PROVISION_PORT.
4. User connects, enters SSID + password → saved to /etc/lumio/wifi.conf
   and applied via nmcli.
5. Hotspot torn down; robot boots normally.

Requires NetworkManager (nmcli) to be installed (default on Raspberry Pi OS).
"""

import logging
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

from lumio.config import (
    HOTSPOT_SSID, HOTSPOT_PASSWORD, HOTSPOT_IP, PROVISION_PORT
)

log = logging.getLogger(__name__)

WIFI_CONF_PATH = "/etc/lumio/wifi.conf"
HOTSPOT_CON_NAME = "lumio-hotspot"

# ── HTML portal page ──────────────────────────────────────────────────────────

_PORTAL_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lumio WiFi Setup</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box}
  body{font-family:'Segoe UI',sans-serif;min-height:100vh;
       background:linear-gradient(135deg,#1a1a2e,#16213e,#0f3460);
       display:flex;align-items:center;justify-content:center}
  .card{background:#fff;border-radius:16px;padding:36px;
        box-shadow:0 20px 60px rgba(0,0,0,.4);max-width:400px;width:90%}
  h1{text-align:center;margin-bottom:8px;color:#0f3460;font-size:1.8em}
  p.sub{text-align:center;color:#666;margin-bottom:24px;font-size:.95em}
  label{display:block;margin-bottom:6px;color:#333;font-weight:600;font-size:.9em}
  input{width:100%;padding:12px 14px;border:2px solid #ddd;border-radius:8px;
        font-size:1em;margin-bottom:18px;transition:border .2s}
  input:focus{outline:none;border-color:#0f3460}
  button{width:100%;padding:14px;background:#0f3460;color:#fff;
         border:none;border-radius:8px;font-size:1.05em;cursor:pointer;
         transition:background .2s}
  button:hover{background:#16213e}
  .msg{margin-top:16px;padding:12px;border-radius:8px;text-align:center;
       font-size:.9em;display:none}
  .msg.ok{background:#d1fae5;color:#065f46}
  .msg.err{background:#fee2e2;color:#991b1b}
  .logo{text-align:center;margin-bottom:20px;font-size:2.5em}
</style>
</head>
<body>
<div class="card">
  <div class="logo">🤖</div>
  <h1>Lumio Setup</h1>
  <p class="sub">Connect Lumio to your WiFi network</p>
  <form method="POST" action="/connect">
    <label>WiFi Network (SSID)</label>
    <input type="text" name="ssid" placeholder="Your network name" required>
    <label>Password</label>
    <input type="password" name="password" placeholder="WiFi password">
    <button type="submit">Connect →</button>
  </form>
  {message}
</div>
</body>
</html>"""

_SUCCESS_HTML = """<!DOCTYPE html>
<html lang="en">
<head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lumio – Connected!</title>
<style>
  body{font-family:'Segoe UI',sans-serif;min-height:100vh;
       background:linear-gradient(135deg,#1a1a2e,#16213e,#0f3460);
       display:flex;align-items:center;justify-content:center;color:#fff;text-align:center}
  .card{background:rgba(255,255,255,.1);border-radius:16px;padding:40px;
        backdrop-filter:blur(12px);max-width:380px}
  h1{font-size:2em;margin-bottom:12px}
  p{color:#cce;font-size:1em;line-height:1.6}
</style></head>
<body>
<div class="card">
  <div style="font-size:3em;margin-bottom:16px">✅</div>
  <h1>Connecting…</h1>
  <p>Lumio is connecting to <strong>{ssid}</strong>.<br>
     This hotspot will close shortly.<br>
     Reconnect to your regular WiFi and find Lumio at<br>
     <strong>http://lumio.local</strong></p>
</div>
</body></html>"""


# ── HTTP handler ──────────────────────────────────────────────────────────────

class _PortalHandler(BaseHTTPRequestHandler):
    """Minimal HTTP server handling the WiFi credential submission."""

    credentials_received: dict | None = None   # class-level flag

    def log_message(self, fmt, *args):
        log.debug("Portal: " + fmt, *args)

    def _send(self, code: int, body: str):
        data = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        # Redirect any path to the root portal (captive-portal behaviour)
        parsed = urlparse(self.path)
        if parsed.path != "/":
            self.send_response(302)
            self.send_header("Location", "/")
            self.end_headers()
            return
        self._send(200, _PORTAL_HTML.format(message=""))

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode(errors="replace")
        params = parse_qs(body)
        ssid = params.get("ssid", [""])[0].strip()
        password = params.get("password", [""])[0]

        if not ssid:
            self._send(200, _PORTAL_HTML.format(
                message='<div class="msg err" style="display:block">SSID cannot be empty.</div>'
            ))
            return

        # Store credentials for the provisioner to act on
        _PortalHandler.credentials_received = {"ssid": ssid, "password": password}
        self._send(200, _SUCCESS_HTML.format(ssid=ssid))


# ── nmcli helpers ─────────────────────────────────────────────────────────────

def _run(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True)


def _wifi_connected() -> bool:
    """True if NetworkManager reports a WiFi connection is active."""
    r = _run(["nmcli", "-t", "-f", "TYPE,STATE", "con", "show", "--active"])
    for line in r.stdout.splitlines():
        kind, _, state = line.partition(":")
        if "wifi" in kind and state.strip() == "activated":
            return True
    return False


def _start_hotspot():
    log.info("Starting hotspot: SSID=%s", HOTSPOT_SSID)
    # Remove stale connection if present
    _run(["nmcli", "con", "delete", HOTSPOT_CON_NAME])
    r = _run([
        "nmcli", "con", "add",
        "type", "wifi",
        "ifname", "wlan0",
        "con-name", HOTSPOT_CON_NAME,
        "autoconnect", "no",
        "ssid", HOTSPOT_SSID,
        "--",
        "802-11-wireless.mode", "ap",
        "802-11-wireless.band", "bg",
        "ipv4.method", "shared",
        "ipv4.addresses", f"{HOTSPOT_IP}/24",
        "wifi-sec.key-mgmt", "wpa-psk",
        "wifi-sec.psk", HOTSPOT_PASSWORD,
    ])
    if r.returncode != 0:
        log.error("Hotspot create failed: %s", r.stderr)
        return False
    r = _run(["nmcli", "con", "up", HOTSPOT_CON_NAME])
    if r.returncode != 0:
        log.error("Hotspot up failed: %s", r.stderr)
        return False
    log.info("Hotspot active at %s", HOTSPOT_IP)
    return True


def _stop_hotspot():
    log.info("Tearing down hotspot")
    _run(["nmcli", "con", "down", HOTSPOT_CON_NAME])
    _run(["nmcli", "con", "delete", HOTSPOT_CON_NAME])


def _apply_wifi(ssid: str, password: str) -> bool:
    log.info("Applying WiFi credentials for SSID: %s", ssid)
    # Save to disk so it survives reboots
    os.makedirs(os.path.dirname(WIFI_CONF_PATH), exist_ok=True)
    with open(WIFI_CONF_PATH, "w") as f:
        f.write(f"SSID={ssid}\n")
        f.write(f"PASSWORD={password}\n")

    # Connect via nmcli (creates or updates the connection profile)
    r = _run([
        "nmcli", "device", "wifi", "connect", ssid,
        "password", password,
        "ifname", "wlan0",
    ])
    if r.returncode != 0:
        log.error("WiFi connect failed: %s", r.stderr)
        return False
    log.info("WiFi connected: %s", ssid)
    return True


# ── Public entry point ────────────────────────────────────────────────────────

def ensure_wifi() -> bool:
    """
    Called at boot.  Returns True once a WiFi connection is established.
    Blocks until connected (either from existing config or provisioning).
    """

    # ── 1. Already connected? ─────────────────────────────────────────────
    if _wifi_connected():
        log.info("WiFi already connected – skipping provisioning")
        return True

    # ── 2. Saved credentials? ─────────────────────────────────────────────
    if os.path.exists(WIFI_CONF_PATH):
        log.info("Found saved WiFi config, attempting reconnect")
        conf: dict[str, str] = {}
        with open(WIFI_CONF_PATH) as f:
            for line in f:
                k, _, v = line.partition("=")
                conf[k.strip()] = v.strip()
        if _apply_wifi(conf.get("SSID", ""), conf.get("PASSWORD", "")):
            time.sleep(5)
            if _wifi_connected():
                return True
            log.warning("Saved credentials failed – falling through to hotspot")

    # ── 3. Start hotspot + captive portal ─────────────────────────────────
    if not _start_hotspot():
        log.error("Could not start hotspot; running without WiFi")
        return False

    _PortalHandler.credentials_received = None
    server = HTTPServer(("0.0.0.0", PROVISION_PORT), _PortalHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    log.info(
        "Captive portal live at http://%s:%d  (SSID: %s  Pass: %s)",
        HOTSPOT_IP, PROVISION_PORT, HOTSPOT_SSID, HOTSPOT_PASSWORD,
    )

    # ── 4. Wait for credentials ───────────────────────────────────────────
    while _PortalHandler.credentials_received is None:
        time.sleep(0.5)

    creds = _PortalHandler.credentials_received
    server.shutdown()

    # ── 5. Apply credentials & tear down hotspot ──────────────────────────
    _stop_hotspot()
    time.sleep(2)  # Let NM settle
    ok = _apply_wifi(creds["ssid"], creds["password"])
    if ok:
        time.sleep(8)  # Give NM time to obtain DHCP lease
        if _wifi_connected():
            log.info("WiFi provisioning complete ✓")
            return True

    log.error("WiFi provisioning failed; network unavailable")
    return False
