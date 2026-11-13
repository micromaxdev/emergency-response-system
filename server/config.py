"""Static configuration and shared paths for the ERS backend server.

Importing this module also makes the pluggable alert handlers (``email_alert``,
``sms_alert``, ``audio_alert``, ``relay_alert``) importable by their bare names,
so it must be imported before any of them.
"""

import json
import os
import sys

SERVER_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(SERVER_DIR)
ALERT_HANDLERS_DIR = os.path.join(SERVER_DIR, "alert_handlers")

LOG_DIR = os.path.join(BASE_DIR, "logs")
DATA_DIR = os.path.join(BASE_DIR, "data")

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(DATA_DIR, exist_ok=True)

if ALERT_HANDLERS_DIR not in sys.path:
    sys.path.append(ALERT_HANDLERS_DIR)

# Network: listen on all interfaces for incoming LoRa gateway / Pico payloads.
HOST = "0.0.0.0"
PORT = 8081

# Ignore repeat triggers with the same device/type/source within this window.
COOLDOWN_SEC = 5

# Filesystem targets.
# ERS_LOG_FILE / ERS_DB_PATH let tests (and unusual deployments) redirect storage.
LOG_FILE = os.environ.get("ERS_LOG_FILE") or os.path.join(LOG_DIR, "emergency_log.jsonl")
DB_PATH = os.environ.get("ERS_DB_PATH") or os.path.join(DATA_DIR, "ers.sqlite")

# -- Site identity (device registration) ---------------------------------------
# ers_setup.py writes /etc/ers/site.json on a customer Pi. For development a
# data/site.json is honoured too. With no site file, site filtering is disabled
# so an un-provisioned dev box behaves exactly as it did before registration.
SITE_FILE_CANDIDATES = [
    os.environ.get("ERS_SITE_FILE", ""),
    "/etc/ers/site.json",
    os.path.join(DATA_DIR, "site.json"),
]

# A commissioned device that is put back "under test" reverts after this long.
TEST_WINDOW_MIN = 15


def load_site() -> dict:
    """Return the site config dict ({} if none is provisioned)."""
    for path in SITE_FILE_CANDIDATES:
        if path and os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except (OSError, ValueError):
                return {}
    return {}


def get_site_id():
    """Return this server's 16-bit site id, or None when no site is provisioned."""
    sid = load_site().get("site_id")
    try:
        return int(sid) if sid is not None else None
    except (TypeError, ValueError):
        return None
