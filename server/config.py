"""Static configuration and shared paths for the ERS backend server.

Importing this module also makes the pluggable alert handlers (``email_alert``,
``sms_alert``, ``audio_alert``, ``relay_alert``) importable by their bare names,
so it must be imported before any of them.
"""

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
LOG_FILE = os.path.join(LOG_DIR, "emergency_log.jsonl")
DB_PATH = os.path.join(DATA_DIR, "ers.sqlite")
