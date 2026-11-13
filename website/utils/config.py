"""Shared paths, constants and the page auto-refresh helper."""

import json
import os

# websites/utils/config.py -> websites/utils -> websites -> <repo root>
PKG_DIR = os.path.dirname(os.path.abspath(__file__))
WEBSITES_DIR = os.path.dirname(PKG_DIR)
ROOT_DIR = os.path.dirname(WEBSITES_DIR)

DB_PATH = os.path.join(ROOT_DIR, "data", "ers.sqlite")
HEARTBEAT_LOG_FILE = os.path.join(ROOT_DIR, "logs", "heartbeat_log.jsonl")
OFFLINE_TIMEOUT_S = 90

MAP_DIR = os.path.join(ROOT_DIR, "assets", "maps")
AUDIO_DIR = os.path.join(ROOT_DIR, "assets", "audios")

os.makedirs(MAP_DIR, exist_ok=True)
os.makedirs(AUDIO_DIR, exist_ok=True)

# Backend server (same host) the dashboard talks to over TCP.
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8081

# Identifiers used to tag dashboard-initiated test drills.
DRILL_DEVICE = "DASHBOARD_DRILL"
DRILL_SOURCE = "test_drill"

# Site identity written by server/setup/ers_setup.py (None when not provisioned).
SITE_FILE_CANDIDATES = [
    os.environ.get("ERS_SITE_FILE", ""),
    "/etc/ers/site.json",
    os.path.join(ROOT_DIR, "data", "site.json"),
]


def get_site_id():
    """Return this site's id (int) or None if no site file exists."""
    for path in SITE_FILE_CANDIDATES:
        if path and os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return int(json.load(f)["site_id"])
            except (OSError, ValueError, KeyError):
                return None
    return None


REFRESH_SEC = 5
RETENTION_DAYS = 30

__all__ = [
    "ROOT_DIR", "DB_PATH", "HEARTBEAT_LOG_FILE", "OFFLINE_TIMEOUT_S",
    "MAP_DIR", "AUDIO_DIR", "SERVER_HOST", "SERVER_PORT",
    "DRILL_DEVICE", "DRILL_SOURCE", "REFRESH_SEC", "RETENTION_DAYS",
    "autorefresh", "get_site_id",
]


def autorefresh():
    """Enable auto-refresh on a page; call once after set_page_config."""
    from streamlit_autorefresh import st_autorefresh
    st_autorefresh(interval=REFRESH_SEC * 1000, key="ers_autorefresh")
