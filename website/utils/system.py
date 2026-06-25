"""System status, control flags, admin alert recipients and server commands."""

import json
import socket
import sqlite3

from .config import SERVER_HOST, SERVER_PORT, DRILL_SOURCE, DB_PATH
from .db import db_connect

__all__ = [
    "get_system_status", "get_control", "set_control", "alerts_active",
    "get_admin_email_recipients", "get_admin_sms_recipients",
    "get_admin_offline_email_recipients", "get_admin_offline_sms_recipients",
    "send_stop_command", "send_ack_offline_alert",
]

_FAILURE_STATES = {"failed", "queue_full"}


def get_system_status():
    """Summarise server reachability, DB health and recent alert channel health."""
    status = {}
    try:
        with socket.create_connection((SERVER_HOST, SERVER_PORT), timeout=2):
            status["server"] = ("ok", f"Reachable on {SERVER_HOST}:{SERVER_PORT}")
    except Exception:
        status["server"] = ("error", "Offline")

    try:
        conn = db_connect()
        count = conn.execute("SELECT COUNT(*) FROM incidents").fetchone()[0]
        conn.close()
        status["db"] = ("ok", f"{count} total records")
    except Exception:
        status["db"] = ("error", "DB Error")

    try:
        conn = db_connect()
        rows = [dict(r) for r in conn.execute(
            f"SELECT audio_status, email_status, sms_status, relay_status "
            f"FROM incidents WHERE trigger_source != '{DRILL_SOURCE}' ORDER BY id DESC LIMIT 10"
        ).fetchall()]
        conn.close()
        for key in ("audio", "email", "sms", "relay"):
            col = f"{key}_status"
            values = [r[col] for r in rows if r.get(col)]
            if not values:
                status[key] = ("unknown", "No data")
                continue
            fails = sum(1 for v in values if v in _FAILURE_STATES)
            status[key] = ("ok", "All OK") if fails == 0 else ("warn", f"{fails} fails")
    except Exception:
        for key in ("audio", "email", "sms", "relay"):
            status[key] = ("error", "Check logs")

    return status


def get_control(key: str) -> str:
    try:
        conn = db_connect()
        if not conn:
            return "0"
        row = conn.execute("SELECT value FROM system_control WHERE key=?", (key,)).fetchone()
        conn.close()
        return row[0] if row else "0"
    except Exception:
        return "0"


def set_control(key: str, value: str):
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("CREATE TABLE IF NOT EXISTS system_control (key TEXT PRIMARY KEY, value TEXT NOT NULL);")
        conn.execute("INSERT OR REPLACE INTO system_control (key,value) VALUES (?,?)", (key, value))
        conn.commit()
    finally:
        conn.close()


def alerts_active() -> bool:
    """Return True if the relay or audio siren is currently running."""
    return get_control("relay_active") == "1" or get_control("audio_active") == "1"


def _admin_recipients(column: str, field: str):
    """Return staff `field` values where admin flag `column` is set."""
    try:
        conn = db_connect()
        if not conn:
            return []
        rows = conn.execute(
            f"SELECT {field} FROM staff WHERE {column}=1 AND {field}!='' AND {field} IS NOT NULL"
        ).fetchall()
        conn.close()
        return [r[0].strip() for r in rows if r[0] and r[0].strip()]
    except Exception:
        return []


def get_admin_email_recipients(alert_type: str) -> list:
    return _admin_recipients(f"admin_email_{alert_type}", "email")


def get_admin_sms_recipients(alert_type: str) -> str:
    return ",".join(_admin_recipients(f"admin_sms_{alert_type}", "phone"))


def get_admin_offline_email_recipients() -> list:
    return _admin_recipients("admin_email_heartbeat_fail", "email")


def get_admin_offline_sms_recipients() -> str:
    return ",".join(_admin_recipients("admin_sms_heartbeat_fail", "phone"))


def send_stop_command():
    """Clear the relay/audio flags and tell the server to stop sirens immediately."""
    set_control("relay_active", "0")
    set_control("audio_active", "0")
    try:
        with socket.create_connection((SERVER_HOST, SERVER_PORT), timeout=3) as s:
            s.sendall(json.dumps({"command": "stop_alerts"}).encode())
    except Exception:
        pass


def send_ack_offline_alert(device_id: str, ack_by: str = "admin"):
    """Tell the server to stop repeating offline alerts for a device."""
    try:
        with socket.create_connection((SERVER_HOST, SERVER_PORT), timeout=3) as s:
            s.sendall(json.dumps({
                "command": "ack_offline_alert",
                "device_id": device_id,
                "ack_by": ack_by,
            }).encode())
        return True, ""
    except Exception as e:
        return False, str(e)
