"""Device online/offline status.

Primary source: devices.last_seen, which the server updates on EVERY packet
(heartbeat, battery report or alert, LoRa or Wi-Fi). Fallback for devices that
are not registered: the legacy heartbeat_log.jsonl (nothing writes it any more,
so this only matters for old data).
"""

import json
import os
from datetime import datetime

from .config import HEARTBEAT_LOG_FILE, OFFLINE_TIMEOUT_S
from .db import db_connect

__all__ = ["get_device_comm_status", "status_from_last_seen"]


def status_from_last_seen(last_seen, timeout_s: int = OFFLINE_TIMEOUT_S):
    """Build the status dict from a 'YYYY-MM-DD HH:MM:SS' last_seen value."""
    if not last_seen:
        return {"state": "unknown", "label": "UNKNOWN", "last_seen": None,
                "detail": "Not heard from yet"}
    ts = datetime.strptime(last_seen, "%Y-%m-%d %H:%M:%S")
    age = (datetime.now() - ts).total_seconds()
    if age <= timeout_s:
        return {"state": "online", "label": "ONLINE", "last_seen": last_seen,
                "detail": f"Last packet {int(age)}s ago"}
    return {"state": "offline", "label": "OFFLINE", "last_seen": last_seen,
            "detail": f"No packet for {int(age)}s"}


def get_device_comm_status(device_id: str, timeout_s: int = OFFLINE_TIMEOUT_S):
    """Return ONLINE/OFFLINE/UNKNOWN status for a device (DB first, then legacy log)."""
    conn = db_connect()
    if conn:
        try:
            row = conn.execute(
                "SELECT last_seen FROM devices WHERE device_id=?", (device_id,)).fetchone()
            if row and row["last_seen"]:
                return status_from_last_seen(row["last_seen"], timeout_s)
        except Exception:
            pass
        finally:
            conn.close()
    return _status_from_log(device_id, timeout_s)


def _status_from_log(device_id: str, timeout_s: int = OFFLINE_TIMEOUT_S):
    if not os.path.exists(HEARTBEAT_LOG_FILE):
        return {
            "state": "unknown",
            "label": "UNKNOWN",
            "last_seen": None,
            "detail": "Heartbeat log not found",
        }

    latest_ts = None
    try:
        with open(HEARTBEAT_LOG_FILE, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except Exception:
                    continue

                if row.get("type") != "heartbeat_received":
                    continue
                if row.get("device_id") != device_id:
                    continue

                ts_str = row.get("server_time")
                if not ts_str:
                    continue

                try:
                    ts = datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S")
                except Exception:
                    continue

                if latest_ts is None or ts > latest_ts:
                    latest_ts = ts

        if latest_ts is None:
            return {
                "state": "unknown",
                "label": "UNKNOWN",
                "last_seen": None,
                "detail": f"No heartbeat found for {device_id}",
            }

        age = (datetime.now() - latest_ts).total_seconds()
        if age <= timeout_s:
            return {
                "state": "online",
                "label": "ONLINE",
                "last_seen": latest_ts.strftime("%Y-%m-%d %H:%M:%S"),
                "detail": f"Last heartbeat {int(age)}s ago",
            }
        return {
            "state": "offline",
            "label": "OFFLINE",
            "last_seen": latest_ts.strftime("%Y-%m-%d %H:%M:%S"),
            "detail": f"No heartbeat for {int(age)}s",
        }

    except Exception as e:
        return {
            "state": "unknown",
            "label": "UNKNOWN",
            "last_seen": None,
            "detail": str(e),
        }
