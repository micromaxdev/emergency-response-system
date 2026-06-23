"""Device online/offline status derived from the heartbeat log."""

import json
import os
from datetime import datetime

from .config import HEARTBEAT_LOG_FILE, OFFLINE_TIMEOUT_S

__all__ = ["get_device_comm_status"]


def get_device_comm_status(device_id: str, timeout_s: int = OFFLINE_TIMEOUT_S):
    """Return ONLINE/OFFLINE/UNKNOWN status for a device from its last heartbeat."""
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
