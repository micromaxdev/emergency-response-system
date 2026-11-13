"""Device records: lookup and CRUD."""

import csv
import io
import re
from datetime import datetime, timedelta

from .db import db_connect
from .heartbeat import status_from_last_seen

__all__ = [
    "get_device_label", "fetch_all_devices", "fetch_known_device_ids",
    "add_device", "update_device", "delete_device",
    # registration / commissioning
    "import_device_list", "fetch_checklist", "activate_device", "set_device_status",
    "start_test_window", "fetch_devices_in_test", "fetch_unregistered_incidents",
    "pick_target_device", "device_comm_state",
    "REQUIRED_CHECKS", "OPTIONAL_CHECKS", "TEST_WINDOW_MIN",
]

UNREGISTERED_PREFIX = "UNREGISTERED-"
DEVICE_ID_RE = re.compile(r"^ERS-[0-9A-F]{8}$")
REQUIRED_CHECKS = ("heartbeat", "personal", "mass", "wearer")
OPTIONAL_CHECKS = ("wifi",)
TEST_WINDOW_MIN = 15  # keep equal to server/config.py TEST_WINDOW_MIN


def get_device_label(device_id: str):
    conn = db_connect()
    if not conn:
        return None
    try:
        row = conn.execute("SELECT label FROM devices WHERE device_id = ?", (device_id,)).fetchone()
        return row[0] if row and row[0] else None
    finally:
        conn.close()


def fetch_all_devices():
    conn = db_connect()
    if not conn:
        return []
    try:
        return [dict(r) for r in conn.execute("""
            SELECT d.*, s.name as staff_name, s.role as staff_role FROM devices d
            LEFT JOIN staff s ON d.assigned_staff_id = s.id ORDER BY d.device_id ASC
        """).fetchall()]
    finally:
        conn.close()


def fetch_known_device_ids():
    """Return device ids seen in incidents but not yet registered in devices."""
    conn = db_connect()
    if not conn:
        return []
    try:
        registered = {r["device_id"] for r in conn.execute("SELECT device_id FROM devices").fetchall()}
        seen = {r[0][len(UNREGISTERED_PREFIX):] if r[0].startswith(UNREGISTERED_PREFIX) else r[0]
                for r in conn.execute(
                    "SELECT DISTINCT device_id FROM incidents WHERE device_id IS NOT NULL").fetchall()}
        return sorted(seen - registered)
    finally:
        conn.close()


def add_device(device_id, label, location, assigned_staff_id=None):
    conn = db_connect()
    try:
        conn.execute(
            "INSERT OR IGNORE INTO devices (device_id, label, location, assigned_staff_id) "
            "VALUES (?,?,?,?)",
            (device_id, label, location, assigned_staff_id or None))
        conn.commit()
    finally:
        conn.close()


def update_device(device_db_id, label, location, assigned_staff_id):
    conn = db_connect()
    try:
        conn.execute(
            "UPDATE devices SET label=?, location=?, assigned_staff_id=? WHERE id=?",
            (label, location, assigned_staff_id or None, device_db_id))
        conn.commit()
    finally:
        conn.close()


def delete_device(device_db_id):
    conn = db_connect()
    try:
        conn.execute("DELETE FROM devices WHERE id=?", (device_db_id,))
        conn.commit()
    finally:
        conn.close()


# -- Registration / commissioning ----------------------------------------------
def import_device_list(csv_text: str, server_site_id=None):
    """Import a provisioning CSV (from tools/provision_pico.py).

    New devices start as 'commissioning'. Returns (added, skipped, errors).
    Rows for a different site, malformed ids and duplicates are reported, not imported.
    """
    added, skipped, errors = 0, 0, []
    conn = db_connect()
    if not conn:
        return 0, 0, ["Database not found"]
    try:
        for n, row in enumerate(csv.DictReader(io.StringIO(csv_text)), start=2):
            dev_id = (row.get("device_id") or "").strip().upper()
            if not DEVICE_ID_RE.match(dev_id):
                errors.append(f"line {n}: bad device_id {row.get('device_id')!r}")
                continue
            try:
                site = int(row["site_id"]) if row.get("site_id") else None
            except ValueError:
                errors.append(f"line {n}: bad site_id")
                continue
            if server_site_id is not None and site is not None and site != server_site_id:
                errors.append(f"line {n}: {dev_id} belongs to site {site}, this server is site {server_site_id}")
                continue
            cur = conn.execute(
                "INSERT OR IGNORE INTO devices (device_id, label, hw_serial, site_id, fw_version, status) "
                "VALUES (?,?,?,?,?, 'commissioning')",
                (dev_id, (row.get("label") or "").strip(), (row.get("hw_serial") or "").strip(),
                 site, (row.get("fw_version") or "").strip()))
            if cur.rowcount:
                added += 1
            else:
                skipped += 1
        conn.commit()
    finally:
        conn.close()
    return added, skipped, errors


def fetch_checklist(device):
    """Return {check: bool} for a device row (dict). 'wearer' is derived live."""
    conn = db_connect()
    result = {c: False for c in REQUIRED_CHECKS + OPTIONAL_CHECKS}
    if conn:
        try:
            for r in conn.execute("SELECT check_name FROM commissioning_checks WHERE device_id=?",
                                  (device["device_id"],)).fetchall():
                if r[0] in result:
                    result[r[0]] = True
        finally:
            conn.close()
    result["wearer"] = bool(device.get("assigned_staff_id"))
    return result


def set_device_status(device_db_id, status):
    if status not in ("imported", "commissioning", "active", "retired"):
        raise ValueError(status)
    conn = db_connect()
    try:
        conn.execute("UPDATE devices SET status=?, test_until=NULL WHERE id=?", (status, device_db_id))
        conn.commit()
    finally:
        conn.close()


def activate_device(device):
    """Activate a commissioning device if its required checklist is complete.

    Returns (ok, missing_checks).
    """
    checks = fetch_checklist(device)
    missing = [c for c in REQUIRED_CHECKS if not checks[c]]
    if missing:
        return False, missing
    set_device_status(device["id"], "active")
    return True, []


def start_test_window(device_db_id, minutes=TEST_WINDOW_MIN):
    """Put a live device under test; the server auto-reverts it when the time is up."""
    until = (datetime.now() + timedelta(minutes=minutes)).strftime("%Y-%m-%d %H:%M:%S")
    conn = db_connect()
    try:
        conn.execute("UPDATE devices SET test_until=? WHERE id=? AND status='active'",
                     (until, device_db_id))
        conn.commit()
    finally:
        conn.close()
    return until


def fetch_devices_in_test():
    """Devices currently commissioning or inside an open test window."""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn = db_connect()
    if not conn:
        return []
    try:
        return [dict(r) for r in conn.execute(
            "SELECT * FROM devices WHERE status='commissioning' "
            "OR (status='active' AND test_until IS NOT NULL AND test_until > ?)", (now,)).fetchall()]
    finally:
        conn.close()


def fetch_unregistered_incidents(hours=24):
    """Recent incidents from devices the server did not recognise."""
    since = (datetime.now() - timedelta(hours=hours)).strftime("%Y-%m-%d %H:%M:%S")
    conn = db_connect()
    if not conn:
        return []
    try:
        return [dict(r) for r in conn.execute(
            "SELECT id, server_time, device_id, emergency_type, message FROM incidents "
            "WHERE device_id LIKE ? AND server_time >= ? ORDER BY id DESC",
            (UNREGISTERED_PREFIX + "%", since)).fetchall()]
    finally:
        conn.close()


def pick_target_device(preferred="PICO_TEST_01"):
    """Device shown in the single-device comms panel on the Dashboard.

    Keeps the old default when that device is registered; otherwise the most
    recently heard active device, so the panel follows the real fleet.
    """
    conn = db_connect()
    if not conn:
        return preferred
    try:
        if conn.execute("SELECT 1 FROM devices WHERE device_id=?", (preferred,)).fetchone():
            return preferred
        row = conn.execute("SELECT device_id FROM devices WHERE status='active' "
                           "ORDER BY last_seen DESC LIMIT 1").fetchone()
        return row[0] if row else preferred
    finally:
        conn.close()


def device_comm_state(device):
    """ONLINE/OFFLINE/UNKNOWN dict for a devices row (uses its last_seen)."""
    return status_from_last_seen(device.get("last_seen"))
