"""Device records: lookup and CRUD."""

from .db import db_connect

__all__ = [
    "get_device_label", "fetch_all_devices", "fetch_known_device_ids",
    "add_device", "update_device", "delete_device",
]


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
        seen = {r[0] for r in conn.execute(
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
