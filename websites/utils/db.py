"""Database connection and staff/devices schema setup."""

import os
import sqlite3

from .config import DB_PATH

__all__ = ["db_connect", "init_extra_tables"]


def db_connect():
    """Return a Row-factory connection, or None if the database is missing."""
    if not os.path.exists(DB_PATH):
        return None
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def init_extra_tables():
    """Create/migrate the staff and devices tables used by the dashboard."""
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("""
        CREATE TABLE IF NOT EXISTS staff (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL, role TEXT, email TEXT, phone TEXT,
            email_alerts_mass INTEGER DEFAULT 0, email_alerts_personal INTEGER DEFAULT 0,
            sms_alerts_mass INTEGER DEFAULT 0, sms_alerts_personal INTEGER DEFAULT 0,
            created_at TEXT DEFAULT (datetime('now'))
        );""")
        conn.execute("""
        CREATE TABLE IF NOT EXISTS devices (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id TEXT NOT NULL UNIQUE, label TEXT, location TEXT,
            assigned_staff_id INTEGER, created_at TEXT DEFAULT (datetime('now')),
            FOREIGN KEY (assigned_staff_id) REFERENCES staff(id)
        );""")
        for col in (
            "admin_email_low_battery",
            "admin_sms_low_battery",
            "admin_email_ups",
            "admin_sms_ups",
            "admin_email_heartbeat_fail",
            "admin_sms_heartbeat_fail",
        ):
            try:
                conn.execute(f"ALTER TABLE staff ADD COLUMN {col} INTEGER DEFAULT 0")
            except sqlite3.OperationalError:
                pass
        conn.commit()
    finally:
        conn.close()
