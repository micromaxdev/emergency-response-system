"""Map scale, gateway coordinates and room zones (experimental localisation)."""

import os
import sqlite3

from .config import DB_PATH
from .db import db_connect
from .system import get_control, set_control

__all__ = [
    "init_map_table", "save_gw_coords", "delete_gw", "fetch_all_gw_coords",
    "save_map_scale", "fetch_map_scale",
    "init_room_zones_table", "fetch_room_zones", "add_room_zone",
    "delete_room_zone", "lookup_room_by_xy",
]

_DEFAULT_GATEWAYS = ["GW1", "GW2", "GW3"]


# -- Map & gateways ------------------------------------------------------------
def init_map_table():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("CREATE TABLE IF NOT EXISTS map_config (key TEXT PRIMARY KEY, val_x REAL, val_y REAL)")
        count = conn.execute("SELECT COUNT(*) FROM map_config").fetchone()[0]
        if count == 0:
            for gw in _DEFAULT_GATEWAYS:
                conn.execute("INSERT OR IGNORE INTO map_config (key, val_x, val_y) VALUES (?, 0.0, 0.0)", (gw,))
        conn.commit()
    finally:
        conn.close()


def save_gw_coords(gw_id, x, y):
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("INSERT OR REPLACE INTO map_config (key, val_x, val_y) VALUES (?, ?, ?)", (gw_id, x, y))
        conn.commit()
    finally:
        conn.close()


def delete_gw(gw_id):
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("DELETE FROM map_config WHERE key = ?", (gw_id,))
        conn.commit()
    finally:
        conn.close()


def fetch_all_gw_coords():
    if not os.path.exists(DB_PATH):
        return {gw: {"x": 0.0, "y": 0.0} for gw in _DEFAULT_GATEWAYS}
    conn = db_connect()
    try:
        rows = conn.execute("SELECT * FROM map_config").fetchall()
        if not rows:
            return {}
        return {row["key"]: {"x": row["val_x"], "y": row["val_y"]} for row in rows}
    except Exception:
        return {}
    finally:
        if conn:
            conn.close()


def save_map_scale(meters_wide):
    set_control("map_meters_wide", str(meters_wide))


def fetch_map_scale():
    val = get_control("map_meters_wide")
    return float(val) if val != "0" else 50.0


# -- Room zones ----------------------------------------------------------------
def init_room_zones_table():
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("""
        CREATE TABLE IF NOT EXISTS room_zones (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            room_name TEXT NOT NULL,
            x_min REAL NOT NULL,
            y_min REAL NOT NULL,
            x_max REAL NOT NULL,
            y_max REAL NOT NULL,
            created_at TEXT DEFAULT (datetime('now'))
        );
        """)
        conn.commit()
    finally:
        conn.close()


def fetch_room_zones():
    conn = db_connect()
    if not conn:
        return []
    try:
        rows = conn.execute("""
            SELECT id, room_name, x_min, y_min, x_max, y_max
            FROM room_zones
            ORDER BY id DESC
        """).fetchall()
        return [dict(r) for r in rows]
    except Exception:
        return []
    finally:
        conn.close()


def add_room_zone(room_name, x_min, y_min, x_max, y_max):
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("""
            INSERT INTO room_zones (room_name, x_min, y_min, x_max, y_max)
            VALUES (?, ?, ?, ?, ?)
        """, (room_name, float(x_min), float(y_min), float(x_max), float(y_max)))
        conn.commit()
    finally:
        conn.close()


def delete_room_zone(zone_id):
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("DELETE FROM room_zones WHERE id=?", (int(zone_id),))
        conn.commit()
    finally:
        conn.close()


def lookup_room_by_xy(x, y):
    """Return the room name whose zone contains (x, y), or None."""
    conn = db_connect()
    if not conn:
        return None
    try:
        rows = conn.execute("""
            SELECT room_name, x_min, y_min, x_max, y_max
            FROM room_zones
        """).fetchall()

        for row in rows:
            x_min = min(float(row["x_min"]), float(row["x_max"]))
            x_max = max(float(row["x_min"]), float(row["x_max"]))
            y_min = min(float(row["y_min"]), float(row["y_max"]))
            y_max = max(float(row["y_min"]), float(row["y_max"]))

            if x_min <= float(x) <= x_max and y_min <= float(y) <= y_max:
                return row["room_name"]

        return None
    except Exception:
        return None
    finally:
        conn.close()
