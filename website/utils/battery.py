"""Client device battery readings."""

from .db import db_connect

__all__ = ["fetch_all_device_batteries"]


def fetch_all_device_batteries():
    conn = db_connect()
    if not conn:
        return []
    try:
        rows = conn.execute("""
            SELECT b.device_id,
                   b.battery_pct,
                   b.battery_v,
                   b.updated_at,
                   COALESCE(d.label, b.device_id) AS label,
                   s.name AS staff_name
            FROM device_battery b
            LEFT JOIN devices d ON d.device_id = b.device_id
            LEFT JOIN staff  s ON s.id = d.assigned_staff_id
            ORDER BY b.device_id ASC
        """).fetchall()
        return [dict(r) for r in rows]
    except Exception:
        return []
    finally:
        conn.close()
