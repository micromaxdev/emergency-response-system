"""Incident queries: listing, latest-with-location, counts and retention purge."""

import json
from datetime import datetime, timedelta

from .config import DRILL_SOURCE
from .db import db_connect

__all__ = [
    "passes_filter", "fetch_incidents", "fetch_latest_incident_with_location",
    "fetch_counts", "count_purgeable", "purge_old_incidents",
]


def passes_filter(incident, type_filter):
    """Return True if an incident matches the dashboard's type filter."""
    if not type_filter or type_filter == "All":
        return True
    return str(incident.get("emergency_type", "")).lower() == str(type_filter).lower()


def fetch_incidents(limit=200):
    conn = db_connect()
    if not conn:
        return []
    try:
        rows = conn.execute(
            "SELECT * FROM incidents ORDER BY id DESC LIMIT ?",
            (limit,)
        ).fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()


def fetch_latest_incident_with_location():
    """Return the most recent incident whose raw_json carries an estimated location."""
    conn = db_connect()
    if not conn:
        return None

    try:
        rows = conn.execute("""
            SELECT id, server_time, device_id, emergency_type, trigger_source,
                   message, triggered_by, raw_json
            FROM incidents
            ORDER BY id DESC
            LIMIT 30
        """).fetchall()

        for row in rows:
            row = dict(row)
            raw_json = row.get("raw_json")
            if not raw_json:
                continue

            try:
                raw = json.loads(raw_json)
            except Exception:
                continue

            loc = raw.get("estimated_location")
            if not loc or "x" not in loc or "y" not in loc:
                continue

            return {
                "id": row.get("id"),
                "server_time": row.get("server_time"),
                "device_id": row.get("device_id"),
                "emergency_type": row.get("emergency_type"),
                "trigger_source": row.get("trigger_source"),
                "message": row.get("message"),
                "triggered_by": row.get("triggered_by"),
                "x": float(loc["x"]),
                "y": float(loc["y"]),
                "estimated_location": loc,
                "raw": raw,
            }

        return None

    except Exception as e:
        print(f"fetch_latest_incident_with_location failed: {e}")
        return None

    finally:
        conn.close()


def fetch_counts():
    conn = db_connect()
    if not conn:
        return {}
    try:
        row = conn.execute(f"""
            SELECT COUNT(*) as total,
            SUM(CASE WHEN emergency_type='mass' AND trigger_source!='{DRILL_SOURCE}' THEN 1 ELSE 0 END) as mass,
            SUM(CASE WHEN emergency_type='personal' AND trigger_source!='{DRILL_SOURCE}' THEN 1 ELSE 0 END) as personal,
            SUM(CASE WHEN trigger_source='{DRILL_SOURCE}' THEN 1 ELSE 0 END) as drills
            FROM incidents
        """).fetchone()
        return dict(row)
    finally:
        conn.close()


def count_purgeable(retention_days):
    conn = db_connect()
    if not conn:
        return 0, ""
    try:
        cutoff = (datetime.now() - timedelta(days=retention_days)).strftime("%Y-%m-%d %H:%M:%S")
        row = conn.execute("SELECT COUNT(*) FROM incidents WHERE server_time < ?", (cutoff,)).fetchone()
        return row[0], cutoff
    finally:
        conn.close()


def purge_old_incidents(retention_days):
    conn = db_connect()
    if not conn:
        return 0, ""
    try:
        cutoff = (datetime.now() - timedelta(days=retention_days)).strftime("%Y-%m-%d %H:%M:%S")
        cur = conn.execute("DELETE FROM incidents WHERE server_time < ?", (cutoff,))
        conn.commit()
        conn.execute("VACUUM")
        conn.commit()
        return cur.rowcount, cutoff
    finally:
        conn.close()
