"""Incident queries: listing, latest-with-location, counts and retention purge."""

import json
from datetime import datetime, timedelta

from .config import DRILL_SOURCE
from .db import db_connect

__all__ = [
    "passes_filter", "fetch_incidents", "fetch_latest_incident_with_location",
    "fetch_counts", "build_log_filters", "count_logs", "fetch_logs",
    "fetch_log_export", "count_purgeable", "purge_old_incidents",
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


def build_log_filters(search="", em_types=None, date_from=None, date_to=None, alert_filter=None):
    """Return SQL WHERE text and params for the admin incident log filters."""
    conditions, params = [], []

    if search:
        conditions.append("""
            (triggered_by LIKE ? OR message LIKE ? OR device_id LIKE ?
             OR emergency_type LIKE ? OR from_ip LIKE ?)
        """)
        s = f"%{search}%"
        params += [s, s, s, s, s]

    if em_types is not None:
        if not em_types:
            conditions.append("1=0")
        else:
            clauses = []
            if "test_drill" in em_types:
                clauses.append("trigger_source = ?")
                params.append(DRILL_SOURCE)
            real = [t for t in em_types if t != "test_drill"]
            if real:
                ph = ",".join("?" * len(real))
                clauses.append(f"(emergency_type IN ({ph}) AND trigger_source != ?)")
                params += real + [DRILL_SOURCE]
            if clauses:
                conditions.append("(" + " OR ".join(clauses) + ")")

    if date_from:
        conditions.append("server_time >= ?")
        params.append(date_from.strftime("%Y-%m-%d 00:00:00"))
    if date_to:
        conditions.append("server_time <= ?")
        params.append(date_to.strftime("%Y-%m-%d 23:59:59"))

    if alert_filter == "email_failed":
        conditions.append("email_status = 'failed'")
    elif alert_filter == "sms_failed":
        conditions.append("sms_status = 'failed'")
    elif alert_filter == "audio_failed":
        conditions.append("audio_status = 'failed'")
    elif alert_filter == "relay_failed":
        conditions.append("relay_status IN ('failed','queue_full')")
    elif alert_filter == "any_failed":
        conditions.append("""
            (email_status = 'failed' OR sms_status = 'failed'
             OR audio_status = 'failed' OR relay_status IN ('failed','queue_full'))
        """)

    where = ("WHERE " + " AND ".join(conditions)) if conditions else ""
    return where, params


def count_logs(search="", em_types=None, date_from=None, date_to=None, alert_filter=None):
    """Count incidents matching the admin log filters."""
    conn = db_connect()
    if not conn:
        return 0
    try:
        where, params = build_log_filters(search, em_types, date_from, date_to, alert_filter)
        row = conn.execute(f"SELECT COUNT(*) FROM incidents {where}", params).fetchone()
        return row[0]
    finally:
        conn.close()


def fetch_logs(search="", em_types=None, date_from=None, date_to=None,
               alert_filter=None, limit=50, offset=0):
    """Fetch a page of incidents matching the admin log filters."""
    conn = db_connect()
    if not conn:
        return []
    try:
        where, params = build_log_filters(search, em_types, date_from, date_to, alert_filter)
        params = params + [limit, offset]
        rows = conn.execute(f"""
            SELECT id, server_time, device_id, triggered_by, emergency_type,
                   trigger_source, message,
                   audio_status, email_status, sms_status,
                   relay_status, relay_error, email_error, sms_error, audio_error
            FROM incidents
            {where}
            ORDER BY id DESC
            LIMIT ? OFFSET ?
        """, params).fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()


def fetch_log_export(days: int):
    """Fetch incidents from the last N days for CSV export."""
    conn = db_connect()
    if not conn:
        return []
    try:
        cutoff = (datetime.now() - timedelta(days=days)).strftime("%Y-%m-%d %H:%M:%S")
        rows = conn.execute("""
            SELECT id, server_time, emergency_type, trigger_source,
                   triggered_by, device_id, message,
                   audio_status, email_status, sms_status, relay_status,
                   from_ip, audio_error, email_error, sms_error, relay_error
            FROM incidents
            WHERE server_time >= ?
            ORDER BY id DESC
        """, (cutoff,)).fetchall()
        return [dict(r) for r in rows]
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
