"""
ers_shared.py — shared config, CSS, and DB helpers for every ERS page.
Import this at the top of each page file.
"""

import sqlite3
import socket
import os
from datetime import datetime, timedelta

# ════════════════════════════════════════════════════════════════════════════════
#  CONFIG — edit here, applies everywhere
# ════════════════════════════════════════════════════════════════════════════════
DB_PATH        = os.path.expanduser("~/ers_server/ers.sqlite")
SERVER_HOST    = "127.0.0.1"
SERVER_PORT    = 8080
DRILL_DEVICE   = "DASHBOARD_DRILL"
DRILL_SOURCE   = "test_drill"
REFRESH_SEC    = 5
RETENTION_DAYS = 30   # ← change this to adjust how many days of alerts are kept

def autorefresh():
    """Call once per page after set_page_config to enable auto-refresh."""
    from streamlit_autorefresh import st_autorefresh
    st_autorefresh(interval=REFRESH_SEC * 1000, key="ers_autorefresh")

# ════════════════════════════════════════════════════════════════════════════════
#  SHARED CSS
# ════════════════════════════════════════════════════════════════════════════════
ERS_CSS = """
<style>
@import url('https://fonts.googleapis.com/css2?family=Bebas+Neue&family=IBM+Plex+Mono:wght@400;600&family=IBM+Plex+Sans:wght@300;400;600&display=swap');

:root {
    --bg:      #ffffff;
    --surface: #ffffff;
    --border:  #1e2530;
    --accent:  #4065a1;
    --warn:    #f4a261;
    --ok:      #2ec4b6;
    --drill:   #f7b731;
    --text:    #111111;
    --muted:   #586069;
}
html, body, [data-testid="stAppViewContainer"] {
    background: var(--bg) !important;
    color: var(--text) !important;
    font-family: 'IBM Plex Sans', sans-serif !important;
}
[data-testid="stSidebar"] {
    background: var(--surface) !important;
    border-right: 1px solid var(--border) !important;
}
.ers-header {
    display: flex; align-items: baseline; gap: 18px;
    padding: 10px 0 24px 0;
    border-bottom: 1px solid var(--border);
    margin-bottom: 28px;
}
.ers-logo {
    font-family: 'Bebas Neue', sans-serif;
    font-size: 3rem; letter-spacing: 6px;
    color: var(--accent); line-height: 1;
}
.ers-sub {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.7rem; color: var(--muted);
    letter-spacing: 3px; text-transform: uppercase; padding-bottom: 4px;
}
.kpi-row { display: flex; gap: 16px; margin-bottom: 28px; flex-wrap: wrap; }
.kpi-card {
    flex: 1; min-width: 120px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 6px; padding: 16px 20px;
    position: relative; overflow: hidden;
}
.kpi-card::before {
    content: ''; position: absolute;
    left: 0; top: 0; bottom: 0; width: 3px;
}
.kpi-card.red::before   { background: var(--accent); }
.kpi-card.amber::before { background: var(--warn); }
.kpi-card.drill::before { background: var(--drill); }
.kpi-card.gray::before  { background: #30363d; }
.kpi-num {
    font-family: 'Bebas Neue', sans-serif;
    font-size: 2.4rem; line-height: 1; color: #111;
}
.kpi-label {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.65rem; color: var(--muted);
    letter-spacing: 2px; text-transform: uppercase; margin-top: 4px;
}
.status-grid { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 24px; }
.status-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 6px; padding: 14px 18px; min-width: 150px; flex: 1;
}
.status-card-label {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.6rem; color: var(--muted);
    letter-spacing: 2px; text-transform: uppercase; margin-bottom: 8px;
}
.status-card-detail {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.65rem; color: var(--muted); margin-top: 6px;
}
.tbl-header {
    display: flex; gap: 10px; padding: 8px 14px;
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.6rem; letter-spacing: 2px;
    color: var(--muted); text-transform: uppercase;
    border-bottom: 1px solid var(--border); margin-bottom: 4px;
}
.inc-row {
    display: flex; align-items: center; gap: 10px;
    padding: 11px 14px;
    border-bottom: 1px solid var(--border);
    border-radius: 4px; transition: background 0.15s;
    font-size: 0.82rem;
}
.inc-row:hover { background: #f5f7fa; }
.inc-row.is-drill { border-left: 3px solid var(--drill); }
.inc-row.is-mass  { border-left: 3px solid var(--accent); }
.badge {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.62rem; font-weight: 600;
    letter-spacing: 1px; padding: 3px 8px;
    border-radius: 3px; text-transform: uppercase; white-space: nowrap;
}
.badge-mass     { background: rgba(230,57,70,0.18);  color: #e63946; border: 1px solid rgba(230,57,70,0.4); }
.badge-personal { background: rgba(244,162,97,0.15); color: #f4a261; border: 1px solid rgba(244,162,97,0.4); }
.badge-drill    { background: rgba(247,183,49,0.15); color: #f7b731; border: 1px solid rgba(247,183,49,0.4); }
.pill {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.58rem; padding: 2px 6px;
    border-radius: 3px; white-space: nowrap; display: inline-block;
}
.pill-ok      { background: rgba(46,196,182,0.12);  color: #2ec4b6; }
.pill-warn    { background: rgba(247,183,49,0.12);  color: #f7b731; }
.pill-fail    { background: rgba(230,57,70,0.12);   color: #e63946; }
.pill-skip    { background: rgba(88,96,105,0.2);    color: #586069; }
.pill-unknown { background: rgba(88,96,105,0.15);   color: #586069; }
.status-dot { width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0; }
.dot-mass     { background: var(--accent); box-shadow: 0 0 4px var(--accent); }
.dot-personal { background: var(--warn); }
.dot-drill    { background: var(--drill); box-shadow: 0 0 4px var(--drill); }
.mono { font-family: 'IBM Plex Mono', monospace; font-size: 0.7rem; color: var(--muted); }
.section-title {
    font-family: 'IBM Plex Mono', monospace; font-size: 0.65rem;
    letter-spacing: 3px; color: var(--muted); text-transform: uppercase;
    margin-bottom: 12px; padding-bottom: 8px;
    border-bottom: 1px solid var(--border);
}
.drill-alert {
    background: rgba(247,183,49,0.08);
    border: 1px solid rgba(247,183,49,0.45);
    border-left: 4px solid var(--drill);
    border-radius: 6px; padding: 16px 20px; margin-bottom: 20px;
    font-family: 'IBM Plex Mono', monospace; font-size: 0.8rem;
    color: var(--drill); letter-spacing: 1px;
}
.drill-alert strong { font-size: 1rem; letter-spacing: 3px; }
.purge-box {
    background: rgba(230,57,70,0.06);
    border: 1px solid rgba(230,57,70,0.3);
    border-radius: 6px; padding: 16px 20px; margin-bottom: 20px;
}
.nav-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 8px; padding: 28px 24px;
    text-decoration: none; display: block;
    transition: border-color 0.15s, box-shadow 0.15s;
    cursor: pointer;
}
.nav-card:hover { border-color: var(--accent); box-shadow: 0 0 0 1px var(--accent); }
.nav-card-icon  { font-size: 2rem; margin-bottom: 10px; }
.nav-card-title {
    font-family: 'Bebas Neue', sans-serif;
    font-size: 1.4rem; letter-spacing: 3px; color: var(--accent);
}
.nav-card-desc  {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.65rem; color: var(--muted); margin-top: 6px;
    letter-spacing: 1px; line-height: 1.6;
}
.stButton > button {
    font-family: 'IBM Plex Mono', monospace !important;
    font-size: 0.75rem !important;
    letter-spacing: 2px !important;
    text-transform: uppercase !important;
    border-radius: 4px !important;
}
</style>
"""

# ════════════════════════════════════════════════════════════════════════════════
#  DB — CORE
# ════════════════════════════════════════════════════════════════════════════════

def db_connect():
    if not os.path.exists(DB_PATH):
        return None
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

def init_extra_tables():
    """Create/migrate staff and devices tables."""
    conn = sqlite3.connect(DB_PATH)
    try:
        # Staff — separate alert flags for mass vs personal emergencies
        conn.execute("""
        CREATE TABLE IF NOT EXISTS staff (
            id                    INTEGER PRIMARY KEY AUTOINCREMENT,
            name                  TEXT NOT NULL,
            role                  TEXT,
            email                 TEXT,
            phone                 TEXT,
            email_alerts_mass     INTEGER DEFAULT 0,
            email_alerts_personal INTEGER DEFAULT 0,
            sms_alerts_mass       INTEGER DEFAULT 0,
            sms_alerts_personal   INTEGER DEFAULT 0,
            created_at            TEXT DEFAULT (datetime('now'))
        );
        """)

        # Migrate: add new columns to existing staff tables
        for col in ("email_alerts_mass", "email_alerts_personal",
                    "sms_alerts_mass", "sms_alerts_personal"):
            try:
                conn.execute(f"ALTER TABLE staff ADD COLUMN {col} INTEGER DEFAULT 0")
            except sqlite3.OperationalError:
                pass  # Already exists

        conn.execute("""
        CREATE TABLE IF NOT EXISTS devices (
            id                INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id         TEXT NOT NULL UNIQUE,
            label             TEXT,
            location          TEXT,
            assigned_staff_id INTEGER,
            created_at        TEXT DEFAULT (datetime('now')),
            FOREIGN KEY (assigned_staff_id) REFERENCES staff(id)
        );
        """)

        # Migrate: add triggered_by to incidents if missing
        try:
            conn.execute("ALTER TABLE incidents ADD COLUMN triggered_by TEXT")
        except sqlite3.OperationalError:
            pass

        conn.commit()
    finally:
        conn.close()

# ── Incidents ─────────────────────────────────────────────────────────────────

def fetch_incidents(limit=200):
    conn = db_connect()
    if not conn:
        return []
    try:
        rows = conn.execute(
            "SELECT * FROM incidents ORDER BY id DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()

def fetch_counts():
    conn = db_connect()
    if not conn:
        return {}
    try:
        row = conn.execute(f"""
            SELECT
                COUNT(*) as total,
                SUM(CASE WHEN emergency_type='mass'
                         AND trigger_source!='{DRILL_SOURCE}' THEN 1 ELSE 0 END) as mass,
                SUM(CASE WHEN emergency_type='personal'
                         AND trigger_source!='{DRILL_SOURCE}' THEN 1 ELSE 0 END) as personal,
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
        row = conn.execute(
            "SELECT COUNT(*) FROM incidents WHERE server_time < ?", (cutoff,)
        ).fetchone()
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

# ── Staff ─────────────────────────────────────────────────────────────────────

def lookup_staff_by_device(device_id: str):
    """Return the staff member assigned to a device_id, or None if unassigned."""
    conn = db_connect()
    if not conn:
        return None
    try:
        row = conn.execute("""
            SELECT s.name, s.role, s.email, s.phone
            FROM devices d
            JOIN staff s ON d.assigned_staff_id = s.id
            WHERE d.device_id = ?
        """, (device_id,)).fetchone()
        return dict(row) if row else None
    finally:
        conn.close()

def get_device_label(device_id: str):
    """Return the friendly label for a device_id, or None if not registered."""
    conn = db_connect()
    if not conn:
        return None
    try:
        row = conn.execute(
            "SELECT label FROM devices WHERE device_id = ?", (device_id,)
        ).fetchone()
        return row[0] if row and row[0] else None
    finally:
        conn.close()

def fetch_all_staff():
    conn = db_connect()
    if not conn:
        return []
    try:
        return [dict(r) for r in conn.execute(
            "SELECT * FROM staff ORDER BY name ASC"
        ).fetchall()]
    finally:
        conn.close()

def add_staff(name, role, email, phone,
              email_alerts_mass=False, email_alerts_personal=False,
              sms_alerts_mass=False, sms_alerts_personal=False):
    conn = db_connect()
    try:
        conn.execute(
            """INSERT INTO staff
               (name, role, email, phone,
                email_alerts_mass, email_alerts_personal,
                sms_alerts_mass, sms_alerts_personal)
               VALUES (?,?,?,?,?,?,?,?)""",
            (name, role, email, phone,
             int(email_alerts_mass), int(email_alerts_personal),
             int(sms_alerts_mass), int(sms_alerts_personal))
        )
        conn.commit()
    finally:
        conn.close()

def update_staff(staff_id, name, role, email, phone,
                 email_alerts_mass, email_alerts_personal,
                 sms_alerts_mass, sms_alerts_personal):
    conn = db_connect()
    try:
        conn.execute(
            """UPDATE staff SET
               name=?, role=?, email=?, phone=?,
               email_alerts_mass=?, email_alerts_personal=?,
               sms_alerts_mass=?, sms_alerts_personal=?
               WHERE id=?""",
            (name, role, email, phone,
             int(email_alerts_mass), int(email_alerts_personal),
             int(sms_alerts_mass), int(sms_alerts_personal),
             staff_id)
        )
        conn.commit()
    finally:
        conn.close()

def delete_staff(staff_id):
    conn = db_connect()
    try:
        # Unassign any devices pointing to this staff member
        conn.execute("UPDATE devices SET assigned_staff_id=NULL WHERE assigned_staff_id=?", (staff_id,))
        conn.execute("DELETE FROM staff WHERE id=?", (staff_id,))
        conn.commit()
    finally:
        conn.close()

# ── Devices ───────────────────────────────────────────────────────────────────

def fetch_all_devices():
    conn = db_connect()
    if not conn:
        return []
    try:
        return [dict(r) for r in conn.execute("""
            SELECT d.*, s.name as staff_name, s.role as staff_role
            FROM devices d
            LEFT JOIN staff s ON d.assigned_staff_id = s.id
            ORDER BY d.device_id ASC
        """).fetchall()]
    finally:
        conn.close()

def fetch_known_device_ids():
    """Pull distinct device_ids seen in incidents that aren't yet registered."""
    conn = db_connect()
    if not conn:
        return []
    try:
        registered = {r["device_id"] for r in conn.execute("SELECT device_id FROM devices").fetchall()}
        seen = {r[0] for r in conn.execute(
            "SELECT DISTINCT device_id FROM incidents WHERE device_id IS NOT NULL"
        ).fetchall()}
        return sorted(seen - registered)
    finally:
        conn.close()

def add_device(device_id, label, location, assigned_staff_id=None):
    conn = db_connect()
    try:
        conn.execute(
            "INSERT OR IGNORE INTO devices (device_id, label, location, assigned_staff_id) VALUES (?,?,?,?)",
            (device_id, label, location, assigned_staff_id or None)
        )
        conn.commit()
    finally:
        conn.close()

def update_device(device_db_id, label, location, assigned_staff_id):
    conn = db_connect()
    try:
        conn.execute(
            "UPDATE devices SET label=?, location=?, assigned_staff_id=? WHERE id=?",
            (label, location, assigned_staff_id or None, device_db_id)
        )
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

# ── System status ─────────────────────────────────────────────────────────────

def get_system_status():
    status = {}

    try:
        with socket.create_connection((SERVER_HOST, SERVER_PORT), timeout=2):
            status["server"] = ("ok", f"Reachable on {SERVER_HOST}:{SERVER_PORT}")
    except ConnectionRefusedError:
        status["server"] = ("error", "Server offline / not running")
    except socket.timeout:
        status["server"] = ("error", "Timeout — not responding")
    except Exception as e:
        status["server"] = ("error", str(e))

    try:
        conn = db_connect()
        count = conn.execute("SELECT COUNT(*) FROM incidents").fetchone()[0]
        conn.close()
        status["db"] = ("ok", f"{count} total incidents")
    except Exception as e:
        status["db"] = ("error", str(e))

    try:
        conn = db_connect()
        row = conn.execute(
            "SELECT server_time FROM incidents ORDER BY id DESC LIMIT 1"
        ).fetchone()
        conn.close()
        status["last_incident"] = ("ok", row[0] if row else "No incidents yet")
    except Exception as e:
        status["last_incident"] = ("error", str(e))

    HEALTHY = {"ok", "queued", "skipped"}
    FAILURE = {"failed", "queue_full"}

    try:
        conn = db_connect()
        rows = [dict(r) for r in conn.execute(f"""
            SELECT audio_status, email_status, sms_status, relay_status
            FROM incidents
            WHERE trigger_source != '{DRILL_SOURCE}'
            ORDER BY id DESC LIMIT 10
        """).fetchall()]
        conn.close()

        for key in ("audio", "email", "sms", "relay"):
            col    = f"{key}_status"
            values = [r[col] for r in rows if r.get(col)]
            if not values:
                status[key] = ("unknown", "No data yet")
                continue
            fails = sum(1 for v in values if v in FAILURE)
            if fails == 0:
                status[key] = ("ok", "All recent OK")
            elif fails < len(values):
                status[key] = ("warn", f"{fails}/{len(values)} recent failures")
            else:
                status[key] = ("error", "All recent failed")
    except Exception as e:
        for key in ("audio", "email", "sms", "relay"):
            status[key] = ("error", str(e))

    try:
        purgeable, _ = count_purgeable(RETENTION_DAYS)
        if purgeable == 0:
            status["retention"] = ("ok", f"No records older than {RETENTION_DAYS}d")
        else:
            status["retention"] = ("warn", f"{purgeable} records older than {RETENTION_DAYS}d")
    except Exception as e:
        status["retention"] = ("error", str(e))

    return status

# ════════════════════════════════════════════════════════════════════════════════
#  SYSTEM CONTROL  (relay / audio stop signals via DB)
# ════════════════════════════════════════════════════════════════════════════════

def get_control(key: str) -> str:
    """Read a system_control flag from the DB. Returns '0' if table missing or key absent."""
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
    """Write a system_control flag. Creates the table if it doesn't exist yet."""
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("""
        CREATE TABLE IF NOT EXISTS system_control (
            key   TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );
        """)
        conn.execute("INSERT OR REPLACE INTO system_control (key,value) VALUES (?,?)", (key, value))
        conn.commit()
    finally:
        conn.close()

def alerts_active() -> bool:
    """Return True if relay or audio is currently running."""
    return get_control("relay_active") == "1" or get_control("audio_active") == "1"

def send_stop_command():
    """
    Write stop flags to the DB AND send a TCP command to the server so it
    reacts immediately rather than waiting for the next poll cycle.
    """
    set_control("relay_active", "0")
    set_control("audio_active", "0")
    # Best-effort TCP nudge — server handles {"command":"stop_alerts"}
    try:
        import socket as _socket, json as _json
        with _socket.create_connection((SERVER_HOST, SERVER_PORT), timeout=3) as s:
            s.sendall(_json.dumps({"command": "stop_alerts"}).encode())
    except Exception:
        pass  # DB flags are sufficient; TCP is just for immediacy

def pill(status, label):
    cls_map = {
        "ok":         "pill-ok",
        "failed":     "pill-fail",
        "queue_full": "pill-fail",
        "skipped":    "pill-skip",
        "queued":     "pill-ok",
    }
    cls = cls_map.get(status, "pill-unknown")
    return f'<span class="pill {cls}">{label}</span>'

def status_pill(state, text):
    cls  = {"ok": "pill-ok", "warn": "pill-warn", "error": "pill-fail"}.get(state, "pill-unknown")
    icon = {"ok": "●", "warn": "▲", "error": "✕", "unknown": "○"}.get(state, "○")
    return f'<span class="pill {cls}">{icon} {text}</span>'

def type_badge(em_type, src):
    if src == DRILL_SOURCE:    return '<span class="badge badge-drill">DRILL</span>'
    if em_type == "mass":      return '<span class="badge badge-mass">MASS</span>'
    return '<span class="badge badge-personal">PERSONAL</span>'

def dot_cls(em_type, src):
    if src == DRILL_SOURCE: return "dot-drill"
    if em_type == "mass":   return "dot-mass"
    return "dot-personal"

def row_cls(em_type, src):
    if src == DRILL_SOURCE: return "is-drill"
    if em_type == "mass":   return "is-mass"
    return ""

def passes_filter(inc, type_filter):
    em  = inc.get("emergency_type", "")
    src = inc.get("trigger_source", "")
    if src == DRILL_SOURCE: return "test_drill" in type_filter
    if em == "mass":        return "mass" in type_filter
    if em == "personal":    return "personal" in type_filter
    return True

def ers_header(subtitle=""):
    return f"""
    <div class="ers-header">
      <div class="ers-logo">ERS</div>
      {"<div class='ers-sub'>" + subtitle + "</div>" if subtitle else ""}
    </div>
    """



# ════════════════════════════════════════════════════════════════════════════════
#  SYSTEM MEMORY  (the location of gateways)
# ════════════════════════════════════════════════════════════════════════════════


def init_map_table():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("""
            CREATE TABLE IF NOT EXISTS map_config (
                key TEXT PRIMARY KEY,
                val_x REAL,
                val_y REAL
            )
        """)

        for gw in ["GW1", "GW2", "GW3"]:
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

def fetch_all_gw_coords():
    if not os.path.exists(DB_PATH):
        return {gw: {"x": 0.0, "y": 0.0} for gw in ["GW1", "GW2", "GW3"]}
    
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    try:
        rows = conn.execute("SELECT * FROM map_config").fetchall()
        if not rows:
            return {gw: {"x": 0.0, "y": 0.0} for gw in ["GW1", "GW2", "GW3"]}
        return {row['key']: {"x": row['val_x'], "y": row['val_y']} for row in rows}
    except sqlite3.OperationalError:
        return {gw: {"x": 0.0, "y": 0.0} for gw in ["GW1", "GW2", "GW3"]}
    finally:
        conn.close()
        
def save_map_scale(meters_wide):
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("INSERT OR REPLACE INTO system_control (key, value) VALUES (?, ?)", 
                     ("map_meters_wide", str(meters_wide)))
        conn.commit()
    finally:
        conn.close()

def fetch_map_scale():
    try:
        val = get_control("map_meters_wide")
        return float(val) if val != "0" else 50.0
    except:
        return 50.0
