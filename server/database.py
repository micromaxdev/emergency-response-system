"""SQLite persistence layer: schema setup, system flags, incidents and lookups."""

import sqlite3

import config
from event_log import log_event, now_str

DB_PATH = config.DB_PATH


# -- Schema --------------------------------------------------------------------
def init_db():
    """Create the database schema if it does not already exist and seed flags."""
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("""
        CREATE TABLE IF NOT EXISTS incidents (
          id               INTEGER PRIMARY KEY AUTOINCREMENT,
          server_time      TEXT,
          device_id        TEXT,
          emergency_type   TEXT,
          trigger_source   TEXT,
          message          TEXT,
          triggered_by     TEXT,
          from_ip          TEXT,
          from_port        INTEGER,
          pico_ts          TEXT,
          audio_status     TEXT,
          audio_result     TEXT,
          audio_error      TEXT,
          email_status     TEXT,
          email_error      TEXT,
          sms_status       TEXT,
          sms_result       TEXT,
          sms_error        TEXT,
          relay_status     TEXT,
          relay_error      TEXT,
          raw_json         TEXT
        );
        """)

        try:
            conn.execute("ALTER TABLE incidents ADD COLUMN triggered_by TEXT")
        except sqlite3.OperationalError:
            pass

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

        for col in (
            "email_alerts_mass",
            "email_alerts_personal",
            "sms_alerts_mass",
            "sms_alerts_personal",
        ):
            try:
                conn.execute(f"ALTER TABLE staff ADD COLUMN {col} INTEGER DEFAULT 0")
            except sqlite3.OperationalError:
                pass

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

        conn.execute("""
        CREATE TABLE IF NOT EXISTS system_control (
            key   TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );
        """)

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

        conn.execute(
            "INSERT OR REPLACE INTO system_control (key,value) VALUES ('relay_active','0')"
        )
        conn.execute(
            "INSERT OR REPLACE INTO system_control (key,value) VALUES ('audio_active','0')"
        )

        conn.commit()
    finally:
        conn.close()


# -- System control flags ------------------------------------------------------
def get_control(key: str) -> str:
    """Read a system_control flag, defaulting to '0' on any error."""
    try:
        conn = sqlite3.connect(DB_PATH)
        row = conn.execute(
            "SELECT value FROM system_control WHERE key=?", (key,)
        ).fetchone()
        conn.close()
        return row[0] if row else "0"
    except Exception:
        return "0"


def set_control(key: str, value: str):
    """Write a system_control flag."""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.execute(
            "INSERT OR REPLACE INTO system_control (key,value) VALUES (?,?)",
            (key, value),
        )
        conn.commit()
        conn.close()
    except Exception as e:
        log_event({
            "server_time": now_str(),
            "type": "set_control_failed",
            "key": key,
            "error": str(e),
        })


# -- Incidents -----------------------------------------------------------------
def insert_incident(row: dict):
    """Persist a single incident record built by the incident handler."""
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.execute("""
            INSERT INTO incidents (
                server_time, device_id, emergency_type, trigger_source, message,
                triggered_by, from_ip, from_port, pico_ts,
                audio_status, audio_result, audio_error,
                email_status, email_error,
                sms_status, sms_result, sms_error,
                relay_status, relay_error, raw_json
            ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
        """, (
            row.get("server_time"),
            row.get("device_id"),
            row.get("emergency_type"),
            row.get("trigger_source"),
            row.get("message"),
            row.get("triggered_by"),
            row.get("from_ip"),
            row.get("from_port"),
            row.get("pico_ts"),
            row.get("audio_status"),
            row.get("audio_result"),
            row.get("audio_error"),
            row.get("email_status"),
            row.get("email_error"),
            row.get("sms_status"),
            row.get("sms_result"),
            row.get("sms_error"),
            row.get("relay_status"),
            row.get("relay_error"),
            row.get("raw_json"),
        ))
        conn.commit()
    finally:
        conn.close()


# -- Alert recipient lookups ---------------------------------------------------
def get_email_recipients(em_type: str) -> list:
    """Return staff emails opted in to the given emergency type's email alerts."""
    col = "email_alerts_mass" if em_type == "mass" else "email_alerts_personal"
    try:
        conn = sqlite3.connect(DB_PATH)
        rows = conn.execute(
            f"SELECT email FROM staff WHERE {col}=1 AND email!='' AND email IS NOT NULL"
        ).fetchall()
        conn.close()
        emails = [r[0].strip() for r in rows if r[0] and r[0].strip()]
        if emails:
            return emails
    except Exception as e:
        log_event({
            "server_time": now_str(),
            "type": "staff_email_lookup_failed",
            "error": str(e),
        })
    return []


def get_sms_recipients(em_type: str) -> str:
    """Return a comma-joined list of staff phones opted in to SMS alerts."""
    col = "sms_alerts_mass" if em_type == "mass" else "sms_alerts_personal"
    try:
        conn = sqlite3.connect(DB_PATH)
        rows = conn.execute(
            f"SELECT phone FROM staff WHERE {col}=1 AND phone!='' AND phone IS NOT NULL"
        ).fetchall()
        conn.close()
        phones = [r[0].strip() for r in rows if r[0] and r[0].strip()]
        if phones:
            return ",".join(phones)
    except Exception as e:
        log_event({
            "server_time": now_str(),
            "type": "staff_sms_lookup_failed",
            "error": str(e),
        })
    return ""


def get_triggered_by(device_id: str):
    """Return 'Name (role)' for the staff member assigned to a device, if any."""
    try:
        conn = sqlite3.connect(DB_PATH)
        row = conn.execute("""
            SELECT s.name, s.role FROM devices d
            JOIN staff s ON d.assigned_staff_id = s.id
            WHERE d.device_id = ?
        """, (device_id,)).fetchone()
        conn.close()
        return f"{row[0]} ({row[1]})" if row else None
    except Exception as e:
        log_event({
            "server_time": now_str(),
            "type": "triggered_by_lookup_failed",
            "error": str(e),
        })
        return None


# -- Room lookup (localisation) ------------------------------------------------
def lookup_room_by_xy(x, y):
    """Return the readable room name whose zone contains (x, y) metres, or None."""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row

        rows = conn.execute("""
            SELECT room_name, x_min, y_min, x_max, y_max
            FROM room_zones
        """).fetchall()

        conn.close()

        for row in rows:
            x_min = min(float(row["x_min"]), float(row["x_max"]))
            x_max = max(float(row["x_min"]), float(row["x_max"]))
            y_min = min(float(row["y_min"]), float(row["y_max"]))
            y_max = max(float(row["y_min"]), float(row["y_max"]))

            if x_min <= float(x) <= x_max and y_min <= float(y) <= y_max:
                return row["room_name"]

        return None

    except Exception as e:
        log_event({
            "server_time": now_str(),
            "type": "room_lookup_failed",
            "error": str(e),
        })
        return None
