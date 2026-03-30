import socket
import json
from datetime import datetime
import os
import sys
import threading
import subprocess
import time
import traceback
import sqlite3

# ── Project paths ─────────────────────────────────────────────────────────────
SERVER_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(SERVER_DIR)
ALERT_HANDLERS_DIR = os.path.join(SERVER_DIR, "alert_handlers")
LOG_DIR = os.path.join(BASE_DIR, "logs")
DATA_DIR = os.path.join(BASE_DIR, "data")

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(DATA_DIR, exist_ok=True)

if ALERT_HANDLERS_DIR not in sys.path:
    sys.path.append(ALERT_HANDLERS_DIR)

from email_alert import send_email_alert
from sms_alert import send_sms_alert
from audio_alert import play_audio_alert, AUDIO_DIR, AUDIO_MAP, get_audio_path
from relay_alert import relay_on, relay_off

# ── Config ────────────────────────────────────────────────────────────────────
LAST_SENT = {}
COOLDOWN_SEC = 5

HOST = "0.0.0.0"
PORT = 8081

LOG_FILE = os.path.join(LOG_DIR, "emergency_log.jsonl")
DB_PATH = os.path.join(DATA_DIR, "ers.sqlite")

# ── Thread state ──────────────────────────────────────────────────────────────
relay_i2c_lock = threading.Lock()
_relay_thread_running = False
_audio_thread_running = False
_audio_proc = None
_audio_proc_lock = threading.Lock()
_thread_lock = threading.Lock()

# ── JSONL logger ──────────────────────────────────────────────────────────────
def log_event(event: dict):
    print(event)
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(json.dumps(event, ensure_ascii=False) + "\n")

# ── DB setup ──────────────────────────────────────────────────────────────────
def init_db():
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

        conn.execute(
            "INSERT OR REPLACE INTO system_control (key,value) VALUES ('relay_active','0')"
        )
        conn.execute(
            "INSERT OR REPLACE INTO system_control (key,value) VALUES ('audio_active','0')"
        )

        conn.commit()
    finally:
        conn.close()

# ── system_control helpers ────────────────────────────────────────────────────
def get_control(key: str) -> str:
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
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "set_control_failed",
            "key": key,
            "error": str(e),
        })

# ── DB incident helpers ───────────────────────────────────────────────────────
def insert_incident(row: dict):
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

# ── Alert recipient helpers ───────────────────────────────────────────────────
def get_email_recipients(em_type: str) -> list:
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
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "staff_email_lookup_failed",
            "error": str(e),
        })
    return []

def get_sms_recipients(em_type: str) -> str:
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
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "staff_sms_lookup_failed",
            "error": str(e),
        })
    return ""

def get_triggered_by(device_id: str):
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
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "triggered_by_lookup_failed",
            "error": str(e),
        })
        return None

# ── Relay loop ────────────────────────────────────────────────────────────────
def _relay_loop():
    global _relay_thread_running
    log_event({
        "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "type": "relay_loop_start",
    })
    try:
        with relay_i2c_lock:
            relay_on(ch=1)

        while get_control("relay_active") == "1":
            time.sleep(1)

        with relay_i2c_lock:
            relay_off(ch=1)
        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "relay_loop_stopped",
        })
    except Exception as e:
        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "relay_loop_error",
            "error": str(e),
            "trace": traceback.format_exc(),
        })
        try:
            with relay_i2c_lock:
                relay_off(ch=1)
        except Exception:
            pass
    finally:
        with _thread_lock:
            _relay_thread_running = False

def start_relay_loop():
    global _relay_thread_running
    with _thread_lock:
        if _relay_thread_running:
            return
        _relay_thread_running = True
    set_control("relay_active", "1")
    threading.Thread(target=_relay_loop, daemon=True).start()

# ── Audio loop ────────────────────────────────────────────────────────────────
def _audio_loop(em_type: str):
    global _audio_thread_running, _audio_proc

    resolved = get_audio_path(em_type)
    if resolved is None:
        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "audio_loop_aborted",
            "reason": f"No audio file found for emergency_type='{em_type}' in {AUDIO_DIR}. Upload one on the Configuration page.",
        })
        with _thread_lock:
            _audio_thread_running = False
        return

    audio_path = str(resolved)

    log_event({
        "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "type": "audio_loop_start",
        "file": audio_path,
    })

    try:
        while get_control("audio_active") == "1":
            cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "error", audio_path]
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                with _audio_proc_lock:
                    _audio_proc = proc

                while proc.poll() is None:
                    if get_control("audio_active") != "1":
                        proc.terminate()
                        proc.wait()
                        break
                    time.sleep(0.5)

            except FileNotFoundError:
                cmd2 = ["mpg123", "-q", audio_path]
                try:
                    proc = subprocess.Popen(
                        cmd2,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    with _audio_proc_lock:
                        _audio_proc = proc
                    while proc.poll() is None:
                        if get_control("audio_active") != "1":
                            proc.terminate()
                            proc.wait()
                            break
                        time.sleep(0.5)
                except Exception as e2:
                    log_event({
                        "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        "type": "audio_loop_player_failed",
                        "error": str(e2),
                    })
                    break

            except Exception as e:
                log_event({
                    "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "type": "audio_loop_error",
                    "error": str(e),
                })
                time.sleep(1)

        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "audio_loop_stopped",
        })
    finally:
        with _audio_proc_lock:
            _audio_proc = None
        with _thread_lock:
            _audio_thread_running = False

def start_audio_loop(em_type: str = "mass"):
    global _audio_thread_running
    with _thread_lock:
        if _audio_thread_running:
            return
        _audio_thread_running = True
    set_control("audio_active", "1")
    threading.Thread(target=_audio_loop, args=(em_type,), daemon=True).start()

# ── Stop all alerts ───────────────────────────────────────────────────────────
def stop_all_alerts():
    set_control("relay_active", "0")
    set_control("audio_active", "0")
    log_event({
        "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "type": "stop_all_alerts_requested",
    })

# ── Core handler ──────────────────────────────────────────────────────────────
def handle_payload(payload: dict, addr):
    device_id = payload.get("device_id") or payload.get("pico_id") or addr[0]
    em_flag = payload.get("emergency", False)

    log_event({
        "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "type": "em_flag_debug",
        "device_id": device_id,
        "emergency_value": em_flag,
    })

    if payload.get("command") == "stop_alerts":
        stop_all_alerts()
        return

    if not em_flag:
        return

    em_type = (payload.get("emergency_type") or "personal").lower()
    msg = payload.get("emergency_message") or "EMERGENCY"
    ts = payload.get("timestamp")
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    src = payload.get("trigger_source", "unknown")
    key = f"{device_id}|{em_type}|{src}"
    now_ts = time.time()
    last_ts = LAST_SENT.get(key, 0)

    if now_ts - last_ts < COOLDOWN_SEC:
        log_event({
            "server_time": now_str,
            "type": "rate_limited",
            "key": key,
            "since_sec": round(now_ts - last_ts, 2),
        })
        return

    LAST_SENT[key] = now_ts
    triggered_by = get_triggered_by(device_id)

    event = {
        "server_time": now_str,
        "device_id": device_id,
        "emergency_type": em_type,
        "message": msg,
        "from_ip": addr[0],
        "from_port": addr[1],
        "pico_ts": ts,
        "triggered_by": triggered_by,
        "raw": payload,
    }

    log_event(event)

    incident = {
        "server_time": now_str,
        "device_id": device_id,
        "emergency_type": em_type,
        "trigger_source": src,
        "message": msg,
        "triggered_by": triggered_by,
        "from_ip": addr[0],
        "from_port": addr[1],
        "pico_ts": ts,
        "raw_json": json.dumps(payload, ensure_ascii=False),
    }

    if em_type != "personal":
        log_event({
            "server_time": now_str,
            "type": "audio_loop_queued",
            "emergency_type": em_type,
        })
        try:
            start_audio_loop(em_type)
            incident["audio_status"] = "ok"
            incident["audio_result"] = "loop_started"
        except Exception as e:
            log_event({
                "server_time": now_str,
                "type": "audio_loop_failed",
                "error": str(e),
            })
            incident["audio_status"] = "failed"
            incident["audio_error"] = str(e)
    else:
        incident["audio_status"] = "skipped"

    log_event({
        "server_time": now_str,
        "type": "email_send_start",
        "em_type": em_type,
    })
    try:
        send_email_alert(event, to_emails=get_email_recipients(em_type))
        log_event({
            "server_time": now_str,
            "type": "email_send_ok",
        })
        incident["email_status"] = "ok"
    except Exception as e:
        log_event({
            "server_time": now_str,
            "type": "email_failed",
            "error": str(e),
        })
        incident["email_status"] = "failed"
        incident["email_error"] = str(e)

    log_event({
        "server_time": now_str,
        "type": "sms_send_start",
        "em_type": em_type,
    })
    try:
        sms_res = send_sms_alert(event, to_number=get_sms_recipients(em_type))
        log_event({
            "server_time": now_str,
            "type": "sms_send_ok",
            "result": sms_res,
        })
        incident["sms_status"] = "ok"
        incident["sms_result"] = str(sms_res)
    except Exception as e:
        log_event({
            "server_time": now_str,
            "type": "sms_failed",
            "error": str(e),
        })
        incident["sms_status"] = "failed"
        incident["sms_error"] = str(e)

    if em_type in ("mass", "drill"):
        log_event({
            "server_time": now_str,
            "type": "relay_loop_queued",
        })
        try:
            start_relay_loop()
            incident["relay_status"] = "active"
        except Exception as e:
            log_event({
                "server_time": now_str,
                "type": "relay_loop_failed",
                "error": str(e),
            })
            incident["relay_status"] = "failed"
            incident["relay_error"] = str(e)
    else:
        log_event({
            "server_time": now_str,
            "type": "relay_skip",
            "reason": "personal",
        })
        incident["relay_status"] = "skipped"

    try:
        insert_incident(incident)
        log_event({
            "server_time": now_str,
            "type": "sqlite_incident_ok",
        })
    except Exception as e:
        log_event({
            "server_time": now_str,
            "type": "sqlite_incident_failed",
            "error": str(e),
        })

# ── TCP server ────────────────────────────────────────────────────────────────
def client_thread(conn, addr):
    try:
        data = conn.recv(4096)
        if not data:
            return
        payload = json.loads(data.decode("utf-8"))
        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "payload_received",
            "raw": payload,
        })
        handle_payload(payload, addr)
    except Exception as e:
        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "server_error",
            "from_ip": addr[0],
            "from_port": addr[1],
            "error": str(e),
            "trace": traceback.format_exc(),
        })
    finally:
        try:
            conn.close()
        except Exception:
            pass

def start_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(50)
    print(f"ERS Server listening on {HOST}:{PORT}")
    print("Relay and audio will loop indefinitely until stopped via dashboard.")
    while True:
        conn, addr = s.accept()
        threading.Thread(target=client_thread, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    init_db()
    start_server()
