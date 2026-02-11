import socket, json
from datetime import datetime
import time as t
import os, sys
import threading
import queue
import time
import traceback   

sys.path.append(os.path.expanduser("~/ers_server"))

from email_alert import send_email_alert
from sms_alert import send_sms_alert
from audio_alert import play_audio_alert
from relay_alert import relay_pulse, relay_off

LAST_SENT = {}
COOLDOWN_SEC = 5

HOST = "0.0.0.0"
PORT = 8080
LOG_FILE = os.path.expanduser("~/ers_server/emergency_log.jsonl")

def log_event(event: dict):
    print(event)
    with open(LOG_FILE, "a") as f:
        f.write(json.dumps(event, ensure_ascii=False) + "\n")


relay_queue = queue.Queue(maxsize=100)
relay_i2c_lock = threading.Lock()

def safe_relay_pulse(**kwargs): # Ensures only one relay action touches I2C at a time
    with relay_i2c_lock:
        relay_pulse(**kwargs)

def safe_relay_off(ch=1):
    with relay_i2c_lock:
        relay_off(ch)

def relay_worker():
    while True:
        job = relay_queue.get()
        try:
            fn = job["fn"]
            kwargs = job["kwargs"]
            meta = job.get("meta", {})

            log_event({
                "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "type": "relay_job_start",
                "meta": meta,
                "kwargs": kwargs
            })

            fn(**kwargs)

            log_event({
                "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "type": "relay_job_ok",
                "meta": meta
            })

        except Exception as e:
            log_event({
                "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "type": "relay_job_failed",
                "error": str(e),
                "trace": traceback.format_exc()
            })
            try:
                safe_relay_off(kwargs.get("ch", 1))
            except:
                pass
        finally:
            relay_queue.task_done()

def start_relay_thread():
    threading.Thread(target=relay_worker, daemon=True).start()

def queue_mass_relay(device_id: str, on_sec: float = 120.0):
    """
    Coalesce behavior: if a job is already queued/running, skip new ones.
    Remove this qsize() check if you WANT multiple queued sirens.
    """
    if relay_queue.qsize() > 0:
        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "relay_coalesced_skip",
            "reason": "job_already_queued_or_running",
            "queue_size": relay_queue.qsize()
        })
        return

    relay_queue.put_nowait({
        "fn": safe_relay_pulse,
        "kwargs": {"ch": 1, "on_sec": float(on_sec), "off_sec": 0.0, "repeat": 1},
        "meta": {"device_id": device_id, "emergency_type": "mass"}
    })



def handle_payload(payload: dict, addr):
    import time

    device_id = payload.get("device_id") or payload.get("pico_id") or addr[0]
    em_flag = payload.get("emergency", False)

    log_event({
        "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "type": "em_flag_debug",
        "device_id": device_id,
        "emergency_value": em_flag
    })

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
            "since_sec": round(now_ts - last_ts, 2)
        })
        return

    LAST_SENT[key] = now_ts

    event = {
        "server_time": now_str,
        "device_id": device_id,
        "emergency_type": em_type,
        "message": msg,
        "from_ip": addr[0],
        "from_port": addr[1],
        "pico_ts": ts,
        "raw": payload
    }

    log_event(event)

    # Audio
    log_event({"server_time": now_str, "type": "audio_play_start", "emergency_type": em_type})
    try:
        audio_res = play_audio_alert(event, block=False, min_interval_sec=0)
        log_event({"server_time": now_str, "type": "audio_play_ok", "result": audio_res})
    except Exception as e:
        log_event({"server_time": now_str, "type": "audio_play_failed", "error": str(e)})

    # Email
    log_event({
        "server_time": now_str,
        "type": "email_send_start",
        "to": os.getenv("EMAIL_TO", "unset"),
        "from": os.getenv("EMAIL_ADDRESS", "unset")
    })
    try:
        send_email_alert(event)
        log_event({"server_time": now_str, "type": "email_send_ok"})
    except Exception as e:
        log_event({"server_time": now_str, "type": "email_failed", "error": str(e)})

    # SMS
    log_event({"server_time": now_str, "type": "sms_send_start", "to": os.getenv("SMS_TO", "unset")})
    try:
        result = send_sms_alert(event)
        log_event({"server_time": now_str, "type": "sms_send_ok", "result": result})
    except Exception as e:
        log_event({"server_time": now_str, "type": "sms_failed", "error": str(e)})

    # Relay
    if em_type == "mass":
        try:
            queue_mass_relay(device_id=device_id, on_sec=120.0) 
            log_event({"server_time": now_str, "type": "relay_queued", "channel": 1})
        except queue.Full:
            log_event({"server_time": now_str, "type": "relay_queue_full", "channel": 1})
    else:
        log_event({"server_time": now_str, "type": "relay_skip", "reason": "personal"})


def client_thread(conn, addr):
    try:
        data = conn.recv(4096)
        if not data:
            return

        payload = json.loads(data.decode("utf-8"))
        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "payload_received",
            "raw": payload
        })
        handle_payload(payload, addr)

    except Exception as e:
        log_event({
            "server_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type": "server_error",
            "from_ip": addr[0],
            "from_port": addr[1],
            "error": str(e)
        })
    finally:
        try:
            conn.close()
        except:
            pass


def start_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    s.bind((HOST, PORT))
    s.listen(50)

    print(f"Mini Emergency Server listening on {HOST}:{PORT}")
    print("Waiting for Pico connections...")

    while True:
        conn, addr = s.accept()

        threading.Thread(target=client_thread, args=(conn, addr), daemon=True).start()


if __name__ == "__main__":
    start_relay_thread()
    start_server()
