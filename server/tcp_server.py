"""TCP listener that accepts one JSON payload per connection and dispatches it."""

import json
import socket
import threading
import traceback

import config
from event_log import log_event, now_str
from incident_handler import handle_payload


def client_thread(conn, addr):
    """Read a single JSON payload from a connection and hand it to the handler."""
    try:
        data = conn.recv(4096)
        if not data:
            return
        payload = json.loads(data.decode("utf-8"))
        log_event({"server_time": now_str(), "type": "payload_received", "raw": payload})
        handle_payload(payload, addr)
    except Exception as e:
        log_event({
            "server_time": now_str(),
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
    """Listen forever, handling each client connection on its own thread."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((config.HOST, config.PORT))
    s.listen(50)
    print(f"ERS Server listening on {config.HOST}:{config.PORT}")
    print("Relay and audio will loop indefinitely until stopped via dashboard.")
    while True:
        conn, addr = s.accept()
        threading.Thread(target=client_thread, args=(conn, addr), daemon=True).start()
