#!/usr/bin/env python3
"""ERS LoRa bridge: Semtech UDP (lora_pkt_fwd) -> ERS server TCP.

Derived from the Pi's ``lora_udp_bridge.py`` (see "lora radio ref.txt"), with:
  * ThingsBoard removed entirely (decision D8),
  * frame v1 decoding (see frame.py) instead of the 4-byte payload,
  * the "map every packet to PICO_TEST_01" workaround removed: the bridge does
    NOT resolve identity. It decodes, filters foreign traffic, and forwards the
    ERS device id; the server maps id -> device -> wearer,
  * PULL_DATA acknowledged (PULL_ACK) so the forwarder sees the link as healthy,
  * retry + on-disk queue so an emergency is not lost while ers-server restarts,
  * RSSI/SNR/frequency forwarded so the server can record link quality.

Runs on the Pi next to lora_pkt_fwd (UDP 127.0.0.1:1730 by default).
"""

import base64
import collections
import json
import os
import socket
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frame  # noqa: E402

REPO_DIR = Path(__file__).resolve().parents[2]

# -- Configuration (env overrides keep the unit file free of code edits) --------
UDP_HOST = os.environ.get("ERS_BRIDGE_UDP_HOST", "127.0.0.1")
UDP_PORT = int(os.environ.get("ERS_BRIDGE_UDP_PORT", "1730"))
ERS_HOST = os.environ.get("ERS_SERVER_HOST", "127.0.0.1")
ERS_PORT = int(os.environ.get("ERS_SERVER_PORT", "8081"))
SOCKET_TIMEOUT_S = 3

LOG_DIR = Path(os.environ.get("ERS_LOG_DIR", REPO_DIR / "logs"))
LORA_LOG_FILE = LOG_DIR / "lora_reception.jsonl"
QUEUE_FILE = LOG_DIR / "lora_bridge_queue.jsonl"
QUEUE_MAX = 500          # oldest dropped beyond this
RETRY_INTERVAL_S = 2

SITE_FILE_CANDIDATES = [
    os.environ.get("ERS_SITE_FILE", ""),
    "/etc/ers/site.json",
    str(REPO_DIR / "data" / "site.json"),
]

STATS = collections.Counter()
_site_id_cache = {"t": 0.0, "v": None}


def get_site_id():
    """This site's id (re-read every 30 s so a re-provision needs no restart)."""
    if time.time() - _site_id_cache["t"] > 30:
        _site_id_cache["t"] = time.time()
        _site_id_cache["v"] = None
        for path in SITE_FILE_CANDIDATES:
            if path and os.path.isfile(path):
                try:
                    with open(path, encoding="utf-8") as f:
                        _site_id_cache["v"] = int(json.load(f)["site_id"])
                except (OSError, ValueError, KeyError):
                    pass
                break
    return _site_id_cache["v"]


# -- Semtech UDP ----------------------------------------------------------------
def parse_semtech_udp(data: bytes):
    """Return (version, token, identifier, body) or None if too short."""
    if len(data) < 4:
        return None
    return data[0], data[1:3], data[3], data[12:]


# -- Decode + filter ------------------------------------------------------------
def build_payload(raw: bytes, rx: dict):
    """Turn raw LoRa bytes into an ERS JSON payload, or (None, reason)."""
    decoded = frame.decode_v1(raw)
    if decoded is None:
        return None, "foreign_or_legacy"

    site = get_site_id()
    if site is not None and decoded["site_id"] != site:
        return None, "other_site"

    meta = frame.EVENT_META.get(decoded["event_code"])
    if meta is None:
        return None, "unknown_event"

    return {
        "device_id": decoded["device_id"],
        "site_id": decoded["site_id"],
        "event_code": decoded["event_code"],
        "emergency": meta["emergency"],
        "emergency_type": meta["emergency_type"],
        "emergency_message": meta["emergency_message"],
        "timestamp_ms": decoded["seq"],  # sequence number, kept for compatibility
        "seq": decoded["seq"],
        "source": "lora",
        "channel": "lora",
        "rssi": rx.get("rssi"),
        "snr": rx.get("lsnr"),
        "freq_mhz": rx.get("freq"),
        "data_rate": rx.get("datr"),
    }, None


# -- Forwarding with retry queue ------------------------------------------------
def forward_to_ers(payload: dict) -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(SOCKET_TIMEOUT_S)
            s.connect((ERS_HOST, ERS_PORT))
            s.sendall(json.dumps(payload).encode("utf-8"))
        return True
    except OSError as e:
        print(f"[BRIDGE] ERS forward failed: {e}", flush=True)
        return False


def _read_queue():
    try:
        with QUEUE_FILE.open(encoding="utf-8") as f:
            return [json.loads(line) for line in f if line.strip()]
    except (OSError, ValueError):
        return []


def _write_queue(items):
    items = items[-QUEUE_MAX:]
    if not items:
        try:
            QUEUE_FILE.unlink()
        except FileNotFoundError:
            pass
        return
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    tmp = QUEUE_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        f.writelines(json.dumps(i) + "\n" for i in items)
    os.replace(tmp, QUEUE_FILE)


def enqueue(payload: dict):
    """Persist an undelivered payload. Heartbeats are not worth queueing."""
    if not payload.get("emergency"):
        return
    items = _read_queue()
    items.append(payload)
    _write_queue(items)
    STATS["queued"] += 1
    print(f"[BRIDGE] queued ({len(items)} pending)", flush=True)


def drain_queue():
    """Try to deliver queued emergencies in order; stop at the first failure."""
    items = _read_queue()
    if not items:
        return
    sent = 0
    for item in items:
        if not forward_to_ers(item):
            break
        sent += 1
    if sent:
        _write_queue(items[sent:])
        STATS["queue_delivered"] += sent
        print(f"[BRIDGE] delivered {sent} queued payload(s)", flush=True)


def deliver(payload: dict):
    # Preserve ordering: anything already queued goes first.
    drain_queue()
    if _read_queue() or not forward_to_ers(payload):
        enqueue(payload)


# -- Logging --------------------------------------------------------------------
def log_lora_reception(rx, raw, received_at, result, payload=None):
    record = {
        "received_at_utc": received_at,
        "result": result,                      # "forwarded" or the drop reason
        "device_id": (payload or {}).get("device_id"),
        "event_code": (payload or {}).get("event_code"),
        "seq": (payload or {}).get("seq"),
        "rssi_dbm": rx.get("rssi"),
        "snr_db": rx.get("lsnr"),
        "frequency_mhz": rx.get("freq"),
        "data_rate": rx.get("datr"),
        "raw_bytes": list(raw),
    }
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        with LORA_LOG_FILE.open("a", encoding="utf-8") as log:
            log.write(json.dumps(record) + "\n")
    except OSError as e:
        print(f"[LORA LOG ERROR] {e}", flush=True)


def process_rxpk(rx: dict):
    b64 = rx.get("data")
    if not b64:
        return
    try:
        raw = base64.b64decode(b64)
    except Exception as e:
        print(f"[BRIDGE] Base64 decode error: {e}", flush=True)
        return
    ts = datetime.now(timezone.utc).isoformat()
    payload, reason = build_payload(raw, rx)
    if payload is None:
        STATS[reason] += 1          # foreign traffic is counted and logged, never processed
        log_lora_reception(rx, raw, ts, reason)
        return
    STATS["forwarded"] += 1
    log_lora_reception(rx, raw, ts, "forwarded", payload)
    print(f"[BRIDGE] {ts} {payload['device_id']} event={payload['event_code']} "
          f"rssi={rx.get('rssi')}", flush=True)
    deliver(payload)


# -- Main loop ------------------------------------------------------------------
def run():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, UDP_PORT))
    sock.settimeout(RETRY_INTERVAL_S)
    print(f"[BRIDGE] UDP {UDP_HOST}:{UDP_PORT} -> ERS tcp {ERS_HOST}:{ERS_PORT}", flush=True)
    last_stats = time.time()

    while True:
        try:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                drain_queue()                       # idle: retry anything queued
                data = None
            if data is not None:
                parsed = parse_semtech_udp(data)
                if parsed:
                    version, token, ident, body = parsed
                    if ident == 0x00:               # PUSH_DATA -> PUSH_ACK
                        sock.sendto(bytes([version]) + token + b"\x01", addr)
                        try:
                            obj = json.loads(body.decode("utf-8"))
                        except Exception as e:
                            print(f"[BRIDGE] JSON parse error: {e}", flush=True)
                            obj = {}
                        for rx in obj.get("rxpk", []):
                            process_rxpk(rx)
                    elif ident == 0x02:             # PULL_DATA -> PULL_ACK
                        sock.sendto(bytes([version]) + token + b"\x04", addr)
            if time.time() - last_stats > 300:
                last_stats = time.time()
                print(f"[BRIDGE] stats {dict(STATS)}", flush=True)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[BRIDGE] Unexpected error: {e}", flush=True)
            time.sleep(1)
    sock.close()


if __name__ == "__main__":
    run()
