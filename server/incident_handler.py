"""Turn an incoming payload into alerts (audio/email/sms/relay) and a DB record."""

import json
import time

import config  # noqa: F401  (ensures alert_handlers/ is on sys.path before the imports below)
from event_log import log_event, now_str
from database import (
    get_email_recipients,
    get_sms_recipients,
    get_triggered_by,
    insert_incident,
    lookup_room_by_xy,
)
from location import safe_estimate_location
from alert_runtime import start_audio_loop, start_relay_loop, stop_all_alerts
from email_alert import send_email_alert
from sms_alert import send_sms_alert

# Last time we acted on a given device|type|source key, for rate limiting.
LAST_SENT = {}


def _is_rate_limited(key: str, now_ts: float) -> bool:
    """Return True (and log) if this key fired within the cooldown window."""
    last_ts = LAST_SENT.get(key, 0)
    if now_ts - last_ts < config.COOLDOWN_SEC:
        log_event({
            "server_time": now_str(),
            "type": "rate_limited",
            "key": key,
            "since_sec": round(now_ts - last_ts, 2),
        })
        return True
    LAST_SENT[key] = now_ts
    return False


def _attach_location(event: dict, payload: dict, when: str) -> dict:
    """Estimate the device's room from RSSI and annotate the event in place.

    Returns the payload to persist (with location attached when available).
    """
    location = safe_estimate_location(payload, when)
    if location is None:
        return payload

    readable_room = lookup_room_by_xy(location["x"], location["y"]) or "Unknown area"
    estimated_location = {
        "x": location["x"],
        "y": location["y"],
        "room_name": readable_room,
        "gateways_used": location["gateways_used"],
        "distance_errors": location.get("distance_errors", []),
    }
    event["estimated_location"] = estimated_location
    event["room_name"] = readable_room

    log_event({
        "server_time": when,
        "type": "room_estimated",
        "device_id": event.get("device_id"),
        "x": location["x"],
        "y": location["y"],
        "room_name": readable_room,
    })

    payload_with_location = dict(payload)
    payload_with_location["estimated_location"] = estimated_location
    payload_with_location["room_name"] = readable_room
    return payload_with_location


def _dispatch_audio(incident: dict, em_type: str, when: str):
    """Start the looping audio siren for non-personal emergencies."""
    if em_type == "personal":
        incident["audio_status"] = "skipped"
        return

    log_event({"server_time": when, "type": "audio_loop_queued", "emergency_type": em_type})
    try:
        start_audio_loop(em_type)
        incident["audio_status"] = "ok"
        incident["audio_result"] = "loop_started"
    except Exception as e:
        log_event({"server_time": when, "type": "audio_loop_failed", "error": str(e)})
        incident["audio_status"] = "failed"
        incident["audio_error"] = str(e)


def _dispatch_email(incident: dict, event: dict, em_type: str, when: str):
    log_event({"server_time": when, "type": "email_send_start", "em_type": em_type})
    try:
        send_email_alert(event, to_emails=get_email_recipients(em_type))
        log_event({"server_time": when, "type": "email_send_ok"})
        incident["email_status"] = "ok"
    except Exception as e:
        log_event({"server_time": when, "type": "email_failed", "error": str(e)})
        incident["email_status"] = "failed"
        incident["email_error"] = str(e)


def _dispatch_sms(incident: dict, event: dict, em_type: str, when: str):
    log_event({"server_time": when, "type": "sms_send_start", "em_type": em_type})
    try:
        sms_res = send_sms_alert(event, to_number=get_sms_recipients(em_type))
        log_event({"server_time": when, "type": "sms_send_ok", "result": sms_res})
        incident["sms_status"] = "ok"
        incident["sms_result"] = str(sms_res)
    except Exception as e:
        log_event({"server_time": when, "type": "sms_failed", "error": str(e)})
        incident["sms_status"] = "failed"
        incident["sms_error"] = str(e)


def _dispatch_relay(incident: dict, em_type: str, when: str):
    """Start the relay siren for mass/drill emergencies only."""
    if em_type not in ("mass", "drill"):
        log_event({"server_time": when, "type": "relay_skip", "reason": "personal"})
        incident["relay_status"] = "skipped"
        return

    log_event({"server_time": when, "type": "relay_loop_queued"})
    try:
        start_relay_loop()
        incident["relay_status"] = "active"
    except Exception as e:
        log_event({"server_time": when, "type": "relay_loop_failed", "error": str(e)})
        incident["relay_status"] = "failed"
        incident["relay_error"] = str(e)


def _persist_incident(incident: dict, when: str):
    try:
        insert_incident(incident)
        log_event({"server_time": when, "type": "sqlite_incident_ok"})
    except Exception as e:
        log_event({"server_time": when, "type": "sqlite_incident_failed", "error": str(e)})


def handle_payload(payload: dict, addr):
    """Process one decoded payload: dispatch alerts and record the incident."""
    device_id = payload.get("device_id") or payload.get("pico_id") or addr[0]
    em_flag = payload.get("emergency", False)

    log_event({
        "server_time": now_str(),
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
    src = payload.get("trigger_source", "unknown")
    when = now_str()

    key = f"{device_id}|{em_type}|{src}"
    if _is_rate_limited(key, time.time()):
        return

    triggered_by = get_triggered_by(device_id)

    event = {
        "server_time": when,
        "device_id": device_id,
        "emergency_type": em_type,
        "message": msg,
        "from_ip": addr[0],
        "from_port": addr[1],
        "pico_ts": ts,
        "triggered_by": triggered_by,
        "raw": payload,
    }

    payload_with_location = _attach_location(event, payload, when)

    log_event(event)

    incident = {
        "server_time": when,
        "device_id": device_id,
        "emergency_type": em_type,
        "trigger_source": src,
        "message": msg,
        "triggered_by": triggered_by,
        "from_ip": addr[0],
        "from_port": addr[1],
        "pico_ts": ts,
        "raw_json": json.dumps(payload_with_location, ensure_ascii=False),
    }

    _dispatch_audio(incident, em_type, when)
    _dispatch_email(incident, event, em_type, when)
    _dispatch_sms(incident, event, em_type, when)
    _dispatch_relay(incident, em_type, when)
    _persist_incident(incident, when)
