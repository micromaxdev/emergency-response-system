import os
import re
from datetime import datetime
from twilio.rest import Client

def _get_env(name: str, required: bool = True) -> str:
    val = os.getenv(name, "").strip()
    if required and not val:
        raise RuntimeError(f"Missing env var: {name}")
    return val

def _format_sms(event: dict) -> str:
    em_type = (event.get("emergency_type") or "unknown").upper()
    device = event.get("device_id") or "unknown_device"
    msg = event.get("message") or "EMERGENCY"
    t = event.get("server_time") or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return f"[ERS {em_type}] {msg}\nDevice: {device}\nTime: {t}"

def _parse_recipients(raw: str) -> list[str]:
    """
    Accept: "+614..., +614..." or "+614...;+614..." or with Chinese comma '，'
    Also tolerates whitespace/newlines.
    """
    if not raw:
        return []
    # Normalize common separators to comma
    raw = raw.replace("，", ",").replace(";", ",").replace("\n", ",").replace("\r", ",")
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    # Optional: de-dup while preserving order
    seen = set()
    out = []
    for p in parts:
        if p not in seen:
            out.append(p)
            seen.add(p)
    return out

def _is_e164(num: str) -> bool:
    # E.164: + and 8-15 digits total (country code included)
    return bool(re.fullmatch(r"\+[1-9]\d{7,14}", num))

def send_sms_alert(event: dict, to_number: str | None = None) -> dict:
    
    account_sid = _get_env("TWILIO_ACCOUNT_SID")
    auth_token = _get_env("TWILIO_AUTH_TOKEN")
    messaging_sid = _get_env("TWILIO_MESSAGING_SERVICE_SID")

    # If caller didn't pass to_number, read from SMS_TO
    raw_to = (to_number.strip() if to_number else _get_env("SMS_TO", required=True))
    recipients = _parse_recipients(raw_to)

    if not recipients:
        raise RuntimeError("No recipients found. Set SMS_TO or pass to_number.")

    # Validate numbers early (helps avoid Twilio 21211)
    bad = [n for n in recipients if not _is_e164(n)]
    if bad:
        raise RuntimeError(
            "Invalid phone number(s) (must be E.164 like +614xxxxxxxx). Bad: "
            + ", ".join(bad)
        )

    body = _format_sms(event)
    client = Client(account_sid, auth_token)

    results = []
    for to in recipients:
        m = client.messages.create(
            messaging_service_sid=messaging_sid,
            body=body,
            to=to,  # IMPORTANT: one recipient per request
        )
        results.append({
            "sid": m.sid,
            "to": to,
            "status": getattr(m, "status", None),
        })

    return {"count": len(results), "results": results}
