"""Site-wide status banners: Internet offline, devices under test, unregistered devices."""

import socket
import time

from .devices import fetch_devices_in_test, fetch_unregistered_incidents

__all__ = ["internet_online", "status_banners_html"]

_CACHE = {"t": 0.0, "ok": True}
_CACHE_TTL_S = 20


def internet_online(timeout_s: float = 1.5) -> bool:
    """True if the Pi can reach the internet (TCP 443). Cached so reruns stay cheap.

    SMS (Twilio), email (SMTP) and NTP all need the wired uplink; when this is
    False the sirens, relay, dashboard and logging still work (decision D4).
    """
    now = time.time()
    if now - _CACHE["t"] < _CACHE_TTL_S:
        return _CACHE["ok"]
    ok = False
    for host in ("1.1.1.1", "8.8.8.8"):
        try:
            with socket.create_connection((host, 443), timeout=timeout_s):
                ok = True
                break
        except OSError:
            continue
    _CACHE.update(t=now, ok=ok)
    return ok


def _banner(color: str, title: str, detail: str) -> str:
    return (f'<div style="background:rgba({color},0.10);border:1px solid rgba({color},0.45);'
            f'border-left:6px solid rgb({color});border-radius:8px;padding:10px 16px;margin-bottom:10px;">'
            f'<strong>{title}</strong> <span style="opacity:0.8">{detail}</span></div>')


def status_banners_html() -> str:
    """HTML for all currently relevant banners ('' when everything is normal)."""
    parts = []
    if not internet_online():
        parts.append(_banner("230,57,70", "Internet: OFFLINE",
                             "SMS and email alerts cannot be sent. Sirens, relay, dashboard and logging still work. "
                             "Check the Ethernet cable."))
    testing = fetch_devices_in_test()
    if testing:
        names = ", ".join(d.get("label") or d["device_id"] for d in testing)
        parts.append(_banner("247,183,49", "Devices in test mode",
                             f"{names}: presses are recorded as TEST and do NOT trigger sirens, relay, SMS or email."))
    unreg = fetch_unregistered_incidents(hours=24)
    if unreg:
        ids = sorted({u["device_id"] for u in unreg})
        parts.append(_banner("230,57,70", "Unregistered device(s) triggered an alert",
                             f"{', '.join(ids)} (last 24 h). Import the device list or register them under "
                             "Configuration. No siren or staff SMS was sent."))
    return "".join(parts)
