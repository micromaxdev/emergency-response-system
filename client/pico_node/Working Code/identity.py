# identity.py - device identity + LoRa frame v1 for the ERS Pico node.
#
# Runs on MicroPython (Pico 2 W / RP2350) and on desktop CPython (unit tests).
#
#   DEVICE_ID  = "ERS-" + first 4 bytes of SHA-256(machine.unique_id()), hex
#                -> derived from the RP2350 factory serial: nothing is typed by
#                   hand, it survives reflashing and cannot drift.
#   device.json = per-device/site data written by tools/provision_pico.py
#                 (site id, hotspot Wi-Fi credentials, server address).
#
# LoRa frame v1 (10 bytes), identical to server/lora/frame.py:
#   [0xE1][site_id 2B][device_id 4B][event][seq hi][seq lo]

try:
    import ujson as json
except ImportError:
    import json
try:
    import hashlib
except ImportError:
    import uhashlib as hashlib
try:
    import ubinascii as binascii
except ImportError:
    import binascii

FW_VERSION = "1.0.0"
MHDR_V1 = 0xE1
CONFIG_FILE = "device.json"

DEFAULT_CONFIG = {
    "site_id": 0,                 # 0 = not provisioned; the server drops/flags it
    "wifi_ssid": "",
    "wifi_pass": "",
    "server_ip": "10.42.0.1",     # the Pi's private hotspot address
    "server_port": 8081,
}


def hw_serial_hex(unique_id=None):
    """The raw hardware serial as hex (stored as hw_serial in the server DB)."""
    if unique_id is None:
        import machine
        unique_id = machine.unique_id()
    return binascii.hexlify(unique_id).decode().upper()


def device_id_bytes(unique_id=None):
    if unique_id is None:
        import machine
        unique_id = machine.unique_id()
    return hashlib.sha256(unique_id).digest()[:4]


def device_id(unique_id=None):
    """ERS-XXXXXXXX (upper-case hex). Stable across reboots and reflashes."""
    return "ERS-" + binascii.hexlify(device_id_bytes(unique_id)).decode().upper()


def load_config(path=CONFIG_FILE):
    """Read device.json, falling back to defaults for anything missing."""
    cfg = dict(DEFAULT_CONFIG)
    try:
        with open(path) as f:
            cfg.update(json.load(f))
    except (OSError, ValueError):
        print("[identity] no usable", path, "- running unprovisioned")
    return cfg


def encode_frame_v1(site_id, dev_id_bytes, event_code, seq):
    seq &= 0xFFFF
    return bytes([
        MHDR_V1,
        (site_id >> 8) & 0xFF, site_id & 0xFF,
        dev_id_bytes[0], dev_id_bytes[1], dev_id_bytes[2], dev_id_bytes[3],
        event_code & 0xFF,
        (seq >> 8) & 0xFF, seq & 0xFF,
    ])
