"""ERS LoRa frame v1 - pure encode/decode, shared by the bridge, tests and tooling.

    byte  0      1..2         3..6           7       8..9
         [0xE1] [site_id 2B] [device_id 4B] [event] [seq hi|lo]      = 10 bytes

0xE1 is a LoRaWAN *proprietary* MHDR (111xxxxx), so real LoRaWAN servers ignore
our frames and we ignore theirs. Low bits carry the version; phase 2 uses 0xE2
and appends [counter 2B][MIC 4B].

The Pico firmware (client/pico_node/Working Code/identity.py) builds the same
bytes; tests/test_frame.py keeps the two in sync.
"""

import hashlib

MHDR_V1 = 0xE1
FRAME_V1_LEN = 10

EVENT_META = {
    1: {"emergency": True, "emergency_type": "mass",
        "emergency_message": "MASS EMERGENCY TRIGGERED (5 rapid presses)"},
    2: {"emergency": True, "emergency_type": "personal",
        "emergency_message": "MANUAL EMERGENCY BUTTON (hold 5s)"},
    3: {"emergency": True, "emergency_type": "personal",
        "emergency_message": "FALL DETECTED"},
    99: {"emergency": False, "emergency_type": "heartbeat",
         "emergency_message": "HEARTBEAT"},
}


def device_id_from_serial(unique_id: bytes) -> str:
    """ERS-XXXXXXXX: first 4 bytes of SHA-256(machine.unique_id()), upper-case hex."""
    return "ERS-" + hashlib.sha256(unique_id).digest()[:4].hex().upper()


def device_id_to_bytes(device_id: str) -> bytes:
    return bytes.fromhex(device_id.split("-", 1)[1])


def encode_v1(site_id: int, device_id: str, event_code: int, seq: int) -> bytes:
    seq &= 0xFFFF
    return (bytes([MHDR_V1]) + (site_id & 0xFFFF).to_bytes(2, "big")
            + device_id_to_bytes(device_id)
            + bytes([event_code & 0xFF, seq >> 8, seq & 0xFF]))


def decode_v1(raw: bytes):
    """Return a dict for a valid v1 frame, else None (foreign/legacy/corrupt)."""
    if len(raw) != FRAME_V1_LEN or raw[0] != MHDR_V1:
        return None
    return {
        "site_id": int.from_bytes(raw[1:3], "big"),
        "device_id": "ERS-" + raw[3:7].hex().upper(),
        "event_code": raw[7],
        "seq": (raw[8] << 8) | raw[9],
    }
