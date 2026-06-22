"""
BLE MQTT RSSI runtime cache + hybrid location estimation.

This module is designed for the backend server, not for GUI display.
It listens to BLE gateway MQTT topics in the background, stores the latest
RSSI per tag/gateway, and provides estimate_location_for_mac().
"""

import json
import time
import threading
from collections import defaultdict

import numpy as np
from scipy.optimize import minimize
import paho.mqtt.client as mqtt

from event_log import log_event


# ----------------------------
# MQTT / BLE Gateway Config
# ----------------------------

BROKER_HOST = "192.168.115.49"
BROKER_PORT = 1883
BROKER_USER = ""
BROKER_PASS = ""

GATEWAY_POSITIONS = {
    "ac233fc25932": [4.0, 20.5],
    "ac233fc26ecb": [15.4, 20.1],
    "ac233fc26ecc": [15.3, 1.4],
    "2805a55eedec": [1.9, 3.7],
}

GATEWAY_TOPICS = {
    "ac233fc25932": {
        "topic": "/gw/ac233fc25932/status",
        "fmt": "list_mac_rssi",
    },
    "ac233fc26ecb": {
        "topic": "/gw/ac233fc26ecb/status",
        "fmt": "list_mac_rssi",
    },
    "ac233fc26ecc": {
        "topic": "/gw/ac233fc26ecc/status",
        "fmt": "list_mac_rssi",
    },
    "2805a55eedec": {
        "topic": "/MKGW3/2805a55eedec/send",
        "fmt": "mkgw3_nested",
    },
}

TX_POWER = -42
PATH_LOSS_EXP = 3.5

ROOM_W = 18.0
ROOM_H = 23.0

GATEWAY_STALE_SEC = 5.0

KALMAN_R = 8.0
KALMAN_Q = 0.1

GATEWAY_WALL_WEIGHT = {
    "ac233fc25932": 1.0,
    "ac233fc26ecb": 1.0,
    "ac233fc26ecc": 1.0,
    "2805a55eedec": 1.0,
}


# ----------------------------
# Runtime State
# ----------------------------

_lock = threading.Lock()
_started = False
_clients = []

# Shape:
# LATEST_RSSI["e7b89a2ee18a"]["ac233fc25932"] = {
#     "rssi": -60.5,
#     "raw_rssi": -61,
#     "ts": 1781572313.123
# }
LATEST_RSSI = defaultdict(dict)


# ----------------------------
# Kalman Filter
# ----------------------------

class RSSIKalman:
    def __init__(self, R=KALMAN_R, Q=KALMAN_Q, initial=-70.0):
        self.R = R
        self.Q = Q
        self.x = initial
        self.P = R
        self.initialized = False

    def update(self, z):
        if not self.initialized:
            self.x = float(z)
            self.initialized = True
        else:
            self.P += self.Q
            K = self.P / (self.P + self.R)
            self.x += K * (float(z) - self.x)
            self.P = (1 - K) * self.P
        return self.x


# Shape:
# KALMAN_FILTERS["e7b89a2ee18a"]["ac233fc25932"] = RSSIKalman()
KALMAN_FILTERS = defaultdict(dict)


def _norm_mac(mac: str) -> str:
    return str(mac).lower().strip().replace(":", "")


def _get_kalman(mac: str, gateway_id: str) -> RSSIKalman:
    if gateway_id not in KALMAN_FILTERS[mac]:
        KALMAN_FILTERS[mac][gateway_id] = RSSIKalman()
    return KALMAN_FILTERS[mac][gateway_id]


# ----------------------------
# MQTT Payload Parsing
# ----------------------------

def parse_entries(payload: bytes, fmt: str):
    """Return list of (mac, rssi) tuples from gateway MQTT payload."""
    out = []

    try:
        data = json.loads(payload.decode("utf-8", errors="replace"))
    except Exception:
        return out

    if fmt == "list_mac_rssi":
        entries = data if isinstance(data, list) else [data]

        for entry in entries:
            if not isinstance(entry, dict):
                continue

            # Skip gateway metadata object, e.g. {"gateway": "...", "timestamp": "..."}
            if "mac" in entry and entry.get("rssi") is not None:
                out.append((entry["mac"], entry["rssi"]))

    elif fmt == "mkgw3_nested":
        readings = data.get("data") if isinstance(data, dict) else None

        if isinstance(readings, list):
            for entry in readings:
                if not isinstance(entry, dict):
                    continue

                if "mac" in entry and entry.get("rssi") is not None:
                    out.append((entry["mac"], entry["rssi"]))

    return out


def _update_rssi(mac: str, gateway_id: str, rssi):
    mac_n = _norm_mac(mac)
    now_ts = time.time()

    with _lock:
        kf = _get_kalman(mac_n, gateway_id)
        filtered_rssi = kf.update(float(rssi))

        LATEST_RSSI[mac_n][gateway_id] = {
            "rssi": filtered_rssi,
            "raw_rssi": float(rssi),
            "ts": now_ts,
        }


# ----------------------------
# Positioning Algorithms
# ----------------------------

def rssi_to_distance(rssi, tx=TX_POWER, n=PATH_LOSS_EXP):
    return max(0.1, 10 ** ((tx - float(rssi)) / (10.0 * n)))


def weighted_centroid(rssi_vals):
    total_weight = 0.0
    wx = 0.0
    wy = 0.0

    for gw_id, rssi in rssi_vals.items():
        if gw_id not in GATEWAY_POSITIONS:
            continue

        linear_weight = 10 ** (float(rssi) / 10.0)
        wall_w = GATEWAY_WALL_WEIGHT.get(gw_id, 1.0)
        w = linear_weight * wall_w

        gx, gy = GATEWAY_POSITIONS[gw_id]
        wx += w * gx
        wy += w * gy
        total_weight += w

    if total_weight == 0:
        return None

    return (wx / total_weight, wy / total_weight)


def trilaterate(distances):
    items = [(gid, d) for gid, d in distances.items() if d is not None]

    if len(items) < 3:
        return None

    def error(pos):
        x, y = pos
        total = 0.0

        for gid, d in items:
            gx, gy = GATEWAY_POSITIONS[gid]
            predicted_d = np.sqrt((x - gx) ** 2 + (y - gy) ** 2)
            total += (predicted_d - d) ** 2

        return total

    x0 = np.mean([GATEWAY_POSITIONS[gid][0] for gid, _ in items])
    y0 = np.mean([GATEWAY_POSITIONS[gid][1] for gid, _ in items])

    result = minimize(
        error,
        [x0, y0],
        method="Nelder-Mead",
        options={"xatol": 0.01, "fatol": 0.01, "maxiter": 1000},
    )

    x, y = result.x
    x = max(0, min(ROOM_W, float(x)))
    y = max(0, min(ROOM_H, float(y)))

    return (x, y)


def rssi_consistency(rssi_vals):
    vals = list(rssi_vals.values())

    if len(vals) < 2:
        return 0.5

    variance = np.var(vals)

    return max(0.0, min(1.0, 1.0 - variance / 200.0))


def hybrid_position(rssi_vals, distances):
    centroid = weighted_centroid(rssi_vals)
    trilat = trilaterate(distances)

    if centroid is None and trilat is None:
        return None

    if trilat is None:
        x, y = centroid
        return (max(0, min(ROOM_W, x)), max(0, min(ROOM_H, y)))

    if centroid is None:
        return trilat

    consistency = rssi_consistency(rssi_vals)
    trilat_weight = consistency * 0.7

    x = trilat_weight * trilat[0] + (1 - trilat_weight) * centroid[0]
    y = trilat_weight * trilat[1] + (1 - trilat_weight) * centroid[1]

    x = max(0, min(ROOM_W, x))
    y = max(0, min(ROOM_H, y))

    return (x, y)


def calculate_distance_errors(x, y, gateways_used):
    errors = []

    for gw in gateways_used:
        gw_id = gw.get("gateway_id")

        if gw_id not in GATEWAY_POSITIONS:
            continue

        gx, gy = GATEWAY_POSITIONS[gw_id]
        estimated_distance = np.sqrt((x - gx) ** 2 + (y - gy) ** 2)
        rssi_distance = gw.get("distance")

        if rssi_distance is None:
            continue

        errors.append({
            "gateway_id": gw_id,
            "rssi": gw.get("rssi"),
            "rssi_distance": round(float(rssi_distance), 2),
            "estimated_distance": round(float(estimated_distance), 2),
            "error": round(float(estimated_distance - rssi_distance), 2),
        })

    return errors


def get_latest_gateways_for_mac(mac: str):
    mac_n = _norm_mac(mac)
    now_ts = time.time()
    gateways = []

    with _lock:
        per_gw = dict(LATEST_RSSI.get(mac_n, {}))

    for gw_id, item in per_gw.items():
        age = now_ts - item["ts"]

        if age > GATEWAY_STALE_SEC:
            continue

        gx, gy = GATEWAY_POSITIONS[gw_id]
        rssi = item["rssi"]
        dist = rssi_to_distance(rssi)

        gateways.append({
            "gateway_id": gw_id,
            "x": gx,
            "y": gy,
            "rssi": rssi,
            "raw_rssi": item.get("raw_rssi"),
            "distance": dist,
            "age_sec": round(age, 2),
        })

    return gateways


def estimate_location_for_mac(mac: str):
    gateways = get_latest_gateways_for_mac(mac)

    if len(gateways) < 2:
        return None

    rssi_vals = {
        gw["gateway_id"]: gw["rssi"]
        for gw in gateways
    }

    distances = {
        gw["gateway_id"]: gw["distance"]
        for gw in gateways
    }

    pos = hybrid_position(rssi_vals, distances)

    if pos is None:
        return None

    x, y = pos
    errors = calculate_distance_errors(x, y, gateways)

    return {
        "x": round(float(x), 2),
        "y": round(float(y), 2),
        "gateways_used": gateways,
        "distance_errors": errors,
    }


# ----------------------------
# MQTT Runtime
# ----------------------------

def _make_client(gateway_id: str):
    cfg = GATEWAY_TOPICS[gateway_id]
    topic = cfg["topic"]
    fmt = cfg["fmt"]

    def on_connect(client, userdata, flags, reason_code=None, properties=None):
        if reason_code is None or reason_code == 0 or str(reason_code) == "Success":
            client.subscribe(topic)
            log_event({
                "type": "ble_mqtt_subscribed",
                "gateway_id": gateway_id,
                "topic": topic,
            })
        else:
            log_event({
                "type": "ble_mqtt_connect_failed",
                "gateway_id": gateway_id,
                "reason": str(reason_code),
            })

    def on_message(client, userdata, msg):
        entries = parse_entries(msg.payload, fmt)

        for mac, rssi in entries:
            _update_rssi(mac, gateway_id, rssi)

    try:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"server-ble-{gateway_id}",
        )
    except AttributeError:
        client = mqtt.Client(client_id=f"server-ble-{gateway_id}")

    client.on_connect = on_connect
    client.on_message = on_message

    if BROKER_USER:
        client.username_pw_set(BROKER_USER, BROKER_PASS)

    return client


def start_ble_location_runtime():
    """Start background MQTT listeners once."""
    global _started, _clients

    if _started:
        return

    _started = True

    for gateway_id in GATEWAY_POSITIONS:
        client = _make_client(gateway_id)

        try:
            client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
            client.loop_start()
            _clients.append(client)
        except Exception as e:
            log_event({
                "type": "ble_mqtt_connection_error",
                "gateway_id": gateway_id,
                "error": str(e),
            })


def debug_cache_snapshot(mac: str = None):
    """Optional helper for debugging."""
    with _lock:
        if mac:
            return dict(LATEST_RSSI.get(_norm_mac(mac), {}))
        return {k: dict(v) for k, v in LATEST_RSSI.items()}
