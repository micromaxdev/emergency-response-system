"""Safe location estimation wrapper.

This module estimates the device location for an emergency payload.

Preferred backend flow:
- BLE MQTT runtime runs in the background and keeps latest RSSI per BLE MAC.
- When an emergency payload arrives, safe_estimate_location() looks up the
  latest RSSI values for the relevant BLE MAC and returns x/y.
"""

import traceback
import numpy as np
from scipy.optimize import minimize

from event_log import log_event

try:
    from ble_location_runtime import estimate_location_for_mac
    BLE_LOCATION_AVAILABLE = True
    BLE_LOCATION_IMPORT_ERROR = None
except Exception as e:
    estimate_location_for_mac = None
    BLE_LOCATION_AVAILABLE = False
    BLE_LOCATION_IMPORT_ERROR = str(e)


def _norm_mac(mac):
    if mac is None:
        return None
    return str(mac).lower().strip().replace(":", "")


def _rssi_to_distance(rssi, measured_power=-42, path_loss_exponent=3.5):
    return max(0.1, 10 ** ((measured_power - float(rssi)) / (10.0 * path_loss_exponent)))


def _estimate_from_payload_gateways(payload: dict):
    """Fallback method: estimate location from gateways included in the payload."""
    gateways = payload.get("gateways")

    if not isinstance(gateways, list) or len(gateways) < 2:
        return None

    measured_power = payload.get("measured_power", -42)
    path_loss_exponent = payload.get("path_loss_exponent", 3.5)
    map_width = payload.get("map_width", 18.0)
    map_height = payload.get("map_height", 23.0)

    usable = []

    for gw in gateways:
        if not isinstance(gw, dict):
            continue

        if "x" not in gw or "y" not in gw or "rssi" not in gw:
            continue

        d = _rssi_to_distance(
            gw["rssi"],
            measured_power=measured_power,
            path_loss_exponent=path_loss_exponent,
        )

        usable.append({
            "gateway_id": gw.get("gateway_id", "unknown"),
            "x": float(gw["x"]),
            "y": float(gw["y"]),
            "rssi": float(gw["rssi"]),
            "distance": d,
        })

    if len(usable) < 2:
        return None

    def centroid():
        total_weight = 0.0
        wx = 0.0
        wy = 0.0

        for gw in usable:
            w = 10 ** (gw["rssi"] / 10.0)
            wx += w * gw["x"]
            wy += w * gw["y"]
            total_weight += w

        if total_weight == 0:
            return None

        return (wx / total_weight, wy / total_weight)

    def trilaterate():
        if len(usable) < 3:
            return None

        def error(pos):
            x, y = pos
            total = 0.0

            for gw in usable:
                predicted_d = np.sqrt((x - gw["x"]) ** 2 + (y - gw["y"]) ** 2)
                total += (predicted_d - gw["distance"]) ** 2

            return total

        x0 = np.mean([gw["x"] for gw in usable])
        y0 = np.mean([gw["y"] for gw in usable])

        result = minimize(
            error,
            [x0, y0],
            method="Nelder-Mead",
            options={"xatol": 0.01, "fatol": 0.01, "maxiter": 1000},
        )

        x, y = result.x
        return (x, y)

    c = centroid()
    t = trilaterate()

    if c is None and t is None:
        return None

    if t is None:
        x, y = c
    elif c is None:
        x, y = t
    else:
        x = 0.6 * t[0] + 0.4 * c[0]
        y = 0.6 * t[1] + 0.4 * c[1]

    x = max(0, min(float(map_width), float(x)))
    y = max(0, min(float(map_height), float(y)))

    distance_errors = []

    for gw in usable:
        estimated_distance = np.sqrt((x - gw["x"]) ** 2 + (y - gw["y"]) ** 2)
        distance_errors.append({
            "gateway_id": gw["gateway_id"],
            "rssi": gw["rssi"],
            "rssi_distance": round(gw["distance"], 2),
            "estimated_distance": round(float(estimated_distance), 2),
            "error": round(float(estimated_distance - gw["distance"]), 2),
        })

    return {
        "x": round(float(x), 2),
        "y": round(float(y), 2),
        "gateways_used": usable,
        "distance_errors": distance_errors,
        "source": "payload_gateways",
    }


def _find_ble_mac(payload: dict):
    """Find the BLE MAC used for RSSI lookup."""
    candidates = [
        payload.get("ble_mac"),
        payload.get("tag_mac"),
        payload.get("device_mac"),
        payload.get("target_mac"),
        payload.get("device_id"),
        payload.get("pico_id"),
    ]

    for value in candidates:
        value = _norm_mac(value)

        if value and len(value) == 12:
            return value

    return None


def safe_estimate_location(payload: dict, now_str: str):
    """Estimate location from BLE MQTT cache using BLE MAC in payload."""
    ble_mac = _find_ble_mac(payload)

    if not ble_mac:
        log_event({
            "server_time": now_str,
            "type": "location_skipped",
            "reason": "No BLE MAC found in payload",
            "payload_keys": list(payload.keys()) if isinstance(payload, dict) else None,
        })
        return None

    if not BLE_LOCATION_AVAILABLE or estimate_location_for_mac is None:
        log_event({
            "server_time": now_str,
            "type": "ble_location_unavailable",
            "reason": str(BLE_LOCATION_IMPORT_ERROR),
            "ble_mac": ble_mac,
        })
        return None

    try:
        location = estimate_location_for_mac(ble_mac)
    except Exception as e:
        log_event({
            "server_time": now_str,
            "type": "location_error",
            "reason": str(e),
            "ble_mac": ble_mac,
        })
        return None

    if not location:
        log_event({
            "server_time": now_str,
            "type": "location_skipped",
            "reason": "No fresh BLE gateway RSSI available",
            "ble_mac": ble_mac,
        })
        return None

    location["source"] = "ble_mqtt_cache"
    location["ble_mac"] = ble_mac

    log_event({
        "server_time": now_str,
        "type": "location_estimated",
        "source": "ble_mqtt_cache",
        "ble_mac": ble_mac,
        "x": location.get("x"),
        "y": location.get("y"),
        "gateways_used": location.get("gateways_used", []),
        "distance_errors": location.get("distance_errors", []),
    })

    return location
