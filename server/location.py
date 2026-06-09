"""Optional RSSI trilateration (experimental localisation).

Wraps ``trilateration.py`` so the rest of the server can estimate a device's
position from gateway RSSI readings without depending on it being importable.
"""

from event_log import log_event

try:
    from trilateration import estimate_location_from_rssi, calculate_distance_error
    TRILATERATION_AVAILABLE = True
    TRILATERATION_IMPORT_ERROR = None
except Exception as e:
    estimate_location_from_rssi = None
    calculate_distance_error = None
    TRILATERATION_AVAILABLE = False
    TRILATERATION_IMPORT_ERROR = str(e)


def safe_estimate_location(payload: dict, now_str: str):
    """Estimate device location from RSSI gateway data, or return None.

    Expected payload shape::

        {
            "gateways": [
                {"gateway_id": "GW1", "x": 0.0, "y": 0.0, "rssi": -73},
                {"gateway_id": "GW2", "x": 10.0, "y": 0.0, "rssi": -75},
                {"gateway_id": "GW3", "x": 0.0, "y": 10.0, "rssi": -79}
            ],
            "measured_power": -59,
            "path_loss_exponent": 2.0,
            "map_width": 10.0,
            "map_height": 10.0
        }

    Returns a location result dict (with ``distance_errors``) or None when the
    payload has no usable gateway data or estimation fails.
    """
    if not TRILATERATION_AVAILABLE:
        log_event({
            "server_time": now_str,
            "type": "trilateration_unavailable",
            "error": TRILATERATION_IMPORT_ERROR or "unknown import error",
        })
        return None

    gateways = payload.get("gateways")

    if not gateways:
        return None

    if not isinstance(gateways, list) or len(gateways) < 3:
        log_event({
            "server_time": now_str,
            "type": "location_skipped",
            "reason": "At least 3 gateways are required.",
            "gateways_received": gateways,
        })
        return None

    try:
        measured_power = payload.get("measured_power", -59)
        path_loss_exponent = payload.get("path_loss_exponent", 2.0)
        map_width = payload.get("map_width")
        map_height = payload.get("map_height")

        location = estimate_location_from_rssi(
            gateways,
            measured_power=measured_power,
            path_loss_exponent=path_loss_exponent,
            map_width=map_width,
            map_height=map_height,
        )

        errors = calculate_distance_error(
            location["x"],
            location["y"],
            location["gateways_used"],
        )

        location["distance_errors"] = errors

        log_event({
            "server_time": now_str,
            "type": "location_estimated",
            "x": location["x"],
            "y": location["y"],
            "gateways_used": location["gateways_used"],
            "distance_errors": errors,
        })

        return location

    except Exception as e:
        import traceback
        log_event({
            "server_time": now_str,
            "type": "location_estimation_failed",
            "error": str(e),
            "trace": traceback.format_exc(),
        })
        return None
