import math


def rssi_to_distance(rssi, measured_power=-59, path_loss_exponent=2.0):
    """
    Convert RSSI value to estimated distance.

    Args:
        rssi: Current RSSI value, for example -70
        measured_power: RSSI value measured at 1 meter
        path_loss_exponent: Environmental path loss factor

    Returns:
        Estimated distance in meters
    """

    if rssi is None:
        raise ValueError("RSSI value is missing.")

    if rssi == 0:
        raise ValueError("RSSI value cannot be 0.")

    distance = 10 ** ((measured_power - rssi) / (10 * path_loss_exponent))
    return distance


def trilaterate_3_points(p1, p2, p3):
    """
    Estimate device location using 3 known gateway points.

    Each point should contain:
    {
        "gateway_id": "gw1",
        "x": 0.0,
        "y": 0.0,
        "distance": 5.0
    }

    Returns:
        {
            "x": estimated_x,
            "y": estimated_y
        }
    """

    x1, y1, r1 = p1["x"], p1["y"], p1["distance"]
    x2, y2, r2 = p2["x"], p2["y"], p2["distance"]
    x3, y3, r3 = p3["x"], p3["y"], p3["distance"]

    A = 2 * (x2 - x1)
    B = 2 * (y2 - y1)
    C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2

    D = 2 * (x3 - x1)
    E = 2 * (y3 - y1)
    F = r1**2 - r3**2 - x1**2 + x3**2 - y1**2 + y3**2

    denominator = A * E - B * D

    if abs(denominator) < 1e-9:
        raise ValueError("Gateway points are collinear or too close to a straight line.")

    x = (C * E - B * F) / denominator
    y = (A * F - C * D) / denominator

    return {
        "x": x,
        "y": y
    }


def estimate_location_from_rssi(
    gateways,
    measured_power=-59,
    path_loss_exponent=2.0,
    map_width=None,
    map_height=None
):
    """
    Estimate device location from gateway coordinates and RSSI values.

    Args:
        gateways: list of gateway dictionaries.
        Example:
        [
            {"gateway_id": "gw1", "x": 0, "y": 0, "rssi": -73},
            {"gateway_id": "gw2", "x": 10, "y": 0, "rssi": -75},
            {"gateway_id": "gw3", "x": 0, "y": 10, "rssi": -79}
        ]

        measured_power: RSSI at 1 meter
        path_loss_exponent: environment factor
        map_width: optional maximum x boundary
        map_height: optional maximum y boundary

    Returns:
        {
            "x": estimated_x,
            "y": estimated_y,
            "gateways_used": [...],
            "success": True
        }
    """

    if gateways is None or len(gateways) < 3:
        raise ValueError("At least 3 gateways are required for 2D trilateration.")

    gateways_with_distance = []

    for gw in gateways[:3]:
        if "x" not in gw or "y" not in gw:
            raise ValueError(f"Gateway {gw.get('gateway_id', 'unknown')} is missing x or y coordinate.")

        if "rssi" not in gw:
            raise ValueError(f"Gateway {gw.get('gateway_id', 'unknown')} is missing RSSI value.")

        distance = rssi_to_distance(
            gw["rssi"],
            measured_power=measured_power,
            path_loss_exponent=path_loss_exponent
        )

        gw_copy = dict(gw)
        gw_copy["distance"] = distance
        gateways_with_distance.append(gw_copy)

    result = trilaterate_3_points(
        gateways_with_distance[0],
        gateways_with_distance[1],
        gateways_with_distance[2]
    )

    estimated_x = result["x"]
    estimated_y = result["y"]

    # Optional map boundary limit
    if map_width is not None:
        estimated_x = max(0, min(estimated_x, map_width))

    if map_height is not None:
        estimated_y = max(0, min(estimated_y, map_height))

    return {
        "x": estimated_x,
        "y": estimated_y,
        "gateways_used": gateways_with_distance,
        "success": True
    }


def calculate_distance_error(x, y, gateways):
    """
    Calculate distance error for checking result quality.
    """

    errors = []

    for gw in gateways:
        calculated_distance = math.sqrt((x - gw["x"]) ** 2 + (y - gw["y"]) ** 2)
        input_distance = gw.get("distance")

        if input_distance is None:
            continue

        errors.append({
            "gateway_id": gw.get("gateway_id", "unknown"),
            "calculated_distance": calculated_distance,
            "input_distance": input_distance,
            "error": calculated_distance - input_distance
        })

    return errors








if __name__ == "__main__":
    gateways = [
        {"gateway_id": "gw1", "x": 0.0, "y": 0.0, "rssi": -74},
        {"gateway_id": "gw2", "x": 10.0, "y": 0.0, "rssi": -76},
        {"gateway_id": "gw3", "x": 0.0, "y": 10.0, "rssi": -76},
    ]

    result = estimate_location_from_rssi(
        gateways,
        measured_power=-59,
        path_loss_exponent=2.0,
        map_width=10,
        map_height=10
    )

    print("Estimated location:")
    print(f"x = {result['x']:.2f} m")
    print(f"y = {result['y']:.2f} m")

    print("\nGateways used:")
    for gw in result["gateways_used"]:
        print(
            gw["gateway_id"],
            "RSSI:", gw["rssi"],
            "distance:", round(gw["distance"], 2), "m"
        )

    errors = calculate_distance_error(
        result["x"],
        result["y"],
        result["gateways_used"]
    )

    print("\nDistance errors:")
    for e in errors:
        print(e)
