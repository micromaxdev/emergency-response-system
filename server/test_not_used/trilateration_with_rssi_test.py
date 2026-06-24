import math


def rssi_to_distance(rssi, measured_power=-59, path_loss_exponent=2.0):
    """
    Convert RSSI value to estimated distance.
    """

    if rssi == 0:
        return None

    return 10 ** ((measured_power - rssi) / (10 * path_loss_exponent))


def trilaterate_3_points(p1, p2, p3):
    """
    Estimate device location using 3 known gateway points and their distances.
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

    return x, y


def calculate_error(x, y, points):
    """
    Check distance error for each gateway.
    """

    errors = []

    for p in points:
        calculated_distance = math.sqrt((x - p["x"]) ** 2 + (y - p["y"]) ** 2)
        input_distance = p["distance"]
        error = calculated_distance - input_distance

        errors.append({
            "gateway_id": p["gateway_id"],
            "calculated_distance": calculated_distance,
            "input_distance": input_distance,
            "error": error
        })

    return errors


if __name__ == "__main__":
    print("=== Trilateration with RSSI Test ===")
    print()

    # These are example gateway coordinates on a simple 2D map.
    # Unit: meters
    gateway_1 = {
        "gateway_id": "gw1",
        "x": 0.0,
        "y": 0.0,
        "rssi": -73
    }

    gateway_2 = {
        "gateway_id": "gw2",
        "x": 10.0,
        "y": 0.0,
        "rssi": -76
    }

    gateway_3 = {
        "gateway_id": "gw3",
        "x": 0.0,
        "y": 10.0,
        "rssi": -81
    }

    measured_power = -59
    path_loss_exponent = 2.0

    gateways = [gateway_1, gateway_2, gateway_3]

    print("=== RSSI to Distance Conversion ===")

    for gw in gateways:
        distance = rssi_to_distance(
            gw["rssi"],
            measured_power=measured_power,
            path_loss_exponent=path_loss_exponent
        )

        gw["distance"] = distance

        print(
            f"{gw['gateway_id']}: "
            f"RSSI = {gw['rssi']} dBm -> "
            f"estimated distance = {distance:.2f} m"
        )

    print()

    try:
        estimated_x, estimated_y = trilaterate_3_points(
            gateway_1,
            gateway_2,
            gateway_3
        )

        print("=== Estimated Device Location ===")
        print(f"x = {estimated_x:.3f} m")
        print(f"y = {estimated_y:.3f} m")
        print()

        print("=== Distance Error Check ===")

        errors = calculate_error(estimated_x, estimated_y, gateways)

        for item in errors:
            print(
                f"{item['gateway_id']}: "
                f"calculated distance = {item['calculated_distance']:.2f} m, "
                f"input distance = {item['input_distance']:.2f} m, "
                f"error = {item['error']:.2f} m"
            )

    except Exception as e:
        print("Trilateration failed:")
        print(e)
