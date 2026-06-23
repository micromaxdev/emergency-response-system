import math


def trilaterate_3_points(p1, p2, p3):
    """
    Estimate device location using 3 known points and their distances.

    Each point should be:
    {
        "x": float,
        "y": float,
        "distance": float
    }
    """

    x1, y1, r1 = p1["x"], p1["y"], p1["distance"]
    x2, y2, r2 = p2["x"], p2["y"], p2["distance"]
    x3, y3, r3 = p3["x"], p3["y"], p3["distance"]

    # Convert circle equations into two linear equations
    # by subtracting equation 1 from equation 2 and 3.
    A = 2 * (x2 - x1)
    B = 2 * (y2 - y1)
    C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2

    D = 2 * (x3 - x1)
    E = 2 * (y3 - y1)
    F = r1**2 - r3**2 - x1**2 + x3**2 - y1**2 + y3**2

    denominator = A * E - B * D

    if abs(denominator) < 1e-9:
        raise ValueError("The three gateway points are collinear or too close to a straight line.")

    x = (C * E - B * F) / denominator
    y = (A * F - C * D) / denominator

    return x, y


def calculate_error(x, y, points):
    """
    Calculate how close the estimated point is to the input distances.
    """

    errors = []

    for i, p in enumerate(points, start=1):
        actual_distance = math.sqrt((x - p["x"]) ** 2 + (y - p["y"]) ** 2)
        expected_distance = p["distance"]
        error = actual_distance - expected_distance

        errors.append({
            "gateway": i,
            "actual_distance": actual_distance,
            "expected_distance": expected_distance,
            "error": error
        })

    return errors


if __name__ == "__main__":
    # Example test:
    # Assume the real device location is around (4, 3)

    gateway_1 = {
        "x": 0.0,
        "y": 0.0,
        "distance": 5.0
    }

    gateway_2 = {
        "x": 10.0,
        "y": 0.0,
        "distance": 6.708
    }

    gateway_3 = {
        "x": 0.0,
        "y": 10.0,
        "distance": 8.062
    }

    points = [gateway_1, gateway_2, gateway_3]

    try:
        estimated_x, estimated_y = trilaterate_3_points(gateway_1, gateway_2, gateway_3)

        print("=== Trilateration Test Result ===")
        print(f"Estimated device location: x = {estimated_x:.3f}, y = {estimated_y:.3f}")
        print()

        print("=== Distance Error Check ===")
        errors = calculate_error(estimated_x, estimated_y, points)

        for item in errors:
            print(
                f"Gateway {item['gateway']}: "
                f"calculated distance = {item['actual_distance']:.3f} m, "
                f"input distance = {item['expected_distance']:.3f} m, "
                f"error = {item['error']:.3f} m"
            )

    except Exception as e:
        print("Trilateration failed:")
        print(e)
