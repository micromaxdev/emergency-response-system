from trilateration import estimate_location_from_rssi, calculate_distance_error


if __name__ == "__main__":
    print("=== Trilateration Module Test ===")
    print()

    gateways = [
        {
            "gateway_id": "gw1",
            "x": 0.0,
            "y": 0.0,
            "rssi": -73
        },
        {
            "gateway_id": "gw2",
            "x": 10.0,
            "y": 0.0,
            "rssi": -75
        },
        {
            "gateway_id": "gw3",
            "x": 0.0,
            "y": 10.0,
            "rssi": -79
        }
    ]

    try:
        result = estimate_location_from_rssi(
            gateways,
            measured_power=-59,
            path_loss_exponent=2.0,
            map_width=10.0,
            map_height=10.0
        )

        print("=== Estimated Device Location ===")
        print(f"x = {result['x']:.3f} m")
        print(f"y = {result['y']:.3f} m")
        print()

        print("=== Gateway Distance Conversion ===")

        for gw in result["gateways_used"]:
            print(
                f"{gw['gateway_id']}: "
                f"RSSI = {gw['rssi']} dBm, "
                f"distance = {gw['distance']:.2f} m, "
                f"coordinate = ({gw['x']}, {gw['y']})"
            )

        print()

        print("=== Distance Error Check ===")

        errors = calculate_distance_error(
            result["x"],
            result["y"],
            result["gateways_used"]
        )

        for item in errors:
            print(
                f"{item['gateway_id']}: "
                f"calculated = {item['calculated_distance']:.2f} m, "
                f"input = {item['input_distance']:.2f} m, "
                f"error = {item['error']:.2f} m"
            )

    except Exception as e:
        print("Test failed:")
        print(e)

