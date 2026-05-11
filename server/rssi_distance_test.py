import math


def rssi_to_distance(rssi, measured_power=-59, path_loss_exponent=2.0):
    """
    Convert RSSI value to estimated distance.

    rssi: current RSSI value, e.g. -70
    measured_power: RSSI value at 1 meter, e.g. -59
    path_loss_exponent: environment factor, usually 2.0 to 3.5 indoors
    """

    if rssi == 0:
        return None

    distance = 10 ** ((measured_power - rssi) / (10 * path_loss_exponent))
    return distance


if __name__ == "__main__":
    print("=== RSSI to Distance Test ===")

    test_rssi_values = [-50, -59, -65, -70, -75, -80, -85, -90]

    measured_power = -59
    path_loss_exponent = 2.0

    print(f"Measured Power at 1 meter: {measured_power} dBm")
    print(f"Path Loss Exponent: {path_loss_exponent}")
    print()

    for rssi in test_rssi_values:
        distance = rssi_to_distance(
            rssi,
            measured_power=measured_power,
            path_loss_exponent=path_loss_exponent
        )

        print(f"RSSI = {rssi} dBm -> estimated distance = {distance:.2f} m")
