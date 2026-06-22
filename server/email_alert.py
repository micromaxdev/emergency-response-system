import os
import smtplib
import tempfile
from email.message import EmailMessage
from datetime import datetime, timezone
from dotenv import load_dotenv

# Use non-GUI backend for Raspberry Pi / server environment
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


# Load SMTP credentials from ~/.env
load_dotenv(os.path.expanduser("~/.env"))

EMAIL_HOST = os.environ["EMAIL_HOST"]
EMAIL_PORT = int(os.environ.get("EMAIL_PORT", "587"))
EMAIL_ADDRESS = os.environ["EMAIL_ADDRESS"]
EMAIL_PASSWORD = os.environ["EMAIL_PASSWORD"]

# Default recipients, comma-separated, e.g. EMAIL_TO=alice@example.com,bob@example.com
# Per-incident recipients are normally passed in by the caller instead.
EMAIL_TO = os.environ.get("EMAIL_TO", "").strip()

# Paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_FILE = os.path.join(BASE_DIR, "map.png")


# These should match your BLE positioning coordinate system.
# Your previous map loaded as 370x528px and gateway coordinates look like metres.
# Adjust these if your map scale is different.
MAP_WIDTH_M = 17.0
MAP_HEIGHT_M = 24.0


def _format_subject(event: dict) -> str:
    em_type = event.get("emergency_type", "unknown")
    device_id = event.get("device_id", "unknown")

    location = event.get("estimated_location") or {}
    room_name = location.get("room_name") or event.get("room_name")

    if room_name:
        return f"[ERS] {em_type.upper()} alert from {device_id} - {room_name}"

    return f"[ERS] {em_type.upper()} alert from {device_id}"


def _get_location(event: dict) -> dict:
    """Return estimated_location safely."""
    location = event.get("estimated_location")
    if isinstance(location, dict):
        return location
    return {}


def _format_body(event: dict) -> str:
    lines = []
    lines.append("Emergency Response System Alert")
    lines.append("-" * 40)

    server_time = event.get("server_time")
    if not server_time:
        server_time = datetime.now(timezone.utc).isoformat()

    location = _get_location(event)
    room_name = location.get("room_name") or event.get("room_name") or "Unknown area"
    x = location.get("x")
    y = location.get("y")
    ble_mac = location.get("ble_mac") or event.get("ble_mac")

    lines.append(f"Server Time: {server_time}")
    lines.append(f"Device ID: {event.get('device_id', '')}")
    lines.append(f"Emergency Type: {event.get('emergency_type', '')}")
    lines.append(f"Message: {event.get('message', '')}")
    lines.append(f"Pico Timestamp: {event.get('pico_ts', '')}")
    lines.append(f"From: {event.get('from_ip', '')}:{event.get('from_port', '')}")
    lines.append("")

    lines.append("Estimated Location")
    lines.append("-" * 40)
    lines.append(f"Room Name: {room_name}")

    if x is not None and y is not None:
        lines.append(f"Coordinates: x={x}, y={y}")

    if ble_mac:
        lines.append(f"BLE MAC: {ble_mac}")

    gateways = location.get("gateways_used") or []
    if gateways:
        lines.append("")
        lines.append("BLE Gateways Used:")
        for gw in gateways:
            gateway_id = gw.get("gateway_id", "")
            rssi = gw.get("rssi", "")
            raw_rssi = gw.get("raw_rssi", "")
            distance = gw.get("distance", "")
            age_sec = gw.get("age_sec", "")

            lines.append(
                f"- {gateway_id}: rssi={rssi}, raw={raw_rssi}, "
                f"distance={distance}, age={age_sec}s"
            )

    lines.append("")
    lines.append("Raw event:")
    lines.append(str(event))

    return "\n".join(lines)


def _create_location_map_image(event: dict) -> str | None:
    """Create a PNG map image with the estimated emergency location marked.

    Returns:
        path to temporary PNG file, or None if no location/map is available.
    """
    location = _get_location(event)

    x = location.get("x")
    y = location.get("y")
    room_name = location.get("room_name") or event.get("room_name") or "Unknown area"
    device_id = event.get("device_id", "unknown")
    em_type = event.get("emergency_type", "unknown")

    if x is None or y is None:
        return None

    if not os.path.exists(MAP_FILE):
        print(f"[EMAIL MAP] map file not found: {MAP_FILE}")
        return None

    try:
        img = mpimg.imread(MAP_FILE)

        fig, ax = plt.subplots(figsize=(7, 10))

        # Display map using metre-based coordinates.
        # origin='upper' keeps image orientation like typical floor plan images.
        ax.imshow(img, extent=[0, MAP_WIDTH_M, 0, MAP_HEIGHT_M], origin="upper")

        # Mark emergency location
        ax.scatter([x], [y], s=220, marker="*", edgecolors="black", linewidths=1.5)

        ax.annotate(
            f"{em_type.upper()} alert\n{room_name}\n({x}, {y})",
            xy=(x, y),
            xytext=(x + 0.5, y + 0.8),
            arrowprops=dict(arrowstyle="->", linewidth=1.5),
            fontsize=10,
            bbox=dict(boxstyle="round,pad=0.4", fc="white", alpha=0.85),
        )

        ax.set_title(f"ERS Emergency Location - {device_id}")
        ax.set_xlabel("x position")
        ax.set_ylabel("y position")
        ax.set_xlim(0, MAP_WIDTH_M)
        ax.set_ylim(0, MAP_HEIGHT_M)
        ax.grid(True, alpha=0.3)

        tmp = tempfile.NamedTemporaryFile(
            prefix="ers_location_",
            suffix=".png",
            delete=False,
        )
        tmp_path = tmp.name
        tmp.close()

        fig.savefig(tmp_path, dpi=150, bbox_inches="tight")
        plt.close(fig)

        return tmp_path

    except Exception as e:
        print(f"[EMAIL MAP] failed to create location map: {e}")
        return None


def send_email_alert(event: dict, to_emails: list[str] | None = None) -> None:
    """Send an alert email with location text and optional map attachment.

    Raises on failure for the caller to log.
    """
    recipients: list[str] = []

    if to_emails:
        recipients = to_emails
    elif EMAIL_TO:
        recipients = [x.strip() for x in EMAIL_TO.split(",") if x.strip()]

    if not recipients:
        raise RuntimeError("EMAIL_TO is empty. Set EMAIL_TO in ~/.env comma-separated.")

    msg = EmailMessage()
    msg["From"] = EMAIL_ADDRESS
    msg["To"] = ", ".join(recipients)
    msg["Subject"] = _format_subject(event)
    msg.set_content(_format_body(event))

    map_path = _create_location_map_image(event)

    if map_path and os.path.exists(map_path):
        with open(map_path, "rb") as f:
            img_data = f.read()

        msg.add_attachment(
            img_data,
            maintype="image",
            subtype="png",
            filename="ers_emergency_location.png",
        )

    try:
        with smtplib.SMTP(EMAIL_HOST, EMAIL_PORT, timeout=20) as smtp:
            smtp.ehlo()
            smtp.starttls()
            smtp.ehlo()
            smtp.login(EMAIL_ADDRESS, EMAIL_PASSWORD)
            smtp.send_message(msg)

    finally:
        # Clean temporary map image
        if map_path and os.path.exists(map_path):
            try:
                os.remove(map_path)
            except Exception:
                pass
