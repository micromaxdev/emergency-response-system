import os
import smtplib
import tempfile
from email.message import EmailMessage
from datetime import datetime, timezone
from html import escape
from pathlib import Path

try:
    from dotenv import load_dotenv
except ImportError:
    def load_dotenv(*_args, **_kwargs):
        return False

from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SERVER_DIR = PROJECT_ROOT / "server"

load_dotenv(PROJECT_ROOT / ".env")
load_dotenv(Path.home() / ".env", override=True)

MAP_FILE = Path(os.getenv("EMAIL_MAP_FILE", str(PROJECT_ROOT / "assets" / "maps" / "image.png")))

MAP_WIDTH_M = float(os.getenv("EMAIL_MAP_WIDTH_M", "17.0"))
MAP_HEIGHT_M = float(os.getenv("EMAIL_MAP_HEIGHT_M", "24.0"))
MAP_MARKER_RADIUS_M = float(os.getenv("EMAIL_MAP_MARKER_RADIUS_M", "0.45"))
INLINE_MAP_CID = "ers_location_map"


def _email_settings() -> dict:
    return {
        "host": os.environ.get("EMAIL_HOST", ""),
        "port": int(os.environ.get("EMAIL_PORT", "587")),
        "address": os.environ.get("EMAIL_ADDRESS", "ers-alerts@example.local"),
        "password": os.environ.get("EMAIL_PASSWORD", ""),
        "fallback_to": os.environ.get("EMAIL_TO", "").strip(),
        "dry_run_file": os.environ.get("EMAIL_DRY_RUN_FILE", "").strip(),
    }


def _parse_email_recipients(raw) -> list[str]:
    if not raw:
        return []

    if isinstance(raw, (list, tuple, set)):
        parts = [str(x).strip() for x in raw if str(x).strip()]
    else:
        raw = str(raw).replace("，", ",").replace(";", ",").replace("\n", ",").replace("\r", ",")
        parts = [x.strip() for x in raw.split(",") if x.strip()]

    seen = set()
    out = []
    for p in parts:
        if p not in seen:
            out.append(p)
            seen.add(p)
    return out


def _format_subject(event: dict) -> str:
    em_type = event.get("emergency_type", "unknown")
    device_id = event.get("device_id", "unknown")
    location = event.get("estimated_location") or {}
    room_name = location.get("room_name") or event.get("room_name")

    if room_name and room_name != "Unknown area":
        return f"[ERS] {em_type.upper()} alert from {device_id} - {room_name}"

    return f"[ERS] {em_type.upper()} alert from {device_id}"


def _get_location(event: dict) -> dict:
    loc = event.get("estimated_location")
    return loc if isinstance(loc, dict) else {}


def _format_body_text(event: dict) -> str:
    lines = []
    lines.append("Emergency Response System Alert")
    lines.append("-" * 40)

    server_time = event.get("server_time") or datetime.now(timezone.utc).isoformat()
    location = _get_location(event)

    room_name = location.get("room_name") or event.get("room_name") or "Unknown area"
    x = location.get("x")
    y = location.get("y")

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

    lines.append("")
    lines.append("Raw event:")
    lines.append(str(event))

    return "\n".join(lines)


def _format_body_html(event: dict, show_inline_map: bool) -> str:
    server_time = escape(str(event.get("server_time") or datetime.now(timezone.utc).isoformat()))
    location = _get_location(event)

    room_name = escape(str(location.get("room_name") or event.get("room_name") or "Unknown area"))
    x = location.get("x")
    y = location.get("y")

    coord_html = ""
    if x is not None and y is not None:
        coord_html = f"<p><b>Coordinates:</b> x={escape(str(x))}, y={escape(str(y))}</p>"

    map_html = ""
    if show_inline_map:
        map_html = """
        <h3>Emergency Location Map</h3>
        <p><img src="cid:ers_location_map" alt="ERS location map" style="max-width:100%; border:1px solid #ccc;"></p>
        """

    html = f"""
    <html>
      <body>
        <h2>Emergency Response System Alert</h2>
        <p><b>Server Time:</b> {server_time}</p>
        <p><b>Device ID:</b> {escape(str(event.get('device_id', '')))}</p>
        <p><b>Emergency Type:</b> {escape(str(event.get('emergency_type', '')))}</p>
        <p><b>Message:</b> {escape(str(event.get('message', '')))}</p>
        <p><b>Pico Timestamp:</b> {escape(str(event.get('pico_ts', '')))}</p>
        <p><b>From:</b> {escape(str(event.get('from_ip', '')))}:{escape(str(event.get('from_port', '')))}</p>

        <h3>Estimated Location</h3>
        <p><b>Room Name:</b> {room_name}</p>
        {coord_html}

        {map_html}

        <h3>Raw event</h3>
        <pre>{escape(str(event))}</pre>
      </body>
    </html>
    """
    return html


def _create_location_map_image(event: dict, output_path: str | os.PathLike | None = None) -> str | None:
    location = _get_location(event)

    try:
        x = float(location.get("x"))
        y = float(location.get("y"))
    except (TypeError, ValueError):
        return None

    room_name = location.get("room_name") or event.get("room_name") or "Unknown area"
    device_id = event.get("device_id", "unknown")
    em_type = event.get("emergency_type", "unknown")

    if not MAP_FILE.exists():
        print(f"[EMAIL MAP] map file not found: {MAP_FILE}")
        return None

    try:
        img = Image.open(MAP_FILE).convert("RGBA")
        draw = ImageDraw.Draw(img, "RGBA")
        width_px, height_px = img.size

        x_px = max(0, min(width_px, int((x / MAP_WIDTH_M) * width_px)))
        y_px = max(0, min(height_px, int(height_px - ((y / MAP_HEIGHT_M) * height_px))))
        radius_px = max(12, int((MAP_MARKER_RADIUS_M / MAP_WIDTH_M) * width_px))

        draw.ellipse(
            (x_px - radius_px, y_px - radius_px, x_px + radius_px, y_px + radius_px),
            fill=(229, 57, 53, 190),
            outline=(0, 0, 0, 255),
            width=4,
        )
        center_radius = max(3, radius_px // 5)
        draw.ellipse(
            (
                x_px - center_radius,
                y_px - center_radius,
                x_px + center_radius,
                y_px + center_radius,
            ),
            fill=(255, 255, 255, 255),
            outline=(0, 0, 0, 255),
            width=1,
        )

        label = f"{str(em_type).upper()} alert\n{room_name}\n({x:.2f}, {y:.2f})"
        font = ImageFont.load_default()
        text_box = draw.multiline_textbbox((0, 0), label, font=font, spacing=4)
        label_w = text_box[2] - text_box[0] + 16
        label_h = text_box[3] - text_box[1] + 14
        label_x = min(max(8, x_px + radius_px + 10), max(8, width_px - label_w - 8))
        label_y = min(max(8, y_px - label_h - 10), max(8, height_px - label_h - 8))

        draw.line((x_px, y_px, label_x, label_y + label_h), fill=(0, 0, 0, 255), width=3)
        draw.rounded_rectangle(
            (label_x, label_y, label_x + label_w, label_y + label_h),
            radius=6,
            fill=(255, 255, 255, 225),
            outline=(0, 0, 0, 200),
            width=2,
        )
        draw.multiline_text((label_x + 8, label_y + 7), label, fill=(0, 0, 0, 255), font=font, spacing=4)

        if output_path is None:
            tmp = tempfile.NamedTemporaryFile(prefix="ers_location_", suffix=".png", delete=False)
            tmp_path = tmp.name
            tmp.close()
        else:
            tmp_path = str(output_path)

        img.save(tmp_path, format="PNG")

        return tmp_path

    except Exception as e:
        print(f"[EMAIL MAP] failed to create location map: {e}")
        return None


def build_email_message(event: dict, to_emails: list[str] | None = None) -> EmailMessage:
    settings = _email_settings()
    recipients = []

    if to_emails:
        recipients = _parse_email_recipients(to_emails)
    elif settings["fallback_to"]:
        recipients = _parse_email_recipients(settings["fallback_to"])

    if not recipients:
        raise RuntimeError("EMAIL_TO is empty. Set EMAIL_TO in ~/.env comma-separated.")

    print(f"[EMAIL DEBUG] recipients={recipients}")
    print(f"[EMAIL DEBUG] subject={_format_subject(event)}")

    msg = EmailMessage()
    msg["From"] = settings["address"]
    msg["To"] = ", ".join(recipients)
    msg["Subject"] = _format_subject(event)

    text_body = _format_body_text(event)

    map_path = _create_location_map_image(event)
    has_inline_map = bool(map_path and os.path.exists(map_path))

    html_body = _format_body_html(event, show_inline_map=has_inline_map)

    # plain text fallback
    msg.set_content(text_body)

    # HTML version
    msg.add_alternative(html_body, subtype="html")

    # Add inline image to HTML part
    if has_inline_map:
        with open(map_path, "rb") as f:
            img_data = f.read()

        html_part = msg.get_payload()[-1]
        html_part.add_related(
            img_data,
            maintype="image",
            subtype="png",
            cid=f"<{INLINE_MAP_CID}>",
            disposition="inline",
            filename="ers_emergency_location.png",
        )

    if map_path and os.path.exists(map_path):
        try:
            os.remove(map_path)
        except Exception:
            pass

    return msg


def send_email_alert(event: dict, to_emails: list[str] | None = None) -> None:
    settings = _email_settings()
    msg = build_email_message(event, to_emails=to_emails)

    if settings["dry_run_file"]:
        dry_run_path = Path(settings["dry_run_file"])
        dry_run_path.parent.mkdir(parents=True, exist_ok=True)
        dry_run_path.write_bytes(msg.as_bytes())
        print(f"[EMAIL DRY RUN] wrote email message to {dry_run_path}")
        return

    missing = [name for name in ("host", "address", "password") if not settings[name]]
    if missing:
        raise RuntimeError(f"Missing email setting(s): {', '.join(missing)}")

    try:
        with smtplib.SMTP(settings["host"], settings["port"], timeout=20) as smtp:
            smtp.ehlo()
            smtp.starttls()
            smtp.ehlo()
            smtp.login(settings["address"], settings["password"])
            smtp.send_message(msg)
    except smtplib.SMTPException as e:
        raise RuntimeError(f"SMTP email send failed: {e}") from e
