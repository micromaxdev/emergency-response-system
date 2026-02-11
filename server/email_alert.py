import os
import smtplib
from email.message import EmailMessage
from datetime import datetime, timezone
from dotenv import load_dotenv

# 读取 ~/.env（你已经配置好了）
load_dotenv(os.path.expanduser("~/.env"))

EMAIL_HOST = os.environ["EMAIL_HOST"]
EMAIL_PORT = int(os.environ.get("EMAIL_PORT", "587"))
EMAIL_ADDRESS = os.environ["EMAIL_ADDRESS"]
EMAIL_PASSWORD = os.environ["EMAIL_PASSWORD"]

# 收件人：建议也放到 env 里，方便改
# 例如 EMAIL_TO=zl017@uowmail.edu.au,other@example.com
EMAIL_TO = os.environ.get("EMAIL_TO", "").strip()

def _format_subject(event: dict) -> str:
    em_type = event.get("emergency_type", "unknown")
    device_id = event.get("device_id", "unknown")
    return f"[ERS] {em_type.upper()} alert from {device_id}"

def _format_body(event: dict) -> str:
    # 你 event 里常见字段：server_time, pico_ts, device_id, emergency_type, message, from_ip, from_port
    lines = []
    lines.append("Emergency Response System Alert")
    lines.append("-" * 40)

    # 时间
    server_time = event.get("server_time")
    if not server_time:
        server_time = datetime.now(timezone.utc).isoformat()

    lines.append(f"Server Time: {server_time}")
    lines.append(f"Device ID: {event.get('device_id', '')}")
    lines.append(f"Emergency Type: {event.get('emergency_type', '')}")
    lines.append(f"Message: {event.get('message', '')}")
    lines.append(f"Pico Timestamp: {event.get('pico_ts', '')}")
    lines.append(f"From: {event.get('from_ip', '')}:{event.get('from_port', '')}")
    lines.append("")
    lines.append("Raw event:")
    lines.append(str(event))
    return "\n".join(lines)

def send_email_alert(event: dict, to_emails: list[str] | None = None) -> None:
    """
    发送报警邮件。失败时抛异常，由上层捕获并记录日志。
    """
    # 解析收件人
    recipients: list[str] = []
    if to_emails:
        recipients = to_emails
    elif EMAIL_TO:
        recipients = [x.strip() for x in EMAIL_TO.split(",") if x.strip()]

    if not recipients:
        raise RuntimeError("EMAIL_TO is empty. Set EMAIL_TO in ~/.env (comma-separated).")

    msg = EmailMessage()
    msg["From"] = EMAIL_ADDRESS
    msg["To"] = ", ".join(recipients)
    msg["Subject"] = _format_subject(event)
    msg.set_content(_format_body(event))

    with smtplib.SMTP(EMAIL_HOST, EMAIL_PORT, timeout=20) as smtp:
        smtp.ehlo()
        smtp.starttls()
        smtp.ehlo()
        smtp.login(EMAIL_ADDRESS, EMAIL_PASSWORD)
        smtp.send_message(msg)
