"""Append-only JSONL event logger shared across the server modules."""

import json
from datetime import datetime

import config


def now_str() -> str:
    """Return the current local time formatted for log records."""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def log_event(event: dict) -> None:
    """Print an event and append it as one JSON line to the log file."""
    print(event)
    with open(config.LOG_FILE, "a", encoding="utf-8") as f:
        f.write(json.dumps(event, ensure_ascii=False) + "\n")
