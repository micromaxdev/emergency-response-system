import os
import subprocess
import time
from pathlib import Path
from typing import Optional, Dict, Any

def _find_project_root(start: Path) -> Path:
    """
    Walk upwards from 'start' to find a directory that contains 'assets/'.
    This makes it robust when this file is placed under pico_dashboard/templates/ etc.
    """
    start = start.resolve()
    for p in [start] + list(start.parents):
        if (p / "assets").exists():
            return p
    # fallback: current dir
    return start

# --- Project paths ---
PROJECT_ROOT = _find_project_root(Path(__file__).parent)
AUDIO_DIR = Path(
    os.getenv("ALERT_AUDIO_DIR", str(PROJECT_ROOT / "assets" / "audios"))
).expanduser()

# --- Audio mapping ---
AUDIO_MAP = {
    "personal": "personal-emergency-alarm.mp3",
    "mass": "mass-emergency-alarm.mp3",
}

DEFAULT_AUDIO = os.getenv(
    "ALERT_AUDIO_FILE",
    str(AUDIO_DIR / AUDIO_MAP["personal"])
)

def _pick_audio_file(event: Dict[str, Any], audio_file: Optional[str]) -> Path:
    """
    Choose audio file in this priority:
    1) explicit audio_file argument
    2) event['emergency_type'] mapped via AUDIO_MAP
    3) DEFAULT_AUDIO
    """
    if audio_file and audio_file.strip():
        file_path = audio_file.strip()
        p = Path(file_path).expanduser()
        return p

    em_type = (event.get("emergency_type") or "").strip().lower()
    if em_type in AUDIO_MAP:
        return (AUDIO_DIR / AUDIO_MAP[em_type]).expanduser()

    return Path(DEFAULT_AUDIO).expanduser()
    
def get_audio_path(em_type: str):
    em_type = (em_type or "").strip().lower()
    if em_type in AUDIO_MAP:
        p = (AUDIO_DIR / AUDIO_MAP[em_type]).expanduser()
    else:
        p = Path(DEFAULT_AUDIO).expanduser()

    if not p.is_absolute():
        p = (PROJECT_ROOT / p).resolve()

    return p if p.exists() else None

def _rate_limit(rl_state: Dict[str, float], key: str, min_interval_sec: float) -> bool:
    """
    Return True if allowed to play now, False if blocked by rate limit.
    """
    now = time.time()
    last = rl_state.get(key, 0.0)
    if now - last < min_interval_sec:
        return False
    rl_state[key] = now
    return True

# Global rate-limit state (simple and effective)
_RL_STATE: Dict[str, float] = {}

def play_audio_alert(
    event: Dict[str, Any],
    audio_file: Optional[str] = None,
    block: bool = False,
    min_interval_sec: float = 2.0,
    stop_previous: bool = False,
) -> Dict[str, Any]:
    """
    Play an alert sound on the server machine.

    Args:
        event: dict that may include "emergency_type" = "personal" or "mass"
        audio_file: optional explicit file path (overrides mapping)
        block: True = blocking, False = async
        min_interval_sec: rate limit to avoid audio spam from frequent events
        stop_previous: if True, stop previous ffplay process before playing new (best effort)

    Returns:
        dict result
    """
    p = _pick_audio_file(event, audio_file)

    # Resolve relative paths against PROJECT_ROOT (helps if you pass "assets/audio/xxx.mp3")
    if not p.is_absolute():
        p = (PROJECT_ROOT / p).resolve()

    if not p.exists():
        raise FileNotFoundError(f"Audio file not found: {p}")

    # Rate-limit based on emergency_type or fallback key
    em_type = (event.get("emergency_type") or "unknown").strip().lower() or "unknown"
    if not _rate_limit(_RL_STATE, key=f"audio:{em_type}", min_interval_sec=min_interval_sec):
        return {
            "played": False,
            "reason": "rate_limited",
            "emergency_type": em_type,
            "file": str(p),
            "min_interval_sec": min_interval_sec,
        }

    # Prefer ffplay for mp3/wav
    cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "error", str(p)]

    # Optionally stop previous ffplay (simple best-effort using pkill is risky; we keep it safe)
    # If you want strict stop-previous, implement a process manager. Here we keep it conservative.
    # stop_previous is kept for API compatibility.

    try:
        if block:
            subprocess.run(cmd, check=True)
            return {"played": True, "player": "ffplay", "file": str(p), "mode": "block", "emergency_type": em_type}
        else:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return {"played": True, "player": "ffplay", "file": str(p), "mode": "async", "emergency_type": em_type}

    except FileNotFoundError:
        # ffplay not installed. Fallback:
        # - mp3: mpg123
        # - wav: aplay
        suffix = p.suffix.lower()

        if suffix == ".mp3":
            cmd2 = ["mpg123", "-q", str(p)]
        else:
            cmd2 = ["aplay", str(p)]

        if block:
            subprocess.run(cmd2, check=True)
            return {"played": True, "player": cmd2[0], "file": str(p), "mode": "block", "emergency_type": em_type}
        else:
            subprocess.Popen(cmd2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return {"played": True, "player": cmd2[0], "file": str(p), "mode": "async", "emergency_type": em_type}
