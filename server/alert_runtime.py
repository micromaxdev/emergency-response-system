"""Background relay and audio sirens that loop until stopped via the dashboard.

State lives in the ``system_control`` table so the Streamlit dashboard can stop a
running siren by flipping a flag; these loops poll that flag and exit cleanly.
"""

import subprocess
import threading
import time
import traceback

import config  # noqa: F401  (ensures alert_handlers/ is on sys.path before the imports below)
from database import get_control, set_control
from event_log import log_event, now_str
from relay_alert import relay_on, relay_off
from audio_alert import AUDIO_DIR, get_audio_path

relay_i2c_lock = threading.Lock()
_relay_thread_running = False
_audio_thread_running = False
_audio_proc = None
_audio_proc_lock = threading.Lock()
_thread_lock = threading.Lock()


# -- Relay siren ---------------------------------------------------------------
def _relay_loop():
    global _relay_thread_running
    log_event({"server_time": now_str(), "type": "relay_loop_start"})
    try:
        with relay_i2c_lock:
            relay_on(ch=1)

        while get_control("relay_active") == "1":
            time.sleep(1)

        with relay_i2c_lock:
            relay_off(ch=1)
        log_event({"server_time": now_str(), "type": "relay_loop_stopped"})
    except Exception as e:
        log_event({
            "server_time": now_str(),
            "type": "relay_loop_error",
            "error": str(e),
            "trace": traceback.format_exc(),
        })
        try:
            with relay_i2c_lock:
                relay_off(ch=1)
        except Exception:
            pass
    finally:
        with _thread_lock:
            _relay_thread_running = False


def start_relay_loop():
    """Start the relay siren in a background thread (no-op if already running)."""
    global _relay_thread_running
    with _thread_lock:
        if _relay_thread_running:
            return
        _relay_thread_running = True
    set_control("relay_active", "1")
    threading.Thread(target=_relay_loop, daemon=True).start()


# -- Audio siren ---------------------------------------------------------------
def _play_until_stopped(cmd):
    """Run an audio player command, terminating it as soon as audio is stopped."""
    global _audio_proc
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with _audio_proc_lock:
        _audio_proc = proc
    while proc.poll() is None:
        if get_control("audio_active") != "1":
            proc.terminate()
            proc.wait()
            break
        time.sleep(0.5)


def _audio_loop(em_type: str):
    global _audio_thread_running, _audio_proc

    resolved = get_audio_path(em_type)
    if resolved is None:
        log_event({
            "server_time": now_str(),
            "type": "audio_loop_aborted",
            "reason": f"No audio file found for emergency_type='{em_type}' in {AUDIO_DIR}. Upload one on the Configuration page.",
        })
        with _thread_lock:
            _audio_thread_running = False
        return

    audio_path = str(resolved)
    log_event({"server_time": now_str(), "type": "audio_loop_start", "file": audio_path})

    try:
        while get_control("audio_active") == "1":
            try:
                _play_until_stopped(
                    ["ffplay", "-nodisp", "-autoexit", "-loglevel", "error", audio_path]
                )
            except FileNotFoundError:
                try:
                    _play_until_stopped(["mpg123", "-q", audio_path])
                except Exception as e2:
                    log_event({
                        "server_time": now_str(),
                        "type": "audio_loop_player_failed",
                        "error": str(e2),
                    })
                    break
            except Exception as e:
                log_event({
                    "server_time": now_str(),
                    "type": "audio_loop_error",
                    "error": str(e),
                })
                time.sleep(1)

        log_event({"server_time": now_str(), "type": "audio_loop_stopped"})
    finally:
        with _audio_proc_lock:
            _audio_proc = None
        with _thread_lock:
            _audio_thread_running = False


def start_audio_loop(em_type: str = "mass"):
    """Start the looping audio siren in a background thread (no-op if running)."""
    global _audio_thread_running
    with _thread_lock:
        if _audio_thread_running:
            return
        _audio_thread_running = True
    set_control("audio_active", "1")
    threading.Thread(target=_audio_loop, args=(em_type,), daemon=True).start()


# -- Stop ----------------------------------------------------------------------
def stop_all_alerts():
    """Signal the relay and audio loops to stop on their next poll."""
    set_control("relay_active", "0")
    set_control("audio_active", "0")
    log_event({"server_time": now_str(), "type": "stop_all_alerts_requested"})
