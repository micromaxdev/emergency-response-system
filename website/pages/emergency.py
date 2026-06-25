"""
pages/5_Emergency.py — Dedicated real mass emergency trigger.
Separate page with triple confirmation to prevent accidental activation.
"""

import streamlit as st
import socket, json, os, sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import utils as ers

st.set_page_config(page_title="ERS · Emergency Trigger", page_icon="🚨", layout="wide")
st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()

# ── Session state ─────────────────────────────────────────────────────────────
if "em_step"   not in st.session_state: st.session_state.em_step   = 0   # 0=idle 1=confirm 2=confirmed 3=fired
if "em_result" not in st.session_state: st.session_state.em_result = None

# ── Trigger function ──────────────────────────────────────────────────────────
def trigger_real_mass():
    payload = {
        "emergency":         True,
        "emergency_type":    "mass",
        "trigger_source":    "dashboard_manual",
        "device_id":         "DASHBOARD_MANUAL",
        "emergency_message": "MASS EMERGENCY — Manually triggered via dashboard",
        "timestamp":         datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    try:
        with socket.create_connection((ers.SERVER_HOST, ers.SERVER_PORT), timeout=5) as s:
            s.sendall(json.dumps(payload).encode("utf-8"))
        return "ok", None
    except ConnectionRefusedError:
        return "error", f"Connection refused — is the server running on {ers.SERVER_HOST}:{ers.SERVER_PORT}?"
    except socket.timeout:
        return "error", "Connection timed out."
    except Exception as e:
        return "error", str(e)

# ── Sidebar ───────────────────────────────────────────────────────────────────
with st.sidebar:
    st.markdown('<div style="font-family:Bebas Neue,sans-serif;font-size:1.8rem;letter-spacing:5px;color:#e63946;margin-bottom:2px;">ERS</div>', unsafe_allow_html=True)
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:3px;margin-bottom:20px;">EMERGENCY TRIGGER</div>', unsafe_allow_html=True)
    st.markdown("---")
    st.markdown("""
    <div style="font-family:'IBM Plex Mono',monospace;font-size:0.68rem;color:#586069;line-height:1.8;">
      This page triggers a <strong style="color:#e63946;">real mass emergency</strong>.<br><br>
      All systems will activate:<br>
      · Audio alarm<br>
      · Email alerts<br>
      · SMS alerts<br>
      · Relay Alert<br><br>
      Three confirmations are required before anything fires.
    </div>
    """, unsafe_allow_html=True)
    st.markdown("---")
    st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;">Server: {ers.SERVER_HOST}:{ers.SERVER_PORT}</div>', unsafe_allow_html=True)

# ── Main ──────────────────────────────────────────────────────────────────────
st.markdown("""
<div class="ers-header">
  <div class="ers-logo" style="color:#e63946;">ERS</div>
  <div class="ers-sub">Mass Emergency Trigger</div>
</div>
""", unsafe_allow_html=True)

# Show stop button prominently if alerts are already running
if ers.alerts_active() and st.session_state.em_step != 3:
    st.markdown("""
    <div style="background:rgba(230,57,70,0.1);border:2px solid #e63946;
                border-radius:8px;padding:20px 24px;margin-bottom:24px;">
      <div style="font-family:'Bebas Neue',sans-serif;font-size:1.4rem;
                  letter-spacing:4px;color:#e63946;margin-bottom:6px;">
        ALERT SYSTEMS CURRENTLY ACTIVE
      </div>
      <div style="font-family:'IBM Plex Mono',monospace;font-size:0.7rem;color:#586069;">
        Relay and/or audio are running. Stop them before triggering a new emergency.
      </div>
    </div>
    """, unsafe_allow_html=True)
    if st.button("STOP ALL ALERTS NOW", width="stretch", type="primary"):
        ers.send_stop_command()
        st.rerun()
    st.markdown("---")

# ── Fired state ───────────────────────────────────────────────────────────────
if st.session_state.em_step == 3:
    if st.session_state.em_result == "ok":
        st.markdown("""
        <div style="background:rgba(230,57,70,0.08);border:2px solid #e63946;
                    border-radius:8px;padding:32px;text-align:center;margin-bottom:32px;">
          <div style="font-family:'Bebas Neue',sans-serif;font-size:3rem;
                      letter-spacing:6px;color:#e63946;margin-bottom:12px;">
            MASS EMERGENCY ACTIVE
          </div>
          <div style="font-family:'IBM Plex Mono',monospace;font-size:0.85rem;color:#586069;">
            All alert systems have been activated.<br>
            Audio · Email · SMS · Relay are all running.
          </div>
        </div>
        """, unsafe_allow_html=True)
        if st.button("STOP ALL ALERTS — SILENCE RELAY & AUDIO",
                     width="stretch", type="primary"):
            ers.send_stop_command()
            st.session_state.em_step   = 0
            st.session_state.em_result = None
            st.rerun()
    else:
        st.error(f"⚠ Failed to reach server: {st.session_state.em_result}")
        if st.button("Reset", width="content"):
            st.session_state.em_step   = 0
            st.session_state.em_result = None
            st.rerun()
    st.stop()

# ── Warning banner (always shown when idle) ───────────────────────────────────
st.markdown("""
<div style="background:rgba(230,57,70,0.06);border:1px solid rgba(230,57,70,0.35);
            border-left:4px solid #e63946;border-radius:6px;
            padding:20px 24px;margin-bottom:32px;">
  <div style="font-family:'IBM Plex Mono',monospace;font-size:0.75rem;
              color:#e63946;letter-spacing:2px;text-transform:uppercase;margin-bottom:8px;">
    ⚠ This is NOT a drill
  </div>
  <div style="font-family:'IBM Plex Mono',monospace;font-size:0.72rem;color:#586069;line-height:1.8;">
    Pressing the button below will trigger a <strong style="color:#111;">real mass emergency</strong>
    and immediately activate all connected alert systems.
    Only proceed if there is a genuine emergency.
  </div>
</div>
""", unsafe_allow_html=True)

# ── Step 0: Initial button ────────────────────────────────────────────────────
if st.session_state.em_step == 0:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:16px;">Step 1 of 3 — Initiate</div>', unsafe_allow_html=True)
    if st.button("TRIGGER MASS EMERGENCY", width="stretch", type="primary"):
        st.session_state.em_step = 1
        st.rerun()

# ── Step 1: First confirmation ────────────────────────────────────────────────
elif st.session_state.em_step == 1:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:16px;">Step 2 of 3 — Confirm</div>', unsafe_allow_html=True)
    st.warning("Are you sure? This will send real alerts to all configured staff members.")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("✓  YES — THIS IS A REAL EMERGENCY", width="stretch", type="primary"):
            st.session_state.em_step = 2
            st.rerun()
    with c2:
        if st.button("✕  CANCEL — DO NOT TRIGGER", width="stretch"):
            st.session_state.em_step = 0
            st.rerun()

# ── Step 2: Final confirmation ────────────────────────────────────────────────
elif st.session_state.em_step == 2:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:16px;">Step 3 of 3 — Final confirmation</div>', unsafe_allow_html=True)
    st.error("⚠ FINAL WARNING — Pressing confirm will immediately activate audio, email, SMS and relay. This cannot be undone.")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("CONFIRM — ACTIVATE ALL SYSTEMS NOW", width="stretch", type="primary"):
            status, err = trigger_real_mass()
            st.session_state.em_result = "ok" if status == "ok" else err
            st.session_state.em_step   = 3
            st.rerun()
    with c2:
        if st.button("✕  CANCEL — STAND DOWN", width="stretch"):
            st.session_state.em_step = 0
            st.rerun()

# ── Footer ────────────────────────────────────────────────────────────────────
st.markdown(f"""
<div style="margin-top:60px;padding-top:14px;border-top:1px solid #1e2530;
            font-family:'IBM Plex Mono',monospace;font-size:0.6rem;color:#30363d;
            letter-spacing:2px;text-align:center;">
  ERS · MASS EMERGENCY TRIGGER · {ers.SERVER_HOST}:{ers.SERVER_PORT}
</div>""", unsafe_allow_html=True)
