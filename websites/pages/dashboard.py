"""
pages/dashboard.py — Live incident dashboard, system status, test drill, retention.
"""

import streamlit as st
import socket, json, os, sys
from datetime import datetime, timedelta
from PIL import Image 
import plotly.express as px

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import utils as ers

st.set_page_config(page_title="ERS · Dashboard", layout="wide")
st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()

# -- Session state -------------------------------------------------------------
if "drill_enabled" not in st.session_state:
    st.session_state.drill_enabled = True
if "drill_result" not in st.session_state:
    st.session_state.drill_result = None
if "purge_confirm" not in st.session_state:
    st.session_state.purge_confirm = False
if "purge_result" not in st.session_state:
    st.session_state.purge_result = None

# -- TCP drill -----------------------------------------------------------------
def trigger_drill():
    payload = {
        "emergency": True,
        "emergency_type": "drill",
        "trigger_source": ers.DRILL_SOURCE,
        "device_id": ers.DRILL_DEVICE,
        "emergency_message": "TEST DRILL — Simulated Mass Emergency",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "drill": True,
    }
    try:
        with socket.create_connection((ers.SERVER_HOST, ers.SERVER_PORT), timeout=5) as s:
            s.sendall(json.dumps(payload).encode("utf-8"))
        return "ok", None
    except ConnectionRefusedError:
        return "error", f"Connection refused — is the server running on {ers.SERVER_HOST}:{ers.SERVER_PORT}?"
    except socket.timeout:
        return "error", "Connection timed out (5s)."
    except Exception as e:
        return "error", str(e)

# -- Sidebar -------------------------------------------------------------------
with st.sidebar:
    st.markdown(
        '<div style="font-family:Bebas Neue,sans-serif;font-size:1.8rem;letter-spacing:5px;color:#4065a1;margin-bottom:2px;">ERS</div>',
        unsafe_allow_html=True
    )
    st.markdown(
        '<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:3px;margin-bottom:20px;">DASHBOARD</div>',
        unsafe_allow_html=True
    )
    st.markdown("---")

    st.markdown(
        '<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:10px;">Drill Controls</div>',
        unsafe_allow_html=True
    )
    drill_toggle = st.toggle("Test Drill Enabled", value=st.session_state.drill_enabled)
    st.session_state.drill_enabled = drill_toggle
    if not drill_toggle:
        st.warning("Test drills are **disabled**.")

    st.markdown("---")
    st.markdown(
        '<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:10px;">Display</div>',
        unsafe_allow_html=True
    )
    type_filter = st.multiselect(
        "Show Types",
        ["mass", "personal", "test_drill"],
        default=["mass", "personal", "test_drill"]
    )
    limit = st.slider("Max Rows", 20, 500, 100, step=20)

    st.markdown("---")
    st.caption(f"Retention: `{ers.RETENTION_DAYS} days`")

# -- Main ----------------------------------------------------------------------
st.markdown(ers.ers_header("Emergency Response System"), unsafe_allow_html=True)

if not os.path.exists(ers.DB_PATH):
    st.error(f"? Database not found at `{ers.DB_PATH}`. Is the ERS server running?")
    st.stop()

# KPIs
counts = ers.fetch_counts()
st.markdown(f"""
<div class="kpi-row">
  <div class="kpi-card red">
    <div class="kpi-num">{counts.get('mass', 0)}</div>
    <div class="kpi-label">Mass Emergencies</div>
  </div>
  <div class="kpi-card amber">
    <div class="kpi-num">{counts.get('personal', 0)}</div>
    <div class="kpi-label">Personal</div>
  </div>
  <div class="kpi-card drill">
    <div class="kpi-num">{counts.get('drills', 0)}</div>
    <div class="kpi-label">Drills Run</div>
  </div>
  <div class="kpi-card gray">
    <div class="kpi-num">{counts.get('total', 0)}</div>
    <div class="kpi-label">Total</div>
  </div>
</div>
""", unsafe_allow_html=True)

# Active alert banner + stop button
if ers.alerts_active():
    st.markdown("""
    <div style="background:rgba(230,57,70,0.1);border:2px solid #e63946;
                border-radius:8px;padding:20px 24px;margin-bottom:24px;
                display:flex;align-items:center;gap:16px;">
      <div style="font-size:1.8rem;"></div>
      <div>
        <div style="font-family:'Bebas Neue',sans-serif;font-size:1.4rem;
                    letter-spacing:4px;color:#e63946;">ALERT SYSTEMS ACTIVE</div>
        <div style="font-family:'IBM Plex Mono',monospace;font-size:0.7rem;
                    color:#586069;margin-top:2px;">
          Relay and/or audio are currently running. Press Stop to silence all systems.
        </div>
      </div>
    </div>
    """, unsafe_allow_html=True)
    if st.button(
        "STOP ALL ALERTS — SILENCE RELAY & AUDIO",
        width="stretch",
        type="primary",
        key="stop_all_alerts_main"
    ):
        ers.send_stop_command()
        st.rerun()
else:
    st.markdown("""
    <div style="background:rgba(46,196,182,0.08);border:1px solid rgba(46,196,182,0.3);
                border-radius:6px;padding:12px 18px;margin-bottom:24px;">
      <span style="font-family:'IBM Plex Mono',monospace;font-size:0.7rem;color:#2ec4b6;">
        ? All alert systems are silent.
      </span>
    </div>
    """, unsafe_allow_html=True)

# System Status
st.markdown('<div class="section-title">System Status</div>', unsafe_allow_html=True)
sys_status = ers.get_system_status()
STATUS_CARDS = [
    ("server", "ERS Server"),
    ("db", "Database"),
    ("last_incident", "Last Incident"),
    ("audio", "Audio"),
    ("email", "Email"),
    ("sms", "SMS"),
    ("relay", "Relay"),
    ("retention", f"Retention ({ers.RETENTION_DAYS}d)"),
]
cards_html = '<div class="status-grid">'
for key, label in STATUS_CARDS:
    state, detail = sys_status.get(key, ("unknown", "—"))
    cards_html += f"""
    <div class="status-card">
      <div class="status-card-label">{label}</div>
      {ers.status_pill(state, state.upper())}
      <div class="status-card-detail">{detail}</div>
    </div>"""
cards_html += "</div>"
st.markdown(cards_html, unsafe_allow_html=True)
st.markdown("---")

# Device Communication Status
st.markdown('<div class="section-title">Device Communication Status</div>', unsafe_allow_html=True)

TARGET_DEVICE_ID = "PICO_TEST_01"
ACK_BY = "Skylar Li"
comm = ers.get_device_comm_status(TARGET_DEVICE_ID)

if comm["state"] == "online":
    box_bg = "rgba(46,196,182,0.10)"
    box_border = "rgba(46,196,182,0.45)"
    text_color = "#2ec4b6"
elif comm["state"] == "offline":
    box_bg = "rgba(230,57,70,0.10)"
    box_border = "rgba(230,57,70,0.45)"
    text_color = "#e63946"
else:
    box_bg = "rgba(247,183,49,0.10)"
    box_border = "rgba(247,183,49,0.45)"
    text_color = "#f7b731"

st.markdown(f"""
<div style="background:{box_bg};
            border:1px solid {box_border};
            border-left:6px solid {text_color};
            border-radius:8px;
            padding:18px 20px;
            margin-bottom:20px;">
  <div style="font-family:'Bebas Neue',sans-serif;
              font-size:1.3rem;
              letter-spacing:3px;
              color:{text_color};">
    {comm["label"]}
  </div>
  <div style="font-family:'IBM Plex Mono',monospace;
              font-size:0.72rem;
              color:#111;
              margin-top:6px;">
    Device: <strong>{TARGET_DEVICE_ID}</strong>
  </div>
  <div style="font-family:'IBM Plex Mono',monospace;
              font-size:0.68rem;
              color:#586069;
              margin-top:6px;">
    {comm["detail"]}
  </div>
  <div style="font-family:'IBM Plex Mono',monospace;
              font-size:0.68rem;
              color:#586069;
              margin-top:4px;">
    Last seen: {comm["last_seen"] or "—"}
  </div>
</div>
""", unsafe_allow_html=True)

if comm["state"] == "offline":
    if st.button(
        f"ACKNOWLEDGE OFFLINE ALERT · {TARGET_DEVICE_ID}",
        width="stretch",
        key=f"ack_offline_{TARGET_DEVICE_ID}"
    ):
        ok, err = ers.send_ack_offline_alert(TARGET_DEVICE_ID, ACK_BY)
        if ok:
            st.success(f"Offline alert for {TARGET_DEVICE_ID} acknowledged by {ACK_BY}.")
        else:
            st.error(f"Failed to acknowledge offline alert: {err}")

st.markdown("---")


# Live Emergency Location Map
st.markdown('<div class="section-title">Live Emergency Location Map</div>', unsafe_allow_html=True)

map_img_path = os.path.join(ers.MAP_DIR, "company_map.png")
current_gws = ers.fetch_all_gw_coords()
map_meters_wide = ers.fetch_map_scale()
latest_location = ers.fetch_latest_incident_with_location()

if not os.path.exists(map_img_path):
    st.info("No map uploaded yet. Please upload a floor plan in Configuration > Dynamic Map & Gateway Setup.")
else:
    img = Image.open(map_img_path)
    img_w, img_h = img.size

    scale = img_w / float(map_meters_wide)
    real_height_m = img_h / scale

    st.caption(
        f"Map size: {float(map_meters_wide):.2f} m wide × {real_height_m:.2f} m high"
    )

    fig = px.imshow(img)

    # Plot configured gateways
    for gw, pos in current_gws.items():
        fig.add_scatter(
            x=[float(pos["x"]) * scale],
            y=[float(pos["y"]) * scale],
            mode="markers+text",
            text=[f"<b>{gw}</b>"],
            textposition="top center",
            marker=dict(
                color="#e74c3c",
                size=15,
                symbol="octagon",
                line=dict(width=2, color="white")
            ),
            name=gw
        )

    # Plot latest emergency location
    if latest_location:
        x_m = latest_location["x"]
        y_m = latest_location["y"]

        fig.add_scatter(
            x=[x_m * scale],
            y=[y_m * scale],
            mode="markers+text",
            text=[f"?? {latest_location['device_id']}"],
            textposition="bottom center",
            marker=dict(
                color="#0066ff",
                size=24,
                symbol="circle",
                line=dict(width=3, color="white")
            ),
            name="Latest Emergency Location",
            hovertemplate=(
                "<b>Latest Emergency</b><br>"
                "Device: %{customdata[0]}<br>"
                "Type: %{customdata[1]}<br>"
                "Trigger: %{customdata[2]}<br>"
                "X: %{customdata[3]:.2f} m<br>"
                "Y: %{customdata[4]:.2f} m<br>"
                "Time: %{customdata[5]}<extra></extra>"
            ),
            customdata=[[
                latest_location["device_id"],
                latest_location["emergency_type"],
                latest_location["trigger_source"],
                x_m,
                y_m,
                latest_location["server_time"],
            ]]
        )

        st.success(
            f"Latest emergency location: "
            f"{latest_location['device_id']} "
            f"at X={x_m:.2f} m, Y={y_m:.2f} m"
        )

        loc_c1, loc_c2, loc_c3, loc_c4 = st.columns(4)
        loc_c1.metric("Device", latest_location["device_id"])
        loc_c2.metric("Type", latest_location["emergency_type"])
        loc_c3.metric("X Position", f"{x_m:.2f} m")
        loc_c4.metric("Y Position", f"{y_m:.2f} m")

    else:
        st.info("No incident with estimated location has been recorded yet.")

    fig.update_layout(
        showlegend=False,
        margin=dict(l=0, r=0, t=0, b=0),
        xaxis=dict(
            visible=True,
            title=f"Map width: {float(map_meters_wide):.2f} m"
        ),
        yaxis=dict(
            visible=True,
            autorange="reversed"
        )
    )

    st.plotly_chart(fig, width="stretch")

st.markdown("---")



# Test Drill
st.markdown('<div class="section-title">Test Drill</div>', unsafe_allow_html=True)
if st.session_state.drill_result == "ok":
    st.markdown("""
    <div class="drill-alert">
      <strong>? TEST DRILL TRIGGERED</strong><br>
      Payload sent — full mass emergency workflow is running on the server.<br>
      Audio · Email · SMS · Relay (120s) are all active. This is a drill.
    </div>""", unsafe_allow_html=True)
elif st.session_state.drill_result:
    st.error(f"Drill failed: {st.session_state.drill_result}")

col_drill, col_clear = st.columns([4, 1])
with col_drill:
    if st.button(
        "TRIGGER TEST DRILL  ·  Mass Emergency Simulation",
        disabled=not st.session_state.drill_enabled,
        width="stretch",
        type="primary",
        key="trigger_test_drill"
    ):
        status, err = trigger_drill()
        st.session_state.drill_result = "ok" if status == "ok" else err
        st.rerun()
with col_clear:
    if st.button("Clear", width="stretch", key="clear_drill"):
        st.session_state.drill_result = None
        st.rerun()

if not st.session_state.drill_enabled:
    st.caption("Enable drills via the sidebar toggle.")

st.markdown("---")

# Alert Retention
st.markdown('<div class="section-title">Alert Retention</div>', unsafe_allow_html=True)
purgeable, cutoff_str = ers.count_purgeable(ers.RETENTION_DAYS)
cutoff_display = (datetime.now() - timedelta(days=ers.RETENTION_DAYS)).strftime("%d %b %Y %H:%M")
st.markdown(f"""
<div class="purge-box">
  <div style="font-family:'IBM Plex Mono',monospace;font-size:0.75rem;color:var(--text);margin-bottom:8px;">
    Retention window: <strong style="color:#111;">{ers.RETENTION_DAYS} days</strong>
    &nbsp;·&nbsp; Cutoff: <strong style="color:#111;">{cutoff_display}</strong>
  </div>
  <div style="font-family:'IBM Plex Mono',monospace;font-size:0.7rem;color:{'#e63946' if purgeable > 0 else '#2ec4b6'};">
    {'? ' + str(purgeable) + ' incident(s) older than ' + str(ers.RETENTION_DAYS) + ' days are eligible for deletion.' if purgeable > 0 else '? All incidents are within the retention window.'}
  </div>
</div>""", unsafe_allow_html=True)

if st.session_state.purge_result:
    deleted, when = st.session_state.purge_result
    st.success(f"? Purged {deleted} incident(s) older than {when}. Database vacuumed.")
    if st.button("Dismiss", key="dismiss_purge"):
        st.session_state.purge_result = None
        st.rerun()

if not st.session_state.purge_confirm:
    btn_label = (
        f"DELETE {purgeable} INCIDENT(S) OLDER THAN {ers.RETENTION_DAYS} DAYS"
        if purgeable > 0 else f"NO RECORDS OLDER THAN {ers.RETENTION_DAYS} DAYS"
    )
    if st.button(
        btn_label,
        width="stretch",
        disabled=(purgeable == 0),
        key="purge_old_incidents"
    ):
        st.session_state.purge_confirm = True
        st.rerun()
else:
    st.warning(f"This will permanently delete **{purgeable} incident(s)** before `{cutoff_str}`. This cannot be undone.")
    col_yes, col_no = st.columns(2)
    with col_yes:
        if st.button(
            "?  YES, DELETE PERMANENTLY",
            width="stretch",
            type="primary",
            key="confirm_delete_old_incidents"
        ):
            deleted, cutoff = ers.purge_old_incidents(ers.RETENTION_DAYS)
            st.session_state.purge_result = (deleted, cutoff)
            st.session_state.purge_confirm = False
            st.rerun()
    with col_no:
        if st.button("?  CANCEL", width="stretch", key="cancel_delete_old_incidents"):
            st.session_state.purge_confirm = False
            st.rerun()

st.markdown("---")

# Incident Log
st.markdown('<div class="section-title">Incident Log</div>', unsafe_allow_html=True)
all_incidents = ers.fetch_incidents(limit=limit)
filtered = [i for i in all_incidents if ers.passes_filter(i, type_filter)]

if not filtered:
    st.markdown(
        '<div style="font-family:IBM Plex Mono,monospace;font-size:0.75rem;color:#586069;padding:20px 0;">No incidents match the current filters.</div>',
        unsafe_allow_html=True
    )
else:
    st.markdown("""
    <div class="tbl-header">
      <span style="width:8px"></span>
      <span style="width:40px">ID</span>
      <span style="width:80px">Type</span>
      <span style="min-width:130px">Device</span>
      <span style="min-width:130px">Triggered By</span>
      <span style="flex:1">Message</span>
      <span style="width:175px">Audio · Email · SMS</span>
      <span style="width:90px">Relay</span>
      <span style="min-width:130px">Time</span>
    </div>""", unsafe_allow_html=True)

    for inc in filtered:
        em = inc.get("emergency_type", "")
        src = inc.get("trigger_source", "")
        device_id = inc.get("device_id") or inc.get("from_ip") or ""
        label = ers.get_device_label(device_id)
        dev_display = label or device_id or "—"
        if len(dev_display) > 22:
            dev_display = dev_display[:21] + "…"

        tb = inc.get("triggered_by")
        if tb:
            triggered_by = f'<span style="font-size:0.78rem;font-weight:600;">{tb}</span>'
        elif src == ers.DRILL_SOURCE:
            triggered_by = '<span style="font-size:0.75rem;color:#f7b731;">Dashboard Drill</span>'
        elif src == "dashboard_manual":
            triggered_by = '<span style="font-size:0.75rem;color:#e63946;">Dashboard Manual</span>'
        else:
            triggered_by = '<span style="font-size:0.75rem;color:#586069;">— unassigned</span>'

        msg = (inc.get("message") or "—")[:50]
        ts = (inc.get("server_time") or "—")[:16]
        alerts = (
            ers.pill(inc.get("audio_status"), f"{inc.get('audio_status') or '—'}") + " " +
            ers.pill(inc.get("email_status"), f"? {inc.get('email_status') or '—'}") + " " +
            ers.pill(inc.get("sms_status"), f"{inc.get('sms_status') or '—'}")
        )
        relay = ers.pill(inc.get("relay_status"), f"? {inc.get('relay_status') or '—'}")

        st.markdown(f"""
        <div class="inc-row {ers.row_cls(em, src)}">
          <span class="status-dot {ers.dot_cls(em, src)}"></span>
          <span class="mono" style="width:40px">#{inc['id']}</span>
          {ers.type_badge(em, src)}
          <span style="min-width:130px;font-family:IBM Plex Mono,monospace;font-size:0.7rem;">{dev_display}</span>
          <span style="min-width:130px">{triggered_by}</span>
          <span style="flex:1;font-size:0.8rem;color:#111111;">{msg}</span>
          <span style="width:175px">{alerts}</span>
          <span style="width:90px">{relay}</span>
          <span class="mono" style="min-width:130px">{ts}</span>
        </div>""", unsafe_allow_html=True)

st.markdown(f"""
<div style="margin-top:40px;padding-top:14px;border-top:1px solid #1e2530;
            font-family:'IBM Plex Mono',monospace;font-size:0.6rem;color:#30363d;
            letter-spacing:2px;text-align:center;">
  ERS · DASHBOARD · retention={ers.RETENTION_DAYS}d
</div>""", unsafe_allow_html=True)
