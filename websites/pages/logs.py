"""
pages/2_Logs.py — Admin incident log.
Features: triggered_by name, log export, mass emergency trigger, filters.
"""

import streamlit as st
import os, sys, csv, io
from datetime import datetime, timedelta

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import utils as ers

st.set_page_config(page_title="ERS · Logs", page_icon="📋", layout="wide")
st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()
def fetch_logs(search="", em_types=None, date_from=None, date_to=None,
               alert_filter=None, limit=200):
    conn = ers.db_connect()
    if not conn:
        return []
    try:
        conditions, params = [], []

        if search:
            conditions.append("""
                (triggered_by LIKE ? OR message LIKE ? OR device_id LIKE ?
                 OR emergency_type LIKE ? OR from_ip LIKE ?)
            """)
            s = f"%{search}%"
            params += [s, s, s, s, s]

        if em_types:
            clauses = []
            if "test_drill" in em_types:
                clauses.append(f"trigger_source = '{ers.DRILL_SOURCE}'")
            real = [t for t in em_types if t != "test_drill"]
            if real:
                ph = ",".join("?" * len(real))
                clauses.append(f"(emergency_type IN ({ph}) AND trigger_source!='{ers.DRILL_SOURCE}')")
                params += real
            if clauses:
                conditions.append("(" + " OR ".join(clauses) + ")")

        if date_from:
            conditions.append("server_time >= ?")
            params.append(date_from.strftime("%Y-%m-%d 00:00:00"))
        if date_to:
            conditions.append("server_time <= ?")
            params.append(date_to.strftime("%Y-%m-%d 23:59:59"))

        if alert_filter == "email_failed":
            conditions.append("email_status='failed'")
        elif alert_filter == "sms_failed":
            conditions.append("sms_status='failed'")
        elif alert_filter == "audio_failed":
            conditions.append("audio_status='failed'")
        elif alert_filter == "relay_failed":
            conditions.append("relay_status IN ('failed','queue_full')")
        elif alert_filter == "any_failed":
            conditions.append("""
                (email_status='failed' OR sms_status='failed'
                 OR audio_status='failed' OR relay_status IN ('failed','queue_full'))
            """)

        where = ("WHERE " + " AND ".join(conditions)) if conditions else ""
        params.append(limit)

        rows = conn.execute(f"""
            SELECT id, server_time, device_id, triggered_by, emergency_type,
                   trigger_source, message,
                   audio_status, email_status, sms_status,
                   relay_status, relay_error, email_error, sms_error, audio_error
            FROM incidents
            {where}
            ORDER BY id DESC LIMIT ?
        """, params).fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()

def fetch_export(days: int):
    """Fetch all incidents from the last N days for CSV export."""
    conn = ers.db_connect()
    if not conn:
        return []
    try:
        cutoff = (datetime.now() - timedelta(days=days)).strftime("%Y-%m-%d %H:%M:%S")
        rows = conn.execute("""
            SELECT id, server_time, emergency_type, trigger_source,
                   triggered_by, device_id, message,
                   audio_status, email_status, sms_status, relay_status,
                   from_ip, audio_error, email_error, sms_error, relay_error
            FROM incidents
            WHERE server_time >= ?
            ORDER BY id DESC
        """, (cutoff,)).fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()

def count_logs():
    conn = ers.db_connect()
    if not conn:
        return 0
    try:
        return conn.execute("SELECT COUNT(*) FROM incidents").fetchone()[0]
    finally:
        conn.close()

def build_csv(rows: list) -> bytes:
    buf = io.StringIO()
    if not rows:
        return b""
    writer = csv.DictWriter(buf, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)
    return buf.getvalue().encode("utf-8")

# ── Real mass emergency trigger ───────────────────────────────────────────────
def trigger_real_mass(initiated_by: str = "Dashboard Admin"):
    payload = {
        "emergency":         True,
        "emergency_type":    "mass",
        "trigger_source":    "dashboard_manual",
        "device_id":         "DASHBOARD_MANUAL",
        "emergency_message": f"MASS EMERGENCY — Manually triggered via dashboard by {initiated_by}",
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
    st.markdown('<div style="font-family:Bebas Neue,sans-serif;font-size:1.8rem;letter-spacing:5px;color:#4065a1;margin-bottom:2px;">ERS</div>', unsafe_allow_html=True)
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:3px;margin-bottom:20px;">LOGS</div>', unsafe_allow_html=True)
    st.markdown("---")

    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:10px;">Search</div>', unsafe_allow_html=True)
    search = st.text_input("Keyword", placeholder="name, message, device…", label_visibility="collapsed")

    st.markdown("---")
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:10px;">Filters</div>', unsafe_allow_html=True)

    em_types = st.multiselect(
        "Incident Type",
        ["mass", "personal", "test_drill"],
        default=["mass", "personal", "test_drill"],
    )

    today     = datetime.now().date()
    date_from = st.date_input("From date", value=today - timedelta(days=30))
    date_to   = st.date_input("To date",   value=today)

    alert_filter = st.selectbox(
        "Alert status",
        ["All", "any_failed", "email_failed", "sms_failed", "audio_failed", "relay_failed"],
        format_func=lambda x: {
            "All": "All statuses", "any_failed": "Any alert failed",
            "email_failed": "Email failed", "sms_failed": "SMS failed",
            "audio_failed": "Audio failed", "relay_failed": "Relay failed",
        }.get(x, x)
    )

    st.markdown("---")
    limit = st.slider("Max rows", 20, 1000, 200, step=20)

    st.markdown("---")
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:10px;">Export</div>', unsafe_allow_html=True)
    export_days = st.number_input("Export last N days", min_value=1, max_value=3650, value=30, step=1)
    export_rows = fetch_export(int(export_days))
    export_filename = f"ers_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_last{export_days}d.csv"
    st.download_button(
        label=f"⬇ Download CSV ({len(export_rows)} rows)",
        data=build_csv(export_rows),
        file_name=export_filename,
        mime="text/csv",
        width="stretch",
        disabled=(len(export_rows) == 0),
    )

# ── Main ──────────────────────────────────────────────────────────────────────
st.markdown(ers.ers_header("Incident Log"), unsafe_allow_html=True)

if not os.path.exists(ers.DB_PATH):
    st.error(f"⚠ Database not found at `{ers.DB_PATH}`.")
    st.stop()

# ════════════════════════════════════════════════════════════════════════════════
#  INCIDENT LOG
# ════════════════════════════════════════════════════════════════════════════════
logs  = fetch_logs(
    search=search, em_types=em_types, date_from=date_from,
    date_to=date_to, alert_filter=None if alert_filter == "All" else alert_filter,
    limit=limit,
)
total = count_logs()

st.markdown(f"""
<div style="font-family:'IBM Plex Mono',monospace;font-size:0.7rem;color:#586069;
            margin-bottom:18px;letter-spacing:1px;">
  Showing <strong style="color:#111;">{len(logs)}</strong> of
  <strong style="color:#111;">{total}</strong> total incidents
  {' · filtered' if (search or alert_filter != 'All') else ''}
</div>
""", unsafe_allow_html=True)

if not logs:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.75rem;color:#586069;padding:20px 0;">No incidents match the current filters.</div>', unsafe_allow_html=True)
else:
    st.markdown("""
    <div class="tbl-header">
      <span style="width:8px"></span>
      <span style="width:45px">ID</span>
      <span style="width:80px">Type</span>
      <span style="min-width:160px">Triggered By</span>
      <span style="flex:1">Message</span>
      <span style="width:210px">Audio · Email · SMS · Relay</span>
      <span style="min-width:140px">Timestamp</span>
    </div>""", unsafe_allow_html=True)

    for inc in logs:
        em  = inc.get("emergency_type", "")
        src = inc.get("trigger_source", "")

        # Triggered by — from stored snapshot in DB, not live lookup
        tb = inc.get("triggered_by")
        if tb:
            triggered_cell = f'<span style="font-weight:600;font-size:0.82rem;">{tb}</span>'
        elif src == ers.DRILL_SOURCE:
            triggered_cell = '<span style="font-size:0.75rem;color:#f7b731;">Dashboard Drill</span>'
        elif src == "dashboard_manual":
            triggered_cell = '<span style="font-size:0.75rem;color:#e63946;">Dashboard Manual</span>'
        else:
            triggered_cell = '<span style="font-size:0.75rem;color:#586069;">— unassigned</span>'

        msg = (inc.get("message") or "—")[:60]
        ts  = (inc.get("server_time") or "—")[:16]

        def apill(sk, ek, icon):
            s = inc.get(sk)
            return ers.pill(s, f"{icon} {s or '—'}")

        alerts = (
            apill("audio_status", "audio_error", "🔊") + " " +
            apill("email_status", "email_error", "✉")  + " " +
            apill("sms_status",   "sms_error",   "📱") + " " +
            ers.pill(inc.get("relay_status"), f"⚡ {inc.get('relay_status') or '—'}")
        )

        errors = []
        if inc.get("audio_error"): errors.append(f"Audio: {inc['audio_error'][:60]}")
        if inc.get("email_error"): errors.append(f"Email: {inc['email_error'][:60]}")
        if inc.get("sms_error"):   errors.append(f"SMS: {inc['sms_error'][:60]}")
        if inc.get("relay_error"): errors.append(f"Relay: {inc['relay_error'][:60]}")
        error_row = ""
        if errors:
            error_row = f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#e63946;padding:2px 14px 8px 30px;">{" · ".join(errors)}</div>'

        st.markdown(f"""
        <div class="inc-row {ers.row_cls(em, src)}">
          <span class="status-dot {ers.dot_cls(em, src)}"></span>
          <span class="mono" style="width:45px">#{inc['id']}</span>
          {ers.type_badge(em, src)}
          <span style="min-width:160px">{triggered_cell}</span>
          <span style="flex:1;font-size:0.8rem;color:#111111;">{msg}</span>
          <span style="width:210px">{alerts}</span>
          <span class="mono" style="min-width:140px">{ts}</span>
        </div>
        {error_row}
        """, unsafe_allow_html=True)

st.markdown(f"""
<div style="margin-top:40px;padding-top:14px;border-top:1px solid #1e2530;
            font-family:'IBM Plex Mono',monospace;font-size:0.6rem;color:#30363d;
            letter-spacing:2px;text-align:center;">
  ERS · LOGS · {ers.DB_PATH}
</div>""", unsafe_allow_html=True)
