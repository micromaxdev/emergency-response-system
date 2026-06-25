"""Admin incident log with filters, CSV export and pagination."""

import csv
import io
import os
import sys
from datetime import datetime, timedelta

import streamlit as st

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import utils as ers

st.set_page_config(page_title="ERS - Logs", layout="wide")
st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()


def build_csv(rows: list) -> bytes:
    """Build a CSV download payload from incident rows."""
    buf = io.StringIO()
    if not rows:
        return b""
    writer = csv.DictWriter(buf, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)
    return buf.getvalue().encode("utf-8")


def reset_page_when_filters_change(signature):
    """Reset log pagination when any filter value changes."""
    if "logs_filter_signature" not in st.session_state:
        st.session_state.logs_filter_signature = signature
    if "logs_page" not in st.session_state:
        st.session_state.logs_page = 1
    if st.session_state.logs_filter_signature != signature:
        st.session_state.logs_filter_signature = signature
        st.session_state.logs_page = 1


with st.sidebar:
    st.markdown(ers.sidebar_title("Logs"), unsafe_allow_html=True)
    st.markdown("---")

    st.markdown(ers.section_caption("Search"), unsafe_allow_html=True)
    search = st.text_input("Keyword", placeholder="name, message, device", label_visibility="collapsed")

    st.markdown("---")
    st.markdown(ers.section_caption("Filters"), unsafe_allow_html=True)

    em_types = st.multiselect(
        "Incident Type",
        ["mass", "personal", "test_drill"],
        default=["mass", "personal", "test_drill"],
    )

    today = datetime.now().date()
    date_from = st.date_input("From date", value=today - timedelta(days=30))
    date_to = st.date_input("To date", value=today)

    alert_filter = st.selectbox(
        "Alert status",
        ["All", "any_failed", "email_failed", "sms_failed", "audio_failed", "relay_failed"],
        format_func=lambda x: {
            "All": "All statuses",
            "any_failed": "Any alert failed",
            "email_failed": "Email failed",
            "sms_failed": "SMS failed",
            "audio_failed": "Audio failed",
            "relay_failed": "Relay failed",
        }.get(x, x),
    )

    st.markdown("---")
    rows_per_page = st.selectbox("Rows per page", [20, 50, 100, 200], index=1)

    st.markdown("---")
    st.markdown(ers.section_caption("Export"), unsafe_allow_html=True)
    export_days = st.number_input("Export last N days", min_value=1, max_value=3650, value=30, step=1)
    export_rows = ers.fetch_log_export(int(export_days))
    export_filename = f"ers_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_last{export_days}d.csv"
    st.download_button(
        label=f"Download CSV ({len(export_rows)} rows)",
        data=build_csv(export_rows),
        file_name=export_filename,
        mime="text/csv",
        width="stretch",
        disabled=(len(export_rows) == 0),
    )


st.markdown(ers.ers_header("Incident Log"), unsafe_allow_html=True)

if not os.path.exists(ers.DB_PATH):
    st.error(f"Database not found at `{ers.DB_PATH}`.")
    st.stop()

filter_signature = (
    search,
    tuple(em_types),
    date_from.isoformat() if date_from else "",
    date_to.isoformat() if date_to else "",
    alert_filter,
    rows_per_page,
)
reset_page_when_filters_change(filter_signature)

active_alert_filter = None if alert_filter == "All" else alert_filter
total = ers.count_logs(
    search=search,
    em_types=em_types,
    date_from=date_from,
    date_to=date_to,
    alert_filter=active_alert_filter,
)
total_pages = max(1, (total + rows_per_page - 1) // rows_per_page)
st.session_state.logs_page = min(max(1, st.session_state.logs_page), total_pages)

page = st.session_state.logs_page
offset = (page - 1) * rows_per_page
logs = ers.fetch_logs(
    search=search,
    em_types=em_types,
    date_from=date_from,
    date_to=date_to,
    alert_filter=active_alert_filter,
    limit=rows_per_page,
    offset=offset,
)

start_row = offset + 1 if total else 0
end_row = min(offset + len(logs), total)
filtered_note = " filtered" if (search or alert_filter != "All" or set(em_types) != {"mass", "personal", "test_drill"}) else ""
st.markdown(f"""
<div class="mono" style="margin-bottom:18px;letter-spacing:1px;">
  Showing <strong style="color:#111;">{start_row}-{end_row}</strong> of
  <strong style="color:#111;">{total}</strong> incidents{filtered_note}
</div>
""", unsafe_allow_html=True)

nav_prev, nav_page, nav_next = st.columns([1, 2, 1])
with nav_prev:
    if st.button("Previous", width="stretch", disabled=(page <= 1 or total == 0)):
        st.session_state.logs_page = max(1, page - 1)
        st.rerun()
with nav_page:
    selected_page = st.number_input(
        "Page",
        min_value=1,
        max_value=total_pages,
        value=page,
        step=1,
        disabled=(total_pages <= 1),
    )
    if selected_page != page:
        st.session_state.logs_page = int(selected_page)
        st.rerun()
with nav_next:
    if st.button("Next", width="stretch", disabled=(page >= total_pages or total == 0)):
        st.session_state.logs_page = min(total_pages, page + 1)
        st.rerun()

if not logs:
    st.markdown('<div class="mono" style="padding:20px 0;">No incidents match the current filters.</div>', unsafe_allow_html=True)
else:
    st.markdown("""
    <div class="tbl-header">
      <span style="width:8px"></span>
      <span style="width:45px">ID</span>
      <span style="width:80px">Type</span>
      <span style="min-width:160px">Triggered By</span>
      <span style="flex:1">Message</span>
      <span style="width:240px">Audio / Email / SMS / Relay</span>
      <span style="min-width:140px">Timestamp</span>
    </div>""", unsafe_allow_html=True)

    for inc in logs:
        em = inc.get("emergency_type", "")
        src = inc.get("trigger_source", "")

        tb = inc.get("triggered_by")
        if tb:
            triggered_cell = f'<span style="font-weight:600;font-size:0.82rem;">{tb}</span>'
        elif src == ers.DRILL_SOURCE:
            triggered_cell = '<span style="font-size:0.75rem;color:#f7b731;">Dashboard Drill</span>'
        elif src == "dashboard_manual":
            triggered_cell = '<span style="font-size:0.75rem;color:#e63946;">Dashboard Manual</span>'
        else:
            triggered_cell = '<span style="font-size:0.75rem;color:#586069;">unassigned</span>'

        msg = (inc.get("message") or "-")[:60]
        ts = (inc.get("server_time") or "-")[:16]
        alerts = " ".join([
            ers.channel_pill("Audio", inc.get("audio_status")),
            ers.channel_pill("Email", inc.get("email_status")),
            ers.channel_pill("SMS", inc.get("sms_status")),
            ers.channel_pill("Relay", inc.get("relay_status")),
        ])

        errors = []
        if inc.get("audio_error"):
            errors.append(f"Audio: {inc['audio_error'][:60]}")
        if inc.get("email_error"):
            errors.append(f"Email: {inc['email_error'][:60]}")
        if inc.get("sms_error"):
            errors.append(f"SMS: {inc['sms_error'][:60]}")
        if inc.get("relay_error"):
            errors.append(f"Relay: {inc['relay_error'][:60]}")
        error_row = ""
        if errors:
            error_row = f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#e63946;padding:2px 14px 8px 30px;">{" / ".join(errors)}</div>'

        st.markdown(f"""
        <div class="inc-row {ers.row_cls(em, src)}">
          <span class="status-dot {ers.dot_cls(em, src)}"></span>
          <span class="mono" style="width:45px">#{inc['id']}</span>
          {ers.type_badge(em, src)}
          <span style="min-width:160px">{triggered_cell}</span>
          <span style="flex:1;font-size:0.8rem;color:#111111;">{msg}</span>
          <span style="width:240px">{alerts}</span>
          <span class="mono" style="min-width:140px">{ts}</span>
        </div>
        {error_row}
        """, unsafe_allow_html=True)

st.markdown(ers.footer(f"ERS / LOGS / {ers.DB_PATH}"), unsafe_allow_html=True)
