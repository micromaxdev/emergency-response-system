"""
pages/home.py — ERS Home Page.
System health dashboard and quick navigation.
"""

import streamlit as st
import os, sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import utils as ers

st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()

# ── Sidebar ───────────────────────────────────────────────────────────────────
with st.sidebar:
    st.markdown(ers.sidebar_title("Emergency Response System"), unsafe_allow_html=True)
    st.markdown("---")
    st.caption(f"Server: `{ers.SERVER_HOST}:{ers.SERVER_PORT}`")
    st.caption(f"DB: `{ers.DB_PATH}`")
    st.caption(f"Retention: `{ers.RETENTION_DAYS} days`")

# ── Header ────────────────────────────────────────────────────────────────────
st.markdown(ers.ers_header("Emergency Response System"), unsafe_allow_html=True)

st.markdown("""
<div style="font-family:'IBM Plex Mono',monospace;font-size:0.8rem;color:#586069;
            margin-bottom:36px;letter-spacing:1px;">
  Select a section below to get started.
</div>
""", unsafe_allow_html=True)

# ── Nav cards ─────────────────────────────────────────────────────────────────
nav_pages = [
    ("Dashboard",     "pages/dashboard.py",     "Live incident feed · System status · Test drill"),
    ("Configuration", "pages/configuration.py", "Manage gateways, devices and alert recipients"),
    ("Staff",         "pages/staff.py",          "Add and manage staff names and contact info"),
    ("Logs",          "pages/logs.py",           "Searchable and filterable admin incident log"),
    ("Battery",       "pages/battery.py",        "UPS voltage · battery level · AC power status"),
    ("Emergency",     "pages/emergency.py",      "Trigger mass emergency alerts immediately"),
]

cols = st.columns(len(nav_pages))
for col, (title, path, desc) in zip(cols, nav_pages):
    with col:
        st.markdown(f"""
        <div class="nav-card">
          <div class="nav-card-title">{title}</div>
          <div class="nav-card-desc" style="min-height: 60px;">{desc}</div>
        </div>
        """, unsafe_allow_html=True)
        st.page_link(path, label=f"Open {title}", width="stretch")

st.markdown("---")

# ── Quick status strip ────────────────────────────────────────────────────────
if os.path.exists(ers.DB_PATH):
    sys_status = ers.get_system_status()
    counts     = ers.fetch_counts()

    st.markdown('<div class="section-title">Quick Status</div>', unsafe_allow_html=True)

    c1, c2, c3, c4, c5, c6 = st.columns(6)
    with c1:
        state, detail = sys_status.get("server", ("unknown", "—"))
        st.markdown(f'<div class="kpi-label">Server</div>{ers.status_pill(state, state.upper())}<div class="mono" style="margin-top:4px;">{detail}</div>', unsafe_allow_html=True)
    with c2:
        state, detail = sys_status.get("db", ("unknown", "—"))
        st.markdown(f'<div class="kpi-label">Database</div>{ers.status_pill(state, state.upper())}<div class="mono" style="margin-top:4px;">{detail}</div>', unsafe_allow_html=True)
    with c3:
        st.markdown(f'<div class="kpi-label">Mass Alerts</div><div class="ers-logo" style="font-size:1.8rem; line-height:1;">{counts.get("mass",0)}</div>', unsafe_allow_html=True)
    with c4:
        st.markdown(f'<div class="kpi-label">Personal</div><div class="ers-logo" style="font-size:1.8rem; color:#f4a261; line-height:1;">{counts.get("personal",0)}</div>', unsafe_allow_html=True)
    with c5:
        st.markdown(f'<div class="kpi-label">Drills Run</div><div class="ers-logo" style="font-size:1.8rem; color:#f7b731; line-height:1;">{counts.get("drills",0)}</div>', unsafe_allow_html=True)
    with c6:
        state, detail = sys_status.get("retention", ("unknown", "—"))
        st.markdown(f'<div class="kpi-label">Retention</div>{ers.status_pill(state, state.upper())}<div class="mono" style="margin-top:4px;">{detail}</div>', unsafe_allow_html=True)
else:
    st.warning(f"Database not found at `{ers.DB_PATH}`. Please start the server first.")

# ── Footer ────────────────────────────────────────────────────────────────────
st.markdown(ers.footer(f"ERS / EMERGENCY RESPONSE SYSTEM / {datetime.now().year}"), unsafe_allow_html=True)
