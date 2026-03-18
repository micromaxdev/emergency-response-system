"""
Home.py — ERS entry point and navigation hub.
Run with: streamlit run Home.py
"""

import streamlit as st
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import ers_shared as ers

st.set_page_config(
    page_title="ERS · Home",
    page_icon="",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

# Ensure new DB tables exist on first run
if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()

# ── Sidebar ───────────────────────────────────────────────────────────────────
with st.sidebar:
    st.markdown('<div style="font-family:Bebas Neue,sans-serif;font-size:1.8rem;letter-spacing:5px;color:#4065a1;margin-bottom:2px;">ERS</div>', unsafe_allow_html=True)
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:3px;margin-bottom:20px;">EMERGENCY RESPONSE SYSTEM</div>', unsafe_allow_html=True)
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
pages = [
    ("","Dashboard",     "pages/1_Dashboard.py",  "Live incident feed · System status · Test drill · Alert retention"),
    ("","Logs",          "pages/2_Logs.py",        "Searchable and filterable admin incident log · CSV export"),
    ("","Staff",         "pages/3_Staff.py",       "Add and manage staff names, roles, emails and phone numbers"),
    ("️","Configuration", "pages/4_Configuration.py", "Assign devices to staff · Manage email and SMS alert recipients"),
    ("","Emergency",     "pages/5_Emergency.py",   "Real mass emergency trigger — activates all alert systems immediately"),
]

cols = st.columns(len(pages))
for col, (icon, title, path, desc) in zip(cols, pages):
    with col:
        st.markdown(f"""
        <div class="nav-card">
          <div class="nav-card-title">{title}</div>
          <div class="nav-card-desc">{desc}</div>
        </div>
        """, unsafe_allow_html=True)
        st.page_link(path, label=f"Open {title} →", use_container_width=True)

st.markdown("---")

# ── Quick status strip ────────────────────────────────────────────────────────
if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()
    sys_status = ers.get_system_status()
    counts     = ers.fetch_counts()

    st.markdown('<div class="section-title">Quick Status</div>', unsafe_allow_html=True)

    c1, c2, c3, c4, c5, c6 = st.columns(6)
    with c1:
        state, detail = sys_status.get("server", ("unknown", "—"))
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px;">Server</div>{ers.status_pill(state, state.upper())}<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;margin-top:4px;">{detail}</div>', unsafe_allow_html=True)
    with c2:
        state, detail = sys_status.get("db", ("unknown", "—"))
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px;">Database</div>{ers.status_pill(state, state.upper())}<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;margin-top:4px;">{detail}</div>', unsafe_allow_html=True)
    with c3:
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px;">Mass Incidents</div><div style="font-family:Bebas Neue,sans-serif;font-size:1.8rem;color:#4065a1;line-height:1;">{counts.get("mass",0)}</div>', unsafe_allow_html=True)
    with c4:
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px;">Personal</div><div style="font-family:Bebas Neue,sans-serif;font-size:1.8rem;color:#f4a261;line-height:1;">{counts.get("personal",0)}</div>', unsafe_allow_html=True)
    with c5:
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px;">Drills Run</div><div style="font-family:Bebas Neue,sans-serif;font-size:1.8rem;color:#f7b731;line-height:1;">{counts.get("drills",0)}</div>', unsafe_allow_html=True)
    with c6:
        state, detail = sys_status.get("retention", ("unknown", "—"))
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px;">Retention</div>{ers.status_pill(state, state.upper())}<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;margin-top:4px;">{detail}</div>', unsafe_allow_html=True)
else:
    st.warning(f"⚠ Database not found at `{ers.DB_PATH}`. Start the ERS server (`python3 test4.py`) first.")

# ── Footer ────────────────────────────────────────────────────────────────────
st.markdown(f"""
<div style="margin-top:40px;padding-top:14px;border-top:1px solid #1e2530;
            font-family:'IBM Plex Mono',monospace;font-size:0.6rem;color:#30363d;
            letter-spacing:2px;text-align:center;">
  ERS · EMERGENCY RESPONSE SYSTEM · {ers.DB_PATH}
</div>
""", unsafe_allow_html=True)
