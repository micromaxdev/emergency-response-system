"""
websites/web_main.py — Entry point for the ERS Web Interface.
st.navigation() controls sidebar order and display names — no file renaming needed.
"""

import streamlit as st
import os, sys
from datetime import datetime

# ════════════════════════════════════════════════════════════════════════════════
#  1. PATH & MODULE IMPORT
# ════════════════════════════════════════════════════════════════════════════════
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

import utils as ers

# ════════════════════════════════════════════════════════════════════════════════
#  2. PAGE CONFIG  — must come before any other st call
# ════════════════════════════════════════════════════════════════════════════════
st.set_page_config(
    page_title="ERS",
    layout="wide",
    initial_sidebar_state="expanded",
)

# ════════════════════════════════════════════════════════════════════════════════
#  3. NAVIGATION — order and names controlled here, no filename prefixes needed
# ════════════════════════════════════════════════════════════════════════════════
pg = st.navigation([
    st.Page("pages/home.py",          title="Home"),
    st.Page("pages/dashboard.py",     title="Dashboard"),
    st.Page("pages/configuration.py", title="Configuration"),
    st.Page("pages/staff.py",         title="Staff"),
    st.Page("pages/logs.py",          title="Logs"),
    st.Page("pages/battery.py",       title="Battery"),
    st.Page("pages/emergency.py",     title="Emergency"),
])
pg.run()
