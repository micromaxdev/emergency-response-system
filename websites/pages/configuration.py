"""
websites/pages/configuration.py — Device, Alert, Audio and Map Management.
Full version with no omissions. Centrally managed via utils.py (ers).
"""

import streamlit as st
import os, sys
import plotly.express as px
from PIL import Image

# ════════════════════════════════════════════════════════════════════════════════
#  1. PATH & MODULE IMPORT — Connect to utils.py
# ════════════════════════════════════════════════════════════════════════════════
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(CURRENT_DIR)
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

import utils as ers  
ers.init_map_table()
if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()

# ════════════════════════════════════════════════════════════════════════════════
#  2. PAGE SETUP & SESSION STATE
# ════════════════════════════════════════════════════════════════════════════════
st.set_page_config(page_title="ERS · Configuration", page_icon="⚙️", layout="wide")
st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

if "gw_coords" not in st.session_state: 
    st.session_state.gw_coords = ers.fetch_all_gw_coords()

if "edit_device_id"   not in st.session_state: st.session_state.edit_device_id   = None
if "delete_device_id" not in st.session_state: st.session_state.delete_device_id = None
if "device_saved"     not in st.session_state: st.session_state.device_saved     = False
if "alert_saved"      not in st.session_state: st.session_state.alert_saved      = False
if "audio_saved"      not in st.session_state: st.session_state.audio_saved      = False

with st.sidebar:
    st.markdown('<div class="ers-logo" style="font-size:1.8rem; letter-spacing:4px;">ERS</div>', unsafe_allow_html=True)
    st.markdown('<div class="ers-sub">CONFIGURATION</div>', unsafe_allow_html=True)
    st.markdown("---")
    st.caption("Sections:")
    st.markdown("- Devices\n- Mass Alerts\n- Personal Alerts\n- Audio\n- Map & Gateways")

st.markdown(ers.ers_header("Configuration"), unsafe_allow_html=True)

if not os.path.exists(ers.DB_PATH):
    st.error(f"⚠ Database not found at `{ers.DB_PATH}`. Ensure data/ers.sqlite exists.")
    st.stop()

all_staff = ers.fetch_all_staff()
staff_options           = {s["id"]: f"{s['name']} ({s.get('role','—')})" for s in all_staff}
staff_options_with_none = {None: "— Unassigned —", **staff_options}

# ════════════════════════════════════════════════════════════════════════════════
#  3. DEVICE MANAGEMENT
# ════════════════════════════════════════════════════════════════════════════════
st.markdown('<div class="section-title">Device Management</div>', unsafe_allow_html=True)

if st.session_state.device_saved:
    st.success("✓ Device configuration saved.")
    st.session_state.device_saved = False

with st.expander("➕  Register New Device", expanded=False):
    unregistered = ers.fetch_known_device_ids()
    with st.form("add_device_form"):
        c1, c2 = st.columns(2)
        with c1:
            if unregistered:
                use_existing = st.selectbox(
                    "Seen device ID (from incidents)",
                    ["(type manually below)"] + unregistered,
                )
            else:
                use_existing = "(type manually below)"
                st.caption("No unregistered device IDs in incidents yet.")
            manual_id = st.text_input("Device ID (manual)", placeholder="e.g. pico_room3")
        with c2:
            new_label    = st.text_input("Friendly Label", placeholder="e.g. Room 3 Panic Button")
            new_location = st.text_input("Location",       placeholder="e.g. Building A · Floor 2")

        assigned_id = st.selectbox(
            "Assign to Staff Member",
            options=list(staff_options_with_none.keys()),
            format_func=lambda k: staff_options_with_none[k],
        )

        if st.form_submit_button("Register Device", use_container_width=True):
            dev_id = use_existing if use_existing != "(type manually below)" else manual_id.strip()
            if not dev_id:
                st.error("A Device ID is required.")
            else:
                ers.add_device(dev_id, new_label.strip(), new_location.strip(), assigned_id)
                st.session_state.device_saved = True
                st.rerun()

devices = ers.fetch_all_devices()

if not devices:
    st.markdown('<div class="mono" style="padding:16px 0;">No devices registered yet.</div>', unsafe_allow_html=True)
else:
    st.markdown(f'<div class="mono" style="margin-bottom:12px;">{len(devices)} registered device(s)</div>', unsafe_allow_html=True)
    st.markdown("""
    <div class="tbl-header">
      <span style="min-width:160px">Device ID</span>
      <span style="min-width:160px">Label</span>
      <span style="flex:1">Location</span>
      <span style="min-width:180px">Assigned Staff</span>
      <span style="width:140px">Actions</span>
    </div>""", unsafe_allow_html=True)

    for d in devices:
        did = d["id"]
        if st.session_state.edit_device_id == did:
            with st.form(f"edit_device_{did}"):
                dc1, dc2 = st.columns(2)
                with dc1:
                    e_label    = st.text_input("Label",    value=d.get("label",""))
                    e_location = st.text_input("Location", value=d.get("location",""))
                with dc2:
                    cur = d.get("assigned_staff_id")
                    e_assigned = st.selectbox(
                        "Assigned Staff",
                        options=list(staff_options_with_none.keys()),
                        format_func=lambda k: staff_options_with_none[k],
                        index=list(staff_options_with_none.keys()).index(cur) if cur in staff_options_with_none else 0,
                    )
                sb1, sb2 = st.columns(2)
                with sb1:
                    if st.form_submit_button("✓ Save", use_container_width=True):
                        ers.update_device(did, e_label.strip(), e_location.strip(), e_assigned)
                        st.session_state.edit_device_id = None
                        st.session_state.device_saved   = True
                        st.rerun()
                with sb2:
                    if st.form_submit_button("✕ Cancel", use_container_width=True):
                        st.session_state.edit_device_id = None
                        st.rerun()
        elif st.session_state.delete_device_id == did:
            st.warning(f"Remove device **{d['device_id']}**? Incident history is kept.")
            dc1, dc2 = st.columns(2)
            with dc1:
                if st.button("✓ Yes, Remove", key=f"devdel_yes_{did}", use_container_width=True, type="primary"):
                    ers.delete_device(did)
                    st.session_state.delete_device_id = None
                    st.rerun()
            with dc2:
                if st.button("✕ Cancel", key=f"devdel_no_{did}", use_container_width=True):
                    st.session_state.delete_device_id = None
                    st.rerun()
        else:
            assign_pill = (
                f'<span class="pill pill-ok">👤 {d["staff_name"]}</span>'
                if d.get("assigned_staff_id")
                else '<span class="pill pill-unknown">— unassigned</span>'
            )
            st.markdown(f"""
            <div class="inc-row">
              <span style="min-width:160px; font-family:IBM Plex Mono,monospace;">{d['device_id']}</span>
              <span style="min-width:160px; font-weight:600;">{d.get('label') or '—'}</span>
              <span style="flex:1; color:#586069;">{d.get('location') or '—'}</span>
              <span style="min-width:180px">{assign_pill}</span>
            </div>""", unsafe_allow_html=True)

            ce, cd, _ = st.columns([1, 1, 6])
            with ce:
                if st.button("Edit", key=f"ed_{did}", use_container_width=True):
                    st.session_state.edit_device_id = did
                    st.rerun()
            with cd:
                if st.button("Remove", key=f"rm_{did}", use_container_width=True):
                    st.session_state.delete_device_id = did
                    st.rerun()

# ════════════════════════════════════════════════════════════════════════════════
#  4. ALERT RECIPIENTS — MASS EMERGENCY
# ════════════════════════════════════════════════════════════════════════════════
st.markdown("---")
st.markdown('<div class="section-title">Mass Emergency Alert Recipients</div>', unsafe_allow_html=True)

if st.session_state.alert_saved:
    st.success("✓ Alert assignments updated.")
    st.session_state.alert_saved = False

if not all_staff:
    st.info("No staff found. Add staff on the Staff page first.")
else:
    fresh = ers.fetch_all_staff()
    mc1, mc2 = st.columns(2)

    with mc1:
        st.markdown('✉ Email — Mass Emergency')
        with st.form("mass_email_form"):
            mass_email_states = {}
            for s in fresh:
                mass_email_states[s["id"]] = st.checkbox(
                    f"{s['name']} ({s.get('role','—')})",
                    value=bool(s.get("email_alerts_mass")),
                    key=f"me_{s['id']}",
                    disabled=not s.get("email"),
                )
            if st.form_submit_button("Save Email Recipients", use_container_width=True):
                for s in fresh:
                    ers.update_staff(
                        s["id"], s["name"], s.get("role",""), s.get("email",""), s.get("phone",""),
                        mass_email_states[s["id"]], bool(s.get("email_alerts_personal")),
                        bool(s.get("sms_alerts_mass")), bool(s.get("sms_alerts_personal"))
                    )
                st.session_state.alert_saved = True
                st.rerun()

    with mc2:
        st.markdown('📱 SMS — Mass Emergency')
        with st.form("mass_sms_form"):
            mass_sms_states = {}
            for s in fresh:
                mass_sms_states[s["id"]] = st.checkbox(
                    f"{s['name']} ({s.get('role','—')})",
                    value=bool(s.get("sms_alerts_mass")),
                    key=f"ms_{s['id']}",
                    disabled=not s.get("phone"),
                )
            if st.form_submit_button("Save SMS Recipients", use_container_width=True):
                for s in fresh:
                    ers.update_staff(
                        s["id"], s["name"], s.get("role",""), s.get("email",""), s.get("phone",""),
                        bool(s.get("email_alerts_mass")), bool(s.get("email_alerts_personal")),
                        mass_sms_states[s["id"]], bool(s.get("sms_alerts_personal"))
                    )
                st.session_state.alert_saved = True
                st.rerun()

# ════════════════════════════════════════════════════════════════════════════════
#  5. ALERT RECIPIENTS — PERSONAL EMERGENCY
# ════════════════════════════════════════════════════════════════════════════════
st.markdown("---")
st.markdown('<div class="section-title">Personal Emergency Alert Recipients</div>', unsafe_allow_html=True)

fresh2 = ers.fetch_all_staff()
pc1, pc2 = st.columns(2)

with pc1:
    st.markdown('✉ Email — Personal Emergency')
    with st.form("personal_email_form"):
        pers_email_states = {}
        for s in fresh2:
            pers_email_states[s["id"]] = st.checkbox(
                f"{s['name']} ({s.get('role','—')})",
                value=bool(s.get("email_alerts_personal")),
                key=f"pe_{s['id']}",
                disabled=not s.get("email"),
            )
        if st.form_submit_button("Save Email Recipients", use_container_width=True):
            for s in fresh2:
                ers.update_staff(
                    s["id"], s["name"], s.get("role",""), s.get("email",""), s.get("phone",""),
                    bool(s.get("email_alerts_mass")), pers_email_states[s["id"]],
                    bool(s.get("sms_alerts_mass")), bool(s.get("sms_alerts_personal"))
                )
            st.session_state.alert_saved = True
            st.rerun()

with pc2:
    st.markdown('📱 SMS — Personal Emergency')
    with st.form("personal_sms_form"):
        pers_sms_states = {}
        for s in fresh2:
            pers_sms_states[s["id"]] = st.checkbox(
                f"{s['name']} ({s.get('role','—')})",
                value=bool(s.get("sms_alerts_personal")),
                key=f"ps_{s['id']}",
                disabled=not s.get("phone"),
            )
        if st.form_submit_button("Save SMS Recipients", use_container_width=True):
            for s in fresh2:
                ers.update_staff(
                    s["id"], s["name"], s.get("role",""), s.get("email",""), s.get("phone",""),
                    bool(s.get("email_alerts_mass")), bool(s.get("email_alerts_personal")),
                    bool(s.get("sms_alerts_mass")), pers_sms_states[s["id"]]
                )
            st.session_state.alert_saved = True
            st.rerun()

# ════════════════════════════════════════════════════════════════════════════════
#  6. AUDIO SETTINGS — Corrected Paths
# ════════════════════════════════════════════════════════════════════════════════
st.markdown("---")
st.markdown('<div class="section-title">Alert Audio Files</div>', unsafe_allow_html=True)

AUDIO_DIR = ers.AUDIO_DIR

if st.session_state.audio_saved:
    st.success(f"✓ Audio files saved to {AUDIO_DIR}")
    st.session_state.audio_saved = False

ac1, ac2 = st.columns(2)

with ac1:
    st.markdown('**🔔 Test / Drill Alert Audio**')
    test_audio = st.file_uploader("Upload test audio", type=["mp3", "wav", "ogg"], key="test_up")
    existing_test = next((f for f in sorted(os.listdir(AUDIO_DIR)) if f.startswith("test_alert_")), None)
    if test_audio:
        st.audio(test_audio)
    elif existing_test:
        st.audio(os.path.join(AUDIO_DIR, existing_test))

with ac2:
    st.markdown('**🚨 Mass Emergency Alert Audio**')
    emergency_audio = st.file_uploader("Upload mass emergency audio", type=["mp3", "wav", "ogg"], key="em_up")
    existing_em = next((f for f in sorted(os.listdir(AUDIO_DIR)) if f.startswith("emergency_alert_")), None)
    if emergency_audio:
        st.audio(emergency_audio)
    elif existing_em:
        st.audio(os.path.join(AUDIO_DIR, existing_em))

if st.button("💾 Save Audio Files", use_container_width=True):
    if test_audio:
        for f in os.listdir(AUDIO_DIR):
            if f.startswith("test_alert_"): os.remove(os.path.join(AUDIO_DIR, f))
        with open(os.path.join(AUDIO_DIR, f"test_alert_{test_audio.name}"), "wb") as fh:
            fh.write(test_audio.getbuffer())
    if emergency_audio:
        for f in os.listdir(AUDIO_DIR):
            if f.startswith("emergency_alert_"): os.remove(os.path.join(AUDIO_DIR, f))
        with open(os.path.join(AUDIO_DIR, f"emergency_alert_{emergency_audio.name}"), "wb") as fh:
            fh.write(emergency_audio.getbuffer())
    st.session_state.audio_saved = True
    st.rerun()

# ════════════════════════════════════════════════════════════════════════════════
#  7. MAP & GATEWAY CONFIGURATION — Dynamic Upgrade
# ════════════════════════════════════════════════════════════════════════════════
st.markdown("---")
st.markdown('<div class="section-title">Dynamic Map & Gateway Setup</div>', unsafe_allow_html=True)

current_gws = ers.fetch_all_gw_coords()
if "map_meters_wide" not in st.session_state:
    st.session_state.map_meters_wide = ers.fetch_map_scale()

col_map1, col_map2 = st.columns([1, 2])

with col_map1:
    st.markdown("### 1. Map & Scale")
    uploaded_map = st.file_uploader("Upload Floor Plan", type=["png", "jpg", "jpeg"], key="map_up")
    if uploaded_map:
        save_path = os.path.join(ers.MAP_DIR, "company_map.png")
        with open(save_path, "wb") as f:
            f.write(uploaded_map.getbuffer())
        st.success("✓ Map saved!")
        st.rerun()

    new_scale = st.number_input(
        "Office Actual Width (Meters)", 
        min_value=1.0, 
        value=float(st.session_state.map_meters_wide),
        step=0.5
    )
    if new_scale != st.session_state.map_meters_wide:
        ers.save_map_scale(new_scale)
        st.session_state.map_meters_wide = new_scale
        st.rerun()

    st.write("---")
    st.markdown("### 2. Manage Gateways")
    
    with st.expander("➕ Add New Gateway"):
        cadd1, cadd2 = st.columns([3, 1])
        new_name = cadd1.text_input("Gateway Name", key="new_gw_name")
        if cadd2.button("Add"):
            if new_name and new_name not in current_gws:
                ers.save_gw_coords(new_name, 0.0, 0.0)
                st.rerun()
            else:
                st.error("Duplicate or invalid name.")

    with st.form("gw_dynamic_form"):
        temp_changes = {}
        for gw_id, pos in current_gws.items():
            st.markdown(f"**📍 {gw_id}**")
            cx, cy, cdel = st.columns([2, 2, 1])
            nx = cx.number_input("X (m)", value=float(pos['x']), key=f"x_{gw_id}")
            ny = cy.number_input("Y (m)", value=float(pos['y']), key=f"y_{gw_id}")
            is_del = cdel.checkbox("🗑️", key=f"del_{gw_id}")
            temp_changes[gw_id] = "DELETE" if is_del else {"x": nx, "y": ny}
        
        if st.form_submit_button("💾 Save All Changes", use_container_width=True):
            for gid, action in temp_changes.items():
                if action == "DELETE":
                    ers.delete_gw(gid)
                else:
                    ers.save_gw_coords(gid, action['x'], action['y'])
            st.session_state.gw_coords = ers.fetch_all_gw_coords()
            st.rerun()

with col_map2:
    st.markdown("### 3. Map Preview")
    map_img_path = os.path.join(ers.MAP_DIR, "company_map.png")
    
    if os.path.exists(map_img_path):
        img = Image.open(map_img_path)
        w, h = img.size
        scale = w / st.session_state.map_meters_wide
        
        fig = px.imshow(img)
        
        for gw, pos in current_gws.items():
            fig.add_scatter(
                x=[float(pos['x']) * scale], 
                y=[float(pos['y']) * scale],
                mode="markers+text",
                text=[f"<b>{gw}</b>"],
                textposition="top center",
                marker=dict(color="#e74c3c", size=15, symbol="octagon", line=dict(width=2, color="white")),
                name=gw
            )
        
        fig.update_layout(
            showlegend=False,
            margin=dict(l=0, r=0, t=0, b=0),
            xaxis=dict(visible=True, title=f"Scale: {st.session_state.map_meters_wide}m total"),
            yaxis=dict(visible=True, autorange="reversed")
        )
        st.plotly_chart(fig, use_container_width=True)
    else:
        st.info("Upload a map image to preview.")

# ── Live summary ──────────────────────────────────────────────────────────────
st.markdown("---")
st.markdown('<div class="section-title">Current Assignment Summary</div>', unsafe_allow_html=True)

final_staff = ers.fetch_all_staff()
sc1, sc2, sc3, sc4 = st.columns(4)
groups = [
    (sc1, "✉ Email · Mass",     "email_alerts_mass",     "email"),
    (sc2, "📱 SMS · Mass",       "sms_alerts_mass",       "phone"),
    (sc3, "✉ Email · Personal",  "email_alerts_personal", "email"),
    (sc4, "📱 SMS · Personal",   "sms_alerts_personal",   "phone"),
]
for col, label, flag, field in groups:
    with col:
        members = [s for s in final_staff if s.get(flag)]
        st.markdown(f'<div class="mono" style="font-size:0.6rem; letter-spacing:1px;">{label} ({len(members)})</div>', unsafe_allow_html=True)
        for s in members:
            st.markdown(f'<div style="font-size:0.75rem; border-bottom:1px solid #f0f0f0; padding:2px 0;">{s["name"]}</div>', unsafe_allow_html=True)

st.markdown('<div style="margin-top:40px; text-align:center; font-family:IBM Plex Mono,monospace; font-size:0.6rem; color:#30363d; letter-spacing:2px;">ERS · CONFIGURATION</div>', unsafe_allow_html=True)
