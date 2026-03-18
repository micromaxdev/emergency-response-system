"""
pages/4_Configuration.py — Device management and alert recipient assignment.
Email/SMS recipients are configured separately for mass and personal emergencies
"""



import streamlit as st
import os, sys
import plotly.express as px
from PIL import Image
import ers_shared as ers
ers.init_map_table()

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) 
MAP_DIR = os.path.join(BASE_DIR, "assets", "maps")
os.makedirs(MAP_DIR, exist_ok=True)

if "gw_coords" not in st.session_state: 
    try: 
        st.session_state.gw_coords = ers.fetch_all_gw_coords() 
    except:  
        st.session_state.gw_coords = { 
         "GW1": {"x": 0.0, "y": 0.0}, 
         "GW2": {"x": 5.0, "y": 0.0}, 
         "GW3": {"x": 2.5, "y": 5.0}, 
         }



sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))


st.set_page_config(page_title="ERS · Configuration", page_icon="⚙️", layout="wide")
st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()

if "edit_device_id"   not in st.session_state: st.session_state.edit_device_id   = None
if "delete_device_id" not in st.session_state: st.session_state.delete_device_id = None
if "device_saved"     not in st.session_state: st.session_state.device_saved     = False
if "alert_saved"      not in st.session_state: st.session_state.alert_saved      = False

with st.sidebar:
    st.markdown('<div style="font-family:Bebas Neue,sans-serif;font-size:1.8rem;letter-spacing:5px;color:#4065a1;margin-bottom:2px;">ERS</div>', unsafe_allow_html=True)
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:3px;margin-bottom:20px;">CONFIGURATION</div>', unsafe_allow_html=True)
    st.markdown("---")
    st.caption("Sections:")
    st.markdown("- Devices\n- Mass Emergency Alerts\n- Personal Emergency Alerts")

st.markdown(ers.ers_header("Configuration"), unsafe_allow_html=True)

if not os.path.exists(ers.DB_PATH):
    st.error(f"⚠ Database not found at `{ers.DB_PATH}`.")
    st.stop()

all_staff = ers.fetch_all_staff()
staff_options           = {s["id"]: f"{s['name']} ({s.get('role','—')})" for s in all_staff}
staff_options_with_none = {None: "— Unassigned —", **staff_options}

# ════════════════════════════════════════════════════════════════════════════════
#  DEVICES
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
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.75rem;color:#586069;padding:16px 0;">No devices registered yet.</div>', unsafe_allow_html=True)
else:
    st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.7rem;color:#586069;margin-bottom:12px;">{len(devices)} registered device(s)</div>', unsafe_allow_html=True)
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
                        index=list(staff_options_with_none.keys()).index(cur)
                              if cur in staff_options_with_none else 0,
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
              <span style="min-width:160px;font-family:IBM Plex Mono,monospace;font-size:0.72rem;">{d['device_id']}</span>
              <span style="min-width:160px;font-weight:600;font-size:0.82rem;">{d.get('label') or '—'}</span>
              <span style="flex:1;font-size:0.78rem;color:#586069;">{d.get('location') or '—'}</span>
              <span style="min-width:180px">{assign_pill}</span>
            </div>""", unsafe_allow_html=True)

            col_edit, col_del, _ = st.columns([1, 1, 6])
            with col_edit:
                if st.button("Edit",   key=f"devedit_{did}",   use_container_width=True):
                    st.session_state.edit_device_id   = did
                    st.session_state.delete_device_id = None
                    st.rerun()
            with col_del:
                if st.button("Remove", key=f"devdelete_{did}", use_container_width=True):
                    st.session_state.delete_device_id = did
                    st.session_state.edit_device_id   = None
                    st.rerun()

# ════════════════════════════════════════════════════════════════════════════════
#  ALERT RECIPIENTS — MASS EMERGENCY
# ════════════════════════════════════════════════════════════════════════════════
st.markdown("---")
st.markdown("""
<div class="section-title">Mass Emergency Alert Recipients</div>
<div style="font-family:'IBM Plex Mono',monospace;font-size:0.7rem;color:#586069;margin-bottom:16px;line-height:1.8;">
  These staff members receive alerts when a <strong>mass emergency</strong> is triggered.
</div>
""", unsafe_allow_html=True)

if st.session_state.alert_saved:
    st.success("✓ Alert assignments updated.")
    st.session_state.alert_saved = False

if not all_staff:
    st.info("No staff found. Add staff on the Staff page first.")
else:
    fresh = ers.fetch_all_staff()
    mc1, mc2 = st.columns(2)

    with mc1:
        st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:12px;">✉ Email — Mass Emergency</div>', unsafe_allow_html=True)
        with st.form("mass_email_form"):
            mass_email_states = {}
            for s in fresh:
                mass_email_states[s["id"]] = st.checkbox(
                    f"{s['name']} ({s.get('role','—')})  —  {s.get('email') or 'no email'}",
                    value=bool(s.get("email_alerts_mass")),
                    key=f"me_{s['id']}",
                    disabled=not s.get("email"),
                )
            if st.form_submit_button("Save", use_container_width=True):
                for s in fresh:
                    ers.update_staff(
                        s["id"], s["name"], s.get("role",""), s.get("email",""), s.get("phone",""),
                        mass_email_states[s["id"]], bool(s.get("email_alerts_personal")),
                        bool(s.get("sms_alerts_mass")), bool(s.get("sms_alerts_personal"))
                    )
                st.session_state.alert_saved = True
                st.rerun()

    with mc2:
        st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:12px;">📱 SMS — Mass Emergency</div>', unsafe_allow_html=True)
        with st.form("mass_sms_form"):
            mass_sms_states = {}
            for s in fresh:
                mass_sms_states[s["id"]] = st.checkbox(
                    f"{s['name']} ({s.get('role','—')})  —  {s.get('phone') or 'no phone'}",
                    value=bool(s.get("sms_alerts_mass")),
                    key=f"ms_{s['id']}",
                    disabled=not s.get("phone"),
                )
            if st.form_submit_button("Save", use_container_width=True):
                for s in fresh:
                    ers.update_staff(
                        s["id"], s["name"], s.get("role",""), s.get("email",""), s.get("phone",""),
                        bool(s.get("email_alerts_mass")), bool(s.get("email_alerts_personal")),
                        mass_sms_states[s["id"]], bool(s.get("sms_alerts_personal"))
                    )
                st.session_state.alert_saved = True
                st.rerun()

# ════════════════════════════════════════════════════════════════════════════════
#  ALERT RECIPIENTS — PERSONAL EMERGENCY
# ════════════════════════════════════════════════════════════════════════════════
st.markdown("---")
st.markdown("""
<div class="section-title">Personal Emergency Alert Recipients</div>
<div style="font-family:'IBM Plex Mono',monospace;font-size:0.7rem;color:#586069;margin-bottom:16px;line-height:1.8;">
  These staff members receive alerts when a <strong>personal emergency</strong> is triggered.
</div>
""", unsafe_allow_html=True)

fresh2 = ers.fetch_all_staff()
pc1, pc2 = st.columns(2)

with pc1:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:12px;">✉ Email — Personal Emergency</div>', unsafe_allow_html=True)
    with st.form("personal_email_form"):
        pers_email_states = {}
        for s in fresh2:
            pers_email_states[s["id"]] = st.checkbox(
                f"{s['name']} ({s.get('role','—')})  —  {s.get('email') or 'no email'}",
                value=bool(s.get("email_alerts_personal")),
                key=f"pe_{s['id']}",
                disabled=not s.get("email"),
            )
        if st.form_submit_button("Save", use_container_width=True):
            for s in fresh2:
                ers.update_staff(
                    s["id"], s["name"], s.get("role",""), s.get("email",""), s.get("phone",""),
                    bool(s.get("email_alerts_mass")), pers_email_states[s["id"]],
                    bool(s.get("sms_alerts_mass")), bool(s.get("sms_alerts_personal"))
                )
            st.session_state.alert_saved = True
            st.rerun()

with pc2:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:12px;">📱 SMS — Personal Emergency</div>', unsafe_allow_html=True)
    with st.form("personal_sms_form"):
        pers_sms_states = {}
        for s in fresh2:
            pers_sms_states[s["id"]] = st.checkbox(
                f"{s['name']} ({s.get('role','—')})  —  {s.get('phone') or 'no phone'}",
                value=bool(s.get("sms_alerts_personal")),
                key=f"ps_{s['id']}",
                disabled=not s.get("phone"),
            )
        if st.form_submit_button("Save", use_container_width=True):
            for s in fresh2:
                ers.update_staff(
                    s["id"], s["name"], s.get("role",""), s.get("email",""), s.get("phone",""),
                    bool(s.get("email_alerts_mass")), bool(s.get("email_alerts_personal")),
                    bool(s.get("sms_alerts_mass")), pers_sms_states[s["id"]]
                )
            st.session_state.alert_saved = True
            st.rerun()

# ════════════════════════════════════════════════════════════════════════════════
#  AUDIO SETTINGS
# ════════════════════════════════════════════════════════════════════════════════
st.markdown("---")
st.markdown("""
<div class="section-title">Alert Audio Files</div>
<div style="font-family:'IBM Plex Mono',monospace;font-size:0.7rem;color:#586069;margin-bottom:16px;line-height:1.8;">
  Upload the audio files that will play when each alert type is triggered.
  Accepted formats: mp3, wav, ogg.
</div>
""", unsafe_allow_html=True)

# Fixed path — must match AUDIO_DIR in audio_alert.py
AUDIO_DIR = '/home/micromax/ers_server/audio'
os.makedirs(AUDIO_DIR, exist_ok=True)

if "audio_saved" not in st.session_state:
    st.session_state.audio_saved = False

if st.session_state.audio_saved:
    st.success("✓ Audio files saved.")
    st.session_state.audio_saved = False

ac1, ac2 = st.columns(2)

with ac1:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:12px;">🔔 Test / Drill Alert Audio</div>', unsafe_allow_html=True)
    test_audio = st.file_uploader(
        "Drag and drop test alert audio here",
        type=["mp3", "wav", "ogg"],
        key="test_audio_upload",
    )
    existing_test = next(
        (f for f in sorted(os.listdir(AUDIO_DIR)) if f.startswith("test_alert_")), None
    )
    if test_audio:
        st.audio(test_audio, format=test_audio.type)
    elif existing_test:
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.7rem;color:#586069;margin-top:6px;">Current file: <strong>{existing_test}</strong></div>', unsafe_allow_html=True)
        st.audio(os.path.join(AUDIO_DIR, existing_test))

with ac2:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:12px;">🚨 Mass Emergency Alert Audio</div>', unsafe_allow_html=True)
    emergency_audio = st.file_uploader(
        "Drag and drop mass emergency audio here",
        type=["mp3", "wav", "ogg"],
        key="emergency_audio_upload",
    )
    existing_emergency = next(
        (f for f in sorted(os.listdir(AUDIO_DIR)) if f.startswith("emergency_alert_")), None
    )
    if emergency_audio:
        st.audio(emergency_audio, format=emergency_audio.type)
    elif existing_emergency:
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.7rem;color:#586069;margin-top:6px;">Current file: <strong>{existing_emergency}</strong></div>', unsafe_allow_html=True)
        st.audio(os.path.join(AUDIO_DIR, existing_emergency))

if st.button("💾 Save Audio Files", use_container_width=False):
    if not test_audio and not emergency_audio:
        st.warning("⚠ No new audio files selected.")
    else:
        if test_audio:
            for f in os.listdir(AUDIO_DIR):
                if f.startswith("test_alert_"):
                    os.remove(os.path.join(AUDIO_DIR, f))
            with open(os.path.join(AUDIO_DIR, "test_alert_" + test_audio.name), "wb") as fh:
                fh.write(test_audio.getbuffer())
        if emergency_audio:
            for f in os.listdir(AUDIO_DIR):
                if f.startswith("emergency_alert_"):
                    os.remove(os.path.join(AUDIO_DIR, f))
            with open(os.path.join(AUDIO_DIR, "emergency_alert_" + emergency_audio.name), "wb") as fh:
                fh.write(emergency_audio.getbuffer())
        st.session_state.audio_saved = True
        st.rerun()
        

# ════════════════════════════════════════════════════════════════════════════════
#  MAP & GATEWAY CONFIGURATION
# ════════════════════════════════════════════════════════════════════════════════
st.markdown("---")
st.markdown('<div class="section-title">Company Map & Gateway Setup</div>', unsafe_allow_html=True)

if "map_meters_wide" not in st.session_state:
    st.session_state.map_meters_wide = ers.fetch_map_scale()

col_map1, col_map2 = st.columns([1, 2])

with col_map1:
    st.markdown("### 1. Map & Scale")
    uploaded_map = st.file_uploader("Upload Floor Plan", type=["png", "jpg", "jpeg"])
    if uploaded_map:
        save_path = os.path.join(MAP_DIR, "company_map.png")
        with open(save_path, "wb") as f:
            f.write(uploaded_map.getbuffer())
        st.success("Map saved!")
        st.rerun()

    new_scale = st.number_input(
        "Office Actual Width (Meters)", 
        min_value=1.0, 
        value=st.session_state.map_meters_wide,
        help="Enter the actual horizontal distance of the office shown in the image."
    )
    if new_scale != st.session_state.map_meters_wide:
        ers.save_map_scale(new_scale)
        st.session_state.map_meters_wide = new_scale
        st.rerun()

    st.markdown("### 2. Gateway Locations")
    with st.form("gw_config_form"):
        temp_coords = {}
        for gw in ["GW1", "GW2", "GW3"]:
            st.markdown(f"**📍 {gw}**")
            c1, c2 = st.columns(2)
            cur_x = st.session_state.gw_coords.get(gw, {}).get("x", 0.0)
            cur_y = st.session_state.gw_coords.get(gw, {}).get("y", 0.0)
            nx = c1.number_input(f"X (m)", value=float(cur_x), key=f"inx_{gw}", step=0.1)
            ny = c2.number_input(f"Y (m)", value=float(cur_y), key=f"iny_{gw}", step=0.1)
            temp_coords[gw] = {"x": nx, "y": ny}
        
        if st.form_submit_button("💾 Save Positions"):
            for gw_id, pos in temp_coords.items():
                ers.save_gw_coords(gw_id, pos['x'], pos['y'])
            st.session_state.gw_coords = temp_coords
            st.rerun()

with col_map2:
    st.markdown("### 3. Map Preview")
    map_img_path = os.path.join(MAP_DIR, "company_map.png")
    
    if os.path.exists(map_img_path):
        img = Image.open(map_img_path)
        w, h = img.size
        
        current_meters_wide = st.session_state.map_meters_wide
        scale = w / current_meters_wide  
        
        fig = px.imshow(img)
        
        for gw, pos in st.session_state.gw_coords.items():
            pixel_x = float(pos['x']) * scale
            pixel_y = float(pos['y']) * scale 
            
            fig.add_scatter(
                x=[pixel_x], y=[pixel_y],
                mode="markers+text",
                text=[f"<b>{gw}</b>"],
                textposition="top center",
                marker=dict(color="#e74c3c", size=15, symbol="octagon", line=dict(width=2, color="white")),
                name=gw
            )
        
        fig.update_layout(
            showlegend=False,
            margin=dict(l=0, r=0, t=0, b=0),
            xaxis=dict(visible=True, title=f"Scale: {current_meters_wide}m total width"),
            yaxis=dict(visible=True, autorange="reversed")
        )
        st.plotly_chart(fig, use_container_width=True)

# ── Live summary ──────────────────────────────────────────────────────────────
st.markdown("---")
st.markdown('<div class="section-title">Current Assignment Summary</div>', unsafe_allow_html=True)

final = ers.fetch_all_staff()

r1, r2, r3, r4 = st.columns(4)
groups = [
    (r1, "✉ Email · Mass",     "email_alerts_mass",     "email"),
    (r2, "📱 SMS · Mass",       "sms_alerts_mass",       "phone"),
    (r3, "✉ Email · Personal",  "email_alerts_personal", "email"),
    (r4, "📱 SMS · Personal",   "sms_alerts_personal",   "phone"),
]
for col, label, flag, contact_field in groups:
    with col:
        members = [s for s in final if s.get(flag)]
        st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:8px;">{label} ({len(members)})</div>', unsafe_allow_html=True)
        if members:
            for s in members:
                st.markdown(f'<div style="font-size:0.8rem;padding:4px 0;border-bottom:1px solid #f0f0f0;"><strong>{s["name"]}</strong><br><span style="color:#586069;font-size:0.72rem;">{s.get(contact_field,"")}</span></div>', unsafe_allow_html=True)
        else:
            st.caption("None assigned.")

st.markdown(f"""
<div style="margin-top:40px;padding-top:14px;border-top:1px solid #1e2530;
            font-family:'IBM Plex Mono',monospace;font-size:0.6rem;color:#30363d;
            letter-spacing:2px;text-align:center;">
  ERS · CONFIGURATION
</div>""", unsafe_allow_html=True)
