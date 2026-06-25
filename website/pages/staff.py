"""
pages/3_Staff.py — Staff directory. Add, edit and remove staff members.
Stores name, role, email, phone. Email/SMS alert flags managed here too.
"""

import streamlit as st
import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import utils as ers

st.set_page_config(page_title="ERS - Staff", layout="wide")
st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

if os.path.exists(ers.DB_PATH):
    ers.init_extra_tables()

# ── Session state ─────────────────────────────────────────────────────────────
if "edit_staff_id"   not in st.session_state: st.session_state.edit_staff_id   = None
if "delete_staff_id" not in st.session_state: st.session_state.delete_staff_id = None
if "staff_saved"     not in st.session_state: st.session_state.staff_saved     = False

ROLES = ["Admin", "Manager", "Security", "Medical", "Warden", "IT", "Other"]

# ── Sidebar ───────────────────────────────────────────────────────────────────
with st.sidebar:
    st.markdown(ers.sidebar_title("Staff Directory"), unsafe_allow_html=True)
    st.markdown("---")
    staff_search = st.text_input("Search staff", placeholder="Name, role, email…")

# ── Main ──────────────────────────────────────────────────────────────────────
st.markdown(ers.ers_header("Staff Directory"), unsafe_allow_html=True)

if not os.path.exists(ers.DB_PATH):
    st.error(f"Database not found at `{ers.DB_PATH}`.")
    st.stop()

if st.session_state.staff_saved:
    st.success("Staff record saved.")
    st.session_state.staff_saved = False

# ── Add new staff ─────────────────────────────────────────────────────────────
with st.expander("Add New Staff Member", expanded=False):
    with st.form("add_staff_form"):
        c1, c2 = st.columns(2)
        with c1:
            new_name  = st.text_input("Full Name *")
            new_role  = st.selectbox("Role", ROLES)
        with c2:
            new_email = st.text_input("Email Address")
            new_phone = st.text_input("Phone Number")

        c3, c4 = st.columns(2)
        with c3:
            new_email_alerts = st.checkbox("Receives Email Alerts (Mass)")
            new_email_pers   = st.checkbox("Receives Email Alerts (Personal)")
        with c4:
            new_sms_alerts = st.checkbox("Receives SMS Alerts (Mass)")
            new_sms_pers   = st.checkbox("Receives SMS Alerts (Personal)")

        submitted = st.form_submit_button("Add Staff Member", width="stretch")
        if submitted:
            if not new_name.strip():
                st.error("Name is required.")
            else:
                ers.add_staff(
                    name=new_name.strip(), role=new_role,
                    email=new_email.strip(), phone=new_phone.strip(),
                    email_alerts_mass=new_email_alerts, email_alerts_personal=new_email_pers,
                    sms_alerts_mass=new_sms_alerts, sms_alerts_personal=new_sms_pers,
                )
                st.session_state.staff_saved = True
                st.rerun()

st.markdown("---")

# ── Staff list ────────────────────────────────────────────────────────────────
st.markdown('<div class="section-title">All Staff</div>', unsafe_allow_html=True)

all_staff = ers.fetch_all_staff()

if staff_search:
    q = staff_search.lower()
    all_staff = [s for s in all_staff if
                 q in (s.get("name") or "").lower() or
                 q in (s.get("role") or "").lower() or
                 q in (s.get("email") or "").lower()]

if not all_staff:
    st.markdown('<div style="font-family:IBM Plex Mono,monospace;font-size:0.75rem;color:#586069;padding:20px 0;">No staff members found. Add one above.</div>', unsafe_allow_html=True)
else:
    # Table header
    st.markdown("""
    <div class="tbl-header">
      <span style="min-width:180px">Name</span>
      <span style="min-width:110px">Role</span>
      <span style="flex:1">Email</span>
      <span style="min-width:130px">Phone</span>
      <span style="width:80px">Email Alert</span>
      <span style="width:80px">SMS Alert</span>
      <span style="width:130px">Actions</span>
    </div>""", unsafe_allow_html=True)

    for s in all_staff:
        sid = s["id"]

        # ── Edit mode ────────────────────────────────────────────────────────
        if st.session_state.edit_staff_id == sid:
            with st.form(f"edit_form_{sid}"):
                ec1, ec2 = st.columns(2)
                with ec1:
                    e_name  = st.text_input("Full Name *", value=s.get("name",""))
                    e_role  = st.selectbox("Role", ROLES, index=ROLES.index(s["role"]) if s.get("role") in ROLES else 0)
                with ec2:
                    e_email = st.text_input("Email", value=s.get("email",""))
                    e_phone = st.text_input("Phone", value=s.get("phone",""))
                ec3, ec4 = st.columns(2)
                with ec3:
                    e_em = st.checkbox("Email · Mass",     value=bool(s.get("email_alerts_mass")))
                    e_ep = st.checkbox("Email · Personal", value=bool(s.get("email_alerts_personal")))
                with ec4:
                    e_sm = st.checkbox("SMS · Mass",       value=bool(s.get("sms_alerts_mass")))
                    e_sp = st.checkbox("SMS · Personal",   value=bool(s.get("sms_alerts_personal")))

                sb1, sb2 = st.columns(2)
                with sb1:
                    save = st.form_submit_button("Save", width="stretch")
                with sb2:
                    cancel = st.form_submit_button("Cancel", width="stretch")

                if save:
                    if not e_name.strip():
                        st.error("Name is required.")
                    else:
                        ers.update_staff(sid, e_name.strip(), e_role, e_email.strip(),
                                         e_phone.strip(), e_em, e_ep, e_sm, e_sp)
                        st.session_state.edit_staff_id = None
                        st.session_state.staff_saved   = True
                        st.rerun()
                if cancel:
                    st.session_state.edit_staff_id = None
                    st.rerun()

        # ── Delete confirmation ───────────────────────────────────────────────
        elif st.session_state.delete_staff_id == sid:
            st.warning(f"Delete **{s['name']}**? This will also unassign them from any devices.")
            dc1, dc2 = st.columns(2)
            with dc1:
                if st.button("Yes, Delete", key=f"del_yes_{sid}", width="stretch", type="primary"):
                    ers.delete_staff(sid)
                    st.session_state.delete_staff_id = None
                    st.rerun()
            with dc2:
                if st.button("Cancel", key=f"del_no_{sid}", width="stretch"):
                    st.session_state.delete_staff_id = None
                    st.rerun()

        # ── Normal row ───────────────────────────────────────────────────────
        else:
            em_on = s.get("email_alerts_mass") or s.get("email_alerts_personal")
            sm_on = s.get("sms_alerts_mass")   or s.get("sms_alerts_personal")
            email_badge = '<span class="pill pill-ok">Email ON</span>' if em_on else '<span class="pill pill-skip">Email OFF</span>'
            sms_badge = '<span class="pill pill-ok">SMS ON</span>' if sm_on else '<span class="pill pill-skip">SMS OFF</span>'

            st.markdown(f"""
            <div class="inc-row">
              <span style="min-width:180px;font-weight:600;">{s.get('name','—')}</span>
              <span style="min-width:110px;font-size:0.78rem;color:#586069;">{s.get('role','—')}</span>
              <span style="flex:1;font-size:0.78rem;">{s.get('email','—')}</span>
              <span style="min-width:130px;font-size:0.78rem;">{s.get('phone','—')}</span>
              <span style="width:80px">{email_badge}</span>
              <span style="width:80px">{sms_badge}</span>
            </div>""", unsafe_allow_html=True)

            col_edit, col_del, _ = st.columns([1, 1, 6])
            with col_edit:
                if st.button("Edit", key=f"edit_{sid}", width="stretch"):
                    st.session_state.edit_staff_id   = sid
                    st.session_state.delete_staff_id = None
                    st.rerun()
            with col_del:
                if st.button("Delete", key=f"delete_{sid}", width="stretch"):
                    st.session_state.delete_staff_id = sid
                    st.session_state.edit_staff_id   = None
                    st.rerun()

# Alert summary
st.markdown("---")
st.markdown('<div class="section-title">Alert Recipients Summary</div>', unsafe_allow_html=True)

email_recipients = [s for s in ers.fetch_all_staff() if s.get("email_alerts_mass") or s.get("email_alerts_personal")]
sms_recipients   = [s for s in ers.fetch_all_staff() if s.get("sms_alerts_mass")   or s.get("sms_alerts_personal")]

c1, c2 = st.columns(2)
with c1:
    st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:8px;">Email Alerts ({len(email_recipients)})</div>', unsafe_allow_html=True)
    if email_recipients:
        for s in email_recipients:
            st.markdown(f'<div style="font-size:0.82rem;padding:4px 0;border-bottom:1px solid #f0f0f0;"><strong>{s["name"]}</strong> &nbsp;<span style="color:#586069;">{s.get("email","")}</span></div>', unsafe_allow_html=True)
    else:
        st.caption("No staff assigned to email alerts.")

with c2:
    st.markdown(f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.65rem;color:#586069;letter-spacing:2px;text-transform:uppercase;margin-bottom:8px;">SMS Alerts ({len(sms_recipients)})</div>', unsafe_allow_html=True)
    if sms_recipients:
        for s in sms_recipients:
            st.markdown(f'<div style="font-size:0.82rem;padding:4px 0;border-bottom:1px solid #f0f0f0;"><strong>{s["name"]}</strong> &nbsp;<span style="color:#586069;">{s.get("phone","")}</span></div>', unsafe_allow_html=True)
    else:
        st.caption("No staff assigned to SMS alerts.")

st.markdown(ers.footer("ERS / STAFF DIRECTORY"), unsafe_allow_html=True)
