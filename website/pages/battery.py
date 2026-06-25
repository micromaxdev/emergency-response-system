"""
pages/battery.py — UPS Battery Manager.
Displays critical UPS metrics: voltage, capacity, charging state,
AC power status and power-adapter status.

Reads I²C (voltage/capacity) directly. AC power state is read from the
DB written by ups_monitor_server.py — which owns the GPIO pin exclusively.
This avoids the "GPIO busy" conflict from multiple lgpio consumers.
"""

import streamlit as st
import os, sys, struct
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import utils as ers

st.set_page_config(
    page_title="ERS - Battery Manager",
    layout="wide",
)
st.markdown(ers.ERS_CSS, unsafe_allow_html=True)
ers.autorefresh()

# ── Extra CSS (battery bar + big metric cards) ────────────────────────────────
st.markdown("""
<style>
.bat-grid {
    display: flex; flex-wrap: wrap; gap: 16px; margin-bottom: 28px;
}
.bat-card {
    flex: 1; min-width: 180px;
    background: #ffffff;
    border: 1px solid #1e2530;
    border-radius: 6px;
    padding: 22px 24px 18px 24px;
    position: relative; overflow: hidden;
}
.bat-card::before {
    content: ''; position: absolute;
    left: 0; top: 0; bottom: 0; width: 3px;
}
.bat-card.blue::before   { background: #4065a1; }
.bat-card.green::before  { background: #2ec4b6; }
.bat-card.amber::before  { background: #f4a261; }
.bat-card.red::before    { background: #e63946; }
.bat-card.gray::before   { background: #586069; }
.bat-val {
    font-family: 'Bebas Neue', sans-serif;
    font-size: 2.6rem; line-height: 1; color: #111;
}
.bat-unit {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 1rem; color: #586069; margin-left: 4px;
}
.bat-lbl {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.62rem; color: #586069;
    letter-spacing: 2px; text-transform: uppercase; margin-top: 6px;
}
.bat-bar-wrap {
    background: #f0f0f0; border-radius: 4px;
    height: 14px; margin: 22px 0 6px 0;
    overflow: hidden; border: 1px solid #1e2530;
}
.bat-bar-fill {
    height: 100%; border-radius: 3px;
    transition: width 0.4s ease;
}
.bat-bar-lbl {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.6rem; color: #586069;
    display: flex; justify-content: space-between;
    letter-spacing: 1px;
}
.power-banner {
    border-radius: 8px; padding: 18px 24px;
    margin-bottom: 24px; display: flex;
    align-items: center; gap: 16px;
}
.power-banner.ok {
    background: rgba(46,196,182,0.07);
    border: 1px solid rgba(46,196,182,0.35);
}
.power-banner.fail {
    background: rgba(230,57,70,0.08);
    border: 2px solid #e63946;
}
.power-banner-icon {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.72rem;
    letter-spacing: 2px;
    text-transform: uppercase;
}
.power-banner-title {
    font-family: 'Bebas Neue', sans-serif;
    font-size: 1.3rem; letter-spacing: 4px;
}
.power-banner-sub {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.68rem; color: #586069; margin-top: 3px;
}
.hw-error-box {
    background: rgba(247,183,49,0.08);
    border: 1px solid rgba(247,183,49,0.45);
    border-left: 4px solid #f7b731;
    border-radius: 6px; padding: 16px 20px; margin-bottom: 24px;
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.75rem; color: #7a6200; line-height: 1.7;
}
.ts-line {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.62rem; color: #586069;
    letter-spacing: 1px; margin-bottom: 20px;
}
</style>
""", unsafe_allow_html=True)

# ════════════════════════════════════════════════════════════════════════════════
#  HARDWARE LAYER
#  Voltage + capacity: read directly via I²C (no conflict, smbus2 is shareable).
#  AC power (GPIO pin 6): read from DB written by ups_monitor_server.py,
#  which is the sole owner of the GPIO. Never claim the pin here.
# ════════════════════════════════════════════════════════════════════════════════

I2C_ADDRESS = 0x36
I2C_BUS     = 1

_hw_errors: list[str] = []

try:
    import smbus2 as _smbus
    _smbus_ok = True
except Exception as _e:
    _smbus_ok = False
    _hw_errors.append(f"smbus2 not available: {_e}")


def _read_voltage_and_capacity():
    """Open I²C bus, read fuel gauge, close. Returns (voltage_V, capacity_pct)."""
    if not _smbus_ok:
        return None, None
    try:
        bus   = _smbus.SMBus(I2C_BUS)
        v_raw = bus.read_word_data(I2C_ADDRESS, 2)
        c_raw = bus.read_word_data(I2C_ADDRESS, 4)
        bus.close()
        voltage  = struct.unpack("<H", struct.pack(">H", v_raw))[0] * 1.25 / 1000 / 16
        capacity = struct.unpack("<H", struct.pack(">H", c_raw))[0] / 256
        return voltage, capacity
    except Exception as e:
        _hw_errors.append(f"I²C read error: {e}")
        return None, None


def _get_pld_state():
    """
    Return True = AC OK, False = power lost, None = not yet known.
    Reads from the system_control DB row written by ups_monitor_server.py.
    GPIO is never touched here — the server owns pin 6 exclusively.
    """
    val = ers.get_control("ups_ac_ok")
    if val == "1":
        return True
    if val == "0":
        return False
    return None  # server hasn't written a reading yet


def _get_charging(ac_ok, capacity):
    """Derive charging state: charging when AC is present and battery not full."""
    if ac_ok is None or capacity is None:
        return None
    return ac_ok and capacity < 99.0

# ════════════════════════════════════════════════════════════════════════════════
#  SIDEBAR
# ════════════════════════════════════════════════════════════════════════════════
last_read = ers.get_control("ups_updated") or "—"

with st.sidebar:
    st.markdown(ers.sidebar_title("Battery Manager"), unsafe_allow_html=True)
    st.markdown("---")
    st.markdown(
        ers.section_caption("UPS Info"),
        unsafe_allow_html=True,
    )
    st.markdown(
        """
        <div style="font-family:'IBM Plex Mono',monospace;font-size:0.68rem;
                    color:#586069;line-height:1.85;">
          Monitors the X120x UPS HAT.<br><br>
          Metrics shown:<br>
          · UPS Voltage<br>
          · Battery Capacity<br>
          · Charging State<br>
          · AC Power Status<br>
          · Power Adapter Status<br><br>
          Refreshes every <strong style="color:#111;">5 s</strong>.
        </div>
        """,
        unsafe_allow_html=True,
    )
    st.markdown("---")
    st.markdown(
        f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;'
        f'color:#586069;">I²C: {"OK" if _smbus_ok else "unavailable"}'
        f'&nbsp;·&nbsp;AC status: from DB</div>'
        f'<div style="font-family:IBM Plex Mono,monospace;font-size:0.58rem;'
        f'color:#586069;margin-top:4px;">GPIO last read: {last_read}</div>',
        unsafe_allow_html=True,
    )

# ════════════════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════════════════
st.markdown(ers.ers_header("Battery Manager"), unsafe_allow_html=True)

# ── Hardware-warning banner (only if something failed to init) ────────────────
if _hw_errors:
    errs_joined = "<br>".join(f"Warning: {e}" for e in _hw_errors)
    st.markdown(
        f'<div class="hw-error-box">'
        f'<strong style="letter-spacing:2px;">HARDWARE WARNING</strong><br>'
        f'{errs_joined}<br><br>'
        f'Metrics below are simulated / unavailable. Run on a Raspberry Pi 5 '
        f'with the X120x HAT to see live values.'
        f'</div>',
        unsafe_allow_html=True,
    )

# ── Read all metrics ──────────────────────────────────────────────────────────
voltage, capacity = _read_voltage_and_capacity()
ac_ok    = _get_pld_state()                  # True / False / None — from DB
charging = _get_charging(ac_ok, capacity)    # derived, no GPIO needed

now_str = datetime.now().strftime("%d %b %Y · %H:%M:%S")
st.markdown(f'<div class="ts-line">Last read: {now_str}</div>', unsafe_allow_html=True)

# ════════════════════════════════════════════════════════════════════════════════
#  POWER STATUS BANNER
# ════════════════════════════════════════════════════════════════════════════════
if ac_ok is None:
    banner_cls   = "fail"
    banner_icon  = "Unknown"
    banner_title = "POWER STATUS UNKNOWN"
    banner_sub   = f"Waiting for first reading from UPS monitor server. Last read: {last_read}"
elif ac_ok:
    banner_cls   = "ok"
    banner_icon  = "OK"
    banner_title = "AC POWER: OK"
    banner_sub   = "Mains supply present · Power adapter functioning normally."
else:
    banner_cls   = "fail"
    banner_icon  = "X"
    banner_title = "POWER LOSS DETECTED"
    banner_sub   = "Mains supply lost OR power adapter failure — running on UPS battery."

st.markdown(
    f'<div class="power-banner {banner_cls}">'
    f'  <div class="power-banner-icon">{banner_icon}</div>'
    f'  <div>'
    f'    <div class="power-banner-title" '
    f'         style="color:{"#2ec4b6" if ac_ok else "#e63946"}">{banner_title}</div>'
    f'    <div class="power-banner-sub">{banner_sub}</div>'
    f'  </div>'
    f'</div>',
    unsafe_allow_html=True,
)

# ════════════════════════════════════════════════════════════════════════════════
#  METRIC CARDS  (voltage · capacity · charging · AC power · adapter)
# ════════════════════════════════════════════════════════════════════════════════
def _fmt(val, decimals=3, unit="", fallback="—"):
    if val is None:
        return fallback, ""
    return f"{val:.{decimals}f}", unit

def _bool_pill(val, true_label, false_label, true_color, false_color):
    """Return an HTML pill for a tri-state bool (True/False/None)."""
    if val is None:
        return '<span class="pill pill-unknown">UNKNOWN</span>'
    if val:
        return f'<span style="font-family:IBM Plex Mono,monospace;font-size:0.75rem;' \
               f'font-weight:600;color:{true_color};">{true_label}</span>'
    return f'<span style="font-family:IBM Plex Mono,monospace;font-size:0.75rem;' \
            f'font-weight:600;color:{false_color};">{false_label}</span>'

volt_val,  volt_u  = _fmt(voltage,  3, "V")
cap_val,   cap_u   = _fmt(capacity, 2, "%")

# Determine card accent colours from live state
cap_num   = capacity if capacity is not None else 0
cap_color = "green" if cap_num >= 60 else ("amber" if cap_num >= 25 else "red")

chg_display = _bool_pill(charging,
                          "ENABLED",  "DISABLED",
                          "#2ec4b6",     "#e63946")
ac_display  = _bool_pill(ac_ok,
                          "AC OK",    "NO MAINS",
                          "#2ec4b6",     "#e63946")
adp_display = _bool_pill(ac_ok,
                          "ADAPTER OK", "ADAPTER FAILURE",
                          "#2ec4b6",        "#e63946")

st.markdown(f"""
<div class="bat-grid">

  <!-- UPS Voltage -->
  <div class="bat-card blue">
    <div class="bat-val">{volt_val}<span class="bat-unit">{volt_u}</span></div>
    <div class="bat-lbl">UPS Voltage</div>
  </div>

  <!-- Battery Capacity -->
  <div class="bat-card {cap_color}">
    <div class="bat-val">{cap_val}<span class="bat-unit">{cap_u}</span></div>
    <div class="bat-lbl">Battery Capacity</div>
  </div>

  <!-- Charging -->
  <div class="bat-card {'green' if charging else 'gray'}">
    <div style="margin-top:6px;">{chg_display}</div>
    <div class="bat-lbl" style="margin-top:10px;">Charging</div>
  </div>

  <!-- AC Power -->
  <div class="bat-card {'green' if ac_ok else 'red'}">
    <div style="margin-top:6px;">{ac_display}</div>
    <div class="bat-lbl" style="margin-top:10px;">AC Power</div>
  </div>

  <!-- Power Adapter -->
  <div class="bat-card {'green' if ac_ok else 'red'}">
    <div style="margin-top:6px;">{adp_display}</div>
    <div class="bat-lbl" style="margin-top:10px;">Power Adapter</div>
  </div>

</div>
""", unsafe_allow_html=True)

# ════════════════════════════════════════════════════════════════════════════════
#  BATTERY BAR
# ════════════════════════════════════════════════════════════════════════════════
st.markdown('<div class="section-title">Battery Level</div>', unsafe_allow_html=True)

if capacity is not None:
    pct       = max(0.0, min(100.0, capacity))
    bar_color = "#2ec4b6" if pct >= 60 else ("#f4a261" if pct >= 25 else "#e63946")
    st.markdown(f"""
    <div class="bat-bar-wrap">
      <div class="bat-bar-fill"
           style="width:{pct:.1f}%; background:{bar_color};"></div>
    </div>
    <div class="bat-bar-lbl">
      <span>0%</span>
      <span style="color:#111;font-weight:600;">{pct:.2f}%</span>
      <span>100%</span>
    </div>
    """, unsafe_allow_html=True)
else:
    st.markdown(
        '<div style="font-family:IBM Plex Mono,monospace;font-size:0.75rem;'
        'color:#586069;padding:12px 0;">Battery level unavailable — I²C not reachable.</div>',
        unsafe_allow_html=True,
    )

st.markdown("---")

# ════════════════════════════════════════════════════════════════════════════════
#  QUICK REFERENCE TABLE
# ════════════════════════════════════════════════════════════════════════════════
st.markdown('<div class="section-title">Status Summary</div>', unsafe_allow_html=True)

def _row(label, value_html):
    return (
        f'<div class="inc-row">'
        f'  <span style="min-width:220px;font-family:IBM Plex Mono,monospace;'
        f'         font-size:0.7rem;letter-spacing:1px;color:#586069;'
        f'         text-transform:uppercase;">{label}</span>'
        f'  <span style="font-family:IBM Plex Mono,monospace;font-size:0.82rem;">{value_html}</span>'
        f'</div>'
    )

volt_html = f'<strong>{volt_val} V</strong>' if voltage is not None else '<span style="color:#586069;">—</span>'
cap_html  = f'<strong>{cap_val} %</strong>'  if capacity is not None else '<span style="color:#586069;">—</span>'

def _state_html(val, t_label, f_label):
    if val is None:
        return '<span style="color:#586069;">Unknown</span>'
    color  = "#2ec4b6" if val else "#e63946"
    label  = t_label  if val else f_label
    return f'<strong style="color:{color};">{label}</strong>'

rows_html = (
    _row("UPS Voltage",    volt_html) +
    _row("Battery Capacity", cap_html) +
    _row("Charging",       _state_html(charging, "Enabled",       "Disabled")) +
    _row("AC Power",       _state_html(ac_ok,    "OK — Mains present", "LOST — No mains supply")) +
    _row("Power Adapter",  _state_html(ac_ok,    "OK — Functioning",   "FAILURE — Adapter fault"))
)
st.markdown(rows_html, unsafe_allow_html=True)

st.markdown("---")

# ════════════════════════════════════════════════════════════════════════════════
#  CLIENT DEVICE BATTERIES
#  Pico devices report battery_pct + battery_v every 15 min via TCP.
#  The server stores them in device_battery; we read here — no GPIO needed.
# ════════════════════════════════════════════════════════════════════════════════
st.markdown('<div class="section-title">Client Device Batteries</div>', unsafe_allow_html=True)

device_batteries = ers.fetch_all_device_batteries()

if not device_batteries:
    st.markdown(
        '<div style="font-family:IBM Plex Mono,monospace;font-size:0.75rem;'
        'color:#586069;padding:12px 0;">'
        'No client device readings yet — Pico devices report every 15 minutes.</div>',
        unsafe_allow_html=True,
    )
else:
    STALE_MINUTES = 20   # flag as stale if no report within this many minutes

    cards_html = '<div class="bat-grid">'
    for dev in device_batteries:
        pct     = dev.get("battery_pct")
        voltage = dev.get("battery_v")
        label   = dev.get("label") or dev.get("device_id") or "—"
        updated = dev.get("updated_at") or "—"

        stale = False
        if updated and updated != "—":
            try:
                age_mins = (
                    datetime.now() - datetime.strptime(updated, "%Y-%m-%d %H:%M:%S")
                ).total_seconds() / 60
                stale = age_mins > STALE_MINUTES
            except Exception:
                pass

        if pct is None:
            bar_w, bar_color, card_cls, pct_str = 0, "#aaa", "gray", "—"
        else:
            bar_w = max(0.0, min(100.0, pct))
            if pct >= 60:
                bar_color, card_cls = "#2ec4b6", "green"
            elif pct >= 25:
                bar_color, card_cls = "#f4a261", "amber"
            else:
                bar_color, card_cls = "#e63946", "red"
            pct_str = f"{pct:.1f}%"

        volt_str    = f"{voltage:.3f} V" if voltage is not None else "—"
        staff_name  = dev.get("staff_name")
        # Show assigned person's name if available, otherwise fall back to device label
        display_name = staff_name if staff_name else label
        stale_html = (
            '<span style="font-family:IBM Plex Mono,monospace;font-size:0.58rem;'
            'color:#f7b731;margin-left:6px;">STALE</span>'
            if stale else ""
        )

        cards_html += f"""
        <div class="bat-card {card_cls}" style="min-width:200px;">
          <div class="bat-val">{pct_str}</div>
          <div class="bat-lbl" style="margin-top:2px;">{display_name}{stale_html}</div>
          <div class="bat-bar-wrap" style="margin:10px 0 6px 0;">
            <div class="bat-bar-fill"
                 style="width:{bar_w:.1f}%; background:{bar_color};"></div>
          </div>
          <div class="bat-bar-lbl">
            <span>0%</span>
            <span>100%</span>
          </div>
          <div style="font-family:IBM Plex Mono,monospace;font-size:0.6rem;
                      color:#586069;margin-top:8px;">
            {volt_str} &nbsp;·&nbsp; {updated}
          </div>
        </div>"""

    cards_html += "</div>"
    st.markdown(cards_html, unsafe_allow_html=True)

# ── Footer ────────────────────────────────────────────────────────────────────
st.markdown(ers.footer(f"ERS / BATTERY MANAGER / X120x UPS HAT / {datetime.now().year}"), unsafe_allow_html=True)
