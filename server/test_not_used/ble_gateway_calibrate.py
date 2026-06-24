#!/usr/bin/env python3
"""
BLE Gateway MQTT Listener + Trilateration + Map Overlay

Map scaling workflow:
  1. Place your map PNG in the same folder as this script
  2. Set MAP_FILE to the filename below
  3. Run the script — it opens in MAP SCALING MODE first
  4. Click two known points on the map (e.g. two corners of the room)
  5. Enter the real distance between those points in metres
  6. The map is now scaled — drag gateways to their real positions

Calibration:
  'c' = calibration mode, click map to set position, SPACE to record
  'f' = fit TX_POWER and PATH_LOSS_EXP from recorded points
  'r' = reset calibration
  'q' = quit

Setup:
    pip install paho-mqtt matplotlib numpy scipy pillow
"""

import json
import threading
import os
from datetime import datetime
from collections import deque

import numpy as np
from scipy.optimize import minimize, curve_fit
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from matplotlib.widgets import Button
import paho.mqtt.client as mqtt

try:
    from PIL import Image
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("[WARN] Pillow not installed — map overlay disabled. Run: pip install pillow")

# ─── EDIT THESE ───────────────────────────────────────────────────────────────

BROKER_HOST = "192.168.115.49"
BROKER_PORT  = 1883
BROKER_USER  = ""
BROKER_PASS  = ""

TARGET_MAC = "e7eabd18a8c3"

# Put your map PNG in the same folder and set the filename here
MAP_FILE = "map.png"

# Initial gateway positions in metres (will be overwritten by drag/cal file)
GATEWAY_POSITIONS = {
    "ac233fc25932": [3.6, 20.5],
    "ac233fc26ecb": [15.4, 19.9],
    "ac233fc26ecc": [16.0, 4.2],
}

# Room size — will be updated after map scaling
ROOM_W = 18
ROOM_H = 23

TX_POWER      = -59
PATH_LOSS_EXP = 2.5

SMOOTH_N  = 20
MAX_TRAIL = 30
CAL_FILE  = "cal_data.json"

# ──────────────────────────────────────────────────────────────────────────────

data_lock   = threading.Lock()
rssi_data   = {gw: deque(maxlen=60)       for gw in GATEWAY_POSITIONS}
rssi_smooth = {gw: deque(maxlen=SMOOTH_N) for gw in GATEWAY_POSITIONS}
pos_trail   = deque(maxlen=MAX_TRAIL)

GATEWAY_IDS = list(GATEWAY_POSITIONS.keys())
COLORS      = ["#2196F3", "#4CAF50", "#F44336"]
GW_COLORS   = {gid: COLORS[i] for i, gid in enumerate(GATEWAY_IDS)}

cal_params     = {"tx_power": TX_POWER, "path_loss_exp": PATH_LOSS_EXP}
cal_mode       = False
cal_click_pos  = None
cal_points     = []

# Map scaling state
map_scale_mode    = False
map_scale_clicks  = []   # list of (x_metres, y_metres) — 2 clicks needed
map_scale_pixels  = []   # pixel coords of the 2 clicks
map_image         = None
map_extent        = None  # [xmin, xmax, ymin, ymax] in metres

# ─── CALIBRATION SAVE/LOAD ───────────────────────────────────────────────────

def save_calibration():
    data = {
        "tx_power":          cal_params["tx_power"],
        "path_loss_exp":     cal_params["path_loss_exp"],
        "cal_points":        cal_points,
        "gateway_positions": {k: list(v) for k, v in GATEWAY_POSITIONS.items()},
        "room_w":            ROOM_W,
        "room_h":            ROOM_H,
        "map_extent":        map_extent,
    }
    with open(CAL_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[CAL] Saved to {CAL_FILE}")


def load_calibration():
    global cal_points, ROOM_W, ROOM_H, map_extent
    if not os.path.exists(CAL_FILE):
        print("[CAL] No calibration file found, using defaults")
        return
    try:
        with open(CAL_FILE) as f:
            data = json.load(f)
        cal_params["tx_power"]      = data.get("tx_power",      TX_POWER)
        cal_params["path_loss_exp"] = data.get("path_loss_exp", PATH_LOSS_EXP)
        cal_points = data.get("cal_points", [])
        for gid, pos in data.get("gateway_positions", {}).items():
            if gid in GATEWAY_POSITIONS:
                GATEWAY_POSITIONS[gid] = pos
        ROOM_W     = data.get("room_w", ROOM_W)
        ROOM_H     = data.get("room_h", ROOM_H)
        map_extent = data.get("map_extent", None)
        print(f"[CAL] Loaded: TX={cal_params['tx_power']}  n={cal_params['path_loss_exp']}  points={len(cal_points)}")
    except Exception as e:
        print(f"[CAL] Load error: {e}")

# ─── PATH LOSS ────────────────────────────────────────────────────────────────

def rssi_to_distance(rssi):
    d = 10 ** ((cal_params["tx_power"] - rssi) / (10.0 * cal_params["path_loss_exp"]))
    return max(0.01, d)


def fit_path_loss(points):
    true_d, measured_r = [], []
    for pt in points:
        px, py = pt["x"], pt["y"]
        for gid, rssi in pt["rssi_per_gw"].items():
            if gid not in GATEWAY_POSITIONS:
                continue
            gx, gy = GATEWAY_POSITIONS[gid]
            d = np.sqrt((px - gx)**2 + (py - gy)**2)
            if d > 0.05:
                true_d.append(d)
                measured_r.append(rssi)
    if len(true_d) < 3:
        return None, None
    true_d     = np.array(true_d)
    measured_r = np.array(measured_r)
    def model(d, a, b):
        return a - b * np.log10(d)
    try:
        popt, _ = curve_fit(model, true_d, measured_r,
                            p0=[-59, 25], bounds=([-100, 5], [-20, 50]))
        return round(float(popt[0]), 1), round(float(popt[1]) / 10.0, 2)
    except Exception as e:
        print(f"[CAL] Fit error: {e}")
        return None, None

# ─── TRILATERATION ────────────────────────────────────────────────────────────

def trilaterate(distances):
    gws = [(gid, distances[gid]) for gid in distances if distances[gid] is not None]
    if len(gws) < 2:
        return None
    def error(pos):
        x, y = pos
        return sum(
            (np.sqrt((x - GATEWAY_POSITIONS[gid][0])**2 +
                     (y - GATEWAY_POSITIONS[gid][1])**2) - d)**2
            for gid, d in gws
        )
    x0 = np.mean([GATEWAY_POSITIONS[gid][0] for gid, _ in gws])
    y0 = np.mean([GATEWAY_POSITIONS[gid][1] for gid, _ in gws])
    result = minimize(error, [x0, y0], method="Nelder-Mead",
                      options={"xatol": 0.01, "fatol": 0.01, "maxiter": 1000})
    x = max(0, min(ROOM_W, result.x[0]))
    y = max(0, min(ROOM_H, result.x[1]))
    return (x, y)

# ─── MQTT ─────────────────────────────────────────────────────────────────────

def make_mqtt_client(gateway_id):
    def on_connect(client, userdata, flags, reason_code, properties):
        if str(reason_code) == "Success":
            topic = f"/gw/{gateway_id}/status"
            client.subscribe(topic)
            print(f"[{gateway_id}] connected → {topic}")
        else:
            print(f"[{gateway_id}] failed: {reason_code}")

    def on_message(client, userdata, msg):
        try:
            entries = json.loads(msg.payload.decode("utf-8", errors="replace"))
            if not isinstance(entries, list):
                entries = [entries]
            for entry in entries:
                if "mac" not in entry:
                    continue
                mac = str(entry["mac"]).lower().strip().replace(":", "")
                if mac != TARGET_MAC.lower().replace(":", ""):
                    continue
                rssi = entry.get("rssi")
                if rssi is None:
                    continue
                ts = datetime.now().strftime("%H:%M:%S")
                print(f"[{ts}] [{gateway_id}]  rssi={rssi}")
                with data_lock:
                    rssi_data[gateway_id].append(rssi)
                    rssi_smooth[gateway_id].append(rssi)
        except Exception as e:
            print(f"[{gateway_id}] error: {e}")

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id=f"listener-{gateway_id}"
    )
    client.on_connect = on_connect
    client.on_message = on_message
    if BROKER_USER:
        client.username_pw_set(BROKER_USER, BROKER_PASS)
    return client


def start_mqtt():
    for gw_id in GATEWAY_IDS:
        c = make_mqtt_client(gw_id)
        try:
            c.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
            c.loop_start()
        except Exception as e:
            print(f"[{gw_id}] connection error: {e}")

# ─── DRAGGABLE GATEWAY ────────────────────────────────────────────────────────

class DraggableGateway:
    def __init__(self, point, annotation, gateway_id):
        self.point      = point
        self.annotation = annotation
        self.gateway_id = gateway_id
        self.dragging   = False
        fig.canvas.mpl_connect("button_press_event",   self.on_press)
        fig.canvas.mpl_connect("button_release_event", self.on_release)
        fig.canvas.mpl_connect("motion_notify_event",  self.on_motion)

    def _click_radius(self):
        return max(ROOM_W, ROOM_H) * 0.05

    def on_press(self, event):
        if event.inaxes != pos_ax or cal_mode or map_scale_mode:
            return
        x, y = GATEWAY_POSITIONS[self.gateway_id]
        if np.sqrt((event.xdata - x)**2 + (event.ydata - y)**2) < self._click_radius():
            self.dragging = True

    def on_release(self, event):
        if self.dragging:
            self.dragging = False
            x, y = GATEWAY_POSITIONS[self.gateway_id]
            print(f"[DRAG] {self.gateway_id} → ({x:.2f}, {y:.2f})")

    def on_motion(self, event):
        if not self.dragging or event.inaxes != pos_ax:
            return
        x = max(0, min(ROOM_W, event.xdata))
        y = max(0, min(ROOM_H, event.ydata))
        GATEWAY_POSITIONS[self.gateway_id][0] = x
        GATEWAY_POSITIONS[self.gateway_id][1] = y
        self.point.set_data([x], [y])
        idx = GATEWAY_IDS.index(self.gateway_id)
        self.annotation.set_position((x + ROOM_W * 0.02, y + ROOM_H * 0.02))
        self.annotation.set_text(f"GW{idx+1}\n({x:.1f},{y:.1f})")
        fig.canvas.draw_idle()

# ─── MATPLOTLIB LAYOUT ────────────────────────────────────────────────────────

fig = plt.figure(figsize=(14, 7), facecolor="#0d1117")
gs  = fig.add_gridspec(3, 2, width_ratios=[1, 1.5], hspace=0.55, wspace=0.35,
                       left=0.06, right=0.97, top=0.92, bottom=0.1)

rssi_axes = [fig.add_subplot(gs[i, 0]) for i in range(3)]
pos_ax    = fig.add_subplot(gs[:, 1])

fig.suptitle(
    "BLE Monitor  |  'm'=scale map  'c'=calibrate  'space'=record  'f'=fit  'r'=reset  'q'=quit",
    color="#58a6ff", fontsize=9, fontfamily="monospace", y=0.98
)

rssi_lines, rssi_text = [], []
for i, (ax, gw_id) in enumerate(zip(rssi_axes, GATEWAY_IDS)):
    ax.set_facecolor("#161b22")
    ax.set_ylim(-100, -10)
    ax.set_xlim(0, 60)
    ax.set_title(f"GW{i+1}  {gw_id[-4:]}", color=COLORS[i],
                 fontsize=8, fontfamily="monospace", pad=3)
    ax.tick_params(colors="#484f58", labelsize=7)
    ax.set_ylabel("dBm", color="#484f58", fontsize=7, fontfamily="monospace")
    for spine in ax.spines.values():
        spine.set_edgecolor("#21262d")
    ax.grid(True, color="#21262d", linewidth=0.5)
    ax.set_xticks([])
    line, = ax.plot([], [], color=COLORS[i], linewidth=1.5)
    rssi_lines.append(line)
    txt = ax.text(0.98, 0.85, "— dBm", transform=ax.transAxes,
                  color=COLORS[i], fontsize=9, fontfamily="monospace",
                  ha="right", va="top")
    rssi_text.append(txt)

pos_ax.set_facecolor("#161b22")
pos_ax.set_xlim(-0.3, ROOM_W + 0.3)
pos_ax.set_ylim(-0.3, ROOM_H + 0.3)
pos_ax.set_aspect("equal")
pos_ax.set_title("Estimated Position  (drag ■ gateways)", color="#e6edf3",
                 fontsize=8, fontfamily="monospace", pad=6)
pos_ax.tick_params(colors="#484f58", labelsize=7)
pos_ax.set_xlabel("x (m)", color="#484f58", fontsize=7, fontfamily="monospace")
pos_ax.set_ylabel("y (m)", color="#484f58", fontsize=7, fontfamily="monospace")
for spine in pos_ax.spines.values():
    spine.set_edgecolor("#21262d")
pos_ax.grid(True, color="#21262d", linewidth=0.5)

map_image_artist = None

room_rect = mpatches.FancyBboxPatch(
    (0, 0), ROOM_W, ROOM_H, boxstyle="square,pad=0",
    linewidth=1.5, edgecolor="#484f58", facecolor="none", zorder=2
)
pos_ax.add_patch(room_rect)

draggables, gw_points, gw_annots = [], {}, {}
for i, (gw_id, pos) in enumerate(GATEWAY_POSITIONS.items()):
    gx, gy = pos
    pt,  = pos_ax.plot([gx], [gy], "s", color=COLORS[i], markersize=12, zorder=10)
    ann  = pos_ax.annotate(
        f"GW{i+1}\n({gx:.1f},{gy:.1f})",
        xy=(gx, gy), xytext=(gx + 0.06, gy + 0.06),
        color=COLORS[i], fontsize=7, fontfamily="monospace", zorder=11
    )
    gw_points[gw_id] = pt
    gw_annots[gw_id] = ann

trail_scatter    = pos_ax.scatter([], [], c=[], cmap="Blues", s=20,
                                   alpha=0.5, zorder=3, vmin=0, vmax=1)
pos_dot,         = pos_ax.plot([], [], "o", color="white", markersize=12,
                                 zorder=7, markeredgecolor="#58a6ff", markeredgewidth=2)
pos_label        = pos_ax.text(0.02, 0.02, "", transform=pos_ax.transAxes,
                                 color="#e6edf3", fontsize=8, fontfamily="monospace")
status_label     = pos_ax.text(0.02, 0.96, "Press 'm' to scale map, or 'c' to calibrate",
                                 transform=pos_ax.transAxes, color="#f78166",
                                 fontsize=8, fontfamily="monospace", va="top")
cal_label        = pos_ax.text(0.98, 0.96, "", transform=pos_ax.transAxes,
                                 color="#3fb950", fontsize=8, fontfamily="monospace",
                                 va="top", ha="right")
cal_click_marker,= pos_ax.plot([], [], "+", color="#f78166",
                                 markersize=16, markeredgewidth=2, zorder=8)
scale_markers    = []
cal_pt_markers   = []
circle_artists   = []

for i, (gw_id, _) in enumerate(GATEWAY_POSITIONS.items()):
    draggables.append(DraggableGateway(gw_points[gw_id], gw_annots[gw_id], gw_id))

# ─── MAP SCALING ─────────────────────────────────────────────────────────────

def load_map_image():
    global map_image
    if not PIL_AVAILABLE:
        return
    if not os.path.exists(MAP_FILE):
        print(f"[MAP] '{MAP_FILE}' not found — place it in the same folder as this script")
        return
    map_image = np.array(Image.open(MAP_FILE))
    print(f"[MAP] Loaded '{MAP_FILE}'  size={map_image.shape[1]}x{map_image.shape[0]}px")


def apply_map_to_axes():
    global map_image_artist, map_extent
    if map_image is None or map_extent is None:
        return
    if map_image_artist is not None:
        map_image_artist.remove()
    map_image_artist = pos_ax.imshow(
        map_image,
        extent=map_extent,
        aspect="auto",
        origin="upper",
        alpha=0.45,
        zorder=0
    )
    pos_ax.set_xlim(map_extent[0] - 0.5, map_extent[1] + 0.5)
    pos_ax.set_ylim(map_extent[2] - 0.5, map_extent[3] + 0.5)
    fig.canvas.draw_idle()
    print(f"[MAP] Applied extent: {map_extent}")


# ─── KEYBOARD + CLICK HANDLERS ───────────────────────────────────────────────

def on_click(event):
    global cal_click_pos, map_scale_mode, map_extent, ROOM_W, ROOM_H

    if event.inaxes != pos_ax or event.button != 1:
        return

    # ── Map scaling ──
    if map_scale_mode:
        map_scale_clicks.append((event.xdata, event.ydata))
        m, = pos_ax.plot([event.xdata], [event.ydata], "D",
                          color="#ffcc00", markersize=10, zorder=9)
        scale_markers.append(m)
        pos_ax.annotate(f"P{len(map_scale_clicks)}",
                         xy=(event.xdata, event.ydata),
                         xytext=(event.xdata + 0.01, event.ydata + 0.01),
                         color="#ffcc00", fontsize=8, fontfamily="monospace", zorder=9)

        if len(map_scale_clicks) == 1:
            status_label.set_text("Click the SECOND reference point on the map")
            fig.canvas.draw_idle()

        elif len(map_scale_clicks) == 2:
            # Ask for real distance via terminal
            status_label.set_text("Check terminal — enter real distance between the 2 points")
            fig.canvas.draw_idle()
            # Run in thread so it doesn't block the GUI
            threading.Thread(target=finish_map_scale, daemon=True).start()
        return

    # ── Calibration click ──
    if cal_mode:
        cal_click_pos = (
            max(0, min(ROOM_W, event.xdata)),
            max(0, min(ROOM_H, event.ydata))
        )
        cal_click_marker.set_data([cal_click_pos[0]], [cal_click_pos[1]])
        status_label.set_text(
            f"Position ({cal_click_pos[0]:.2f}, {cal_click_pos[1]:.2f}) — press SPACE to record"
        )
        fig.canvas.draw_idle()


def finish_map_scale():
    global map_scale_mode, ROOM_W, ROOM_H, map_extent
    try:
        p1 = map_scale_clicks[0]
        p2 = map_scale_clicks[1]
        real_dist = float(input("\n[MAP] Enter real distance between the 2 clicked points (metres): "))

        # pixel distance between the two clicks (in current axis units)
        pixel_dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        scale = real_dist / pixel_dist  # metres per axis-unit

        # recompute extent so the full image spans correctly
        if map_image is not None:
            h_px, w_px = map_image.shape[:2]
            # find where p1 is in the image
            # use p1 as origin (0,0) and scale
            # extent = [left, right, bottom, top] in metres
            x_left  = p1[0] * scale * -1  # this won't work unless we anchor properly
            # better: treat p1 as (0,0) in metre space
            left   = -p1[0] * scale
            right  = left + w_px * (real_dist / (pixel_dist))
            bottom = -p1[1] * scale
            top    = bottom + h_px * (real_dist / (pixel_dist))

            # simpler robust approach: extent fits image to real dimensions
            # where p1→p2 vector defines the scale
            map_extent = [left, right, bottom, top]
            ROOM_W = right - left
            ROOM_H = top   - bottom

        map_scale_mode = False
        map_scale_clicks.clear()

        # update room and axes on main thread
        pos_ax.set_xlim(left - 0.5,  right + 0.5)
        pos_ax.set_ylim(bottom - 0.5, top   + 0.5)
        apply_map_to_axes()
        status_label.set_text(f"Map scaled! Room ≈ {ROOM_W:.1f}x{ROOM_H:.1f}m — drag gateways into position")
        status_label.set_color("#3fb950")
        fig.canvas.draw_idle()
        print(f"[MAP] Scale set: {scale:.3f} m/unit  Room: {ROOM_W:.1f}x{ROOM_H:.1f}m")

    except Exception as e:
        print(f"[MAP] Scaling error: {e}")
        map_scale_mode = False
        map_scale_clicks.clear()


def on_key(event):
    global cal_mode, cal_click_pos, map_scale_mode

    # ── Map scale mode ──
    if event.key == "m":
        if map_image is None:
            print(f"[MAP] No map loaded — place '{MAP_FILE}' in the same folder")
            status_label.set_text(f"No map found — place '{MAP_FILE}' in script folder")
            fig.canvas.draw_idle()
            return
        map_scale_mode = True
        map_scale_clicks.clear()
        for m in scale_markers:
            try:
                m.remove()
            except Exception:
                pass
        scale_markers.clear()

        # show the raw image first (unscaled, fills axes)
        global map_extent
        if map_image is not None:
            h_px, w_px = map_image.shape[:2]
            map_extent = [0, w_px, 0, h_px]
            apply_map_to_axes()
            pos_ax.set_xlim(-w_px * 0.05, w_px * 1.05)
            pos_ax.set_ylim(-h_px * 0.05, h_px * 1.05)

        status_label.set_text("SCALE MODE: click the FIRST reference point on the map")
        status_label.set_color("#ffcc00")
        fig.canvas.draw_idle()
        print("[MAP] Scale mode — click 2 known points on the map")

    # ── Calibration ──
    elif event.key == "c":
        cal_mode      = True
        cal_click_pos = None
        status_label.set_text("CAL MODE: click your position, then press SPACE")
        status_label.set_color("#f78166")
        fig.canvas.draw_idle()

    elif event.key == " " and cal_mode:
        if cal_click_pos is None:
            print("[CAL] Click your position on the map first")
            return
        with data_lock:
            snap = {gw: list(rssi_smooth[gw]) for gw in GATEWAY_IDS}
        rssi_per_gw = {gw: sum(v)/len(v) for gw, v in snap.items() if v}
        if len(rssi_per_gw) < 2:
            print("[CAL] Not enough RSSI data yet")
            return
        cal_points.append({"x": cal_click_pos[0], "y": cal_click_pos[1],
                            "rssi_per_gw": rssi_per_gw})
        m, = pos_ax.plot([cal_click_pos[0]], [cal_click_pos[1]], "x",
                          color="#f78166", markersize=10, markeredgewidth=2, zorder=7)
        pos_ax.annotate(f"P{len(cal_points)}", xy=cal_click_pos,
                         xytext=(cal_click_pos[0] + ROOM_W*0.02, cal_click_pos[1] + ROOM_H*0.02),
                         color="#f78166", fontsize=7, fontfamily="monospace")
        cal_pt_markers.append(m)
        cal_click_pos = None
        cal_click_marker.set_data([], [])
        status_label.set_text(f"Recorded P{len(cal_points)} — move and repeat, or press 'f' to fit")
        cal_label.set_text(f"{len(cal_points)} cal pts")
        fig.canvas.draw_idle()
        print(f"[CAL] Recorded point {len(cal_points)}")

    elif event.key == "f":
        if len(cal_points) < 3:
            status_label.set_text(f"Need ≥3 points (have {len(cal_points)})")
            fig.canvas.draw_idle()
            return
        tx, n = fit_path_loss(cal_points)
        if tx is not None:
            cal_params["tx_power"]      = tx
            cal_params["path_loss_exp"] = n
            save_calibration()
            cal_mode = False
            msg = f"Calibrated! TX={tx}dBm  n={n}"
            status_label.set_text(msg)
            status_label.set_color("#3fb950")
            fig.canvas.draw_idle()
            print(f"[CAL] {msg}")

    elif event.key == "r":
        cal_points.clear()
        cal_mode      = False
        cal_click_pos = None
        for m in cal_pt_markers:
            try: m.remove()
            except: pass
        cal_pt_markers.clear()
        cal_click_marker.set_data([], [])
        cal_params["tx_power"]      = TX_POWER
        cal_params["path_loss_exp"] = PATH_LOSS_EXP
        status_label.set_text("Calibration reset")
        status_label.set_color("#f78166")
        cal_label.set_text("")
        fig.canvas.draw_idle()
        print("[CAL] Reset")

    elif event.key == "q":
        plt.close("all")


fig.canvas.mpl_connect("key_press_event",    on_key)
fig.canvas.mpl_connect("button_press_event", on_click)

# ─── ANIMATION ────────────────────────────────────────────────────────────────

def animate(frame):
    global circle_artists

    with data_lock:
        rssi_snap   = {gw: list(rssi_data[gw])   for gw in GATEWAY_IDS}
        smooth_snap = {gw: list(rssi_smooth[gw])  for gw in GATEWAY_IDS}

    for i, gw_id in enumerate(GATEWAY_IDS):
        vals = rssi_snap[gw_id]
        if vals:
            rssi_lines[i].set_data(list(range(len(vals))), vals)
            rssi_axes[i].set_xlim(0, max(60, len(vals)))
            latest = vals[-1]
            rssi_text[i].set_text(f"{latest} dBm")
            rssi_text[i].set_color(
                "#3fb950" if latest >= -60 else
                "#d29922" if latest >= -75 else "#f78166"
            )

    for c in circle_artists:
        try: c.remove()
        except: pass
    circle_artists = []

    distances = {}
    for gw_id in GATEWAY_IDS:
        vals = smooth_snap[gw_id]
        if vals:
            avg   = sum(vals) / len(vals)
            d     = rssi_to_distance(avg)
            distances[gw_id] = d
            gx, gy = GATEWAY_POSITIONS[gw_id]
            circ = plt.Circle((gx, gy), d, color=GW_COLORS[gw_id],
                               fill=False, linewidth=1, linestyle="--", alpha=0.3, zorder=2)
            pos_ax.add_patch(circ)
            circle_artists.append(circ)
        else:
            distances[gw_id] = None

    pos = trilaterate(distances)
    if pos is not None:
        x, y = pos
        with data_lock:
            pos_trail.append((x, y))
            trail = list(pos_trail)
        if len(trail) > 1:
            trail_scatter.set_offsets(np.c_[[p[0] for p in trail], [p[1] for p in trail]])
            trail_scatter.set_array(np.linspace(0, 1, len(trail)))
        pos_dot.set_data([x], [y])
        pos_label.set_text(
            f"x={x:.2f}m  y={y:.2f}m  |  TX={cal_params['tx_power']}  n={cal_params['path_loss_exp']}"
        )

    return rssi_lines + rssi_text + [pos_dot, pos_label, trail_scatter,
                                      status_label, cal_label, cal_click_marker]


if __name__ == "__main__":
    load_map_image()
    load_calibration()

    if map_image is not None and map_extent is not None:
        apply_map_to_axes()

    print(f"\nBroker  : {BROKER_HOST}:{BROKER_PORT}")
    print(f"Target  : {TARGET_MAC}")
    print(f"Map     : {MAP_FILE} {'✓ loaded' if map_image is not None else '✗ not found'}")
    print(f"TX Power: {cal_params['tx_power']}  |  Path Loss: {cal_params['path_loss_exp']}")
    print(f"Cal pts : {len(cal_points)}")
    print("-" * 50)
    print("Keys: 'm'=scale map  'c'=calibrate  'space'=record  'f'=fit  'r'=reset  'q'=quit")
    print("-" * 50)

    mqtt_thread = threading.Thread(target=start_mqtt, daemon=True)
    mqtt_thread.start()

    ani = animation.FuncAnimation(
        fig, animate, interval=500, blit=False, cache_frame_data=False
    )

    plt.show()
