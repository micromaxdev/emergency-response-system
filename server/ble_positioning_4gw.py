#!/usr/bin/env python3
"""
BLE Gateway MQTT Listener + Hybrid Positioning + Room Detection (4 GATEWAYS)

Coordinate estimation (hybrid centroid / trilateration) is an intermediate
step; the final OUTPUT is the ROOM the tag is in, determined by which room
rectangle the estimated position falls inside.

Features:
  - Per-gateway Kalman filtering of RSSI
  - Staleness handling: gateways not heard recently are dropped
  - Interactive room drawing (rectangles on the map)
  - Room detection with STABILITY-BIASED hysteresis (no doorway flicker)
  - Per-gateway MQTT topic + JSON format (handles the MKGW3 gateway which
    sits on a different topic path than the ac233f 'gw' gateways)
  - Coverage-polygon overlay: shows the convex hull of the gateways so you
    can see which rooms fall INSIDE the well-covered zone

Calibration:
  'c' -> click position -> SPACE to record -> repeat 6+ spots -> 'f' to fit
Room drawing:
  'b' -> click two opposite corners -> type name in terminal. 'x' deletes last.

Keys:
  'm' scale map   'c' calibrate   'space' record   'f' fit   'r' reset cal
  'w' mode        'b' draw room   'x' delete room   'h' toggle hull   'q' quit

Setup:
    pip install paho-mqtt matplotlib numpy scipy pillow
"""

import json
import threading
import os
from datetime import datetime
from collections import deque

import numpy as np
from scipy.optimize import minimize, differential_evolution
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
import paho.mqtt.client as mqtt

try:
    from PIL import Image
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# ─── EDIT THESE ───────────────────────────────────────────────────────────────

BROKER_HOST = "192.168.115.49"
BROKER_PORT  = 1883
BROKER_USER  = ""
BROKER_PASS  = ""

TARGET_MAC = "e7b89a2ee18a"

MAP_FILE = "map.png"

GATEWAY_POSITIONS = {
    "ac233fc25932": [4.0,  20.5],   # GW1 top-left   (kitchen/service)
    "ac233fc26ecb": [15.4, 20.1],   # GW2 top-right  (shakif)
    "ac233fc26ecc": [15.3, 1.4],    # GW3 bottom-right (tory)
    "2805a55eedec": [1.9,  3.7],    # GW4 bottom-left  (bathrooms) 
}

GATEWAY_TOPICS = {
    "ac233fc25932": {"topic": "/gw/ac233fc25932/status",        "fmt": "list_mac_rssi"},
    "ac233fc26ecb": {"topic": "/gw/ac233fc26ecb/status",        "fmt": "list_mac_rssi"},
    "ac233fc26ecc": {"topic": "/gw/ac233fc26ecc/status",        "fmt": "list_mac_rssi"},
    # MKGW3 gateway: different topic (.../send) and a nested "data" array where
    # each entry is one overheard device with its own "mac" + "rssi".
    "2805a55eedec": {"topic": "/MKGW3/2805a55eedec/send",        "fmt": "mkgw3_nested"},
}

# Per-gateway wall penalty — 1.0 = full trust, lower = trusted less (more walls)
GATEWAY_WALL_WEIGHT = {
    "ac233fc25932": 1.0,
    "ac233fc26ecb": 1.0,
    "ac233fc26ecc": 1.0,
    "2805a55eedec": 1.0,
}

ROOM_W = 18.0
ROOM_H = 23.0

TX_POWER      = -42
PATH_LOSS_EXP = 3.5

SMOOTH_N  = 5
MAX_TRAIL = 10
CAL_FILE  = "cal_data.json"

POSITION_MODE = "hybrid"   # "hybrid", "centroid", "trilateration"

# ─── KALMAN / ROOM TUNING ──────────────────────────────────────────────────────

KALMAN_R = 8.0     # measurement noise (variance of raw RSSI, ~5-15 indoors)
KALMAN_Q = 0.1    # process noise (smaller = smoother / slower to react)

GATEWAY_STALE_SEC = 5.0   # gateway dropped if unheard for this long

ROOM_MARGIN_M       = 0.5  # metres inside a room boundary to count as "solidly in"
ROOM_PERSIST_FRAMES = 4    # consecutive frames in a new room before switching

SHOW_HULL = True           # draw gateway coverage polygon (toggle with 'h')

# ──────────────────────────────────────────────────────────────────────────────

data_lock   = threading.Lock()
rssi_data   = {gw: deque(maxlen=60)       for gw in GATEWAY_POSITIONS}
rssi_smooth = {gw: deque(maxlen=SMOOTH_N) for gw in GATEWAY_POSITIONS}
pos_trail   = deque(maxlen=MAX_TRAIL)

GATEWAY_IDS   = list(GATEWAY_POSITIONS.keys())
# 4 colours now (was 3) — adding a 4th prevents an index error in plotting.
COLORS        = ["#2196F3", "#4CAF50", "#F44336", "#FF9800"]
GW_COLORS     = {gid: COLORS[i] for i, gid in enumerate(GATEWAY_IDS)}
MODES         = ["hybrid", "centroid", "trilateration"]
mode_index    = MODES.index(POSITION_MODE)

cal_params    = {"tx_power": TX_POWER, "path_loss_exp": PATH_LOSS_EXP}
cal_mode      = False
cal_click_pos = None
cal_points    = []

map_scale_mode   = False
map_scale_clicks = []
map_image        = None
map_extent       = None
map_image_artist = None

scale_markers  = []
cal_pt_markers = []
circle_artists = []
hull_artist    = None

# ─── ROOM STATE ────────────────────────────────────────────────────────────────
rooms              = []
room_draw_mode     = False
room_draw_clicks   = []
room_draw_markers  = []
room_patches       = []
room_label_artists = []

current_room       = None
candidate_room     = None
candidate_count    = 0

# ─── KALMAN FILTER ─────────────────────────────────────────────────────────────

class RSSIKalman:
    """Scalar Kalman filter for one gateway's RSSI stream."""
    def __init__(self, R=KALMAN_R, Q=KALMAN_Q, initial=-70.0):
        self.R = R
        self.Q = Q
        self.x = initial
        self.P = R
        self.initialized = False
        self.last_update = None

    def update(self, z, now=None):
        if not self.initialized:
            self.x = z
            self.initialized = True
        else:
            self.P += self.Q
            K = self.P / (self.P + self.R)
            self.x += K * (z - self.x)
            self.P = (1 - K) * self.P
        self.last_update = now if now is not None else datetime.now()
        return self.x

    def is_fresh(self, now=None):
        if not self.initialized or self.last_update is None:
            return False
        now = now if now is not None else datetime.now()
        return (now - self.last_update).total_seconds() <= GATEWAY_STALE_SEC


kalman = {gw: RSSIKalman() for gw in GATEWAY_POSITIONS}

# ─── SAVE / LOAD ──────────────────────────────────────────────────────────────

def save_calibration():
    data = {
        "tx_power":            cal_params["tx_power"],
        "path_loss_exp":       cal_params["path_loss_exp"],
        "cal_points":          cal_points,
        "gateway_positions":   {k: list(v) for k, v in GATEWAY_POSITIONS.items()},
        "gateway_wall_weight": GATEWAY_WALL_WEIGHT,
        "room_w":              ROOM_W,
        "room_h":              ROOM_H,
        "map_extent":          map_extent,
        "rooms":               rooms,
    }
    with open(CAL_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[CAL] Saved to {CAL_FILE}")


def load_calibration():
    global cal_points, ROOM_W, ROOM_H, map_extent, rooms
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
                GATEWAY_POSITIONS[gid] = list(pos)
        for gid, w in data.get("gateway_wall_weight", {}).items():
            if gid in GATEWAY_WALL_WEIGHT:
                GATEWAY_WALL_WEIGHT[gid] = w
        ROOM_W     = data.get("room_w",     ROOM_W)
        ROOM_H     = data.get("room_h",     ROOM_H)
        map_extent = data.get("map_extent", None)
        rooms      = data.get("rooms", [])
        print(f"[CAL] Loaded: TX={cal_params['tx_power']}  n={cal_params['path_loss_exp']}  points={len(cal_points)}  rooms={len(rooms)}")
    except Exception as e:
        print(f"[CAL] Load error: {e}")

# ─── PATH LOSS ────────────────────────────────────────────────────────────────

def rssi_to_distance(rssi, tx=None, n=None):
    tx = tx if tx is not None else cal_params["tx_power"]
    n  = n  if n  is not None else cal_params["path_loss_exp"]
    return max(0.1, 10 ** ((tx - rssi) / (10.0 * n)))

# ─── POSITIONING ALGORITHMS ──────────────────────────────────────────────────

def weighted_centroid(rssi_vals):
    total_weight = 0
    wx, wy = 0.0, 0.0
    for gw_id, rssi in rssi_vals.items():
        if gw_id not in GATEWAY_POSITIONS:
            continue
        linear_weight = 10 ** (rssi / 10.0)
        wall_w        = GATEWAY_WALL_WEIGHT.get(gw_id, 1.0)
        w             = linear_weight * wall_w
        gx, gy   = GATEWAY_POSITIONS[gw_id]
        wx      += w * gx
        wy      += w * gy
        total_weight += w
    if total_weight == 0:
        return None
    return (wx / total_weight, wy / total_weight)


def _trilaterate_raw(distances):
    gws = [(gid, d) for gid, d in distances.items() if d is not None]
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
                      options={"xatol": 0.01, "fatol": 0.01, "maxiter": 2000})
    return tuple(result.x)


def trilaterate(distances):
    pos = _trilaterate_raw(distances)
    if pos is None:
        return None
    x = max(0, min(ROOM_W, pos[0]))
    y = max(0, min(ROOM_H, pos[1]))
    return (x, y)


def rssi_consistency(rssi_vals):
    if not rssi_vals:
        return 0.0
    vals = list(rssi_vals.values())
    if len(vals) < 2:
        return 0.5
    variance = np.var(vals)
    return max(0.0, min(1.0, 1.0 - variance / 200.0))


def hybrid_position(rssi_vals, distances):
    centroid = weighted_centroid(rssi_vals)
    trilat   = trilaterate(distances)
    if centroid is None and trilat is None:
        return None
    if centroid is None:
        return trilat
    if trilat is None:
        return centroid
    consistency = rssi_consistency(rssi_vals)
    trilat_weight = consistency * 0.7
    x = trilat_weight * trilat[0] + (1 - trilat_weight) * centroid[0]
    y = trilat_weight * trilat[1] + (1 - trilat_weight) * centroid[1]
    x = max(0, min(ROOM_W, x))
    y = max(0, min(ROOM_H, y))
    return (x, y)


def get_position(rssi_vals, distances):
    mode = MODES[mode_index]
    if mode == "centroid":
        pos = weighted_centroid(rssi_vals)
        if pos:
            return (max(0, min(ROOM_W, pos[0])), max(0, min(ROOM_H, pos[1])))
        return None
    elif mode == "trilateration":
        return trilaterate(distances)
    else:
        return hybrid_position(rssi_vals, distances)

# ─── CONVEX HULL (coverage polygon) ────────────────────────────────────────────

def convex_hull(points):
    """Andrew's monotone chain. Returns hull vertices CCW. Pure-python, no deps."""
    pts = sorted(set(map(tuple, points)))
    if len(pts) <= 2:
        return pts
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

# ─── ROOM LOGIC ────────────────────────────────────────────────────────────────

def _norm_rect(x0, y0, x1, y1):
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def room_containing(pos, margin=0.0):
    if pos is None:
        return None
    x, y = pos
    best = None
    best_area = None
    for rm in rooms:
        x0, y0, x1, y1 = rm["x0"], rm["y0"], rm["x1"], rm["y1"]
        if (x0 + margin) <= x <= (x1 - margin) and (y0 + margin) <= y <= (y1 - margin):
            area = (x1 - x0) * (y1 - y0)
            if best_area is None or area < best_area:
                best = rm["name"]
                best_area = area
    return best


def update_room_decision(pos):
    global current_room, candidate_room, candidate_count
    solid = room_containing(pos, margin=ROOM_MARGIN_M)
    loose = room_containing(pos, margin=0.0)

    if current_room is not None and loose == current_room:
        candidate_room = None
        candidate_count = 0
        return current_room

    if current_room is None:
        if solid is not None:
            current_room = solid
            candidate_room = None
            candidate_count = 0
        return current_room

    if solid is not None:
        if solid == candidate_room:
            candidate_count += 1
        else:
            candidate_room = solid
            candidate_count = 1
        if candidate_count >= ROOM_PERSIST_FRAMES:
            current_room = candidate_room
            candidate_room = None
            candidate_count = 0
    else:
        candidate_count = max(0, candidate_count - 1)
        if candidate_count == 0:
            candidate_room = None
    return current_room

# ─── CALIBRATION FIT ─────────────────────────────────────────────────────────

def fit_path_loss(points):
    if len(points) < 3:
        return None, None

    def total_position_error(params):
        tx, n = params
        total_err = 0
        for pt in points:
            true_x, true_y = pt["x"], pt["y"]
            dists = {gid: rssi_to_distance(rssi, tx=tx, n=n)
                     for gid, rssi in pt["rssi_per_gw"].items()
                     if gid in GATEWAY_POSITIONS}
            rssi_vals = pt["rssi_per_gw"]
            if MODES[mode_index] == "centroid":
                pos = weighted_centroid(rssi_vals)
                if pos:
                    pos = (max(0, min(ROOM_W, pos[0])), max(0, min(ROOM_H, pos[1])))
            else:
                pos = hybrid_position(rssi_vals, dists)
            if pos is not None:
                total_err += (pos[0] - true_x)**2 + (pos[1] - true_y)**2
            else:
                total_err += ROOM_W**2 + ROOM_H**2
        return total_err

    bounds = [(-90, -20), (1.0, 5.0)]
    try:
        result = differential_evolution(
            total_position_error, bounds,
            seed=42, maxiter=500, tol=0.001, workers=1, polish=True
        )
        tx_fit = round(float(result.x[0]), 1)
        n_fit  = round(float(result.x[1]), 2)
        err    = result.fun / len(points)
        print(f"[CAL] Fit: TX={tx_fit}  n={n_fit}  avg_pos_err={err:.2f}m²  ({np.sqrt(err):.2f}m RMS)")
        return tx_fit, n_fit
    except Exception as e:
        print(f"[CAL] Fit error: {e}")
        return None, None

# ─── MQTT ─────────────────────────────────────────────────────────────────────

def parse_entries(payload, fmt):
    """Return a list of (mac, rssi) tuples from a payload, per the format.
    A tag may legitimately appear multiple times in one message (the gateway
    batches several overheard advertising packets); we return all of them and
    feed each into the Kalman filter."""
    out = []
    try:
        data = json.loads(payload.decode("utf-8", errors="replace"))
    except Exception:
        return out

    if fmt == "list_mac_rssi":
        # ac233f gateways: a list (or single obj) of {"mac":.., "rssi":..}
        entries = data if isinstance(data, list) else [data]
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            if "mac" in entry and entry.get("rssi") is not None:
                out.append((str(entry["mac"]), entry["rssi"]))

    elif fmt == "mkgw3_nested":
        # MKGW3 gateway: {"device_info":{"mac":<gw>}, "data":[{"mac":<tag>,"rssi":..}, ...]}
        # The top-level mac is the GATEWAY's own mac — ignore it; read "data".
        readings = data.get("data") if isinstance(data, dict) else None
        if isinstance(readings, list):
            for entry in readings:
                if not isinstance(entry, dict):
                    continue
                if "mac" in entry and entry.get("rssi") is not None:
                    out.append((str(entry["mac"]), entry["rssi"]))

    return out


def make_mqtt_client(gateway_id):
    cfg   = GATEWAY_TOPICS[gateway_id]
    topic = cfg["topic"]
    fmt   = cfg["fmt"]

    def on_connect(client, userdata, flags, reason_code=None, properties=None):
        # Compatible with both paho-mqtt v1 and v2
        if reason_code is None or reason_code == 0 or str(reason_code) == "Success":
            client.subscribe(topic)
            print(f"[{gateway_id}] connected → subscribed: {topic}")
        else:
            print(f"[{gateway_id}] connect failed: {reason_code}")
         

    def on_message(client, userdata, msg):
        try:
            for mac, rssi in parse_entries(msg.payload, fmt):
                mac_n = mac.lower().strip().replace(":", "")
                if mac_n != TARGET_MAC.lower().replace(":", ""):
                    continue
                ts = datetime.now().strftime("%H:%M:%S")
                print(f"[{ts}] [{gateway_id}]  rssi={rssi}")
                with data_lock:
                    rssi_data[gateway_id].append(rssi)
                    rssi_smooth[gateway_id].append(rssi)
                    kalman[gateway_id].update(rssi)
        except Exception as e:
            print(f"[{gateway_id}] error: {e}")
                         
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=f"listener-{gateway_id}")
    except AttributeError:
        client = mqtt.Client(client_id=f"listener-{gateway_id}")
    
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
    def __init__(self, point, halo, annotation, gateway_id):
        self.point      = point
        self.halo       = halo
        self.annotation = annotation
        self.gateway_id = gateway_id
        self.dragging   = False
        fig.canvas.mpl_connect("button_press_event",   self.on_press)
        fig.canvas.mpl_connect("button_release_event", self.on_release)
        fig.canvas.mpl_connect("motion_notify_event",  self.on_motion)

    def on_press(self, event):
        if event.inaxes != pos_ax or cal_mode or map_scale_mode or room_draw_mode:
            return
        x, y = GATEWAY_POSITIONS[self.gateway_id]
        r = max(ROOM_W, ROOM_H) * 0.04
        if np.sqrt((event.xdata - x)**2 + (event.ydata - y)**2) < r:
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
        self.halo.set_data([x], [y])
        idx = GATEWAY_IDS.index(self.gateway_id)
        off = max(ROOM_W, ROOM_H) * 0.02
        self.annotation.set_position((x + off, y + off))
        self.annotation.set_text(f"GW{idx+1}\n({x:.1f},{y:.1f})")
        fig.canvas.draw_idle()

# ─── MATPLOTLIB LAYOUT ────────────────────────────────────────────────────────

fig = plt.figure(figsize=(15, 8.5), facecolor="#0d1117")
gs  = fig.add_gridspec(4, 2, width_ratios=[1, 1.6], hspace=0.7, wspace=0.3,
                       left=0.06, right=0.97, top=0.93, bottom=0.08)

rssi_axes = [fig.add_subplot(gs[i, 0]) for i in range(4)]   # 4 panels now
pos_ax    = fig.add_subplot(gs[:, 1])

fig.suptitle(
    "BLE 4-GW | 'm'map 'c'cal 'space'rec 'f'fit 'r'reset 'w'mode 'b'room 'x'del 'h'hull 'q'quit",
    color="#58a6ff", fontsize=8, fontfamily="monospace", y=0.985
)

rssi_lines, rssi_text = [], []
for i, (ax, gw_id) in enumerate(zip(rssi_axes, GATEWAY_IDS)):
    ax.set_facecolor("#161b22")
    ax.set_ylim(-100, -10)
    ax.set_xlim(0, 60)
    ax.set_title(f"GW{i+1}  {gw_id[-4:]}", color=COLORS[i],
                 fontsize=8, fontfamily="monospace", pad=2)
    ax.tick_params(colors="#484f58", labelsize=6)
    ax.set_ylabel("dBm", color="#484f58", fontsize=6, fontfamily="monospace")
    for spine in ax.spines.values():
        spine.set_edgecolor("#21262d")
    ax.grid(True, color="#21262d", linewidth=0.5)
    ax.set_xticks([])
    line, = ax.plot([], [], color=COLORS[i], linewidth=1.4)
    rssi_lines.append(line)
    ktxt = ax.text(0.98, 0.82, "— dBm", transform=ax.transAxes,
                   color=COLORS[i], fontsize=8.5, fontfamily="monospace",
                   ha="right", va="top")
    rssi_text.append(ktxt)

pos_ax.set_facecolor("#161b22")
pos_ax.set_xlim(-0.5, ROOM_W + 0.5)
pos_ax.set_ylim(-0.5, ROOM_H + 0.5)
pos_ax.set_aspect("equal")
pos_ax.set_title("Estimated Position + Rooms  (drag ■ gateways, 'b' rooms, 'h' hull)",
                 color="#e6edf3", fontsize=8, fontfamily="monospace", pad=6)
pos_ax.tick_params(colors="#484f58", labelsize=7)
pos_ax.set_xlabel("x (m)", color="#484f58", fontsize=7, fontfamily="monospace")
pos_ax.set_ylabel("y (m)", color="#484f58", fontsize=7, fontfamily="monospace")
for spine in pos_ax.spines.values():
    spine.set_edgecolor("#21262d")
pos_ax.grid(True, color="#21262d", linewidth=0.5)

room_rect = mpatches.FancyBboxPatch(
    (0, 0), ROOM_W, ROOM_H, boxstyle="square,pad=0",
    linewidth=1.5, edgecolor="#484f58", facecolor="none", zorder=2
)
pos_ax.add_patch(room_rect)

gw_points, gw_annots, draggables = {}, {}, []
for i, (gw_id, pos) in enumerate(GATEWAY_POSITIONS.items()):
    gx, gy = pos
    halo, = pos_ax.plot([gx], [gy], "s", color="white",
                         markersize=20, zorder=9, alpha=0.25)
    pt,   = pos_ax.plot([gx], [gy], "s", color=COLORS[i],
                         markersize=13, zorder=10,
                         markeredgecolor="white", markeredgewidth=1.5)
    ann   = pos_ax.annotate(
        f"GW{i+1}\n({gx:.1f},{gy:.1f})",
        xy=(gx, gy), xytext=(gx + ROOM_W*0.02, gy + ROOM_H*0.02),
        color=COLORS[i], fontsize=7, fontfamily="monospace", zorder=11,
        bbox=dict(boxstyle="round,pad=0.2", fc="#0d1117", alpha=0.7, ec="none")
    )
    gw_points[gw_id] = (pt, halo)
    gw_annots[gw_id] = ann
    draggables.append(DraggableGateway(pt, halo, ann, gw_id))

trail_scatter     = pos_ax.scatter([], [], c=[], cmap="Blues", s=25,
                                    alpha=0.5, zorder=3, vmin=0, vmax=1)
pos_dot,          = pos_ax.plot([], [], "o", color="white", markersize=14,
                                  zorder=12, markeredgecolor="#58a6ff", markeredgewidth=2)
pos_label         = pos_ax.text(0.02, 0.02, "", transform=pos_ax.transAxes,
                                  color="#e6edf3", fontsize=8, fontfamily="monospace",
                                  bbox=dict(boxstyle="round,pad=0.3", fc="#0d1117", alpha=0.7, ec="none"))
status_label      = pos_ax.text(0.02, 0.97, "Press 'm' to load map, 'c' to calibrate, 'b' to draw rooms",
                                  transform=pos_ax.transAxes, color="#f78166",
                                  fontsize=8, fontfamily="monospace", va="top",
                                  bbox=dict(boxstyle="round,pad=0.3", fc="#0d1117", alpha=0.7, ec="none"))
mode_label        = pos_ax.text(0.98, 0.97, f"mode: {MODES[mode_index]}",
                                  transform=pos_ax.transAxes, color="#3fb950",
                                  fontsize=8, fontfamily="monospace", va="top", ha="right",
                                  bbox=dict(boxstyle="round,pad=0.3", fc="#0d1117", alpha=0.7, ec="none"))
room_status_label = pos_ax.text(0.5, 0.97, "ROOM: —", transform=pos_ax.transAxes,
                                  color="#bc8cff", fontsize=11, fontfamily="monospace",
                                  va="top", ha="center", fontweight="bold",
                                  bbox=dict(boxstyle="round,pad=0.4", fc="#0d1117", alpha=0.85, ec="#bc8cff"))
cal_label         = pos_ax.text(0.98, 0.91, "", transform=pos_ax.transAxes,
                                  color="#d29922", fontsize=8, fontfamily="monospace",
                                  va="top", ha="right",
                                  bbox=dict(boxstyle="round,pad=0.3", fc="#0d1117", alpha=0.7, ec="none"))
cal_click_marker, = pos_ax.plot([], [], "+", color="#f78166",
                                  markersize=18, markeredgewidth=2.5, zorder=13)

# ─── ROOM DRAWING / HULL DRAW ──────────────────────────────────────────────────

def redraw_rooms():
    global room_patches, room_label_artists
    for p in room_patches:
        try: p.remove()
        except Exception: pass
    for t in room_label_artists:
        try: t.remove()
        except Exception: pass
    room_patches = []
    room_label_artists = []
    for rm in rooms:
        x0, y0, x1, y1 = rm["x0"], rm["y0"], rm["x1"], rm["y1"]
        w, h = x1 - x0, y1 - y0
        is_current = (rm["name"] == current_room)
        patch = mpatches.Rectangle(
            (x0, y0), w, h,
            linewidth=2 if is_current else 1.2,
            edgecolor="#bc8cff" if is_current else "#6e7681",
            facecolor="#bc8cff" if is_current else "#30363d",
            alpha=0.25 if is_current else 0.12, zorder=1
        )
        pos_ax.add_patch(patch)
        room_patches.append(patch)
        lbl = pos_ax.text(
            x0 + w / 2, y0 + h / 2, rm["name"],
            color="#bc8cff" if is_current else "#8b949e",
            fontsize=9, fontfamily="monospace", ha="center", va="center",
            zorder=2, fontweight="bold" if is_current else "normal"
        )
        room_label_artists.append(lbl)


def redraw_hull():
    """Draw the gateway coverage polygon (convex hull). Rooms inside it are
    well-covered; rooms outside are being extrapolated and will be less reliable."""
    global hull_artist
    if hull_artist is not None:
        try: hull_artist.remove()
        except Exception: pass
        hull_artist = None
    if not SHOW_HULL:
        fig.canvas.draw_idle()
        return
    pts = [tuple(GATEWAY_POSITIONS[g]) for g in GATEWAY_IDS]
    hull = convex_hull(pts)
    if len(hull) >= 3:
        hull_artist = mpatches.Polygon(
            hull, closed=True, fill=False,
            edgecolor="#58a6ff", linewidth=1.5, linestyle=(0, (6, 4)),
            alpha=0.6, zorder=4
        )
        pos_ax.add_patch(hull_artist)
    fig.canvas.draw_idle()


def finish_room_draw():
    global room_draw_mode
    try:
        (cx0, cy0), (cx1, cy1) = room_draw_clicks[0], room_draw_clicks[1]
        x0, y0, x1, y1 = _norm_rect(cx0, cy0, cx1, cy1)
        name = input("\n[ROOM] Enter a name for this room: ").strip()
        if not name:
            name = f"Room {len(rooms) + 1}"
        rooms.append({"name": name, "x0": x0, "y0": y0, "x1": x1, "y1": y1})
        save_calibration()
        print(f"[ROOM] Added '{name}'  ({x0:.1f},{y0:.1f})-({x1:.1f},{y1:.1f})")
    except Exception as e:
        print(f"[ROOM] Error: {e}")
    finally:
        room_draw_mode = False
        room_draw_clicks.clear()
        for m in room_draw_markers:
            try: m.remove()
            except Exception: pass
        room_draw_markers.clear()
        redraw_rooms()
        status_label.set_text(f"Room added. {len(rooms)} rooms total. 'b' for another.")
        status_label.set_color("#3fb950")
        fig.canvas.draw_idle()

# ─── MAP ──────────────────────────────────────────────────────────────────────

def load_map_image():
    global map_image
    if not PIL_AVAILABLE:
        print("[MAP] Install pillow: pip install pillow")
        return
    if not os.path.exists(MAP_FILE):
        print(f"[MAP] '{MAP_FILE}' not found in script folder")
        return
    map_image = np.array(Image.open(MAP_FILE))
    print(f"[MAP] Loaded '{MAP_FILE}'  {map_image.shape[1]}x{map_image.shape[0]}px")


def apply_map_to_axes():
    global map_image_artist
    if map_image is None or map_extent is None:
        return
    if map_image_artist is not None:
        try: map_image_artist.remove()
        except Exception: pass
    map_image_artist = pos_ax.imshow(
        map_image, extent=map_extent, aspect="auto", origin="upper",
        alpha=0.5, zorder=0
    )
    fig.canvas.draw_idle()


def finish_map_scale():
    global map_scale_mode, ROOM_W, ROOM_H, map_extent
    try:
        p1 = map_scale_clicks[0]
        p2 = map_scale_clicks[1]
        real_dist  = float(input("\n[MAP] Enter real distance between the 2 points (metres): ").strip())
        pixel_dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        scale      = real_dist / pixel_dist
        if map_image is not None:
            h_px, w_px = map_image.shape[:2]
            left   = -p1[0] * scale
            right  = left   + w_px * scale
            bottom = -p1[1] * scale
            top    = bottom + h_px * scale
            map_extent = [left, right, bottom, top]
            ROOM_W = right - left
            ROOM_H = top   - bottom
        map_scale_mode = False
        map_scale_clicks.clear()
        apply_map_to_axes()
        pos_ax.set_xlim(map_extent[0] - 1, map_extent[1] + 1)
        pos_ax.set_ylim(map_extent[2] - 1, map_extent[3] + 1)
        room_rect.set_width(ROOM_W)
        room_rect.set_height(ROOM_H)
        redraw_hull()
        status_label.set_text(f"Map scaled! {ROOM_W:.1f}x{ROOM_H:.1f}m — drag gateways into position")
        status_label.set_color("#3fb950")
        fig.canvas.draw_idle()
        print(f"[MAP] Room: {ROOM_W:.1f}x{ROOM_H:.1f}m  scale={scale:.4f}m/px")
    except Exception as e:
        print(f"[MAP] Error: {e}")
        map_scale_mode = False
        map_scale_clicks.clear()

# ─── CLICK / KEY ───────────────────────────────────────────────────────────────

def on_click(event):
    global cal_click_pos, map_extent
    if event.inaxes != pos_ax or event.button != 1:
        return

    if room_draw_mode:
        room_draw_clicks.append((event.xdata, event.ydata))
        m, = pos_ax.plot([event.xdata], [event.ydata], "P",
                          color="#bc8cff", markersize=11, zorder=14)
        room_draw_markers.append(m)
        if len(room_draw_clicks) == 1:
            status_label.set_text("Click the OPPOSITE corner of the room")
        elif len(room_draw_clicks) == 2:
            status_label.set_text("Check terminal — enter the room name")
            threading.Thread(target=finish_room_draw, daemon=True).start()
        fig.canvas.draw_idle()
        return

    if map_scale_mode:
        map_scale_clicks.append((event.xdata, event.ydata))
        m, = pos_ax.plot([event.xdata], [event.ydata], "D",
                          color="#ffcc00", markersize=10, zorder=14)
        scale_markers.append(m)
        pos_ax.annotate(f"P{len(map_scale_clicks)}", xy=(event.xdata, event.ydata),
                         xytext=(event.xdata * 1.005, event.ydata * 1.005),
                         color="#ffcc00", fontsize=8, zorder=14)
        if len(map_scale_clicks) == 1:
            status_label.set_text("Click the SECOND reference point")
        elif len(map_scale_clicks) == 2:
            status_label.set_text("Check terminal — enter the real distance")
            threading.Thread(target=finish_map_scale, daemon=True).start()
        fig.canvas.draw_idle()
        return

    if cal_mode:
        cal_click_pos = (max(0, min(ROOM_W, event.xdata)),
                         max(0, min(ROOM_H, event.ydata)))
        cal_click_marker.set_data([cal_click_pos[0]], [cal_click_pos[1]])
        status_label.set_text(
            f"Position ({cal_click_pos[0]:.2f}, {cal_click_pos[1]:.2f}) — press SPACE to record")
        fig.canvas.draw_idle()


def on_key(event):
    global cal_mode, cal_click_pos, map_scale_mode, map_extent, mode_index
    global room_draw_mode, SHOW_HULL

    if event.key == "m":
        if map_image is None:
            status_label.set_text(f"No map — place '{MAP_FILE}' in script folder")
            fig.canvas.draw_idle()
            return
        map_scale_mode = True
        map_scale_clicks.clear()
        for m in scale_markers:
            try: m.remove()
            except: pass
        scale_markers.clear()
        h_px, w_px = map_image.shape[:2]
        pos_ax.set_xlim(-w_px * 0.02, w_px * 1.02)
        pos_ax.set_ylim(-h_px * 0.02, h_px * 1.02)
        if map_image_artist is not None:
            try: map_image_artist.remove()
            except: pass
        pos_ax.imshow(map_image, extent=[0.0, float(w_px), 0.0, float(h_px)],
                      aspect="auto", origin="upper", alpha=0.5, zorder=0)
        status_label.set_text("SCALE MODE: click FIRST reference point on map")
        status_label.set_color("#ffcc00")
        fig.canvas.draw_idle()

    elif event.key == "b":
        if ROOM_W <= 0 or ROOM_H <= 0:
            status_label.set_text("Scale the map first ('m') before drawing rooms")
            fig.canvas.draw_idle()
            return
        room_draw_mode = True
        room_draw_clicks.clear()
        for m in room_draw_markers:
            try: m.remove()
            except: pass
        room_draw_markers.clear()
        status_label.set_text("ROOM MODE: click FIRST corner of the room")
        status_label.set_color("#bc8cff")
        fig.canvas.draw_idle()

    elif event.key == "x":
        if rooms:
            removed = rooms.pop()
            save_calibration()
            redraw_rooms()
            status_label.set_text(f"Deleted room '{removed['name']}'. {len(rooms)} left.")
            status_label.set_color("#f78166")
            fig.canvas.draw_idle()
            print(f"[ROOM] Deleted '{removed['name']}'")
        else:
            status_label.set_text("No rooms to delete")
            fig.canvas.draw_idle()

    elif event.key == "h":
        SHOW_HULL = not SHOW_HULL
        redraw_hull()
        status_label.set_text(f"Coverage hull {'ON' if SHOW_HULL else 'OFF'}")
        status_label.set_color("#58a6ff")

    elif event.key == "w":
        mode_index = (mode_index + 1) % len(MODES)
        mode_label.set_text(f"mode: {MODES[mode_index]}")
        status_label.set_text(f"Switched to {MODES[mode_index]} mode")
        status_label.set_color("#58a6ff")
        fig.canvas.draw_idle()
        print(f"[MODE] {MODES[mode_index]}")

    elif event.key == "c":
        cal_mode = True
        cal_click_pos = None
        status_label.set_text("CAL MODE: click your position on the map, then SPACE")
        status_label.set_color("#f78166")
        fig.canvas.draw_idle()

    elif event.key == " " and cal_mode:
        if cal_click_pos is None:
            print("[CAL] Click your position first")
            return
        with data_lock:
            snap = {gw: kalman[gw].x for gw in GATEWAY_IDS if kalman[gw].is_fresh()}
        rssi_per_gw = dict(snap)
        if len(rssi_per_gw) < 2:
            print("[CAL] Not enough fresh RSSI data yet")
            return
        cal_points.append({"x": cal_click_pos[0], "y": cal_click_pos[1],
                            "rssi_per_gw": rssi_per_gw})
        off = max(ROOM_W, ROOM_H) * 0.015
        m, = pos_ax.plot([cal_click_pos[0]], [cal_click_pos[1]], "x",
                          color="#f78166", markersize=12, markeredgewidth=2.5, zorder=13)
        pos_ax.annotate(f"P{len(cal_points)}", xy=cal_click_pos,
                         xytext=(cal_click_pos[0] + off, cal_click_pos[1] + off),
                         color="#f78166", fontsize=7, fontfamily="monospace", zorder=13)
        cal_pt_markers.append(m)
        cal_click_pos = None
        cal_click_marker.set_data([], [])
        status_label.set_text(f"Recorded P{len(cal_points)} — move to next spot or press 'f' to fit")
        cal_label.set_text(f"{len(cal_points)} cal pts")
        fig.canvas.draw_idle()
        print(f"[CAL] P{len(cal_points)}: ({cal_points[-1]['x']:.1f},{cal_points[-1]['y']:.1f})  rssi={rssi_per_gw}")

    elif event.key == "f":
        if len(cal_points) < 3:
            status_label.set_text(f"Need ≥3 cal points (have {len(cal_points)})")
            fig.canvas.draw_idle()
            return
        status_label.set_text(f"Fitting {len(cal_points)} points... ~10s")
        fig.canvas.draw_idle()
        print(f"[CAL] Fitting with {len(cal_points)} points...")

        def do_fit():
            tx, n = fit_path_loss(cal_points)
            if tx is not None:
                cal_params["tx_power"]      = tx
                cal_params["path_loss_exp"] = n
                save_calibration()
                msg = f"Calibrated! TX={tx}  n={n}  ({len(cal_points)} pts)"
                status_label.set_text(msg)
                status_label.set_color("#3fb950")
                fig.canvas.draw_idle()
                print(f"[CAL] {msg}")
            else:
                status_label.set_text("Fit failed — add more spread-out points and retry")
                fig.canvas.draw_idle()
        threading.Thread(target=do_fit, daemon=True).start()

    elif event.key == "r":
        cal_points.clear()
        cal_mode = False
        cal_click_pos = None
        for m in cal_pt_markers:
            try: m.remove()
            except: pass
        cal_pt_markers.clear()
        cal_click_marker.set_data([], [])
        cal_params["tx_power"]      = TX_POWER
        cal_params["path_loss_exp"] = PATH_LOSS_EXP
        status_label.set_text("Calibration reset (rooms kept)")
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

    now = datetime.now()
    with data_lock:
        rssi_snap = {gw: list(rssi_data[gw]) for gw in GATEWAY_IDS}
        kf_snap   = {gw: (kalman[gw].x, kalman[gw].is_fresh(now)) for gw in GATEWAY_IDS}

    for i, gw_id in enumerate(GATEWAY_IDS):
        vals = rssi_snap[gw_id]
        if vals:
            rssi_lines[i].set_data(list(range(len(vals))), vals)
            rssi_axes[i].set_xlim(0, max(60, len(vals)))
            kx, fresh = kf_snap[gw_id]
            rssi_text[i].set_text(f"{kx:.1f} dBm{'' if fresh else ' (stale)'}")
            rssi_text[i].set_color(
                "#3fb950" if (fresh and kx >= -60) else
                "#d29922" if (fresh and kx >= -75) else "#f78166")

    for c in circle_artists:
        try: c.remove()
        except: pass
    circle_artists = []

    rssi_vals = {}
    distances = {}
    for gw_id in GATEWAY_IDS:
        kx, fresh = kf_snap[gw_id]
        if fresh:
            rssi_vals[gw_id] = kx
            distances[gw_id] = rssi_to_distance(kx)
            if MODES[mode_index] in ("trilateration", "hybrid"):
                gx, gy = GATEWAY_POSITIONS[gw_id]
                circ = plt.Circle((gx, gy), distances[gw_id],
                                   color=GW_COLORS[gw_id], fill=False,
                                   linewidth=1, linestyle="--", alpha=0.3, zorder=2)
                pos_ax.add_patch(circ)
                circle_artists.append(circ)
        else:
            distances[gw_id] = None

    pos = get_position(rssi_vals, distances)
    prev_room = current_room
    if pos is not None:
        x, y = pos
        with data_lock:
            pos_trail.append((x, y))
            trail = list(pos_trail)
        if len(trail) > 1:
            trail_scatter.set_offsets(np.c_[[p[0] for p in trail], [p[1] for p in trail]])
            trail_scatter.set_array(np.linspace(0, 1, len(trail)))
        pos_dot.set_data([x], [y])
        room_now = update_room_decision(pos)
        room_status_label.set_text(f"ROOM: {room_now if room_now else '—'}")
        consistency = rssi_consistency(rssi_vals)
        n_fresh = sum(1 for _, f in kf_snap.values() if f)
        pos_label.set_text(
            f"x={x:.1f}m  y={y:.1f}m\n"
            f"room={room_now if room_now else '—'}\n"
            f"gw_live={n_fresh}/4   consistency={consistency:.2f}\n"
            f"TX={cal_params['tx_power']}  n={cal_params['path_loss_exp']}")

    if current_room != prev_room:
        redraw_rooms()
        print(f"[ROOM] -> {current_room}")

    return rssi_lines + rssi_text + [pos_dot, pos_label, trail_scatter,
                                      status_label, mode_label, cal_label,
                                      room_status_label, cal_click_marker]


if __name__ == "__main__":
    load_map_image()
    load_calibration()
    if map_image is not None and map_extent is not None:
        apply_map_to_axes()
    redraw_rooms()
    redraw_hull()

    print(f"\nBroker  : {BROKER_HOST}:{BROKER_PORT}")
    print(f"Target  : {TARGET_MAC}")
    print(f"Map     : {MAP_FILE} {'✓' if map_image is not None else '✗ not found'}")
    print(f"Gateways: {len(GATEWAY_IDS)}")
    for i, g in enumerate(GATEWAY_IDS):
        print(f"   GW{i+1} {g}  pos={GATEWAY_POSITIONS[g]}  topic={GATEWAY_TOPICS[g]['topic']}")
    print(f"Mode    : {MODES[mode_index]}   Kalman R={KALMAN_R} Q={KALMAN_Q}")
    print(f"Rooms   : {len(rooms)}   Hysteresis: margin={ROOM_MARGIN_M}m persist={ROOM_PERSIST_FRAMES}")
    print("-" * 64)
    print(">>> If GW4 (2805a55eedec) shows no [HH:MM:SS] rssi lines, fix its")
    print(">>> 'topic' in GATEWAY_TOPICS — check the exact path in MQTT Explorer.")
    print("-" * 64)

    mqtt_thread = threading.Thread(target=start_mqtt, daemon=True)
    mqtt_thread.start()

    ani = animation.FuncAnimation(fig, animate, interval=500,
                                  blit=False, cache_frame_data=False)
    plt.show()
