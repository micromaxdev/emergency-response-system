#!/usr/bin/env python3
"""
BLE Gateway MQTT Listener + Trilateration Position Estimator
Gateways can be dragged to their real positions on the map.

Setup:
    pip install paho-mqtt matplotlib numpy scipy
"""

import json
import threading
from datetime import datetime
from collections import deque

import numpy as np
from scipy.optimize import minimize
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches

# ─── EDIT THESE ───────────────────────────────────────────────────────────────

BROKER_HOST = "192.168.115.49"
BROKER_PORT  = 1883
BROKER_USER  = ""
BROKER_PASS  = ""

TARGET_MAC = "e7eabd18a8c3"

# Initial gateway positions in metres (x, y) — drag them on the map to adjust
GATEWAY_POSITIONS = {
    "ac233fc25932": [0.0, 0.0],
    "ac233fc26ecb": [1.0, 2.0],
    "ac233fc26ecc": [2.0, 0.0],
}

# Room size in metres
ROOM_W = 18
ROOM_H = 23

# Path loss model: distance = 10 ^ ((TX_POWER - rssi) / (10 * PATH_LOSS_EXP))
TX_POWER      = -50
PATH_LOSS_EXP = 3.5

SMOOTH_N  = 12
MAX_TRAIL = 30

# ──────────────────────────────────────────────────────────────────────────────

data_lock   = threading.Lock()
rssi_data   = {gw: deque(maxlen=60)       for gw in GATEWAY_POSITIONS}
rssi_smooth = {gw: deque(maxlen=SMOOTH_N) for gw in GATEWAY_POSITIONS}
pos_trail   = deque(maxlen=MAX_TRAIL)

GATEWAY_IDS = list(GATEWAY_POSITIONS.keys())
COLORS      = ["#2196F3", "#4CAF50", "#F44336"]
GW_COLORS   = {gid: COLORS[i] for i, gid in enumerate(GATEWAY_IDS)}

# ─── RSSI → DISTANCE ──────────────────────────────────────────────────────────

def rssi_to_distance(rssi):
    return 10 ** ((TX_POWER - rssi) / (10.0 * PATH_LOSS_EXP))

# ─── TRILATERATION ────────────────────────────────────────────────────────────

def trilaterate(distances):
    gws = [(gid, distances[gid]) for gid in distances if distances[gid] is not None]
    if len(gws) < 2:
        return None

    def error(pos):
        x, y = pos
        total = 0
        for gid, d in gws:
            gx, gy = GATEWAY_POSITIONS[gid]
            estimated = np.sqrt((x - gx)**2 + (y - gy)**2)
            total += (estimated - d) ** 2
        return total

    x0 = np.mean([GATEWAY_POSITIONS[gid][0] for gid, _ in gws])
    y0 = np.mean([GATEWAY_POSITIONS[gid][1] for gid, _ in gws])

    result = minimize(error, [x0, y0], method="Nelder-Mead",
                      options={"xatol": 0.01, "fatol": 0.01, "maxiter": 1000})
    x, y = result.x
    x = max(0, min(ROOM_W, x))
    y = max(0, min(ROOM_H, y))
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
    def __init__(self, point, label, annotation, gateway_id):
        self.point      = point
        self.label      = label
        self.annotation = annotation
        self.gateway_id = gateway_id
        self.dragging   = False
        self.connect()

    def connect(self):
        self.cidpress   = self.point.figure.canvas.mpl_connect("button_press_event",   self.on_press)
        self.cidrelease = self.point.figure.canvas.mpl_connect("button_release_event", self.on_release)
        self.cidmotion  = self.point.figure.canvas.mpl_connect("motion_notify_event",  self.on_motion)

    def on_press(self, event):
        if event.inaxes != self.point.axes:
            return
        x, y = GATEWAY_POSITIONS[self.gateway_id]
        dist = np.sqrt((event.xdata - x)**2 + (event.ydata - y)**2)
        if dist < 0.15:  # click radius in metres
            self.dragging = True

    def on_release(self, event):
        if self.dragging:
            self.dragging = False
            x, y = GATEWAY_POSITIONS[self.gateway_id]
            print(f"[DRAG] {self.gateway_id} moved to ({x:.2f}, {y:.2f})")

    def on_motion(self, event):
        if not self.dragging or event.inaxes != self.point.axes:
            return
        x = max(0, min(ROOM_W, event.xdata))
        y = max(0, min(ROOM_H, event.ydata))
        GATEWAY_POSITIONS[self.gateway_id][0] = x
        GATEWAY_POSITIONS[self.gateway_id][1] = y
        self.point.set_data([x], [y])
        self.annotation.set_position((x + 0.06, y + 0.06))
        short = self.gateway_id[-4:]
        idx   = GATEWAY_IDS.index(self.gateway_id)
        self.annotation.set_text(f"GW{idx+1}\n({x:.1f},{y:.1f})")
        self.point.figure.canvas.draw_idle()

# ─── MATPLOTLIB LAYOUT ────────────────────────────────────────────────────────

fig = plt.figure(figsize=(13, 7), facecolor="#0d1117")
fig.suptitle("BLE Indoor Position Estimator  |  drag gateways to set positions",
             color="#58a6ff", fontsize=11, fontfamily="monospace", y=0.98)

gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.4], hspace=0.55, wspace=0.35,
                      left=0.07, right=0.96, top=0.93, bottom=0.07)

rssi_axes  = [fig.add_subplot(gs[i, 0]) for i in range(3)]
pos_ax     = fig.add_subplot(gs[:, 1])

rssi_lines = []
rssi_text  = []

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

# Position map
pos_ax.set_facecolor("#161b22")
pos_ax.set_xlim(-0.3, ROOM_W + 0.3)
pos_ax.set_ylim(-0.3, ROOM_H + 0.3)
pos_ax.set_aspect("equal")
pos_ax.set_title("Estimated Position  (drag ■ to move gateways)",
                 color="#e6edf3", fontsize=8, fontfamily="monospace", pad=6)
pos_ax.tick_params(colors="#484f58", labelsize=7)
pos_ax.set_xlabel("x (m)", color="#484f58", fontsize=7, fontfamily="monospace")
pos_ax.set_ylabel("y (m)", color="#484f58", fontsize=7, fontfamily="monospace")
for spine in pos_ax.spines.values():
    spine.set_edgecolor("#21262d")
pos_ax.grid(True, color="#21262d", linewidth=0.5)

# Room boundary
room_rect = mpatches.FancyBboxPatch(
    (0, 0), ROOM_W, ROOM_H,
    boxstyle="square,pad=0",
    linewidth=1.5, edgecolor="#484f58", facecolor="none"
)
pos_ax.add_patch(room_rect)

# Draggable gateway markers
draggables = []
gw_points  = {}
gw_annots  = {}

for i, (gw_id, (gx, gy)) in enumerate(GATEWAY_POSITIONS.items()):
    pt, = pos_ax.plot([gx], [gy], "s", color=COLORS[i], markersize=12,
                      zorder=5, picker=5)
    ann = pos_ax.annotate(
        f"GW{i+1}\n({gx:.1f},{gy:.1f})",
        xy=(gx, gy), xytext=(gx + 0.06, gy + 0.06),
        color=COLORS[i], fontsize=7, fontfamily="monospace", zorder=6
    )
    gw_points[gw_id] = pt
    gw_annots[gw_id] = ann
    draggables.append(DraggableGateway(pt, f"GW{i+1}", ann, gw_id))

# Trail and position dot
trail_scatter = pos_ax.scatter([], [], c=[], cmap="Blues", s=20,
                                alpha=0.4, zorder=3, vmin=0, vmax=1)
pos_dot,      = pos_ax.plot([], [], "o", color="white", markersize=12,
                              zorder=6, markeredgecolor="#58a6ff", markeredgewidth=2)
pos_label     = pos_ax.text(0.02, 0.02, "", transform=pos_ax.transAxes,
                              color="#e6edf3", fontsize=8, fontfamily="monospace")

circle_artists = []

# ─── ANIMATION ────────────────────────────────────────────────────────────────

def animate(frame):
    global circle_artists

    with data_lock:
        rssi_snap   = {gw: list(rssi_data[gw])   for gw in GATEWAY_IDS}
        smooth_snap = {gw: list(rssi_smooth[gw])  for gw in GATEWAY_IDS}
        trail_snap  = list(pos_trail)

    # update RSSI graphs
    for i, gw_id in enumerate(GATEWAY_IDS):
        vals = rssi_snap[gw_id]
        if vals:
            rssi_lines[i].set_data(list(range(len(vals))), vals)
            rssi_axes[i].set_xlim(0, max(60, len(vals)))
            latest = vals[-1]
            rssi_text[i].set_text(f"{latest} dBm")
            rssi_text[i].set_color(
                "#3fb950" if latest >= -60 else
                "#d29922" if latest >= -75 else
                "#f78166"
            )

    # remove old circles
    for c in circle_artists:
        c.remove()
    circle_artists = []

    # compute distances and draw circles
    distances = {}
    for gw_id in GATEWAY_IDS:
        vals = smooth_snap[gw_id]
        if vals:
            avg_rssi = sum(vals) / len(vals)
            d = rssi_to_distance(avg_rssi)
            distances[gw_id] = d
            gx, gy = GATEWAY_POSITIONS[gw_id]
            circ = plt.Circle((gx, gy), d, color=GW_COLORS[gw_id],
                               fill=False, linewidth=1, linestyle="--", alpha=0.3)
            pos_ax.add_patch(circ)
            circle_artists.append(circ)
        else:
            distances[gw_id] = None

    # trilaterate
    pos = trilaterate(distances)
    if pos is not None:
        x, y = pos
        with data_lock:
            pos_trail.append((x, y))
            trail_snap = list(pos_trail)

        if len(trail_snap) > 1:
            tx = [p[0] for p in trail_snap]
            ty = [p[1] for p in trail_snap]
            trail_scatter.set_offsets(np.c_[tx, ty])
            trail_scatter.set_array(np.linspace(0, 1, len(trail_snap)))

        pos_dot.set_data([x], [y])
        pos_label.set_text(f"x={x:.2f}m  y={y:.2f}m")

    return rssi_lines + rssi_text + [pos_dot, pos_label, trail_scatter]


if __name__ == "__main__":
    print(f"\nBroker  : {BROKER_HOST}:{BROKER_PORT}")
    print(f"Target  : {TARGET_MAC}")
    print(f"\nGateway starting positions (drag to adjust):")
    for i, (gid, pos) in enumerate(GATEWAY_POSITIONS.items()):
        print(f"  GW{i+1} {gid} → {pos}")
    print(f"\nTX Power: {TX_POWER} dBm  |  Path Loss Exp: {PATH_LOSS_EXP}")
    print("-" * 50)

    mqtt_thread = threading.Thread(target=start_mqtt, daemon=True)
    mqtt_thread.start()

    ani = animation.FuncAnimation(
        fig, animate, interval=500, blit=False, cache_frame_data=False
    )

    plt.show()
