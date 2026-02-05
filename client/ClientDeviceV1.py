# Pico W - Combined Emergency Script
# Mass: 5 rapid presses -> MASS emergency
# Personal: hold 5s -> PERSONAL emergency
# Fall detection (MPU6050) -> PERSONAL emergency (WITH 10s CANCEL WINDOW)
#   During the 10s: if the button is pressed once, cancel the fall (do not send).

import time, math, _thread
from machine import Pin, I2C
import network
import socket
import ujson

# ---------------- User config ----------------
WIFI_SSID = "add wifi ssid"
WIFI_PASS = "add wifi pass"

PI5_IP   = "add pi5 IP"
PI5_PORT = 8080

DEVICE_ID = "PICO_TEST_01"

# ---------------- Button config (GPIO) ----------------
BUTTON_GPIO = 15
button = Pin(BUTTON_GPIO, Pin.IN, Pin.PULL_UP)  # active-low

def read_pressed() -> int:
    return 1 if button.value() == 0 else 0  # pressed=1

def wait_for_release(read_pressed_fn, max_ms=8000):
    """Block briefly until the button is released (prevents 'stuck' after trigger)."""
    t0 = time.ticks_ms()
    while read_pressed_fn():
        if time.ticks_diff(time.ticks_ms(), t0) > max_ms:
            break
        time.sleep_ms(20)

# ---------------- Mass emergency config ----------------
WINDOW_MS = 2200
DEBOUNCE_MS = 80
MASS_COOLDOWN_MS = 2500
REQUIRED_PRESSES = 5

# ---------------- Personal emergency config ----------------
HOLD_SECONDS = 5
CHECK_MS = 50
PERSONAL_COOLDOWN_MS = 1200

# ---------------- Fall detection thresholds ----------------
FALL_THRESHOLD_LOW  = 0.4   # g
FALL_THRESHOLD_HIGH = 3.5   # g
FALL_MIN_DURATION   = 100   # ms
FALL_MAX_DURATION   = 1500  # ms
FALL_COOLDOWN_MS    = 10000 # ms (prevents repeat fall spamming)

# Fall cancel window
FALL_CANCEL_WINDOW_MS = 10_000  # 10 seconds to cancel by single press

# ---------------- Network behavior ----------------
WIFI_CONNECT_TIMEOUT_MS = 4000   # keep loop responsive
SOCKET_TIMEOUT_S = 2

# ---------------- Hardware ----------------
led = Pin("LED", Pin.OUT)
led.off() 

# ---------------- Wi-Fi + Send ----------------
def wifi_connect(timeout_ms=WIFI_CONNECT_TIMEOUT_MS):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if wlan.isconnected():
        return wlan

    print("Connecting Wi-Fi...")
    wlan.connect(WIFI_SSID, WIFI_PASS)
    t0 = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            print("Wi-Fi connect timeout")
            return wlan
        time.sleep_ms(150)

    print("Wi-Fi connected:", wlan.ifconfig())
    return wlan

def send_to_pi5(payload: dict, wlan) -> bool:
    s = None
    try:
        if not wlan.isconnected():
            print("Wi-Fi dropped, quick reconnect...")
            wlan = wifi_connect(WIFI_CONNECT_TIMEOUT_MS)

        if not wlan.isconnected():
            print("No Wi-Fi, cannot send right now.")
            return False

        s = socket.socket()
        s.settimeout(SOCKET_TIMEOUT_S)
        s.connect((PI5_IP, PI5_PORT))
        s.send(ujson.dumps(payload).encode("utf-8"))
        s.close()
        return True
    except Exception as e:
        print("Send failed:", e)
        try:
            if s:
                s.close()
        except:
            pass
        return False

def send_emergency(em_type: str, msg: str, wlan, extra: dict=None) -> bool:
    now_ms = time.ticks_ms()
    payload = {
        "device_id": DEVICE_ID,
        "emergency": True,
        "emergency_type": em_type,   # "mass" or "personal"
        "emergency_message": msg,
        "timestamp_ms": now_ms,      # local tick-ms
        "seq": now_ms,               # simple sequence for de-dupe on Pi5
    }
    if extra:
        payload.update(extra)

    ok = send_to_pi5(payload, wlan)
    print("Sent to Pi5:", ok, payload)
    return ok

# ---------------- Shared state for fall / sensors ----------------
latest_accel = {"x": 0.0, "y": 0.0, "z": 0.0}
latest_gyro  = {"x": 0.0, "y": 0.0, "z": 0.0}
latest_heart_rate = 0

fall_detected_flag = False
fall_detected_at_ms = 0

data_lock = _thread.allocate_lock()

# ---------------- MPU6050----------------
MPU_ADDR = 0x68

# Registers
PWR_MGMT_1   = 0x6B
CONFIG       = 0x1A
SMPLRT_DIV   = 0x19
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

ACCEL_RANGE_REGVAL = 0x10  # ±8g
GYRO_RANGE_REGVAL  = 0x08  # ±500 dps
DLPF_CFG_REGVAL    = 0x03  # ~42 Hz
SMPLRT_DIV_REGVAL  = 0x04

LSB_PER_G   = {0x00: 16384.0, 0x08: 8192.0, 0x10: 4096.0, 0x18: 2048.0}
LSB_PER_DPS = {0x00: 131.0,   0x08: 65.5,   0x10: 32.8,   0x18: 16.4}

def to_i16(msb, lsb):
    v = (msb << 8) | lsb
    return v - 65536 if v & 0x8000 else v

def write_reg(i2c, reg, val, tries=8):
    for _ in range(tries):
        try:
            i2c.writeto_mem(MPU_ADDR, reg, bytes([val]))
            return
        except OSError:
            time.sleep_ms(30)
    raise OSError("EIO writing reg 0x%02X" % reg)

def read_reg(i2c, reg, tries=8):
    for _ in range(tries):
        try:
            return i2c.readfrom_mem(MPU_ADDR, reg, 1)[0]
        except OSError:
            time.sleep_ms(30)
    raise OSError("EIO reading reg 0x%02X" % reg)

class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr

        # Match your init exactly
        write_reg(self.i2c, PWR_MGMT_1, 0x00)      # wake
        time.sleep_ms(100)
        write_reg(self.i2c, CONFIG, DLPF_CFG_REGVAL)
        write_reg(self.i2c, SMPLRT_DIV, SMPLRT_DIV_REGVAL)
        write_reg(self.i2c, ACCEL_CONFIG, ACCEL_RANGE_REGVAL)
        write_reg(self.i2c, GYRO_CONFIG,  GYRO_RANGE_REGVAL)
        time.sleep_ms(20)

        a_cfg = read_reg(self.i2c, ACCEL_CONFIG)
        g_cfg = read_reg(self.i2c, GYRO_CONFIG)
        c_cfg = read_reg(self.i2c, CONFIG)
        s_div = read_reg(self.i2c, SMPLRT_DIV)

        print("MPU CONFIG:", "ACCEL_CONFIG", hex(a_cfg), "GYRO_CONFIG", hex(g_cfg),
              "CONFIG", hex(c_cfg), "SMPLRT_DIV", hex(s_div))

        self.lsb_per_g   = LSB_PER_G[ACCEL_RANGE_REGVAL]
        self.lsb_per_dps = LSB_PER_DPS[GYRO_RANGE_REGVAL]

    def read_accel_gyro(self):
        # 14 bytes: accel(6), temp(2), gyro(6)
        d = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 14)

        ax = to_i16(d[0], d[1]) / self.lsb_per_g
        ay = to_i16(d[2], d[3]) / self.lsb_per_g
        az = to_i16(d[4], d[5]) / self.lsb_per_g

        gx = to_i16(d[8],  d[9])  / self.lsb_per_dps
        gy = to_i16(d[10], d[11]) / self.lsb_per_dps
        gz = to_i16(d[12], d[13]) / self.lsb_per_dps

        return ax, ay, az, gx, gy, gz

# ---------------- Threads ----------------
def fall_detection_thread(mpu):
    global fall_detected_flag, fall_detected_at_ms, latest_accel, latest_gyro

    print("[Fall] thread started")
    fall_state = "normal"
    freefall_start = 0
    cooldown_start = 0
    error_count = 0

    while True:
        try:
            ax, ay, az, gx, gy, gz = mpu.read_accel_gyro()
            total_accel = math.sqrt(ax*ax + ay*ay + az*az)

            # Update shared latest sensor values (LOCK SAFE)
            data_lock.acquire()
            try:
                latest_accel = {"x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)}
                latest_gyro  = {"x": round(gx, 2), "y": round(gy, 2), "z": round(gz, 2)}
            finally:
                data_lock.release()

            error_count = 0

            if fall_state == "normal":
                if total_accel < FALL_THRESHOLD_LOW:
                    fall_state = "freefall"
                    freefall_start = time.ticks_ms()

            elif fall_state == "freefall":
                dt = time.ticks_diff(time.ticks_ms(), freefall_start)

                if total_accel > FALL_THRESHOLD_HIGH:
                    if FALL_MIN_DURATION < dt < FALL_MAX_DURATION:
                        impact_time = time.ticks_ms()
                        print("[FALL] IMPACT |a|=", round(total_accel, 2), "g after", dt, "ms")

                        data_lock.acquire()
                        try:
                            fall_detected_flag = True
                            fall_detected_at_ms = impact_time
                        finally:
                            data_lock.release()

                        fall_state = "cooldown"
                        cooldown_start = impact_time
                    else:
                        fall_state = "normal"

                elif dt > FALL_MAX_DURATION:
                    fall_state = "normal"

            elif fall_state == "cooldown":
                if time.ticks_diff(time.ticks_ms(), cooldown_start) > FALL_COOLDOWN_MS:
                    fall_state = "normal"

            time.sleep_ms(100)

        except OSError:
            error_count += 1
            if error_count % 20 == 1:
                print("[Fall] I2C error")
            time.sleep_ms(200)

        except Exception as e:
            print("[Fall] error:", e)
            time.sleep(1)

# ---------------- Init sensors ----------------
time.sleep(1)

# MATCH your table script: I2C0 SDA=GP0, SCL=GP1, freq=100k
I2C_FREQ = 100_000
i2c0 = I2C(0, sda=Pin(0), scl=Pin(1), freq=I2C_FREQ)
time.sleep_ms(200)

scan = i2c0.scan()
print("I2C scan:", [hex(x) for x in scan])
if MPU_ADDR not in scan:
    raise RuntimeError("MPU6050 not found at 0x68 (check wiring/power)")

mpu = MPU6050(i2c0)
print("MPU6050 initialized (matched config)")

_thread.start_new_thread(fall_detection_thread, (mpu,))
print("Fall detection thread started")

# ---------------- Button state ----------------
prev_pressed = 0
last_edge = 0

press_count = 0
window_start = 0
mass_cooldown_until = 0

press_start = None
personal_triggered = False
last_print_sec = -1
personal_cooldown_until = 0

global_cooldown_until = 0
fall_send_cooldown_until = 0

# ---------------- Fall pending (NEW) ----------------
fall_pending = False
fall_pending_start_ms = 0
fall_pending_extra = None 

def start_fall_pending(extra: dict, start_ms: int):
    global fall_pending, fall_pending_start_ms, fall_pending_extra
    fall_pending = True
    fall_pending_start_ms = start_ms
    fall_pending_extra = extra
    led.on()
    print("[FALL] Pending for 10s (press button once to cancel).")

def cancel_fall_pending():
    global fall_pending, fall_pending_start_ms, fall_pending_extra
    fall_pending = False
    fall_pending_start_ms = 0
    fall_pending_extra = None
    led.off()
    print("[FALL] Cancelled by button press.")

def confirm_send_fall(wlan):
    global fall_pending, fall_pending_start_ms, fall_pending_extra
    extra = fall_pending_extra
    fall_pending = False
    fall_pending_start_ms = 0
    fall_pending_extra = None
    led.off()

    print("!!! FALL CONFIRMED (no cancel) -> PERSONAL EMERGENCY !!!")
    send_emergency("personal", "FALL DETECTED", wlan, extra=extra)

print("READY:")
print(f" - Rapid press GPIO{BUTTON_GPIO} 5 times -> MASS EMERGENCY")
print(f" - Hold GPIO{BUTTON_GPIO} {HOLD_SECONDS} seconds -> PERSONAL EMERGENCY")
print(" - Fall detection -> LED ON, 10s cancel window -> then PERSONAL EMERGENCY")
print(f"Target Pi5: {PI5_IP}:{PI5_PORT}")

wlan = wifi_connect()

# ---------------- Main loop ----------------
while True:
    now = time.ticks_ms()

    # ---- Global cooldown (prevents back-to-back triggers)
    if time.ticks_diff(now, global_cooldown_until) < 0:
        time.sleep_ms(CHECK_MS)
        continue

    # ---- Read button early (needed for fall-cancel edge handling)
    pressed = read_pressed()
    rising_edge = (prev_pressed == 0 and pressed == 1)

    # ---- Handle fall pending cancel window FIRST
    # If the wearer presses once during the 10s window, we cancel and we DO NOT
    # let that press affect mass/personal logic (prevents accidental triggers).
    if fall_pending:
        if rising_edge:
            cancel_fall_pending()
            wait_for_release(read_pressed)
            prev_pressed = 0
            time.sleep_ms(CHECK_MS)
            continue

        # Time to confirm send?
        if time.ticks_diff(now, fall_pending_start_ms) >= FALL_CANCEL_WINDOW_MS:
            confirm_send_fall(wlan)
            fall_send_cooldown_until = time.ticks_add(now, FALL_COOLDOWN_MS)
            global_cooldown_until = time.ticks_add(now, 800)
            time.sleep_ms(CHECK_MS)
            prev_pressed = pressed
            continue

    # ---- Check fall event (from thread)
    fall_alert = False
    accel = None
    gyro = None
    fall_time_ms = 0

    data_lock.acquire()
    try:
        if fall_detected_flag:
            fall_alert = True
            fall_detected_flag = False
            fall_time_ms = fall_detected_at_ms
        accel = latest_accel.copy()
        gyro  = latest_gyro.copy()
    finally:
        data_lock.release()

    # Start pending window instead of sending immediately
    if fall_alert and (not fall_pending) and time.ticks_diff(now, fall_send_cooldown_until) >= 0:
        extra = {
            "trigger_source": "fall_detection",
            "fall_timestamp_ms": fall_time_ms,
            "accelerometer": accel,
            "gyroscope": gyro,
            "mpu_config": {
                "accel_range_reg": hex(ACCEL_RANGE_REGVAL),
                "gyro_range_reg": hex(GYRO_RANGE_REGVAL),
                "dlpf_config": hex(DLPF_CFG_REGVAL),
                "smplrt_div": hex(SMPLRT_DIV_REGVAL),
                "i2c_freq": I2C_FREQ,
            }
        }
        start_fall_pending(extra, now)

    # ---- MASS logic: count rapid presses (rising edges)
    if time.ticks_diff(now, mass_cooldown_until) >= 0:
        if rising_edge:
            # debounce
            if time.ticks_diff(now, last_edge) > DEBOUNCE_MS:
                last_edge = now

                if press_count == 0:
                    window_start = now
                    press_count = 1
                else:
                    if time.ticks_diff(now, window_start) > WINDOW_MS:
                        window_start = now
                        press_count = 1
                    else:
                        press_count += 1

                print("[PRESS] count =", press_count)

                if press_count >= REQUIRED_PRESSES and time.ticks_diff(now, window_start) <= WINDOW_MS:
                    print("!!! MASS EMERGENCY TRIGGERED !!!")

                    send_emergency(
                        "mass",
                        "MASS EMERGENCY TRIGGERED (5 rapid presses)",
                        wlan,
                        extra={"press_count": press_count, "window_ms": WINDOW_MS, "trigger_source": "rapid_presses"}
                    )

                    press_count = 0
                    window_start = 0
                    mass_cooldown_until = time.ticks_add(now, MASS_COOLDOWN_MS)
                    global_cooldown_until = time.ticks_add(now, 1500)

                    # reset personal hold state
                    press_start = None
                    personal_triggered = False
                    last_print_sec = -1

                    wait_for_release(read_pressed)
                    prev_pressed = 0
                    time.sleep_ms(CHECK_MS)
                    continue

    # ---- PERSONAL logic: hold for 5 seconds
    if time.ticks_diff(now, personal_cooldown_until) >= 0:
        if pressed:
            if press_start is None:
                press_start = now
                personal_triggered = False
                last_print_sec = -1
                print("Button pressed - hold for {} seconds".format(HOLD_SECONDS))
            else:
                held = time.ticks_diff(now, press_start) / 1000.0
                sec = int(held)

                if sec != last_print_sec and sec < HOLD_SECONDS:
                    last_print_sec = sec
                    remaining = HOLD_SECONDS - sec
                    print("Hold for {} more seconds.".format(remaining))

                if held >= HOLD_SECONDS and not personal_triggered:
                    personal_triggered = True
                    print(" EMERGENCY ALERT TRIGGERED! (PERSONAL) ")

                    send_emergency(
                        "personal",
                        "MANUAL EMERGENCY BUTTON (hold 5s)",
                        wlan,
                        extra={"hold_seconds": HOLD_SECONDS, "trigger_source": "manual_hold"}
                    )

                    personal_cooldown_until = time.ticks_add(now, PERSONAL_COOLDOWN_MS)
                    global_cooldown_until = time.ticks_add(now, 800)

                    wait_for_release(read_pressed)
                    press_start = None
                    personal_triggered = False
                    last_print_sec = -1
                    prev_pressed = 0
                    time.sleep_ms(CHECK_MS)
                    continue

        else:
            if press_start is not None and not personal_triggered:
                print("Button released")
            press_start = None
            personal_triggered = False
            last_print_sec = -1

    prev_pressed = pressed
    time.sleep_ms(CHECK_MS)



