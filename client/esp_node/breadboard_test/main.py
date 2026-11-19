"""Micromax ERS vibration bench build for the XIAO ESP32-S3 + Wio-SX1262.

Default bench procedure
    1. Open the USB serial console and reset with S2 released.
    2. Five seconds after the ready message, a synthetic fall starts the
       normal ten-second cancellation window. The motor and pending LED turn on.
    3. Press S2 once during the window. Both outputs turn off and the console
       reports cancellation. Reset and repeat without pressing S2. Both
       outputs turn off before the alert dispatch path is reached.
    4. For a gateway test, set BENCH_SEND_TEST_ALERT = True and reset.
       Timeout then sends a real fall event (code 3) to the test gateway.

Bench mode runs one synthetic fall per reset, bypasses the MPU6050 and
pauses manual gestures, battery telemetry and heartbeats. Set
VIBRATION_BENCH_TEST = False to restore normal ERS operation with continuous
fall-pending vibration. Set FALL_VIBRATION_ENABLED = False to disable it.

Run with the XIAO ESP32-S3 MicroPython build. Copy the existing sx1262.py,
sx126x.py and _sx126x.py beside this file on the device. The radio pin map
is for the kit's board-to-board connection.

Breadboard pin map (XIAO pad -> ESP32-S3 GPIO)
    D0 -> GPIO1  S2 manual alert and fall cancel button
    D1 -> GPIO2  Q1 vibration motor switch, active high
    D2 -> GPIO3  R2/R3 battery divider midpoint, ADC1
    D3 -> GPIO4  Low-battery WARN LED, active low
    D4 -> GPIO5  MPU6050 SDA
    D5 -> GPIO6  MPU6050 SCL
          GPIO48 Wio board fall-pending LED, active high

The schematic labels U6 with the ESP32-C3 SKU. Use the actual
ESP32-S3 kit for this firmware. In normal mode, the MPU6050 must appear at 0x68 on
startup. Its SDA and SCL lines need pull-ups to 3.3 V.

Set the fitted hardware options under USER CONFIG. The breadboard uses
an external resistor from D0 to 3.3 V and S2 from D0 to ground. D0 reads
high when released and low when pressed. WARN uses active-low wiring.

With the button configured, five rapid presses send a mass emergency
and a five-second hold sends a personal emergency. A detected fall opens
a ten-second window before the personal fall alert is sent. The button
can cancel the alert only when its input is configured and functional.
"""

import time, math, _thread
from machine import ADC, I2C, Pin
import network
import socket
import ujson


# ================================================================
#  USER CONFIG
# ================================================================

# Enter the test network details locally. An empty SSID skips Wi-Fi.
WIFI_SSID = "2KF2197U00607-BFW"
WIFI_PASS = "MmxLetsUIn2014"

PI5_IP   = "192.168.115.50"
PI5_PORT = 8081

# Retain the original dashboard identity and its LoRa mapping.
DEVICE_ID = "PICO_TEST_03"

# Enter the fitted divider resistances in ohms. R2 runs from V_BATT to
# D2/GPIO3, and R3 runs from D2/GPIO3 to GND. Check the midpoint at 4.2 V
# before connecting it to the ADC. Leave either value unset for bring-up.
BATTERY_R2_OHMS = None
BATTERY_R3_OHMS = None

# Provisional voltage-based estimate. Verify the 3.4 V endpoint on the
# ESP32-S3 power circuit and compare reported voltage with a multimeter.
BATTERY_MIN_V = 3.4
BATTERY_MAX_V = 4.2
LOW_BATTERY_THRESHOLD = 20          # percent
BATTERY_CHECK_INTERVAL_MS = 5_000
BATTERY_SEND_INTERVAL_MS = 900_000
HEARTBEAT_INTERVAL_MS = 30_000

# S2 closes to ground. Choose the input bias after checking the wiring:
# None       - disable the button during bring-up
# "external" - resistor from D0 to 3.3 V, S2 from D0 to ground
# "internal" - remove the added resistor, S2 from D0 to ground
# Both enabled settings read 1 released and 0 pressed.
BUTTON_PULL_UP = "external"

# WARN LED on D3 starts high (off) and goes low on a low-battery
# reading. Battery monitoring requires the fitted R2 and R3 values.
WARN_LED_ACTIVE_LEVEL = 0

# Verify the motor and Q1 circuit can run continuously for ten seconds.
# D1 drives Q1 high throughout the pending window when enabled.
FALL_VIBRATION_ENABLED = True

# One synthetic fall per reset, five seconds after the ready message.
# Enable live sending only when using the ERS test gateway/dashboard.
VIBRATION_BENCH_TEST = True
BENCH_START_DELAY_MS = 5_000
BENCH_SEND_TEST_ALERT = False


# ================================================================
#  BUTTON CONFIG
# ================================================================
BUTTON_GPIO = 1  # XIAO D0
button = None
if BUTTON_PULL_UP == "external":
    button = Pin(BUTTON_GPIO, Pin.IN, pull=None)
elif BUTTON_PULL_UP == "internal":
    button = Pin(BUTTON_GPIO, Pin.IN, pull=Pin.PULL_UP)


def read_pressed() -> int:
    """Return 1 for a valid active-low press, or 0 if disabled or released."""
    return int(button is not None and button.value() == 0)

def wait_for_release(read_pressed_fn, max_ms=8000):
    t0 = time.ticks_ms()
    while read_pressed_fn():
        if time.ticks_diff(time.ticks_ms(), t0) > max_ms:
            break
        time.sleep_ms(20)


# ================================================================
#  EMERGENCY TIMING CONFIG
# ================================================================
WINDOW_MS          = 2200
DEBOUNCE_MS        = 80
MASS_COOLDOWN_MS   = 2500
REQUIRED_PRESSES   = 5

HOLD_SECONDS          = 5
CHECK_MS              = 50
PERSONAL_COOLDOWN_MS  = 1200

FALL_THRESHOLD_LOW  = 0.4
FALL_THRESHOLD_HIGH = 3.5
FALL_MIN_DURATION   = 100
FALL_MAX_DURATION   = 1500
FALL_COOLDOWN_MS    = 10000

FALL_CANCEL_WINDOW_MS = 10_000

WIFI_CONNECT_TIMEOUT_MS = 20_000
SOCKET_TIMEOUT_S        = 2


# ================================================================
#  ESP32-S3 OUTPUTS
# ================================================================
FALL_LED_GPIO = 48        # Active-high green LED on the Wio-SX1262 board
WARN_LED_GPIO = 4         # XIAO D3
MOTOR_GPIO = 2            # XIAO D1, Q1 low-side switch
LORA_RX_ENABLE_GPIO = 38  # Wio RF_SW. Low for the transmit-only ERS path.

led = Pin(FALL_LED_GPIO, Pin.OUT, value=0)
battery_led = Pin(WARN_LED_GPIO, Pin.IN)
if WARN_LED_ACTIVE_LEVEL in (0, 1):
    battery_led.init(Pin.OUT, value=1 - WARN_LED_ACTIVE_LEVEL)

motor = Pin(MOTOR_GPIO, Pin.OUT, value=0)
lora_rx_enable = Pin(LORA_RX_ENABLE_GPIO, Pin.OUT, value=0)


# ================================================================
#  TRANSPORT — LoRa PRIMARY, Wi-Fi FALLBACK
# ================================================================

# ---- Event codes ----
EVENT_CODE_MASS          = 1
EVENT_CODE_PERSONAL_HOLD = 2
EVENT_CODE_FALL          = 3

# ---- LoRa transport state ----
sx          = None
lora_ready  = False
_wlan       = None   # internal Wi-Fi handle

# ---- Wi-Fi helpers ----
def wifi_connect_if_needed(timeout_ms=WIFI_CONNECT_TIMEOUT_MS):
    global _wlan
    if not WIFI_SSID:
        return False
    if _wlan is None:
        _wlan = network.WLAN(network.STA_IF)
        _wlan.active(True)
    if _wlan.isconnected():
        return True
    print("Connecting Wi-Fi...")
    try:
        _wlan.disconnect()
    except:
        pass
    time.sleep_ms(300)
    _wlan.connect(WIFI_SSID, WIFI_PASS)
    t0 = time.ticks_ms()
    while not _wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            print("Wi-Fi connect timeout")
            return False
        time.sleep_ms(150)
    print("Wi-Fi connected:", _wlan.ifconfig())
    return True

def send_via_wifi(payload: dict) -> bool:
    s = None
    try:
        if not wifi_connect_if_needed():
            print("[Wi-Fi] Unavailable or unconfigured")
            return False
        s = socket.socket()
        s.settimeout(SOCKET_TIMEOUT_S)
        s.connect((PI5_IP, PI5_PORT))
        s.send(ujson.dumps(payload).encode("utf-8"))
        s.close()
        print("[Wi-Fi] Sent:", payload)
        return True
    except Exception as e:
        print("[Wi-Fi] Send failed:", e)
        try:
            if s:
                s.close()
        except:
            pass
        return False

# ---- LoRa helpers ----
def get_device_code(device_id: str) -> int:
    if device_id == "PICO_TEST_02":
        return 1
    return 255

def encode_for_lorawan(payload: dict) -> bytes:
    device_code = get_device_code(payload.get("device_id", ""))
    event_code  = payload.get("event_code", 0)
    seq         = payload.get("seq", 0) & 0xFFFF
    return bytes([device_code, event_code, (seq >> 8) & 0xFF, seq & 0xFF])

def lora_init() -> bool:
    global lora_ready, sx
    try:
        from sx1262 import SX1262
        # D8/D9/D10 carry SPI. CS, DIO1, RESET and BUSY use the kit's
        # underside connector. The driver's "gpio" parameter is BUSY.
        lora_rx_enable.off()
        sx = SX1262(spi_bus=1, clk=7, mosi=9, miso=8,
                    cs=41, irq=39, rst=42, gpio=40)
        status = sx.begin(freq=917.2, bw=125.0, sf=7, cr=5, syncWord=0x34,
                          power=14, currentLimit=60.0, preambleLength=8,
                          implicit=False, crcOn=True, txIq=False, rxIq=False,
                          tcxoVoltage=1.8, useRegulatorLDO=False, blocking=True)
        if status != 0:
            raise OSError("SX1262 initialisation status {}".format(status))
        print("[LoRa] Init OK")
        lora_ready = True
        return True
    except Exception as e:
        print("[LoRa] Init failed:", e)
        lora_ready = False
        sx = None
        return False

def lora_send_bytes(data: bytes) -> bool:
    global lora_ready, sx
    for attempt in range(2):
        try:
            if not lora_ready or sx is None:
                print("[LoRa] Init attempt", attempt + 1)
                if not lora_init():
                    continue
            print("[LoRa] TX attempt", attempt + 1, "bytes:", list(data))
            sent, status = sx.send(data)
            if status != 0 or sent != len(data):
                raise OSError("SX1262 TX status {}, bytes {}/{}".format(
                    status, sent, len(data)
                ))
            print("[LoRa] TX complete locally")
            return True
        except Exception as e:
            print("[LoRa] TX exception:", e)
            lora_ready = False
            sx = None
    print("[LoRa] TX FAIL")
    return False

def send_via_lorawan(payload: dict) -> bool:
    try:
        return lora_send_bytes(encode_for_lorawan(payload))
    except Exception as e:
        print("[LoRa] send_via_lorawan error:", e)
        return False

def send_with_fallback(payload: dict):
    if send_via_lorawan(payload):
        return True, "lorawan"
    if send_via_wifi(payload):
        return True, "wifi"
    return False, "none"

# ---- Emergency payload builder ----
def send_emergency(event_code: int, em_type: str, msg: str, extra: dict = None) -> bool:
    now_ms = time.ticks_ms()
    payload = {
        "device_id":         DEVICE_ID,
        "event_code":        event_code,
        "emergency":         True,
        "emergency_type":    em_type,
        "emergency_message": msg,
        "timestamp_ms":      now_ms,
        "seq":               now_ms,
    }
    if extra:
        payload.update(extra)
    ok, channel = send_with_fallback(payload)
    print("Sent result:", ok, "| channel:", channel, "| payload:", payload)
    return ok


# ================================================================
#  SHARED SENSOR STATE
# ================================================================
latest_accel = {"x": 0.0, "y": 0.0, "z": 0.0}
latest_gyro  = {"x": 0.0, "y": 0.0, "z": 0.0}
latest_heart_rate = 0

fall_detected_flag = False
fall_detected_at_ms = 0

data_lock = _thread.allocate_lock()


# ================================================================
#  MPU6050
# ================================================================
MPU_ADDR     = 0x68
PWR_MGMT_1   = 0x6B
CONFIG       = 0x1A
SMPLRT_DIV   = 0x19
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

ACCEL_RANGE_REGVAL = 0x10   # ±8g
GYRO_RANGE_REGVAL  = 0x08   # ±500 dps
DLPF_CFG_REGVAL    = 0x03
SMPLRT_DIV_REGVAL  = 0x04

LSB_PER_G   = {0x00: 16384.0, 0x08: 8192.0, 0x10: 4096.0, 0x18: 2048.0}
LSB_PER_DPS = {0x00: 131.0,   0x08: 65.5,   0x10: 32.8,   0x18: 16.4}

def to_i16(msb, lsb):
    v = (msb << 8) | lsb
    return v - 65536 if v & 0x8000 else v

def write_reg(i2c, reg, val, tries=10):
    for _ in range(tries):
        try:
            i2c.writeto_mem(MPU_ADDR, reg, bytes([val]))
            return
        except OSError:
            time.sleep_ms(50)
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
        self.i2c  = i2c
        self.addr = addr
        write_reg(self.i2c, PWR_MGMT_1, 0x00)
        time.sleep_ms(100)
        write_reg(self.i2c, CONFIG,       DLPF_CFG_REGVAL)
        write_reg(self.i2c, SMPLRT_DIV,   SMPLRT_DIV_REGVAL)
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
        d  = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 14)
        ax = to_i16(d[0], d[1]) / self.lsb_per_g
        ay = to_i16(d[2], d[3]) / self.lsb_per_g
        az = to_i16(d[4], d[5]) / self.lsb_per_g
        gx = to_i16(d[8],  d[9])  / self.lsb_per_dps
        gy = to_i16(d[10], d[11]) / self.lsb_per_dps
        gz = to_i16(d[12], d[13]) / self.lsb_per_dps
        return ax, ay, az, gx, gy, gz


# ================================================================
#  FALL DETECTION THREAD
# ================================================================
def fall_detection_thread(mpu):
    global fall_detected_flag, fall_detected_at_ms, latest_accel, latest_gyro

    print("[Fall] thread started")
    fall_state      = "normal"
    freefall_start  = 0
    cooldown_start  = 0
    error_count     = 0

    while True:
        try:
            ax, ay, az, gx, gy, gz = mpu.read_accel_gyro()
            total_accel = math.sqrt(ax*ax + ay*ay + az*az)

            data_lock.acquire()
            try:
                latest_accel = {"x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)}
                latest_gyro  = {"x": round(gx, 2), "y": round(gy, 2), "z": round(gz, 2)}
            finally:
                data_lock.release()

            error_count = 0

            if fall_state == "normal":
                if total_accel < FALL_THRESHOLD_LOW:
                    fall_state     = "freefall"
                    freefall_start = time.ticks_ms()

            elif fall_state == "freefall":
                dt = time.ticks_diff(time.ticks_ms(), freefall_start)
                if total_accel > FALL_THRESHOLD_HIGH:
                    if FALL_MIN_DURATION < dt < FALL_MAX_DURATION:
                        impact_time = time.ticks_ms()
                        print("[FALL] IMPACT |a|=", round(total_accel, 2), "g after", dt, "ms")
                        data_lock.acquire()
                        try:
                            fall_detected_flag  = True
                            fall_detected_at_ms = impact_time
                        finally:
                            data_lock.release()
                        fall_state     = "cooldown"
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


# ================================================================
#  SENSOR INIT
# ================================================================
MPU_I2C_FREQ = 50_000
if not VIBRATION_BENCH_TEST:
    time.sleep(2)  # Allow the power rail and sensor to settle.

    # MPU6050: XIAO D4 (GPIO5) is SDA and D5 (GPIO6) is SCL.
    # Both lines require pull-ups to 3.3 V. AD0 low selects address 0x68.
    i2c0 = I2C(0, sda=Pin(5), scl=Pin(6), freq=MPU_I2C_FREQ)
    time.sleep_ms(500)

    scan0 = i2c0.scan()
    print("I2C0 scan:", [hex(x) for x in scan0])
    if MPU_ADDR not in scan0:
        raise RuntimeError("MPU6050 not found at 0x68 (check wiring/power)")

    mpu = MPU6050(i2c0)
    print("MPU6050 initialized")

BATTERY_ADC_GPIO = 3  # XIAO D2, ADC1 and a reset strapping pin
battery_adc = None
battery_divider_gain = None
if VIBRATION_BENCH_TEST:
    print("[Bench] Battery monitoring paused.")
elif BATTERY_R2_OHMS is None or BATTERY_R3_OHMS is None:
    print("[Battery] Monitoring disabled. Set the fitted R2/R3 values.")
elif BATTERY_R2_OHMS <= 0 or BATTERY_R3_OHMS <= 0:
    print("[Battery] Monitoring disabled. R2 and R3 must be positive.")
else:
    battery_divider_gain = (BATTERY_R2_OHMS + BATTERY_R3_OHMS) / BATTERY_R3_OHMS
    # Leave headroom below the S3 ADC's nominal upper measurement limit.
    # This check only controls software. Verify the divider before wiring.
    if BATTERY_MAX_V / battery_divider_gain > 2.5:
        print("[Battery] Monitoring disabled. Check the divider ratio.")
    else:
        try:
            battery_adc = ADC(Pin(BATTERY_ADC_GPIO), atten=ADC.ATTN_11DB)
        except (OSError, ValueError) as e:
            print("[Battery] ADC setup failed:", e)

if not VIBRATION_BENCH_TEST:
    _thread.start_new_thread(fall_detection_thread, (mpu,))
    print("Fall detection thread started")


# ================================================================
#  BATTERY HELPER
# ================================================================
def check_battery():
    """Read V_BATT through R2/R3 and update the optional low-battery LED."""
    if battery_adc is None:
        return None, None
    try:
        adc_voltage = battery_adc.read_uv() / 1_000_000
        # A missing battery on USB power can leave the divider near zero.
        if adc_voltage < 0.15:
            return None, None
        battery_voltage = adc_voltage * battery_divider_gain
        pct = (battery_voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0
        pct = max(0.0, min(100.0, pct))

        if WARN_LED_ACTIVE_LEVEL in (0, 1):
            low_battery = pct < LOW_BATTERY_THRESHOLD
            battery_led.value(WARN_LED_ACTIVE_LEVEL if low_battery else 1 - WARN_LED_ACTIVE_LEVEL)

        return pct, battery_voltage
    except Exception as e:
        print("Battery read error:", e)
        return None, None

def send_battery_to_pi5(pct, voltage):
    """Send latest battery reading to the Pi 5 dashboard (Wi-Fi only)."""
    if pct is None:
        return
    payload = {
        "device_id":   DEVICE_ID,
        "emergency":   False,
        "battery_pct": pct,
        "battery_v":   voltage,
        "timestamp_ms": time.ticks_ms(),
    }
    send_via_wifi(payload)


# ================================================================
#  FALL-PENDING HELPERS
# ================================================================
fall_pending          = False
fall_pending_start_ms = 0
fall_pending_extra    = None

def start_fall_pending(extra: dict, start_ms: int):
    global fall_pending, fall_pending_start_ms, fall_pending_extra
    fall_pending          = True
    fall_pending_start_ms = start_ms
    fall_pending_extra    = extra
    led.on()
    # The fall state owns the motor. No timed pulse ends it mid-window.
    if FALL_VIBRATION_ENABLED:
        motor.on()
        print("[Motor] ON | ms:", time.ticks_ms())
    if button is None:
        print("[FALL] Pending for 10s. Button cancellation unavailable.")
    else:
        print("[FALL] Pending for 10s (press button once to cancel).")

def cancel_fall_pending():
    global fall_pending, fall_pending_start_ms, fall_pending_extra
    fall_pending          = False
    fall_pending_start_ms = 0
    fall_pending_extra    = None
    motor.off()
    led.off()
    print("[Motor] OFF | ms:", time.ticks_ms())
    print("[FALL] Cancelled by button press.")

def confirm_send_fall():
    global fall_pending, fall_pending_start_ms, fall_pending_extra
    extra              = fall_pending_extra
    fall_pending       = False
    fall_pending_start_ms = 0
    fall_pending_extra = None
    motor.off()
    led.off()
    print("[Motor] OFF | ms:", time.ticks_ms())
    print("!!! FALL CONFIRMED (no cancel) -> PERSONAL EMERGENCY !!!")
    if VIBRATION_BENCH_TEST and not BENCH_SEND_TEST_ALERT:
        print("[Bench] Would send fall event 3. Test transmission is disabled.")
        return
    message = "BENCH TEST FALL" if VIBRATION_BENCH_TEST else "FALL DETECTED"
    send_emergency(EVENT_CODE_FALL, "personal", message, extra=extra)


# ================================================================
#  BUTTON STATE
# ================================================================
prev_pressed  = 0
last_edge     = 0

press_count       = 0
window_start      = 0
mass_cooldown_until = 0

press_start            = None
personal_triggered     = False
last_print_sec         = -1
personal_cooldown_until = 0

global_cooldown_until    = 0
fall_send_cooldown_until = 0

last_battery_check_ms = 0   # tracks when we last read the ADC and updated the LED
last_battery_send_ms  = 0   # tracks when we last sent battery data to Pi 5
last_heartbeat_ms     = 0   # tracks when we last sent a heartbeat
_last_battery_pct     = None
_last_battery_v       = None


# ================================================================
#  STARTUP
# ================================================================
print("READY:")
if VIBRATION_BENCH_TEST:
    print("[Bench] Start with S2 released.")
    print("[Bench] Real fall transmission:", BENCH_SEND_TEST_ALERT)
elif button is None:
    print("[Button] Disabled. Check the D0 wiring and set BUTTON_PULL_UP.")
    print("[Fall] Automatic alerts are active. Button cancellation is unavailable.")
else:
    print(f" - Rapid press GPIO{BUTTON_GPIO} 5 times -> MASS EMERGENCY")
    print(f" - Hold GPIO{BUTTON_GPIO} {HOLD_SECONDS} seconds -> PERSONAL EMERGENCY")
if not VIBRATION_BENCH_TEST:
    print(" - Fall detection -> 10s pending window -> then PERSONAL EMERGENCY")
print(f"Target Pi5: {PI5_IP}:{PI5_PORT}")
if WARN_LED_ACTIVE_LEVEL in (0, 1):
    print(f"Low-battery LED (GPIO{WARN_LED_GPIO}) threshold: {LOW_BATTERY_THRESHOLD}%")
else:
    print("[Battery] WARN output disabled. Confirm LED polarity.")

if not VIBRATION_BENCH_TEST or BENCH_SEND_TEST_ALERT:
    lora_init()

bench_fall_due = None
if VIBRATION_BENCH_TEST:
    print("[Bench] Synthetic fall in", BENCH_START_DELAY_MS / 1000, "seconds.")
    bench_fall_due = time.ticks_add(time.ticks_ms(), BENCH_START_DELAY_MS)
else:
    # Update the WARN LED and send the initial reading to the dashboard.
    _last_battery_pct, _last_battery_v = check_battery()
    last_battery_check_ms = time.ticks_ms()
    send_battery_to_pi5(_last_battery_pct, _last_battery_v)
    last_battery_send_ms = time.ticks_ms()


# ================================================================
#  MAIN LOOP
# ================================================================
try:
    while True:
        now = time.ticks_ms()

        # ---- Global cooldown
        if time.ticks_diff(now, global_cooldown_until) < 0:
            time.sleep_ms(CHECK_MS)
            continue

        # ---- Read button
        pressed     = read_pressed()
        rising_edge = (prev_pressed == 0 and pressed == 1)

        # ---- Fall-pending cancel window
        if fall_pending:
            if rising_edge:
                cancel_fall_pending()
                wait_for_release(read_pressed)
                prev_pressed = 0
                time.sleep_ms(CHECK_MS)
                continue

            if time.ticks_diff(now, fall_pending_start_ms) >= FALL_CANCEL_WINDOW_MS:
                confirm_send_fall()
                fall_send_cooldown_until = time.ticks_add(now, FALL_COOLDOWN_MS)
                global_cooldown_until    = time.ticks_add(now, 800)
                time.sleep_ms(CHECK_MS)
                prev_pressed = pressed
                continue

            prev_pressed = pressed
            time.sleep_ms(CHECK_MS)
            continue

        # Exercise the same pending, cancellation and confirmation handlers
        # without requiring a physical fall or a connected MPU6050.
        if VIBRATION_BENCH_TEST:
            if bench_fall_due is not None and time.ticks_diff(now, bench_fall_due) >= 0:
                bench_fall_due = None
                start_fall_pending({"trigger_source": "vibration_bench_test"}, now)
                if rising_edge:
                    cancel_fall_pending()
                    wait_for_release(read_pressed)
                    pressed = 0
            prev_pressed = pressed
            time.sleep_ms(CHECK_MS)
            continue

        # ---- Check fall event from thread
        fall_alert   = False
        accel        = None
        gyro         = None
        fall_time_ms = 0

        data_lock.acquire()
        try:
            if fall_detected_flag:
                fall_alert          = True
                fall_detected_flag  = False
                fall_time_ms        = fall_detected_at_ms
            accel = latest_accel.copy()
            gyro  = latest_gyro.copy()
        finally:
            data_lock.release()

        if fall_alert and (not fall_pending) and time.ticks_diff(now, fall_send_cooldown_until) >= 0:
            extra = {
                "trigger_source":   "fall_detection",
                "fall_timestamp_ms": fall_time_ms,
                "accelerometer":    accel,
                "gyroscope":        gyro,
                "mpu_config": {
                    "accel_range_reg": hex(ACCEL_RANGE_REGVAL),
                    "gyro_range_reg":  hex(GYRO_RANGE_REGVAL),
                    "dlpf_config":     hex(DLPF_CFG_REGVAL),
                    "smplrt_div":      hex(SMPLRT_DIV_REGVAL),
                    "i2c_freq":        MPU_I2C_FREQ,
                }
            }
            # Discard a partly completed manual gesture when the button
            # switches to its cancellation role.
            press_count = 0
            window_start = 0
            press_start = None
            personal_triggered = False
            last_print_sec = -1
            start_fall_pending(extra, now)
            # A new press can cancel immediately, even on the first poll.
            if rising_edge:
                cancel_fall_pending()
                wait_for_release(read_pressed)
                pressed = 0
            prev_pressed = pressed
            time.sleep_ms(CHECK_MS)
            continue

        # ---- MASS logic: 5 rapid presses
        if time.ticks_diff(now, mass_cooldown_until) >= 0:
            if rising_edge:
                if time.ticks_diff(now, last_edge) > DEBOUNCE_MS:
                    last_edge = now
                    if press_count == 0:
                        window_start = now
                        press_count  = 1
                    else:
                        if time.ticks_diff(now, window_start) > WINDOW_MS:
                            window_start = now
                            press_count  = 1
                        else:
                            press_count += 1

                    print("[PRESS] count =", press_count)

                    if press_count >= REQUIRED_PRESSES and time.ticks_diff(now, window_start) <= WINDOW_MS:
                        print("!!! MASS EMERGENCY TRIGGERED !!!")
                        send_emergency(
                            EVENT_CODE_MASS,
                            "mass",
                            "MASS EMERGENCY TRIGGERED (5 rapid presses)",
                            extra={"press_count": press_count, "window_ms": WINDOW_MS,
                                   "trigger_source": "rapid_presses"}
                        )
                        press_count   = 0
                        window_start  = 0
                        mass_cooldown_until   = time.ticks_add(now, MASS_COOLDOWN_MS)
                        global_cooldown_until = time.ticks_add(now, 1500)
                        press_start        = None
                        personal_triggered = False
                        last_print_sec     = -1
                        wait_for_release(read_pressed)
                        prev_pressed = 0
                        time.sleep_ms(CHECK_MS)
                        continue

        # ---- PERSONAL logic: hold for 5 seconds
        if time.ticks_diff(now, personal_cooldown_until) >= 0:
            if pressed:
                if press_start is None:
                    press_start        = now
                    personal_triggered = False
                    last_print_sec     = -1
                    print("Button pressed - hold for {} seconds".format(HOLD_SECONDS))
                else:
                    held = time.ticks_diff(now, press_start) / 1000.0
                    sec  = int(held)

                    if sec != last_print_sec and sec < HOLD_SECONDS:
                        last_print_sec = sec
                        print("Hold for {} more seconds.".format(HOLD_SECONDS - sec))

                    if held >= HOLD_SECONDS and not personal_triggered:
                        personal_triggered = True
                        print(" EMERGENCY ALERT TRIGGERED! (PERSONAL) ")
                        send_emergency(
                            EVENT_CODE_PERSONAL_HOLD,
                            "personal",
                            "MANUAL EMERGENCY BUTTON (hold 5s)",
                            extra={"hold_seconds": HOLD_SECONDS, "trigger_source": "manual_hold"}
                        )
                        personal_cooldown_until = time.ticks_add(now, PERSONAL_COOLDOWN_MS)
                        global_cooldown_until   = time.ticks_add(now, 800)
                        wait_for_release(read_pressed)
                        press_start        = None
                        personal_triggered = False
                        last_print_sec     = -1
                        prev_pressed       = 0
                        time.sleep_ms(CHECK_MS)
                        continue
            else:
                if press_start is not None and not personal_triggered:
                    print("Button released")
                press_start        = None
                personal_triggered = False
                last_print_sec     = -1

        # ---- Heartbeat (every 30 s)
        if time.ticks_diff(now, last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS:
            last_heartbeat_ms = now
            hb_payload = {
                "device_id":      DEVICE_ID,
                "event_code":     99,
                "emergency":      False,
                "emergency_type": "heartbeat",
                "message":        "HEARTBEAT",
                "timestamp_ms":   now,
                "seq":            now,
                "status":         "online",
            }
            hb_ok, hb_ch = send_with_fallback(hb_payload)
            print("[HEARTBEAT] sent via", hb_ch, "ok:", hb_ok)

        # ---- Battery LED check (every 5 s)
        if time.ticks_diff(now, last_battery_check_ms) >= BATTERY_CHECK_INTERVAL_MS:
            last_battery_check_ms = now
            _last_battery_pct, _last_battery_v = check_battery()

        # ---- Battery Pi 5 send (every 15 min)
        if time.ticks_diff(now, last_battery_send_ms) >= BATTERY_SEND_INTERVAL_MS:
            last_battery_send_ms = now
            send_battery_to_pi5(_last_battery_pct, _last_battery_v)

        prev_pressed = pressed
        time.sleep_ms(CHECK_MS)
finally:
    # A serial Ctrl+C or unexpected loop exit clears the motor output.
    motor.off()
    led.off()
