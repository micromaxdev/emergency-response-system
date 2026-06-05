# Pico W - Combined Emergency Script + INA219 Battery Monitor
# Mass: 5 rapid presses -> MASS emergency
# Personal: hold 5s -> PERSONAL emergency
# Fall detection (MPU6050) -> PERSONAL emergency (WITH 10s CANCEL WINDOW)
#   During the 10s: if the button is pressed once, cancel the fall (do not send).
# Battery: INA219 monitors battery %; GPIO14 LED turns ON if below LOW_BATTERY_THRESHOLD


import time, math, _thread
from machine import Pin, I2C
import network
import socket
import ujson

# ================================================================
#  INA219 DRIVER
# ================================================================

# Config Register (R/W)
_REG_CONFIG        = 0x00
_REG_SHUNTVOLTAGE  = 0x01
_REG_BUSVOLTAGE    = 0x02
_REG_POWER         = 0x03
_REG_CURRENT       = 0x04
_REG_CALIBRATION   = 0x05

class BusVoltageRange:
    RANGE_16V = 0x00
    RANGE_32V = 0x01

class Gain:
    DIV_1_40MV  = 0x00
    DIV_2_80MV  = 0x01
    DIV_4_160MV = 0x02
    DIV_8_320MV = 0x03

class ADCResolution:
    ADCRES_9BIT_1S    = 0x00
    ADCRES_10BIT_1S   = 0x01
    ADCRES_11BIT_1S   = 0x02
    ADCRES_12BIT_1S   = 0x03
    ADCRES_12BIT_2S   = 0x09
    ADCRES_12BIT_4S   = 0x0A
    ADCRES_12BIT_8S   = 0x0B
    ADCRES_12BIT_16S  = 0x0C
    ADCRES_12BIT_32S  = 0x0D
    ADCRES_12BIT_64S  = 0x0E
    ADCRES_12BIT_128S = 0x0F

class Mode:
    POWERDOW             = 0x00
    SVOLT_TRIGGERED      = 0x01
    BVOLT_TRIGGERED      = 0x02
    SANDBVOLT_TRIGGERED  = 0x03
    ADCOFF               = 0x04
    SVOLT_CONTINUOUS     = 0x05
    BVOLT_CONTINUOUS     = 0x06
    SANDBVOLT_CONTINUOUS = 0x07

class INA219:
    def __init__(self, i2c, addr=0x40):
        """
        i2c  - an already-constructed machine.I2C object
        addr - I2C address of the INA219 chip (default 0x40; use 0x43 if A0+A1 tied high)
        """
        self.i2c  = i2c
        self.addr = addr
        self._cal_value  = 0
        self._current_lsb = 0
        self._power_lsb   = 0
        self.set_calibration_32V_2A()

    def read(self, address):
        data = self.i2c.readfrom_mem(self.addr, address, 2)
        return (data[0] * 256) + data[1]

    def write(self, address, data):
        temp = [0, 0]
        temp[1] = data & 0xFF
        temp[0] = (data & 0xFF00) >> 8
        self.i2c.writeto_mem(self.addr, address, bytes(temp))

    def set_calibration_32V_2A(self):
        """Configure INA219 for up to 32 V / 2 A (0.1-ohm shunt assumed)."""
        self._current_lsb = 1
        self._cal_value   = 4096
        self._power_lsb   = 0.002
        self.write(_REG_CALIBRATION, self._cal_value)

        self.bus_voltage_range    = BusVoltageRange.RANGE_32V
        self.gain                 = Gain.DIV_8_320MV
        self.bus_adc_resolution   = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode                 = Mode.SANDBVOLT_CONTINUOUS
        self.config = (
            self.bus_voltage_range    << 13 |
            self.gain                 << 11 |
            self.bus_adc_resolution   << 7  |
            self.shunt_adc_resolution << 3  |
            self.mode
        )
        self.write(_REG_CONFIG, self.config)

    def getShuntVoltage_mV(self):
        value = self.read(_REG_SHUNTVOLTAGE)
        if value > 32767:
            value -= 65535
        return value * 0.01

    def getBusVoltage_V(self):
        self.read(_REG_BUSVOLTAGE)
        return (self.read(_REG_BUSVOLTAGE) >> 3) * 0.004

    def getCurrent_mA(self):
        value = self.read(_REG_CURRENT)
        if value > 32767:
            value -= 65535
        return value * self._current_lsb


# ================================================================
#  USER CONFIG
# ================================================================

WIFI_SSID = "2KF2197U00607-BFW"
WIFI_PASS = "MmxLetsUIn2014"

PI5_IP   = "192.168.115.49"
PI5_PORT = 8081

DEVICE_ID = "PICO_TEST_01"

# ----------------------------------------------------------------
#  Battery calibration
# ----------------------------------------------------------------
# BATTERY_MIN_V: voltage at which the battery is considered empty (0%).
#   Set this to the voltage at which your battery actually shuts down.
#   Measured shutdown voltage was ~3.4 V, so use that here.
BATTERY_MIN_V = 3.4                 # V

# BATTERY_MAX_V: voltage at which the battery is considered full (100%).
#   Standard LiPo/Li-ion max charge voltage.
BATTERY_MAX_V = 4.2                 # V

# ----------------------------------------------------------------
#  Battery warning
# ----------------------------------------------------------------
# Change this value to set the threshold at which the low-battery
# LED (GPIO14) turns on.  Range: 0–100 (percent).
LOW_BATTERY_THRESHOLD = 20          # %

# How often the INA219 is read and the LED state is updated
BATTERY_CHECK_INTERVAL_MS = 5_000   # 5 seconds

# How often the battery reading is sent to the Pi 5 dashboard
BATTERY_SEND_INTERVAL_MS  = 900_000 # 15 minutes
HEARTBEAT_INTERVAL_MS     = 30_000  # 30 seconds

# INA219 I2C address — 0x40 default; 0x43 if A0+A1 shorted to VS+
INA219_ADDR = 0x43

# ----------------------------------------------------------------
#  BLE trigger (Seeed XIAO nRF52840)
# ----------------------------------------------------------------
# GPIO pin that pulses HIGH for 1 second to tell the XIAO to start
# BLE advertising. Connect Pico W GPIO16 → XIAO D2, shared GND.
BLE_TRIGGER_GPIO    = 16
BLE_TRIGGER_PULSE_MS = 1000   # how long to hold the pin HIGH (ms)


# ================================================================
#  BUTTON CONFIG
# ================================================================
BUTTON_GPIO = 15
button = Pin(BUTTON_GPIO, Pin.IN, Pin.PULL_UP)  # active-low

def read_pressed() -> int:
    return 1 if button.value() == 0 else 0

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

WIFI_CONNECT_TIMEOUT_MS = 15000
SOCKET_TIMEOUT_S        = 2


# ================================================================
#  HARDWARE — LEDs
# ================================================================
led = Pin("LED", Pin.OUT)   # built-in LED: fall-pending indicator
led.off()

battery_led = Pin(14, Pin.OUT)  # GPIO14: low-battery warning LED
battery_led.off()

ble_trigger = Pin(BLE_TRIGGER_GPIO, Pin.OUT)  # GPIO16: XIAO BLE trigger
ble_trigger.off()


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

    if _wlan is None:
        _wlan = network.WLAN(network.STA_IF)
        _wlan.active(True)

    if _wlan.isconnected():
        print("Already connected:", _wlan.ifconfig())
        return True

    print("Connecting Wi-Fi...")
    print("SSID:", WIFI_SSID)

    try:
        _wlan.disconnect()
    except:
        pass

    time.sleep_ms(500)
    _wlan.connect(WIFI_SSID, WIFI_PASS)

    t0 = time.ticks_ms()
    while not _wlan.isconnected():
        status = _wlan.status()
        print("Wi-Fi status:", status)

        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            print("Wi-Fi connect timeout")
            print("Final status:", _wlan.status())
            print("ifconfig:", _wlan.ifconfig())
            return False

        time.sleep_ms(1000)

    print("Wi-Fi connected:", _wlan.ifconfig())
    return True

def send_via_wifi(payload: dict) -> bool:
    s = None
    try:
        if not wifi_connect_if_needed():
            print("[Wi-Fi] Not available")
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
    if device_id == "PICO_TEST_01":
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
        sx = SX1262(spi_bus=1, clk=10, mosi=11, miso=12,
                    cs=3, irq=20, rst=15, gpio=2)
        sx.begin(freq=917.2, bw=125.0, sf=7, cr=5, syncWord=0x34,
                 power=14, currentLimit=60.0, preambleLength=8,
                 implicit=False, crcOn=True, txIq=False, rxIq=False,
                 tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
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
            sx.send(data)
            print("[LoRa] TX OK")
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
time.sleep(2)  # longer settle time — UPS board needs more time on power-up

I2C_FREQ     = 100_000
MPU_I2C_FREQ = 50_000   # slower = more reliable for MPU6050 over longer wires/UPS board

# I2C0 — MPU6050  (SDA=GP0, SCL=GP1)
i2c0 = I2C(0, sda=Pin(0), scl=Pin(1), freq=MPU_I2C_FREQ)
time.sleep_ms(500)

scan0 = i2c0.scan()
print("I2C0 scan:", [hex(x) for x in scan0])
if MPU_ADDR not in scan0:
    raise RuntimeError("MPU6050 not found at 0x68 (check wiring/power)")

mpu = MPU6050(i2c0)
print("MPU6050 initialized")

# I2C1 — INA219   (SDA=GP2, SCL=GP3)
i2c1 = I2C(1, sda=Pin(6), scl=Pin(7), freq=I2C_FREQ)
time.sleep_ms(200)

scan1 = i2c1.scan()
print("I2C1 scan:", [hex(x) for x in scan1])

ina219 = None
if INA219_ADDR in scan1:
    ina219 = INA219(i2c1, addr=INA219_ADDR)
    print("INA219 initialized at", hex(INA219_ADDR))
else:
    print("WARNING: INA219 not found — battery monitoring disabled")

_thread.start_new_thread(fall_detection_thread, (mpu,))
print("Fall detection thread started")


# ================================================================
#  BATTERY HELPER
# ================================================================
def check_battery():
    """Read INA219, update the GPIO14 warning LED, return (pct, voltage).
    Does NOT send to Pi 5 — that happens separately every 15 minutes."""
    if ina219 is None:
        return None, None
    try:
        bus_voltage = ina219.getBusVoltage_V()
        pct = (bus_voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0
        pct = max(0.0, min(100.0, pct))

        battery_led.on() if pct < LOW_BATTERY_THRESHOLD else battery_led.off()

        return pct, bus_voltage
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
#  BLE TRIGGER HELPER
# ================================================================
ble_trigger_off_at_ms = 0   # when to lower the trigger pin (0 = already low)

def trigger_ble_alert():
    """Pulse the BLE trigger pin HIGH so the XIAO starts advertising.
    Non-blocking — the main loop lowers the pin after BLE_TRIGGER_PULSE_MS."""
    global ble_trigger_off_at_ms
    ble_trigger.on()
    ble_trigger_off_at_ms = time.ticks_add(time.ticks_ms(), BLE_TRIGGER_PULSE_MS)
    print("[BLE] Trigger pin HIGH — XIAO will advertise for 10 s")


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
    print("[FALL] Pending for 10s (press button once to cancel).")

def cancel_fall_pending():
    global fall_pending, fall_pending_start_ms, fall_pending_extra
    fall_pending          = False
    fall_pending_start_ms = 0
    fall_pending_extra    = None
    led.off()
    print("[FALL] Cancelled by button press.")

def confirm_send_fall():
    global fall_pending, fall_pending_start_ms, fall_pending_extra
    extra              = fall_pending_extra
    fall_pending       = False
    fall_pending_start_ms = 0
    fall_pending_extra = None
    led.off()
    print("!!! FALL CONFIRMED (no cancel) -> PERSONAL EMERGENCY !!!")
    send_emergency(EVENT_CODE_FALL, "personal", "FALL DETECTED", extra=extra)
    trigger_ble_alert()


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

last_battery_check_ms = 0   # tracks when we last read INA219 + updated LED
last_battery_send_ms  = 0   # tracks when we last sent battery data to Pi 5
last_heartbeat_ms     = 0   # tracks when we last sent a heartbeat
_last_battery_pct     = None
_last_battery_v       = None


# ================================================================
#  STARTUP
# ================================================================
print("READY:")
print(f" - Rapid press GPIO{BUTTON_GPIO} 5 times -> MASS EMERGENCY")
print(f" - Hold GPIO{BUTTON_GPIO} {HOLD_SECONDS} seconds -> PERSONAL EMERGENCY")
print(" - Fall detection -> LED ON, 10s cancel window -> then PERSONAL EMERGENCY")
print(f"Target Pi5: {PI5_IP}:{PI5_PORT}")
print(f"Low-battery LED (GPIO14) threshold: {LOW_BATTERY_THRESHOLD}%")

lora_init()

# Run an immediate battery check so the LED state is correct from the start
_last_battery_pct, _last_battery_v = check_battery()
last_battery_check_ms = time.ticks_ms()
# Also send the first reading to Pi 5 straight away
send_battery_to_pi5(_last_battery_pct, _last_battery_v)
last_battery_send_ms = time.ticks_ms()


# ================================================================
#  MAIN LOOP
# ================================================================
while True:
    now = time.ticks_ms()

    # ---- Global cooldown
    if time.ticks_diff(now, global_cooldown_until) < 0:
        time.sleep_ms(CHECK_MS)
        continue

    # ---- BLE trigger pin — lower after pulse duration
    if ble_trigger_off_at_ms and time.ticks_diff(now, ble_trigger_off_at_ms) >= 0:
        ble_trigger.off()
        ble_trigger_off_at_ms = 0
        print("[BLE] Trigger pin LOW")

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
                "i2c_freq":        I2C_FREQ,
            }
        }
        start_fall_pending(extra, now)

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
                    trigger_ble_alert()
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

    prev_pressed = pressed
    time.sleep_ms(CHECK_MS)