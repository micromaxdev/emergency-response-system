"""
config.py
---------
Shared configuration for all Pico emergency-button units
(PICO_TEST_01, Thomas, Skylar). Committed to git — contains no
secrets, only deployment and hardware constants.

DEPLOYMENT-SPECIFIC: update PI5_IP / PI5_PORT when moving this
fleet to a new site. All units at one site share the same Pi 5,
so this lives here (shared) rather than in secrets.py (per-device).

Per-device identity (WIFI_SSID/PASS, DEVICE_ID, DEVICE_CODE) lives
in secrets.py instead — see secrets_template.py.
"""

# ================================================================
#  Pi 5 dashboard server (same for all units at one deployment site)
# ================================================================
PI5_IP   = "192.168.115.49"
PI5_PORT = 8081

# ================================================================
#  GPIO pin assignments
# ================================================================
PIN_BUTTON      = 13     # Rewired from GP15 -- GP15 is hardwired to LoRa_RESET
                          # on the SX1262 HAT, conflicting with the button.
                          # GP13 is confirmed unused on both the LoRa HAT and
                          # the UPS HAT. Physically rewire all 3 units before
                          # using this value.
PIN_BATTERY_LED = 14
PIN_BLE_TRIGGER = 16

PIN_I2C0_SDA = 0   # MPU6050 bus
PIN_I2C0_SCL = 1
PIN_I2C1_SDA = 6   # INA219 bus -- matches the UPS HAT's documented
PIN_I2C1_SCL = 7   # "Voltage/Current monitor" SDA/SCL pins

# ================================================================
#  I2C addresses
# ================================================================
MPU6050_ADDR = 0x68
INA219_ADDR  = 0x43   # 0x40 default; 0x43 if A0+A1 tied high

# ================================================================
#  I2C bus speeds
# ================================================================
I2C0_FREQ_HZ = 50_000    # MPU6050 — slower = more reliable over UPS board wiring
I2C1_FREQ_HZ = 100_000   # INA219

# ================================================================
#  Button timing — MASS emergency (5 rapid presses)
# ================================================================
MASS_REQUIRED_PRESSES = 5
MASS_WINDOW_MS        = 2200   # all required presses must land inside this window
MASS_DEBOUNCE_MS      = 80
MASS_COOLDOWN_MS      = 2500

# ================================================================
#  Button timing — PERSONAL emergency (hold)
# ================================================================
PERSONAL_HOLD_SECONDS = 5
PERSONAL_COOLDOWN_MS  = 1200
BUTTON_POLL_MS         = 50

# ================================================================
#  Fall detection (MPU6050 free-fall + impact pattern)
# ================================================================
FALL_THRESHOLD_LOW_G  = 0.4
FALL_THRESHOLD_HIGH_G = 3.5
FALL_MIN_DURATION_MS  = 100
FALL_MAX_DURATION_MS  = 1500
FALL_COOLDOWN_MS      = 10_000
FALL_CANCEL_WINDOW_MS = 10_000   # time window to cancel via button after a fall is detected

MPU_ACCEL_RANGE_REG = 0x10   # +/-8g
MPU_GYRO_RANGE_REG  = 0x08   # +/-500 dps
MPU_DLPF_CFG_REG    = 0x03
MPU_SMPLRT_DIV_REG  = 0x04

# ================================================================
#  Battery monitoring (INA219)
# ================================================================
BATTERY_MIN_V              = 3.4   # measured shutdown voltage = 0%
BATTERY_MAX_V              = 4.2   # full charge = 100%
LOW_BATTERY_THRESHOLD_PCT  = 20

BATTERY_CHECK_INTERVAL_MS = 5_000      # read INA219 + update warning LED
BATTERY_SEND_INTERVAL_MS  = 900_000    # send reading to Pi 5 (15 min)

# ================================================================
#  Heartbeat ("I'm alive" message to Pi 5)
# ================================================================
HEARTBEAT_INTERVAL_MS = 30_000

# ================================================================
#  BLE trigger pulse (signal line to XIAO nRF52840 advertiser)
# ================================================================
BLE_TRIGGER_PULSE_MS = 1000

# ================================================================
#  Networking
# ================================================================
WIFI_CONNECT_TIMEOUT_MS = 15_000   # generous — WiFi is only the fallback path
SOCKET_TIMEOUT_S        = 2