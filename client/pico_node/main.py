"""
main.py
--------
Entry point for the Pico 2 W emergency-button node. Wires together
the lib/ modules and runs the main control loop.

Emergencies:
  - MASS:     N rapid button presses -> immediate emergency
  - PERSONAL: button held for N seconds -> immediate emergency
  - FALL:     MPU6050 detects a fall -> N-second cancel window,
              then emergency unless the button is pressed once
              during that window

All emergencies and periodic heartbeats are sent via LoRaWAN first,
falling back to WiFi if LoRa delivery fails. See lib/transport.py.

Per-device identity (DEVICE_ID, DEVICE_CODE, WiFi credentials) comes
from secrets.py, which is unique per physical unit and never
committed to git. Everything else in this file is identical across
all units in the fleet.
"""

import time
from machine import Pin, I2C

import config
import secrets
from transport import (
    Transport,
    EVENT_CODE_MASS, EVENT_CODE_PERSONAL_HOLD,
    EVENT_CODE_FALL, EVENT_CODE_HEARTBEAT,
)
from emergency_button import EmergencyButton, ButtonEvent
from fall_detector import FallDetector
from fall_pending import FallPendingState
from battery_monitor import BatteryMonitor
from ble_trigger import BleTrigger


def build_payload(event_code, emergency, emergency_type=None,
                   message=None, extra=None):
    now_ms = time.ticks_ms()
    payload = {
        "device_id":    secrets.DEVICE_ID,
        "event_code":   event_code,
        "emergency":    emergency,
        "timestamp_ms": now_ms,
        "seq":          now_ms,
    }
    if emergency_type is not None:
        payload["emergency_type"] = emergency_type
    if message is not None:
        payload["emergency_message"] = message
    if extra:
        payload.update(extra)
    return payload


def main():
    print(f"READY: device={secrets.DEVICE_ID} code={secrets.DEVICE_CODE}")

    # ---- LEDs and button ----
    fall_led    = Pin("LED", Pin.OUT)          # built-in LED: fall-pending indicator
    fall_led.off()
    battery_led = Pin(config.PIN_BATTERY_LED, Pin.OUT)
    ble_pin     = Pin(config.PIN_BLE_TRIGGER, Pin.OUT)
    button_pin  = Pin(config.PIN_BUTTON, Pin.IN, Pin.PULL_UP)

    # ---- I2C buses ----
    time.sleep(2)   # UPS board needs extra settle time on power-up
    i2c0 = I2C(0, sda=Pin(config.PIN_I2C0_SDA), scl=Pin(config.PIN_I2C0_SCL),
               freq=config.I2C0_FREQ_HZ)
    time.sleep_ms(500)
    i2c1 = I2C(1, sda=Pin(config.PIN_I2C1_SDA), scl=Pin(config.PIN_I2C1_SCL),
               freq=config.I2C1_FREQ_HZ)
    time.sleep_ms(200)

    scan0 = i2c0.scan()
    print("[I2C0] scan:", [hex(a) for a in scan0])
    if config.MPU6050_ADDR not in scan0:
        raise RuntimeError("MPU6050 not found at 0x68 -- check wiring/power")

    scan1 = i2c1.scan()
    print("[I2C1] scan:", [hex(a) for a in scan1])

    # ---- Subsystems ----
    fall_detector = FallDetector(
        i2c0, config.MPU6050_ADDR,
        config.MPU_ACCEL_RANGE_REG, config.MPU_GYRO_RANGE_REG,
        config.MPU_DLPF_CFG_REG, config.MPU_SMPLRT_DIV_REG,
        config.FALL_THRESHOLD_LOW_G, config.FALL_THRESHOLD_HIGH_G,
        config.FALL_MIN_DURATION_MS, config.FALL_MAX_DURATION_MS,
        config.FALL_COOLDOWN_MS,
    )
    fall_detector.start()
    fall_pending = FallPendingState(fall_led, config.FALL_CANCEL_WINDOW_MS)

    battery = BatteryMonitor(
        i2c1, config.INA219_ADDR, battery_led,
        config.BATTERY_MIN_V, config.BATTERY_MAX_V,
        config.LOW_BATTERY_THRESHOLD_PCT,
    )

    button = EmergencyButton(
        button_pin,
        config.MASS_REQUIRED_PRESSES, config.MASS_WINDOW_MS,
        config.MASS_DEBOUNCE_MS, config.MASS_COOLDOWN_MS,
        config.PERSONAL_HOLD_SECONDS, config.PERSONAL_COOLDOWN_MS,
    )

    ble = BleTrigger(ble_pin, config.BLE_TRIGGER_PULSE_MS)

    transport = Transport(
        secrets.DEVICE_CODE, secrets.WIFI_SSID, secrets.WIFI_PASS,
        config.PI5_IP, config.PI5_PORT,
        config.WIFI_CONNECT_TIMEOUT_MS, config.SOCKET_TIMEOUT_S,
    )

    # Send one battery reading immediately so the LED + Pi5 reflect
    # reality from boot, rather than waiting for the first interval.
    pct, voltage = battery.check()
    if pct is not None:
        transport.send(build_payload(
            EVENT_CODE_HEARTBEAT, False,
            extra={"battery_pct": pct, "battery_v": voltage}))

    last_battery_check_ms = time.ticks_ms()
    last_battery_send_ms  = time.ticks_ms()
    last_heartbeat_ms     = time.ticks_ms()
    global_cooldown_until = 0

    print(f" - Rapid press GPIO{config.PIN_BUTTON} "
          f"{config.MASS_REQUIRED_PRESSES} times -> MASS EMERGENCY")
    print(f" - Hold GPIO{config.PIN_BUTTON} "
          f"{config.PERSONAL_HOLD_SECONDS}s -> PERSONAL EMERGENCY")
    print(f" - Fall detection -> LED on, "
          f"{config.FALL_CANCEL_WINDOW_MS // 1000}s cancel window")
    print(f" - Target Pi5: {config.PI5_IP}:{config.PI5_PORT}")

    # ---- Main loop ----
    while True:
        now = time.ticks_ms()

        if time.ticks_diff(now, global_cooldown_until) < 0:
            time.sleep_ms(config.BUTTON_POLL_MS)
            continue

        ble.update()

        if time.ticks_diff(now, last_heartbeat_ms) >= config.HEARTBEAT_INTERVAL_MS:
            last_heartbeat_ms = now
            ok, channel = transport.send(
                build_payload(EVENT_CODE_HEARTBEAT, False, message="HEARTBEAT"))
            print("[Heartbeat] sent via", channel, "ok:", ok)

        if time.ticks_diff(now, last_battery_check_ms) >= config.BATTERY_CHECK_INTERVAL_MS:
            last_battery_check_ms = now
            battery.check()

        if time.ticks_diff(now, last_battery_send_ms) >= config.BATTERY_SEND_INTERVAL_MS:
            last_battery_send_ms = now
            pct, voltage = battery.last_reading()
            if pct is not None:
                transport.send(build_payload(
                    EVENT_CODE_HEARTBEAT, False,
                    extra={"battery_pct": pct, "battery_v": voltage}))

        rising_edge = button.poll_edge()

        # ---- Fall-pending cancel window takes priority over mass/personal ----
        if fall_pending.pending:
            if rising_edge:
                fall_pending.cancel()
                button.wait_for_release()
            elif fall_pending.window_expired(now):
                extra = fall_pending.confirm()
                print("!!! FALL CONFIRMED -> PERSONAL EMERGENCY !!!")
                transport.send(build_payload(
                    EVENT_CODE_FALL, True, "personal", "FALL DETECTED", extra=extra))
                ble.trigger()
                global_cooldown_until = time.ticks_add(now, 800)
            time.sleep_ms(config.BUTTON_POLL_MS)
            continue

        # ---- Check for a new fall event from the background thread ----
        fall_happened, fall_time_ms, accel, gyro = fall_detector.take_fall_event()
        if fall_happened:
            extra = {
                "trigger_source":    "fall_detection",
                "fall_timestamp_ms": fall_time_ms,
                "accelerometer":     accel,
                "gyroscope":         gyro,
            }
            fall_pending.start(extra, now)

        # ---- MASS / PERSONAL gestures ----
        event = button.process_gesture(rising_edge)

        if event == ButtonEvent.MASS:
            print("!!! MASS EMERGENCY TRIGGERED !!!")
            transport.send(build_payload(
                EVENT_CODE_MASS, True, "mass", "MASS EMERGENCY TRIGGERED",
                extra={"trigger_source": "rapid_presses"}))
            global_cooldown_until = time.ticks_add(now, 1500)

        elif event == ButtonEvent.PERSONAL:
            print("!!! PERSONAL EMERGENCY TRIGGERED !!!")
            transport.send(build_payload(
                EVENT_CODE_PERSONAL_HOLD, True, "personal",
                "MANUAL EMERGENCY BUTTON (hold)",
                extra={"trigger_source": "manual_hold"}))
            ble.trigger()
            global_cooldown_until = time.ticks_add(now, 800)

        time.sleep_ms(config.BUTTON_POLL_MS)


main()