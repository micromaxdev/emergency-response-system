"""Temporary Pico MPU6050 acquisition mode. Does not run the ERS application."""
import sys
import time
import machine
import ubinascii
import ujson
import os

SDA_PIN = 0
SCL_PIN = 1
I2C_HZ = 50000
MPU_ADDR = 0x68
TARGET_HZ = 100
ACCEL_CONFIG = 0x10  # +/-8 g, matching the inspected Drive code
GYRO_CONFIG = 0x08  # +/-500 degrees/s
DLPF_CONFIG = 0x03
SAMPLE_DIVIDER = 0x04


def signed(msb, lsb):
    value = (msb << 8) | lsb
    return value - 65536 if value & 0x8000 else value


def run():
    # Disable radios and keep the external BLE trigger low in this test mode.
    try:
        import network
        network.WLAN(network.STA_IF).active(False)
        network.WLAN(network.AP_IF).active(False)
    except (ImportError, AttributeError):
        pass
    machine.Pin(16, machine.Pin.OUT, value=0)
    i2c = machine.I2C(0, sda=machine.Pin(SDA_PIN), scl=machine.Pin(SCL_PIN), freq=I2C_HZ)
    time.sleep_ms(500)
    if MPU_ADDR not in i2c.scan():
        raise RuntimeError('MPU6050 missing at 0x68 on GP0/GP1. Check wiring and power.')
    registers = ((0x6B, 0x00), (0x1A, DLPF_CONFIG), (0x19, SAMPLE_DIVIDER),
                 (0x1C, ACCEL_CONFIG), (0x1B, GYRO_CONFIG))
    for reg, value in registers:
        i2c.writeto_mem(MPU_ADDR, reg, bytes([value]))
        time.sleep_ms(20)
    actual = {str(reg): i2c.readfrom_mem(MPU_ADDR, reg, 1)[0] for reg, _ in registers}
    for reg, expected in registers:
        if actual[str(reg)] != expected:
            raise RuntimeError('MPU configuration readback failed at register %s' % hex(reg))
    meta = dict(protocol='ERS_IMU_V1', mode='acquisition_only', sensor='MPU6050',
                session_id=ubinascii.hexlify(os.urandom(8)).decode(),
                device_id=ubinascii.hexlify(machine.unique_id()).decode(),
                target_hz=TARGET_HZ, accel_range_g=8, gyro_range_dps=500,
                accel_units='g', gyro_units='degrees_per_second',
                i2c_hz=I2C_HZ, sda_pin=SDA_PIN, scl_pin=SCL_PIN,
                registers=actual, detector_state='NOT_RUNNING',
                timestamp='elapsed microseconds at read start', firmware='ERS_IMU_1.0')
    period = 1000000 // TARGET_HZ
    previous = time.ticks_us()
    elapsed = 0
    next_due = previous
    next_meta = previous
    sequence = 0
    errors = 0
    while True:
        now = time.ticks_us()
        elapsed += time.ticks_diff(now, previous)
        previous = now
        if time.ticks_diff(now, next_meta) >= 0:
            print('@META ' + ujson.dumps(meta))
            next_meta = time.ticks_add(now, 2000000)
        if time.ticks_diff(now, next_due) < 0:
            time.sleep_us(200)
            continue
        next_due = time.ticks_add(now, period)
        try:
            data = i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)
            raw = [signed(data[i], data[i + 1]) for i in (0, 2, 4, 8, 10, 12)]
            clipped = int(any(abs(x) >= 32760 for x in raw))
            values = [x / 4096.0 for x in raw[:3]] + [x / 65.5 for x in raw[3:]]
            # Raw integer values are preserved alongside converted values.
            print('S,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%d' %
                  ((sequence, elapsed) + tuple(values) + (','.join(str(x) for x in raw), clipped)))
            sequence += 1
        except OSError as exc:
            errors += 1
            print('@ERROR ' + ujson.dumps(dict(device_us=elapsed, count=errors, message=str(exc))))
            time.sleep_ms(20)
        # A slow USB host can block print. Real timestamps reveal that gap.


if __name__ == '__main__':
    run()
