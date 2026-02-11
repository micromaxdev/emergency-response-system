import time
from smbus2 import SMBus


DEVICE_BUS = 1
DEVICE_ADDR = 0x10
RELAY_CH = 1
RELAY_ON = 0xFF
RELAY_OFF = 0x00


def _bus():
    return SMBus(DEVICE_BUS)


def relay_on(ch: int = RELAY_CH):
    bus = _bus()
    try:
        bus.write_byte_data(DEVICE_ADDR, ch, RELAY_ON)
    finally:
        bus.close()


def relay_off(ch: int = RELAY_CH):
    bus = _bus()
    try:
        bus.write_byte_data(DEVICE_ADDR, ch, RELAY_OFF)
    finally:
        bus.close()


def relay_pulse(ch: int = RELAY_CH, on_sec: float = 2.0, off_sec: float = 0.0, repeat: int = 1):
    for _ in range(max(1, int(repeat))):
        relay_on(ch)
        time.sleep(on_sec)
        relay_off(ch)
        if off_sec > 0:
            time.sleep(off_sec)


if __name__ == "__main__":
    print("Testing relay channel:", RELAY_CH)
    relay_pulse(RELAY_CH, on_sec=1.0, off_sec=1.0, repeat=3)
    print("Done.")
