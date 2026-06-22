"""
secrets_template.py
--------------------
TEMPLATE ONLY. Copy this file to `secrets.py` on each physical device
and fill in real values. `secrets.py` is gitignored and must NEVER
be committed — it contains WiFi credentials and is unique per unit.

Setup for a new Pico:
    1. cp secrets_template.py secrets.py
    2. Fill in the values below for this specific unit
    3. Copy secrets.py onto the device (it stays local, never goes to git)
"""
# --- WiFi ---
WIFI_SSID = ""
WIFI_PASS = ""

# --- Device identity (UNIQUE per physical unit) ---
DEVICE_ID   = ""   # e.g. "PICO_TEST_01", "Thomas", "Skylar"
DEVICE_CODE = 0    # 1 byte, sent in every LoRa packet. Must match what's
                    # registered on the Pi 5 server for this DEVICE_ID.
                    # Registry: PICO_TEST_01=1, Thomas=2, Skylar=3