"""ERS backend server entry point.

Run on the Raspberry Pi from the project root::

    python3 server/server.py

Listens for JSON emergency payloads over TCP (port 8081) and fans each one out to
the audio, email, SMS and relay alert handlers, persisting every incident to the
SQLite database. Relay and audio sirens loop until stopped from the dashboard.

Responsibilities are split across sibling modules:
    config            - paths, host/port, constants
    event_log         - JSONL event logging
    database          - SQLite schema, flags, incidents, lookups
    location          - optional RSSI trilateration (experimental)
    alert_runtime     - background relay/audio siren loops
    incident_handler  - payload -> alerts + incident record
    tcp_server        - the listening socket
"""


from database import init_db
from tcp_server import start_server
from ble_location_runtime import start_ble_location_runtime

if __name__ == "__main__":
    init_db()

    # Start BLE Gateway MQTT listener in the background.
    # This keeps the latest RSSI cache ready for emergency location lookup.
    start_ble_location_runtime()
    print("[BLE] BLE location runtime started")

    start_server()
