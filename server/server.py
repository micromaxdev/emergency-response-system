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

if __name__ == "__main__":
    init_db()
    start_server()
