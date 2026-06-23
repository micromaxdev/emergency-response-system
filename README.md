# Emergency Response System (ERS)

A LoRa-based emergency alerting system developed through ECTE351 and extended
in‑house. Battery‑powered **Pi Pico / LoRa terminal devices** send emergency
triggers to a **Raspberry Pi backend server**, which fans each alert out to
audio sirens, a relay (mass‑notification siren), email and SMS, and records every
incident in a SQLite database. A **Streamlit dashboard** provides live monitoring,
staff/device management, logs and configuration.

> Location estimation (RSSI trilateration) and the BLE gateway tooling are
> **experimental** and under active development.

## Architecture

```
emergency-response-system/
├── server/                     Raspberry Pi backend (TCP listener + alert fan‑out)
│   ├── server.py               Entry point  →  python3 server/server.py
│   ├── config.py               Paths, host/port, constants
│   ├── event_log.py            JSONL event logger
│   ├── database.py             SQLite schema, control flags, incidents, lookups
│   ├── incident_handler.py     Payload → alerts (audio/email/sms/relay) + DB record
│   ├── alert_runtime.py        Background relay/audio siren loops
│   ├── tcp_server.py           Listening socket
│   ├── location.py             Optional RSSI trilateration wrapper (experimental)
│   ├── alert_handlers/         Pluggable alerts: email_alert, sms_alert, audio_alert, relay_alert
│   ├── trilateration.py        RSSI→distance + 2D trilateration (experimental)
│   ├── ble_gateway_*.py        BLE/MQTT gateway listener & calibration (experimental)
│   └── *_test.py               Localisation test scripts (experimental)
├── websites/                   Streamlit dashboard
│   ├── web_main.py             Entry point  →  streamlit run websites/web_main.py
│   ├── utils/                  Shared package: config, styles, db, incidents, staff,
│   │                           devices, battery, system, mapping, heartbeat
│   └── pages/                  Home, Dashboard, Configuration, Staff, Logs, Battery, Emergency
├── client/                     Pi Pico / LoRa terminal firmware (MicroPython) + SX126x drivers
├── assets/
│   ├── maps/                   Floor plans
│   └── audios/                 Alert sound files (mass / personal)
├── data/                       ers.sqlite — runtime database (created on first run)
├── logs/                       emergency_log.jsonl — runtime event log
├── requirements.txt
└── README.md
```

The server and the dashboard run as **two separate processes** and communicate
only through the shared SQLite database (`data/ers.sqlite`) and a small TCP
command channel (the dashboard tells the server to stop sirens, etc.).

## Prerequisites

- **Python 3.10+**
- Python packages: `pip install -r requirements.txt`
- **Audio playback** (server host): `ffplay` (from FFmpeg) preferred, or `mpg123`.
- **Relay HAT** (Raspberry Pi only): I²C enabled (`raspi-config`); the `smbus2`
  package drives the relay over I²C address `0x10`.

### Environment variables

Alert handlers read credentials from `~/.env`:

```ini
# Email (SMTP)
EMAIL_HOST=smtp.example.com
EMAIL_PORT=587
EMAIL_ADDRESS=alerts@example.com
EMAIL_PASSWORD=app-password
EMAIL_TO=fallback@example.com        # optional default recipients (comma-separated)

# SMS (Twilio)
TWILIO_ACCOUNT_SID=ACxxxxxxxx
TWILIO_AUTH_TOKEN=xxxxxxxx
TWILIO_MESSAGING_SERVICE_SID=MGxxxxxxxx
SMS_TO=+61400000000                  # optional default recipient(s)
```

Per‑incident email/SMS recipients are normally taken from the **staff** table
(managed on the dashboard); the `EMAIL_TO` / `SMS_TO` values are fallbacks.

## Running

From the project root:

```bash
# 1. (first time) create and activate a virtual environment, install deps
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# 2. Start the backend server — listens on TCP :8081 for device payloads
python3 server/server.py

# 3. In a second shell, start the dashboard
python3 -m streamlit run websites/web_main.py
```

`server/server.py` creates the database schema on startup. Relay and audio
sirens, once triggered by a mass emergency, loop until stopped from the
dashboard.

### Sending a test payload

```bash
printf '{"device_id":"PICO1","emergency":true,"emergency_type":"mass","emergency_message":"TEST"}' \
  | nc <server-ip> 8081
```

This should produce a log line in `logs/emergency_log.jsonl`, a new row in the
`incidents` table, and (on a configured Pi) audio + relay activation. Send
`{"command":"stop_alerts"}` to silence them.

## Experimental: location & BLE

The localisation features are not yet production‑ready:

- `server/trilateration.py` — RSSI→distance and 2D trilateration maths.
- `server/location.py` — wraps trilateration into the incident pipeline; only runs
  when a payload includes a `gateways` array.
- `server/ble_gateway_listener.py`, `server/ble_gateway_calibrate.py`,
  `server/cal_data.json` — BLE/MQTT gateway listener, live position plot and
  RSSI calibration. These need extra packages: `paho-mqtt`, `scipy`, `matplotlib`.
- `server/*_test.py` — standalone scripts for exploring the localisation maths.

## Notes

- Hardware design files (`schematics/`) and the compiled LoRa concentrator HAL
  (`sx1302_hal_rpi5-master/`) are kept on the deployment machine but are not
  tracked in version control.
