# Walkthrough: device registration (Option A-lite, plan Rev 3)

Implemented on branch `shakif/client-rego`. **Nothing is committed.** `git rm --cached` has staged three file removals (see Repo hygiene); everything else is unstaged or untracked.

**Verification:** 37 unit tests pass (`python -m unittest discover tests`). They cover frame/ID derivation, bridge decoding, server resolution, provisioning and dashboard import. **Nothing was run on a Pico, the Pi, the SX1302 HAT, the hotspot or systemd.** See "Needs hardware checking".

## How it works now

```
Pico: ID = ERS-<sha256(machine.unique_id())[:4]>    device.json = site id + hotspot creds
  LoRa : [0xE1][site 2B][device 4B][event][seq 2B] -> lora_pkt_fwd -> server/lora/bridge.py ┐ decode + site filter only
  Wi-Fi: JSON {device_id, site_id, channel:"wifi", fw, ...} -> 10.42.0.1:8081 ──────────────┤
                                                                                            ▼
                             incident_handler: device_id -> devices row -> status
          active -> normal | commissioning/testing -> TEST record, no alerts | retired -> silent | unknown -> UNREGISTERED-…
```

The three acceptance criteria are met in code and covered by tests:

| Criterion | Where |
|---|---|
| Distinct, stable ID per client | `identity.py` derives it from the chip serial. Tests: stable across calls, distinct per chip, Pico and server derive the same ID. |
| Alerts attributed to the right device and wearer | The server resolves ID to wearer for both transports (`incident_handler.py`). The bridge's "everything is PICO_TEST_01" mapping is gone. |
| Unknown IDs identified, not attributed | Stored as `UNREGISTERED-<id>` with no wearer. No siren, relay or staff SMS. Admin email, dashboard banner. |

## Changes by directory

### `client/pico_node/Working Code/`
- **`identity.py` (new).** `device_id()`, `hw_serial_hex()`, `load_config()` (reads `device.json`) and `encode_frame_v1()`. It runs on MicroPython and CPython so the tests can import it.
- **`main.py`.**
  - Removed the hard-coded Wi-Fi credentials (they were committed in plaintext), the hand-typed `DEVICE_ID` and `get_device_code()`, which returned 255 for every device except `PICO_TEST_02`. That was the root cause of `UNKNOWN_255`.
  - `DEVICE_ID`, the site ID and the Wi-Fi/server settings now come from `identity`.
  - LoRa packets are frame v1.
  - Wi-Fi JSON now carries `site_id`, `channel:"wifi"` and `fw`. The Wi-Fi path returns False when no SSID is provisioned, so the LoRa path is unaffected.
- **`../.gitignore`** now ignores `device.json`.
- The debloated `client/pico_node/main.py`, `config.py` and `lib/` are untouched.

**Decision: folder name.** The plan says to rename `Working Code` to a name without spaces. I left it in place. A rename breaks `.micropico` and anyone's muscle memory, and `provision_pico.py` takes `--src`, so spaces do no harm. It is a one-line `git mv` if you want it.

### `server/`
- **`config.py`.**
  - Env overrides `ERS_DB_PATH` and `ERS_LOG_FILE` for tests and unusual deployments.
  - `get_site_id()` reads `/etc/ers/site.json` (then `data/site.json`). With no site file, site filtering is off, so a dev box behaves as before.
  - `TEST_WINDOW_MIN = 15`.
- **`database.py`.**
  - Idempotent migrations. The `devices` table gains `hw_serial`, `site_id`, `status`, `last_seen`, `last_rssi`, `last_snr`, `last_channel`, `battery_pct`, `fw_version` and `test_until`. The `incidents` table gains `is_test` and `device_status`. A new `commissioning_checks` table holds the checklist.
  - Existing devices become `status='active'`, so already-registered devices keep alerting.
  - New helpers: `get_device`, `effective_status`, `touch_device`, `tick_checklist`, `start_test_window`, `get_admin_email_recipients`.
  - Admin flag columns on `staff` are now also created here.
- **`incident_handler.py`.** `handle_payload` is restructured.
  - Order: stop command, site filter, device lookup, `touch_device` on every packet, then status dispatch.
  - Heartbeats and battery packets update `last_seen` before the `if not em_flag: return`. Previously Wi-Fi heartbeats were dropped.
  - Commissioning and test-window devices tick the checklist (heartbeat, personal, mass, Wi-Fi).
  - The old alert dispatchers are unchanged and used only for `active`.
  - Legacy payloads (`PICO_TEST_01` and similar) still work if registered.
- **`lora/frame.py` (new).** Pure encode/decode for frame v1 and the SHA-256 ID derivation, shared by the bridge and tests.
- **`lora/bridge.py` (new).** Derived from `lora_udp_bridge.py` in your reference file.
  - Removed ThingsBoard entirely and the hard-coded token.
  - Frame v1 decode, site filter, foreign traffic counted and logged to `logs/lora_reception.jsonl` with a `result` field.
  - `PULL_ACK` added. RSSI, SNR and frequency forwarded in the JSON.
  - Retry plus on-disk queue (`logs/lora_bridge_queue.jsonl`, emergencies only, 500 max).
  - Binds to `127.0.0.1` by default instead of `0.0.0.0`, so a LAN host cannot inject fake packets.
- **`lora/global_conf.json` (new).** Copied from your reference file. Only change: `server_address` is `127.0.0.1` instead of `localhost` (avoids an IPv6 `::1` resolution surprise).
- **`lora/reset_concentrator.sh` (new).** See hardware checks below.
- **`setup/systemd/*.service` (new).** Four units with `@ERS_DIR@`-style placeholders: pkt-fwd (reset `ExecStartPre`), bridge (venv, retry, `After=ers-server`), server (`OOMScoreAdjust=-900`, linger runtime dir, `/etc/ers/ers.env`), dashboard (port 8501, `OOMScoreAdjust=-500`).
- **`setup/ers_setup.py` (new).** Implements plan section 5. Supports `--dry-run` (tested here: prints every action). Idempotent, and keeps the hotspot password unless `--rotate-wifi`.
- **`setup/deploy.sh` (new).** Stop, back up `data/ logs/ assets/maps/`, checkout, restore, migrate, start, smoke test. It uses `git update-index --assume-unchanged` on the three formerly tracked runtime files so the checkout can delete them safely (they are restored from the backup).

### `website/`
- **`utils/db.py`.** The dashboard creates the same registration columns and table (either process may start first). The column list is duplicated from `server/database.py`, with a comment saying so.
- **`utils/devices.py`.**
  - `import_device_list` (CSV; starts devices as `commissioning`; rejects bad IDs and other-site rows).
  - `fetch_checklist`, `activate_device` (blocked until heartbeat, personal, mass and wearer pass; Wi-Fi is optional), `set_device_status`, `start_test_window`.
  - `fetch_devices_in_test`, `fetch_unregistered_incidents`, `pick_target_device`.
  - `fetch_known_device_ids` strips the `UNREGISTERED-` prefix so registering offers the real ID.
- **`utils/heartbeat.py`.** Online/offline now reads `devices.last_seen`, with the old log as a fallback.
- **`utils/banners.py` (new).** Internet OFFLINE (TCP 443 check, cached 20 s), devices-in-test and unregistered-device banners. Shown on Home, Dashboard and Configuration.
- **`utils/incidents.py`.** The live feed and counts exclude `is_test` rows. The Logs page still shows everything as the audit trail.
- **`utils/config.py`, `__init__.py`.** `get_site_id()` and the banners export.
- **`pages/configuration.py`.** New "Import device list" expander. Each device row shows a status pill, online state, link quality, battery and the commissioning checklist, plus Activate, "Test for 15 min", Retire and Reactivate.
- **`pages/dashboard.py`.** The single-device comms panel was hard-coded to `PICO_TEST_01`. It now keeps that default when registered, otherwise follows the most recently heard active device.

### `tools/provision_pico.py` (new)
- Writes `device.json` from the site file, copies the firmware with `mpremote` (`main.py` last), verifies each file with SHA-256 computed on the device, reads back the real ID and appends a row to the device-list CSV.
- A hash mismatch aborts and adds nothing to the CSV (T10).
- `--stage-dir DIR --fake-uid HEX` provisions into a folder with no hardware.
- Needs `pip install mpremote` for real devices.

### `tests/` (new)
`test_frame.py`, `test_registration.py` and `test_provision.py`. They use a temp DB and stubbed alert handlers, so no real sirens, relay, email or SMS fire, and the dev `data/ers.sqlite` is not touched.

### Repo hygiene
- `.gitignore` adds `data/exports/`, `data/site.json`, `assets/maps/company_map.png`.
- **Staged (not committed):** `git rm --cached` for `data/ers.sqlite`, `logs/emergency_log.jsonl`, `assets/maps/company_map.png` (decision D12). The files are still on disk.
- `requirements.txt` adds `paho-mqtt`. The BLE runtime already imports it.

## Not changed
`alert_handlers/*`, `alert_runtime.py`, `tcp_server.py`, `location.py`, `ble_location_runtime.py`, the other dashboard pages, the debloated client and `FallTesting`.

## Needs hardware checking / outstanding

1. **Not run on hardware.** Pico, Pi, hotspot and systemd are all untested. Run test plan T1 to T13 on the Pi.
2. **`reset_concentrator.sh`.** Your reference file has the bridge and `global_conf.json` but not `reset_lgw.sh` or `start_lora.sh`. The script prefers the HAL's own `reset_lgw.sh` and otherwise uses a `pinctrl` sequence (GPIO23 reset, GPIO18 power-enable) modelled on Semtech's stock script. Verify the pins and `pinctrl` on your HAT.
3. **HAL build and pinning.** `ers_setup.py` expects the Pi 5 fork at `~/sx1302_hal_rpi5-master` (or `--hal-dir`) and only runs `make` if `lora_pkt_fwd` is missing. Automatic download of a pinned release (D7) is **not implemented**, because I don't know which fork or tag you want.
4. **Hotspot isolation.** NetworkManager `ipv4.method shared` gives the Picos DHCP but also NATs. I add an nftables drop (`iifname wlan0`, in a NetworkManager dispatcher script) so they can't reach Ethernet. Confirm it works on the Pi, in particular that the hostname/mDNS access in section 1 of the plan still holds.
5. **RSSI offset `-215.4`** is still the unverified value from your config (plan section 11).
6. **Rotate the ThingsBoard token** and delete the device in the ThingsBoard account. It was exposed in plaintext in `lora_udp_bridge.py`. Also rotate the Wi-Fi password that was committed in `Working Code/main.py` history.
7. **Stale registered device.** The dev DB has `UNKNOWN_255` registered as a device. The migration marks it `active`, so it would still alert if anything sent it. Nothing does now. Delete it on the dashboard.
8. **Admin recipients.** "Unregistered device" emails go to staff with the existing *heartbeat-fail* admin email flag. Say if you want a dedicated flag and UI checkbox.
9. **Old Pi units.** Nothing disables the old `lora-pkt-fwd` / `lora-udp-bridge` units. `ers_setup.py` prints the command, to be run after the new units are verified. Both can't run at once (UDP 1730).
10. **Before the hotspot step on the dev Pi, plug in Ethernet** (plan section 7), or SSH, Tailscale and the Picos' current Wi-Fi path drop together.
11. **Phase 2** (signed `0xE2` frames) is not implemented. `0xE1` reserves the space.
12. **BLE gateways** still publish to mosquitto and may need to join the hotspot or LAN (plan section 11).
13. **README** not updated. It still shows the old layout and the 4-byte payload.

## Typical flow after deploying
1. `sudo python3 server/setup/ers_setup.py --site <name> --site-id <n>`. This writes `/etc/ers/site.json` and exports `data/exports/site-<name>.json`.
2. Per Pico: `python tools/provision_pico.py --site data/exports/site-<name>.json --label "Reception"`.
3. Dashboard, Configuration, "Import device list" (the CSV in `data/exports/`). Assign wearers, press each button, then Activate.
