"""Server-side identity resolution: active / commissioning / retired / unregistered.

Uses a throwaway SQLite DB and stubbed alert handlers, so no sirens, relay,
email or SMS are touched. Run: python -m unittest discover tests
"""

import os
import sqlite3
import sys
import tempfile
import types
import unittest
from datetime import datetime, timedelta

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SERVER = os.path.join(ROOT, "server")

_TMP = tempfile.TemporaryDirectory()
os.environ["ERS_DB_PATH"] = os.path.join(_TMP.name, "ers.sqlite")
os.environ["ERS_LOG_FILE"] = os.path.join(_TMP.name, "events.jsonl")
os.environ["ERS_SITE_FILE"] = os.path.join(_TMP.name, "site.json")
sys.path.insert(0, SERVER)

CALLS = []


def _install_stubs():
    rt = types.ModuleType("alert_runtime")
    rt.start_audio_loop = lambda t: CALLS.append(("audio", t))
    rt.start_relay_loop = lambda: CALLS.append(("relay",))
    rt.stop_all_alerts = lambda: CALLS.append(("stop",))
    em = types.ModuleType("email_alert")
    em.send_email_alert = lambda ev, to_emails=None: CALLS.append(("email", ev["device_id"], tuple(to_emails or ())))
    sms = types.ModuleType("sms_alert")
    sms.send_sms_alert = lambda ev, to_number="": CALLS.append(("sms", ev["device_id"])) or "ok"
    loc = types.ModuleType("location")
    loc.safe_estimate_location = lambda payload, when: None
    for m in (rt, em, sms, loc):
        sys.modules[m.__name__] = m


_install_stubs()
import config  # noqa: E402
import database  # noqa: E402
import incident_handler as ih  # noqa: E402

ADDR = ("127.0.0.1", 5555)
SITE_FILE = os.path.join(_TMP.name, "site.json")
ORIG_DB = os.path.join(_TMP.name, "ers.sqlite")


def lora(dev, event, site=7, **kw):
    meta = {1: ("mass", True), 2: ("personal", True), 3: ("personal", True), 99: ("heartbeat", False)}[event]
    p = {"device_id": dev, "site_id": site, "event_code": event, "emergency": meta[1],
         "emergency_type": meta[0], "emergency_message": "x", "seq": event, "source": "lora",
         "channel": "lora", "rssi": -90, "snr": 7.0}
    p.update(kw)
    return p


class RegistrationTests(unittest.TestCase):
    def setUp(self):
        # Other test modules also touch os.environ, so pin the paths on the module itself.
        config.SITE_FILE_CANDIDATES = [SITE_FILE]
        config.DB_PATH = database.DB_PATH = ORIG_DB
        config.LOG_FILE = os.environ["ERS_LOG_FILE"]
        for f in (database.DB_PATH, SITE_FILE):
            if os.path.exists(f):
                os.remove(f)
        with open(SITE_FILE, "w") as f:
            f.write('{"site_id": 7}')
        database.init_db()
        self.conn = sqlite3.connect(database.DB_PATH)
        self.conn.execute("INSERT INTO staff (name, role, email, admin_email_heartbeat_fail) "
                          "VALUES ('Alice','Nurse','a@x.com',0)")
        self.conn.execute("INSERT INTO staff (name, role, email, admin_email_heartbeat_fail) "
                          "VALUES ('Admin','IT','admin@x.com',1)")
        self.conn.execute("INSERT INTO devices (device_id, label, assigned_staff_id, status) "
                          "VALUES ('ERS-AAAAAAAA','Alice pendant',1,'active')")
        self.conn.execute("INSERT INTO devices (device_id, label, status) "
                          "VALUES ('ERS-BBBBBBBB','New unit','commissioning')")
        self.conn.execute("INSERT INTO devices (device_id, label, status) VALUES ('ERS-CCCCCCCC','Old','retired')")
        self.conn.commit()
        CALLS.clear()
        ih.LAST_SENT.clear()

    def tearDown(self):
        self.conn.close()

    def incidents(self):
        c = sqlite3.connect(database.DB_PATH)
        c.row_factory = sqlite3.Row
        rows = [dict(r) for r in c.execute("SELECT * FROM incidents ORDER BY id")]
        c.close()
        return rows

    def device(self, dev_id):
        c = sqlite3.connect(database.DB_PATH)
        c.row_factory = sqlite3.Row
        row = dict(c.execute("SELECT * FROM devices WHERE device_id=?", (dev_id,)).fetchone())
        c.close()
        return row

    # -- active: normal path, correct wearer, link quality recorded
    def test_active_device_alerts_with_correct_wearer(self):
        ih.handle_payload(lora("ERS-AAAAAAAA", 1), ADDR)
        inc = self.incidents()[0]
        self.assertEqual(inc["device_id"], "ERS-AAAAAAAA")
        self.assertEqual(inc["triggered_by"], "Alice (Nurse)")
        self.assertEqual(inc["is_test"], 0)
        kinds = {c[0] for c in CALLS}
        self.assertTrue({"audio", "relay", "email", "sms"} <= kinds)
        dev = self.device("ERS-AAAAAAAA")
        self.assertEqual((dev["last_rssi"], dev["last_snr"], dev["last_channel"]), (-90, 7.0, "lora"))
        self.assertIsNotNone(dev["last_seen"])

    def test_heartbeat_updates_last_seen_without_incident(self):
        ih.handle_payload(lora("ERS-AAAAAAAA", 99), ADDR)
        self.assertEqual(self.incidents(), [])
        self.assertIsNotNone(self.device("ERS-AAAAAAAA")["last_seen"])

    def test_wifi_battery_report_updates_battery(self):
        ih.handle_payload({"device_id": "ERS-AAAAAAAA", "site_id": 7, "emergency": False,
                           "battery_pct": 81, "channel": "wifi"}, ADDR)
        dev = self.device("ERS-AAAAAAAA")
        self.assertEqual(dev["battery_pct"], 81)
        self.assertEqual(dev["last_channel"], "wifi")

    # -- unregistered (acceptance criterion 3)
    def test_unregistered_flagged_no_wearer_no_siren(self):
        ih.handle_payload(lora("ERS-DEADBEEF", 1), ADDR)
        inc = self.incidents()[0]
        self.assertEqual(inc["device_id"], "UNREGISTERED-ERS-DEADBEEF")
        self.assertIsNone(inc["triggered_by"])
        self.assertEqual(inc["device_status"], "unregistered")
        kinds = [c[0] for c in CALLS]
        self.assertNotIn("audio", kinds)
        self.assertNotIn("relay", kinds)
        self.assertNotIn("sms", kinds)
        self.assertIn(("email", "UNREGISTERED-ERS-DEADBEEF", ("admin@x.com",)), CALLS)

    def test_unregistered_never_inherits_another_devices_wearer(self):
        ih.handle_payload(lora("ERS-AAAAAAAA", 2), ADDR)
        ih.handle_payload(lora("ERS-DEADBEEF", 2), ADDR)
        a, b = self.incidents()
        self.assertEqual(a["triggered_by"], "Alice (Nurse)")
        self.assertIsNone(b["triggered_by"])

    # -- commissioning (D9)
    def test_commissioning_records_test_and_ticks_checklist_without_alerts(self):
        ih.handle_payload(lora("ERS-BBBBBBBB", 99), ADDR)
        ih.handle_payload(lora("ERS-BBBBBBBB", 1), ADDR)
        ih.handle_payload(lora("ERS-BBBBBBBB", 2), ADDR)
        self.assertEqual(CALLS, [])
        incs = self.incidents()
        self.assertTrue(all(i["is_test"] == 1 for i in incs))
        self.assertTrue(incs[0]["message"].startswith("[TEST]"))
        ticks = {r[0] for r in self.conn.execute(
            "SELECT check_name FROM commissioning_checks WHERE device_id='ERS-BBBBBBBB'")}
        self.assertEqual(ticks, {"heartbeat", "mass", "personal"})

    def test_wifi_tick(self):
        ih.handle_payload({"device_id": "ERS-BBBBBBBB", "site_id": 7, "emergency": False,
                           "event_code": 99, "channel": "wifi"}, ADDR)
        ticks = {r[0] for r in self.conn.execute(
            "SELECT check_name FROM commissioning_checks WHERE device_id='ERS-BBBBBBBB'")}
        self.assertIn("wifi", ticks)

    def test_commissioning_device_does_not_block_live_device(self):
        ih.handle_payload(lora("ERS-BBBBBBBB", 1), ADDR)
        self.assertEqual(CALLS, [])
        ih.handle_payload(lora("ERS-AAAAAAAA", 1), ADDR)
        self.assertIn("relay", [c[0] for c in CALLS])

    def test_test_window_suppresses_then_reverts(self):
        until = (datetime.now() + timedelta(minutes=5)).strftime("%Y-%m-%d %H:%M:%S")
        self.conn.execute("UPDATE devices SET test_until=? WHERE device_id='ERS-AAAAAAAA'", (until,))
        self.conn.commit()
        ih.handle_payload(lora("ERS-AAAAAAAA", 1), ADDR)
        self.assertEqual(CALLS, [])
        self.assertEqual(self.incidents()[0]["is_test"], 1)
        past = (datetime.now() - timedelta(minutes=1)).strftime("%Y-%m-%d %H:%M:%S")
        self.conn.execute("UPDATE devices SET test_until=? WHERE device_id='ERS-AAAAAAAA'", (past,))
        self.conn.commit()
        ih.LAST_SENT.clear()
        ih.handle_payload(lora("ERS-AAAAAAAA", 1), ADDR)
        self.assertIn("relay", [c[0] for c in CALLS])

    # -- retired
    def test_retired_recorded_but_silent(self):
        ih.handle_payload(lora("ERS-CCCCCCCC", 1), ADDR)
        self.assertEqual(CALLS, [])
        self.assertEqual(self.incidents()[0]["device_status"], "retired")

    # -- site filter (T5)
    def test_other_site_ignored(self):
        ih.handle_payload(lora("ERS-AAAAAAAA", 1, site=9), ADDR)
        self.assertEqual(CALLS, [])
        self.assertEqual(self.incidents(), [])

    def test_no_site_file_means_no_filtering(self):
        os.remove(SITE_FILE)
        ih.handle_payload(lora("ERS-AAAAAAAA", 1, site=9), ADDR)
        self.assertEqual(len(self.incidents()), 1)

    # -- previous behaviour kept
    def test_legacy_registered_device_still_works(self):
        self.conn.execute("INSERT INTO devices (device_id, assigned_staff_id) VALUES ('PICO_TEST_01', 1)")
        self.conn.commit()
        ih.handle_payload({"device_id": "PICO_TEST_01", "emergency": True, "emergency_type": "mass",
                           "emergency_message": "legacy"}, ADDR)
        self.assertEqual(self.incidents()[0]["triggered_by"], "Alice (Nurse)")
        self.assertIn("relay", [c[0] for c in CALLS])

    def test_stop_command_still_works(self):
        ih.handle_payload({"command": "stop_alerts"}, ADDR)
        self.assertEqual(CALLS, [("stop",)])

    def test_migration_gives_existing_devices_active_status(self):
        c = sqlite3.connect(":memory:")
        c.execute("CREATE TABLE devices (id INTEGER PRIMARY KEY, device_id TEXT UNIQUE, label TEXT)")
        c.execute("INSERT INTO devices (device_id) VALUES ('OLD_1')")
        database._add_columns(c, "devices", database.DEVICE_MIGRATION_COLUMNS)
        self.assertEqual(c.execute("SELECT status FROM devices").fetchone()[0], "active")


if __name__ == "__main__":
    unittest.main()
