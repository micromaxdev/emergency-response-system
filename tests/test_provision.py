"""Provisioning tool (no hardware: DirTransport) and dashboard device-list import."""

import csv
import json
import os
import sqlite3
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "server", "lora"))

import frame  # noqa: E402
import provision_pico as pp  # noqa: E402

SITE = {"site_name": "acme", "site_id": 7, "hotspot_ssid": "ERS-acme",
        "hotspot_password": "pw123456", "hotspot_ip": "10.42.0.1", "server_port": 8081}
UID = "e66164084397a523"


class ProvisionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.csv = os.path.join(self.tmp.name, "devices.csv")
        self.dev = os.path.join(self.tmp.name, "pico")

    def tearDown(self):
        self.tmp.cleanup()

    def test_device_json(self):
        self.assertEqual(pp.build_device_json(SITE), {
            "site_id": 7, "wifi_ssid": "ERS-acme", "wifi_pass": "pw123456",
            "server_ip": "10.42.0.1", "server_port": 8081})

    def test_firmware_list_puts_main_last_and_skips_secrets(self):
        names = pp.firmware_files(pp.DEFAULT_SRC)
        self.assertEqual(names[-1], "main.py")
        self.assertIn("identity.py", names)
        self.assertNotIn("secrets.py", names)

    def test_successful_provision_writes_verified_files_and_csv(self):
        row = pp.provision(pp.DirTransport(self.dev, UID), SITE, pp.DEFAULT_SRC, self.csv, "Reception")
        self.assertEqual(row["device_id"], frame.device_id_from_serial(bytes.fromhex(UID)))
        self.assertEqual(json.load(open(os.path.join(self.dev, "device.json")))["site_id"], 7)
        rows = list(csv.DictReader(open(self.csv)))
        self.assertEqual([r["device_id"] for r in rows], [row["device_id"]])
        self.assertEqual(rows[0]["label"], "Reception")

    def test_reprovisioning_same_device_does_not_duplicate(self):
        pp.provision(pp.DirTransport(self.dev, UID), SITE, pp.DEFAULT_SRC, self.csv)
        pp.provision(pp.DirTransport(self.dev, UID), SITE, pp.DEFAULT_SRC, self.csv)
        self.assertEqual(len(list(csv.DictReader(open(self.csv)))), 1)

    def test_corrupted_transfer_is_caught_and_device_not_listed(self):  # T10
        with self.assertRaises(pp.ProvisionError):
            pp.provision(pp.DirTransport(self.dev, UID, corrupt={"main.py"}), SITE, pp.DEFAULT_SRC, self.csv)
        self.assertFalse(os.path.exists(self.csv))

    def test_cli_stage_mode(self):
        site_file = os.path.join(self.tmp.name, "site.json")
        json.dump(SITE, open(site_file, "w"))
        rc = pp.main(["--site", site_file, "--stage-dir", self.dev, "--csv", self.csv])
        self.assertEqual(rc, 0)

    def test_two_chips_two_ids(self):
        r1 = pp.provision(pp.DirTransport(self.dev, UID), SITE, pp.DEFAULT_SRC, self.csv)
        r2 = pp.provision(pp.DirTransport(self.dev + "2", "e66164084397a524"), SITE, pp.DEFAULT_SRC, self.csv)
        self.assertNotEqual(r1["device_id"], r2["device_id"])


class DashboardImportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.db = os.path.join(self.tmp.name, "ers.sqlite")
        sys.path.insert(0, os.path.join(ROOT, "website"))
        for m in [m for m in sys.modules if m == "utils" or m.startswith("utils.")]:
            del sys.modules[m]
        import utils as ers
        import utils.db as udb
        udb.DB_PATH = self.db
        self.ers = ers
        ers.init_extra_tables()
        conn = sqlite3.connect(self.db)
        conn.execute("""CREATE TABLE incidents (id INTEGER PRIMARY KEY AUTOINCREMENT, server_time TEXT,
                        device_id TEXT, emergency_type TEXT, message TEXT)""")
        conn.commit()
        conn.close()

    def tearDown(self):
        self.tmp.cleanup()

    def test_import_starts_in_commissioning_and_flags_problems(self):
        text = ("device_id,hw_serial,site_id,fw_version,label\n"
                "ERS-11111111,AA,7,1.0.0,Front desk\n"
                "ERS-22222222,BB,9,1.0.0,Wrong site\n"
                "bogus,CC,7,1.0.0,Bad id\n"
                "ERS-11111111,AA,7,1.0.0,Duplicate\n")
        added, skipped, errors = self.ers.import_device_list(text, server_site_id=7)
        self.assertEqual((added, skipped, len(errors)), (1, 1, 2))
        dev = [d for d in self.ers.fetch_all_devices() if d["device_id"] == "ERS-11111111"][0]
        self.assertEqual(dev["status"], "commissioning")

    def test_activation_requires_checklist(self):
        self.ers.import_device_list("device_id,site_id\nERS-11111111,7\n", 7)
        dev = self.ers.fetch_all_devices()[0]
        ok, missing = self.ers.activate_device(dev)
        self.assertFalse(ok)
        self.assertEqual(set(missing), {"heartbeat", "personal", "mass", "wearer"})
        conn = sqlite3.connect(self.db)
        conn.execute("INSERT INTO staff (name) VALUES ('Bob')")
        conn.execute("UPDATE devices SET assigned_staff_id=1")
        conn.executemany("INSERT INTO commissioning_checks (device_id, check_name) VALUES (?,?)",
                         [("ERS-11111111", c) for c in ("heartbeat", "personal", "mass")])
        conn.commit()
        conn.close()
        ok, missing = self.ers.activate_device(self.ers.fetch_all_devices()[0])
        self.assertTrue(ok, missing)
        self.assertEqual(self.ers.fetch_all_devices()[0]["status"], "active")

    def test_unregistered_id_offered_for_registration_without_prefix(self):
        conn = sqlite3.connect(self.db)
        conn.execute("INSERT INTO incidents (server_time, device_id) VALUES (datetime('now','localtime'), 'UNREGISTERED-ERS-DEADBEEF')")
        conn.commit()
        conn.close()
        self.assertEqual(self.ers.fetch_known_device_ids(), ["ERS-DEADBEEF"])
        self.assertEqual(len(self.ers.fetch_unregistered_incidents()), 1)

    def test_status_from_last_seen(self):
        from datetime import datetime, timedelta
        fmt = "%Y-%m-%d %H:%M:%S"
        self.assertEqual(self.ers.status_from_last_seen((datetime.now() - timedelta(seconds=10)).strftime(fmt))["state"], "online")
        self.assertEqual(self.ers.status_from_last_seen((datetime.now() - timedelta(hours=1)).strftime(fmt))["state"], "offline")
        self.assertEqual(self.ers.status_from_last_seen(None)["state"], "unknown")


if __name__ == "__main__":
    unittest.main()
