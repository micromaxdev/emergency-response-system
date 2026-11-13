"""Frame v1, device identity and bridge decoding. Run: python -m unittest discover tests"""

import base64
import importlib.util
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "server", "lora"))

import frame  # noqa: E402


def load_pico_identity():
    path = os.path.join(ROOT, "client", "pico_node", "Working Code", "identity.py")
    spec = importlib.util.spec_from_file_location("pico_identity", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


UID_A = bytes.fromhex("e66164084397a523")
UID_B = bytes.fromhex("e66164084397a524")


class IdentityTests(unittest.TestCase):
    def setUp(self):
        self.ident = load_pico_identity()

    def test_pico_and_server_derive_the_same_id(self):
        self.assertEqual(self.ident.device_id(UID_A), frame.device_id_from_serial(UID_A))

    def test_distinct_chips_get_distinct_ids(self):
        self.assertNotEqual(self.ident.device_id(UID_A), self.ident.device_id(UID_B))

    def test_id_is_stable(self):
        self.assertEqual(self.ident.device_id(UID_A), self.ident.device_id(UID_A))
        self.assertRegex(self.ident.device_id(UID_A), r"^ERS-[0-9A-F]{8}$")

    def test_pico_frame_matches_server_encoder(self):
        did = self.ident.device_id(UID_A)
        pico = self.ident.encode_frame_v1(7, self.ident.device_id_bytes(UID_A), 2, 0x1234)
        self.assertEqual(pico, frame.encode_v1(7, did, 2, 0x1234))
        self.assertEqual(len(pico), 10)
        self.assertEqual(pico[0], 0xE1)

    def test_roundtrip(self):
        did = frame.device_id_from_serial(UID_A)
        got = frame.decode_v1(frame.encode_v1(65535, did, 99, 70000))
        self.assertEqual(got, {"site_id": 65535, "device_id": did, "event_code": 99, "seq": 70000 & 0xFFFF})

    def test_foreign_and_legacy_frames_rejected(self):
        self.assertIsNone(frame.decode_v1(bytes([255, 1, 0, 0])))        # legacy 4-byte
        self.assertIsNone(frame.decode_v1(bytes(17)))                    # LoRaWAN-like uplink
        self.assertIsNone(frame.decode_v1(bytes([0x40]) + bytes(9)))     # LoRaWAN data MHDR


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        os.environ["ERS_LOG_DIR"] = self.tmp.name
        os.environ["ERS_SITE_FILE"] = os.path.join(self.tmp.name, "site.json")
        with open(os.environ["ERS_SITE_FILE"], "w") as f:
            f.write('{"site_id": 7}')
        sys.modules.pop("bridge", None)
        import bridge
        self.bridge = bridge
        bridge._site_id_cache["t"] = 0.0

    def tearDown(self):
        self.tmp.cleanup()

    def test_valid_frame_becomes_payload(self):
        did = frame.device_id_from_serial(UID_A)
        raw = frame.encode_v1(7, did, 1, 5)
        payload, reason = self.bridge.build_payload(raw, {"rssi": -80, "lsnr": 9.5, "freq": 917.2})
        self.assertIsNone(reason)
        self.assertEqual(payload["device_id"], did)
        self.assertEqual(payload["emergency_type"], "mass")
        self.assertEqual(payload["rssi"], -80)
        self.assertEqual(payload["channel"], "lora")

    def test_other_site_dropped(self):
        raw = frame.encode_v1(8, frame.device_id_from_serial(UID_A), 1, 5)
        self.assertEqual(self.bridge.build_payload(raw, {}), (None, "other_site"))

    def test_foreign_traffic_dropped(self):
        self.assertEqual(self.bridge.build_payload(bytes(20), {}), (None, "foreign_or_legacy"))

    def test_unknown_event_dropped(self):
        raw = frame.encode_v1(7, frame.device_id_from_serial(UID_A), 42, 5)
        self.assertEqual(self.bridge.build_payload(raw, {}), (None, "unknown_event"))

    def test_process_rxpk_logs_foreign_and_queues_when_server_down(self):
        self.bridge.ERS_PORT = 1  # nothing listens: forward fails
        raw = frame.encode_v1(7, frame.device_id_from_serial(UID_A), 1, 5)
        self.bridge.process_rxpk({"data": base64.b64encode(raw).decode(), "rssi": -70})
        self.assertEqual(len(self.bridge._read_queue()), 1)          # emergency kept for retry
        hb = frame.encode_v1(7, frame.device_id_from_serial(UID_A), 99, 6)
        self.bridge.process_rxpk({"data": base64.b64encode(hb).decode()})
        self.assertEqual(len(self.bridge._read_queue()), 1)          # heartbeat not queued
        self.bridge.process_rxpk({"data": base64.b64encode(bytes(17)).decode()})
        self.assertEqual(self.bridge.STATS["foreign_or_legacy"], 1)


if __name__ == "__main__":
    unittest.main()
