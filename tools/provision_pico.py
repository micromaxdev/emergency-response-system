#!/usr/bin/env python3
"""Bench-provision one Pico for a customer site (about 1 minute per unit).

    python tools/provision_pico.py --site data/exports/site-acme.json
    python tools/provision_pico.py --site site.json --port COM5 --label "Reception"

What it does, in order:
  1. writes device.json (site id + hotspot credentials) from the site file,
  2. copies the firmware (*.py from the firmware folder) with mpremote,
  3. VERIFIES every file by SHA-256 computed on the device (MicroPico is not
     used for transfers: it corrupts files). A mismatch aborts, nothing is
     added to the device list,
  4. reads back the device's real ID (derived from its factory serial),
  5. appends it to the device-list CSV that is imported on the dashboard,
  6. prints a label line for the sticker.

Without hardware, `--stage-dir DIR --fake-uid HEX` "provisions" into a folder so
the whole flow (hashing, device.json, CSV) can be exercised and tested.

Requires `pip install mpremote` for real devices (not needed for --stage-dir).
"""

import argparse
import csv
import datetime
import hashlib
import json
import os
import re
import subprocess
import sys

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SRC = os.path.join(REPO_DIR, "client", "pico_node", "Working Code")
CSV_FIELDS = ["device_id", "hw_serial", "site_id", "fw_version", "label", "provisioned_at"]
DEVICE_ID_RE = re.compile(r"^ERS-[0-9A-F]{8}$")

# Never copy these to a device even if they sit in the firmware folder.
EXCLUDE = {"secrets.py", "device.json"}


class ProvisionError(Exception):
    pass


# -- Pure helpers (unit tested) --------------------------------------------------
def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def load_site(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        site = json.load(f)
    for key in ("site_id", "hotspot_ssid", "hotspot_password"):
        if key not in site:
            raise ProvisionError(f"site file {path} is missing '{key}'")
    if not 1 <= int(site["site_id"]) <= 0xFFFF:
        raise ProvisionError("site_id must be 1..65535")
    return site


def build_device_json(site: dict) -> dict:
    return {
        "site_id": int(site["site_id"]),
        "wifi_ssid": site["hotspot_ssid"],
        "wifi_pass": site["hotspot_password"],
        "server_ip": site.get("hotspot_ip", "10.42.0.1"),
        "server_port": int(site.get("server_port", 8081)),
    }


def firmware_files(src_dir: str) -> list:
    """Firmware *.py files, main.py last so the device never half-boots."""
    names = sorted(n for n in os.listdir(src_dir)
                   if n.endswith(".py") and n not in EXCLUDE)
    if "main.py" not in names or "identity.py" not in names:
        raise ProvisionError(f"{src_dir} must contain main.py and identity.py")
    names.remove("main.py")
    return names + ["main.py"]


def parse_identity_output(text: str):
    """Parse the 'ERS-XXXXXXXX SERIALHEX FWVER' line printed by the device."""
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 3 and DEVICE_ID_RE.match(parts[0]):
            return parts[0], parts[1], parts[2]
    raise ProvisionError(f"could not read device identity from: {text!r}")


def append_csv(path: str, row: dict) -> bool:
    """Append a device row. Returns False if the device_id is already listed."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    existing = set()
    exists = os.path.exists(path)
    if exists:
        with open(path, newline="", encoding="utf-8") as f:
            existing = {r["device_id"] for r in csv.DictReader(f)}
    if row["device_id"] in existing:
        return False
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if not exists:
            w.writeheader()
        w.writerow(row)
    return True


# -- Transports -------------------------------------------------------------------
class MpremoteTransport:
    """Real device over USB via mpremote."""

    def __init__(self, port=None):
        self.base = [sys.executable, "-m", "mpremote"]
        if port:
            self.base += ["connect", port]

    def _run(self, *args):
        res = subprocess.run(self.base + list(args), capture_output=True, text=True)
        if res.returncode != 0:
            raise ProvisionError(f"mpremote {' '.join(args)} failed: {res.stderr.strip() or res.stdout.strip()}")
        return res.stdout

    def put(self, local_path, remote_name):
        self._run("fs", "cp", local_path, ":" + remote_name)

    def device_sha256(self, remote_name):
        code = ("import hashlib,binascii\n"
                "h=hashlib.sha256()\n"
                f"f=open({remote_name!r},'rb')\n"
                "while True:\n"
                " b=f.read(512)\n"
                " if not b: break\n"
                " h.update(b)\n"
                "f.close()\n"
                "print('SHA256:'+binascii.hexlify(h.digest()).decode())\n")
        out = self._run("exec", code)
        m = re.search(r"SHA256:([0-9a-f]{64})", out)
        if not m:
            raise ProvisionError(f"no hash returned for {remote_name}: {out!r}")
        return m.group(1)

    def read_identity(self):
        out = self._run("exec", "import identity,machine\n"
                        "print(identity.device_id(), identity.hw_serial_hex(), identity.FW_VERSION)")
        return parse_identity_output(out)

    def reset(self):
        try:
            self._run("reset")
        except ProvisionError:
            pass  # the port drops while rebooting


class DirTransport:
    """No hardware: 'provisions' into a folder with a fake chip serial (tests/dry runs)."""

    def __init__(self, directory, fake_uid_hex, corrupt=None):
        self.dir = directory
        self.uid = bytes.fromhex(fake_uid_hex)
        self.corrupt = corrupt or set()
        os.makedirs(directory, exist_ok=True)

    def put(self, local_path, remote_name):
        with open(local_path, "rb") as f:
            data = f.read()
        if remote_name in self.corrupt:          # simulate a bad transfer
            data = data[:-1] + b"#"
        with open(os.path.join(self.dir, remote_name), "wb") as f:
            f.write(data)

    def device_sha256(self, remote_name):
        with open(os.path.join(self.dir, remote_name), "rb") as f:
            return sha256_bytes(f.read())

    def read_identity(self):
        device_id = "ERS-" + hashlib.sha256(self.uid).digest()[:4].hex().upper()
        fw = "unknown"
        ident = os.path.join(self.dir, "identity.py")
        if os.path.exists(ident):
            with open(ident, encoding="utf-8") as f:
                m = re.search(r'FW_VERSION\s*=\s*"([^"]+)"', f.read())
            fw = m.group(1) if m else fw
        return device_id, self.uid.hex().upper(), fw

    def reset(self):
        pass


# -- Orchestration ------------------------------------------------------------------
def provision(transport, site, src_dir, csv_path, label="", work_dir=None):
    """Provision one device. Returns the CSV row dict. Raises ProvisionError."""
    import tempfile
    names = firmware_files(src_dir)
    device_json = json.dumps(build_device_json(site), indent=2).encode("utf-8")

    with tempfile.TemporaryDirectory(dir=work_dir) as tmp:
        dj_path = os.path.join(tmp, "device.json")
        with open(dj_path, "wb") as f:
            f.write(device_json)

        to_send = [("device.json", dj_path)] + [(n, os.path.join(src_dir, n)) for n in names]
        expected = {}
        for remote, local in to_send:
            with open(local, "rb") as f:
                expected[remote] = sha256_bytes(f.read())
            transport.put(local, remote)
            print(f"  copied {remote}")

        # Verify EVERYTHING before the device is trusted or listed.
        for remote, want in expected.items():
            got = transport.device_sha256(remote)
            if got != want:
                raise ProvisionError(f"hash mismatch on {remote}: expected {want[:12]}..., device has {got[:12]}...")
            print(f"  verified {remote}")

    device_id, hw_serial, fw = transport.read_identity()
    row = {
        "device_id": device_id, "hw_serial": hw_serial, "site_id": int(site["site_id"]),
        "fw_version": fw, "label": label,
        "provisioned_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    added = append_csv(csv_path, row)
    transport.reset()
    print(f"  {'added to' if added else 'already in'} {csv_path}")
    return row


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", required=True, help="site file exported by ers_setup.py")
    ap.add_argument("--src", default=DEFAULT_SRC, help="firmware folder")
    ap.add_argument("--port", help="serial port (default: mpremote auto-detect)")
    ap.add_argument("--csv", help="device list CSV (default data/exports/devices-<site>.csv)")
    ap.add_argument("--label", default="", help="friendly label, e.g. 'Reception desk'")
    ap.add_argument("--stage-dir", help="no hardware: provision into this folder instead")
    ap.add_argument("--fake-uid", default="e66164084397a523", help="chip serial hex for --stage-dir")
    args = ap.parse_args(argv)

    try:
        site = load_site(args.site)
        csv_path = args.csv or os.path.join(
            REPO_DIR, "data", "exports", f"devices-{site.get('site_name', site['site_id'])}.csv")
        transport = (DirTransport(args.stage_dir, args.fake_uid) if args.stage_dir
                     else MpremoteTransport(args.port))
        print(f"Provisioning for site {site.get('site_name', '')} (id {site['site_id']})")
        row = provision(transport, site, args.src, csv_path, args.label)
    except (ProvisionError, OSError, ValueError) as e:
        print(f"FAILED: {e}", file=sys.stderr)
        return 1

    print(f"\nOK  {row['device_id']}   (chip {row['hw_serial']}, fw {row['fw_version']})")
    print(f"LABEL: {row['device_id']} | site {row['site_id']} | {row['label']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
