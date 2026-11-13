#!/usr/bin/env python3
"""One-shot ERS server setup for a Raspberry Pi 5 (run once with sudo).

    sudo python3 server/setup/ers_setup.py --site acme --site-id 7

What it does (plan section 5):
  1. hostname ers-<site> (mDNS via avahi)  ->  http://ers-<site>.local:8501
  2. private Wi-Fi hotspot on wlan0 for the Picos' backup path (10.42.0.1),
     isolated from Ethernet; Ethernet stays the only internet route
  3. checks the LoRa HAL build (lora_pkt_fwd)
  4. venv + requirements, mosquitto + avahi
  5. /etc/ers/site.json and /etc/ers/ers.env (mode 600)
  6. systemd units from server/setup/systemd (placeholders filled in), linger
  7. exports the site file used by tools/provision_pico.py
  8. customer hardening (disable Tailscale / Pi Connect / VNC / cups / ModemManager)

Use --dry-run to print every action without changing anything. The script is
idempotent: re-running keeps the existing site id / hotspot password unless
--rotate-wifi is given.

STATUS: written against the plan but NOT yet exercised on hardware. The hotspot
isolation (step 2) and HAL build (step 3) are the parts most worth checking.
"""

import argparse
import json
import os
import re
import secrets
import shutil
import string
import subprocess
import sys
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[2]
SYSTEMD_SRC = REPO_DIR / "server" / "setup" / "systemd"
ETC_DIR = Path("/etc/ers")
SITE_FILE = ETC_DIR / "site.json"
ENV_FILE = ETC_DIR / "ers.env"
UNITS = ["ers-lora-pkt-fwd", "ers-lora-bridge", "ers-server", "ers-dashboard"]
HOTSPOT_CON = "ers-hotspot"
HOTSPOT_IP = "10.42.0.1"
HARDEN_SERVICES = ["tailscaled", "rpi-connect", "rpi-connect-wayvnc", "wayvnc", "cups", "ModemManager"]

DRY = False


def log(msg):
    print(f"[setup] {msg}", flush=True)


def run(cmd, check=True, **kw):
    log("$ " + " ".join(str(c) for c in cmd))
    if DRY:
        return subprocess.CompletedProcess(cmd, 0, "", "")
    return subprocess.run(cmd, check=check, text=True, **kw)


def write_file(path: Path, content: str, mode=0o644):
    log(f"write {path} (mode {oct(mode)})")
    if DRY:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    os.chmod(path, mode)


def slugify(name: str) -> str:
    slug = re.sub(r"[^a-z0-9-]+", "-", name.lower()).strip("-")
    if not slug:
        sys.exit("--site must contain letters or digits")
    return slug[:30]


def random_password(n=16) -> str:
    alphabet = string.ascii_letters + string.digits
    return "".join(secrets.choice(alphabet) for _ in range(n))


def load_existing_site() -> dict:
    try:
        return json.loads(SITE_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


# -- Steps ----------------------------------------------------------------------
def step_hostname(slug):
    run(["hostnamectl", "set-hostname", f"ers-{slug}"])
    run(["systemctl", "enable", "--now", "avahi-daemon"], check=False)


def step_hotspot(site):
    """NetworkManager AP on wlan0. Ethernet (eth0) stays the only default route."""
    run(["nmcli", "connection", "delete", HOTSPOT_CON], check=False)
    run(["nmcli", "connection", "add", "type", "wifi", "ifname", "wlan0",
         "con-name", HOTSPOT_CON, "autoconnect", "yes",
         "ssid", site["hotspot_ssid"], "mode", "ap",
         "802-11-wireless.band", "bg",
         "ipv4.method", "shared", "ipv4.addresses", f"{HOTSPOT_IP}/24",
         "wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", site["hotspot_password"]])
    # 'shared' mode gives the Picos DHCP but also NATs to Ethernet. Block that:
    # the Picos only ever need the ERS server (decision D5).
    dispatcher = f"""#!/bin/sh
# Installed by ers_setup.py: keep hotspot clients away from the Ethernet LAN/internet.
if [ "$1" = "wlan0" ] && [ "$2" = "up" ]; then
    nft delete table inet ers_isolation 2>/dev/null
    nft -f - <<'NFT'
table inet ers_isolation {{
  chain forward {{
    type filter hook forward priority -10; policy accept;
    iifname "wlan0" drop
  }}
}}
NFT
fi
"""
    write_file(Path("/etc/NetworkManager/dispatcher.d/90-ers-isolate"), dispatcher, 0o755)
    run(["nmcli", "connection", "up", HOTSPOT_CON], check=False)


def step_hal(hal_dir: Path, skip_build: bool):
    fwd = hal_dir / "packet_forwarder" / "lora_pkt_fwd"
    if fwd.exists():
        log(f"found {fwd}")
        return
    if skip_build:
        log("WARNING: lora_pkt_fwd missing and --skip-hal-build given")
        return
    if not hal_dir.is_dir():
        sys.exit(f"HAL folder {hal_dir} not found. Place the Pi 5 sx1302_hal fork there "
                 "(or pass --hal-dir). Pinned download is still an open item.")
    run(["make", "-C", str(hal_dir)])


def step_python(user):
    venv = REPO_DIR / "venv"
    if not (venv / "bin" / "python").exists():
        run([sys.executable, "-m", "venv", str(venv)])
    run([str(venv / "bin" / "pip"), "install", "-r", str(REPO_DIR / "requirements.txt")])
    run(["chown", "-R", f"{user}:{user}", str(venv)], check=False)
    run(["apt-get", "install", "-y", "mosquitto", "avahi-daemon", "network-manager", "nftables"])


def step_config(site, user):
    write_file(SITE_FILE, json.dumps(site, indent=2) + "\n", 0o640)
    if not ENV_FILE.exists() or DRY:
        write_file(ENV_FILE, (
            "# ERS secrets (mode 600). Read by ers-server, ers-lora-bridge, ers-dashboard.\n"
            "# EMAIL_HOST=smtp.example.com\nEMAIL_PORT=587\n# EMAIL_ADDRESS=\n# EMAIL_PASSWORD=\n"
            "# TWILIO_ACCOUNT_SID=\n# TWILIO_AUTH_TOKEN=\n# TWILIO_FROM=\n"), 0o600)
    run(["chown", f"{user}:{user}", str(ENV_FILE)], check=False)
    run(["chown", f"root:{user}", str(SITE_FILE)], check=False)


def step_units(user, hal_dir):
    if DRY:
        uid = 1000
    else:
        import pwd  # POSIX only
        uid = pwd.getpwnam(user).pw_uid
    subs = {"@ERS_USER@": user, "@ERS_UID@": str(uid),
            "@ERS_DIR@": str(REPO_DIR), "@HAL_DIR@": str(hal_dir)}
    for unit in UNITS:
        text = (SYSTEMD_SRC / f"{unit}.service").read_text(encoding="utf-8")
        for k, v in subs.items():
            text = text.replace(k, v)
        write_file(Path(f"/etc/systemd/system/{unit}.service"), text)
    run(["chmod", "+x", str(REPO_DIR / "server/lora/reset_concentrator.sh")])
    run(["systemctl", "daemon-reload"])
    run(["loginctl", "enable-linger", user])
    # Old, pre-repo units are disabled only once the new ones are verified (plan step 2):
    log("NOTE: old 'lora-pkt-fwd' / 'lora-udp-bridge' units are NOT touched; disable them "
        "yourself after verifying: sudo systemctl disable --now lora-pkt-fwd lora-udp-bridge")
    for unit in UNITS:
        run(["systemctl", "enable", unit])


def step_export(site, out: Path):
    log(f"export site file -> {out}")
    if DRY:
        return
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(site, indent=2) + "\n", encoding="utf-8")
    os.chmod(out, 0o600)


def step_harden():
    for svc in HARDEN_SERVICES:
        run(["systemctl", "disable", "--now", svc], check=False)


def main():
    global DRY
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", required=True, help="site name, e.g. acme (-> hostname ers-acme)")
    ap.add_argument("--site-id", type=int, required=True, help="2-byte site id 1..65535")
    ap.add_argument("--user", default=os.environ.get("SUDO_USER") or "pi")
    ap.add_argument("--hal-dir", default=None, help="sx1302 HAL folder (default ~user/sx1302_hal_rpi5-master)")
    ap.add_argument("--export", default=None, help="where to write the site file for provisioning")
    ap.add_argument("--rotate-wifi", action="store_true", help="generate a new hotspot password")
    ap.add_argument("--skip-hotspot", action="store_true")
    ap.add_argument("--skip-hal-build", action="store_true")
    ap.add_argument("--skip-hardening", action="store_true",
                    help="keep Tailscale/Pi Connect (dev Pi only; customer units must be hardened)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    DRY = args.dry_run

    if not 1 <= args.site_id <= 0xFFFF:
        sys.exit("--site-id must be 1..65535")
    if not DRY and os.geteuid() != 0:
        sys.exit("run with sudo (or use --dry-run)")

    slug = slugify(args.site)
    home = Path(os.path.expanduser(f"~{args.user}"))
    hal_dir = Path(args.hal_dir) if args.hal_dir else home / "sx1302_hal_rpi5-master"

    old = load_existing_site()
    password = old.get("hotspot_password")
    if args.rotate_wifi or not password:
        password = random_password()
    site = {
        "site_name": slug,
        "site_id": args.site_id,
        "hotspot_ssid": f"ERS-{slug}",
        "hotspot_password": password,
        "hotspot_ip": HOTSPOT_IP,
        "server_port": 8081,
    }
    export = Path(args.export) if args.export else REPO_DIR / "data" / "exports" / f"site-{slug}.json"

    step_hostname(slug)
    if not args.skip_hotspot:
        step_hotspot(site)
    step_hal(hal_dir, args.skip_hal_build)
    step_python(args.user)
    step_config(site, args.user)
    step_units(args.user, hal_dir)
    step_export(site, export)
    if not args.skip_hardening:
        step_harden()

    log("done. Dashboard: http://ers-%s.local:8501 (LAN) or http://%s:8501 (hotspot)" % (slug, HOTSPOT_IP))
    log("Start services: sudo systemctl start " + " ".join(UNITS))
    if DRY:
        log("(dry run: nothing was changed)")


if __name__ == "__main__":
    main()
