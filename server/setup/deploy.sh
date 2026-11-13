#!/bin/bash
# Safe deploy of a branch onto the Pi.
#
#   server/setup/deploy.sh [branch]        (default: feature/device-registration)
#
# Why not just `git pull`? The Pi has local runtime files that older commits
# tracked (data/ers.sqlite, logs/emergency_log.jsonl, assets/maps/company_map.png).
# A commit that untracks them DELETES them from the working tree on checkout.
# So: stop -> back up -> checkout -> restore -> migrate -> start -> smoke test.
set -euo pipefail

BRANCH="${1:-feature/device-registration}"
ERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP="$HOME/ers-backups/$STAMP"
UNITS=(ers-dashboard ers-lora-bridge ers-server ers-lora-pkt-fwd)
PAYLOAD='{"device_id":"DEPLOY_SMOKE","emergency":false}'

cd "$ERS_DIR"

echo "== stopping services"
for u in "${UNITS[@]}"; do sudo systemctl stop "$u" 2>/dev/null || true; done

echo "== backing up to $BACKUP"
mkdir -p "$BACKUP"
for p in data logs assets/maps; do
    [ -e "$p" ] && cp -a "$p" "$BACKUP/$(echo "$p" | tr / _)"
done

echo "== checking out $BRANCH"
# Locally modified runtime files would make git refuse to delete them when the
# target branch untracks them. They are backed up above, so tell git to ignore
# their modifications for the switch (code edits elsewhere are still protected).
git update-index --assume-unchanged data/ers.sqlite logs/emergency_log.jsonl     assets/maps/company_map.png 2>/dev/null || true
git fetch --all --prune
git checkout "$BRANCH"
git pull --ff-only || true

echo "== restoring runtime data"
mkdir -p data logs assets/maps
[ -d "$BACKUP/data" ]        && cp -a "$BACKUP/data/."        data/
[ -d "$BACKUP/logs" ]        && cp -a "$BACKUP/logs/."        logs/
[ -d "$BACKUP/assets_maps" ] && cp -a "$BACKUP/assets_maps/." assets/maps/

echo "== installing requirements + migrating database"
if [ -x venv/bin/pip ]; then venv/bin/pip install -q -r requirements.txt; fi
PY=python3; [ -x venv/bin/python ] && PY=venv/bin/python
"$PY" - <<'PYEOF'
import sys
sys.path.insert(0, "server")
import database
database.init_db()          # idempotent: adds registration columns/tables
print("database migrated")
PYEOF

echo "== starting services"
sudo systemctl daemon-reload
for u in ers-server ers-lora-pkt-fwd ers-lora-bridge ers-dashboard; do
    sudo systemctl start "$u" || echo "WARNING: $u failed to start"
done

echo "== smoke test"
sleep 3
"$PY" - <<PYEOF
import socket, sys
try:
    with socket.create_connection(("127.0.0.1", 8081), timeout=3) as s:
        s.sendall(b'$PAYLOAD')
    print("server answering on 8081: OK")
except OSError as e:
    print("SMOKE TEST FAILED:", e); sys.exit(1)
PYEOF
systemctl is-active "${UNITS[@]}" || true
echo "== done. Backup kept at $BACKUP"
