#!/bin/bash
# Reset pulse for the SX1302/SX1303 concentrator HAT (BCM GPIO23, power-enable GPIO18).
#
# Run as ExecStartPre of ers-lora-pkt-fwd so a hung concentrator recovers on every
# (re)start. Mirrors Semtech's reset_lgw.sh sequence.
#
# NOT YET VERIFIED ON HARDWARE: "lora radio ref.txt" only shows that the HAL
# folder contains its own reset_lgw.sh (lora_pkt_fwd also calls ./reset_lgw.sh
# itself). If that script exists it is preferred; the pinctrl sequence below is
# the fallback taken from the stock Semtech script for the Pi 5 (pinctrl).
# Check pin numbers against the HAT before relying on this.

set -u
RESET_PIN="${ERS_LGW_RESET_PIN:-23}"
POWER_PIN="${ERS_LGW_POWER_PIN:-18}"
HAL_DIR="${ERS_HAL_DIR:-$HOME/sx1302_hal_rpi5-master}"

if [ -x "$HAL_DIR/packet_forwarder/reset_lgw.sh" ]; then
    cd "$HAL_DIR/packet_forwarder" && exec ./reset_lgw.sh start
fi

if ! command -v pinctrl >/dev/null 2>&1; then
    echo "reset_concentrator: neither reset_lgw.sh nor pinctrl found; skipping reset" >&2
    exit 0   # do not block the service: the forwarder will try its own reset
fi

pinctrl set "$POWER_PIN" op dh
pinctrl set "$RESET_PIN" op dh
sleep 0.1
pinctrl set "$RESET_PIN" op dl
sleep 0.1
echo "reset_concentrator: reset pulse sent (reset=GPIO$RESET_PIN power=GPIO$POWER_PIN)"
