#!/usr/bin/env bash
# usage (root): capture-run.sh <set-name> <touches> <print-file>
# Captures 3-frame bursts per touch into /root/g5125-capture/<set-name>.
set -euo pipefail
proj=$(cd "$(dirname "$0")/.." && pwd)
out=/root/g5125-capture/$1
install -d -m700 /root/g5125-capture "$out"
systemctl stop fprintd.service 2>/dev/null || true
export GOODIX5125_PSK_FILE=/var/lib/fprint/goodix5125-materials/psk.bin
export GOODIX5125_ENGINE_DLL=/usr/lib/libfprint-2/goodix5125/AlgoChicago.dll
export GOODIX5125_CALIBRATION_FILE=/var/lib/fprint/goodix5125-materials/calibration.dat
export GOODIX5125_BASELINE_FILE=/var/lib/fprint/goodix5125-materials/baseline.raw
export GOODIX5125_CAPTURE_DIR=$out GOODIX5125_BURST=3
export G_MESSAGES_DEBUG=libfprint-goodixtls5125
export LD_LIBRARY_PATH=$proj/build/libfprint
exec "$proj/build/g5125-capture" "${3:?print file required}" "$2"
