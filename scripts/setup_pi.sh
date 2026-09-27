#!/usr/bin/env bash
# Run on the Pi after extracting the prepared bundle. Does not open USB devices.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
robot_python="${ROBOT_PYTHON:-python3}"
"$robot_python" -c 'import sys; assert sys.version_info >= (3, 12), "Python 3.12+ required; set ROBOT_PYTHON to its executable"'
if [[ ! -x .venv/bin/python ]]; then
    "$robot_python" -m venv .venv
fi
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-wireless.txt
.venv/bin/python -m pip install 'lerobot[feetech] @ git+https://github.com/huggingface/lerobot.git@e595b7902714ba51f91e47523f66f89c5181b649'
.venv/bin/python -m pip check
.venv/bin/python -c 'from lerobot.robots.so_follower import SO101Follower; import cv2, zmq, serial; print("Pi software imports passed. Run ./robot devices, confirm config/pi.local.json, then ./robot pi.")'
