#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
if [[ -x .venv/bin/python ]]; then
    robot_python=.venv/bin/python
elif [[ -x lerobot/act-athon/bin/python ]]; then
    robot_python=lerobot/act-athon/bin/python
else
    echo "No project environment found. On the Pi, run bash scripts/setup_pi.sh first." >&2
    exit 1
fi
exec "$robot_python" -m mobile_robot.launch "$@"
