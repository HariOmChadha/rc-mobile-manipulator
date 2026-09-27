#!/bin/bash
set -euo pipefail
source_rule=/home/czarhc/act-athon/work/serial-permissions/99-act-athon-serial.rules
destination_rule=/etc/udev/rules.d/99-act-athon-serial.rules
if [[ -e "$destination_rule" ]]; then
    cp -p "$destination_rule" "$destination_rule.backup.$(date +%s)"
fi
install -o root -g root -m 0644 "$source_rule" "$destination_rule"
udevadm control --reload-rules
udevadm trigger --action=change /sys/class/tty/ttyACM0 /sys/class/tty/ttyUSB0
udevadm settle --timeout=5
ls -l /dev/ttyACM0 /dev/ttyUSB0
