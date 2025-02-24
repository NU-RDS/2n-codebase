#!/bin/bash
# setup_teensy.sh
# This script creates a udev rule for the Teensy 4.1 device and reloads udev.

RULE_FILE="/etc/udev/rules.d/50-teensy.rules"
RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", MODE:="0666", SYMLINK+="teensy4"'

# Ensure the script is run as root
if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root. Try: sudo $0"
  exit 1
fi

echo "Creating udev rule at ${RULE_FILE}..."
echo "$RULE" > "$RULE_FILE"

echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "Udev rule installed and service reloaded successfully."
