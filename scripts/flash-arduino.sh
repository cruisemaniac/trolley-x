#!/usr/bin/env bash
# Compile and upload the Uno firmware. Stop the ROS base node first.
set -euo pipefail

REPO="$HOME/dissertation-trolley-x"
SKETCH="${1:-$REPO/firmware/Trolley-X-unified-Spinal-setup}"
FQBN="arduino:avr:uno"
PORT="${PORT:-/dev/arduino}"

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "arduino-cli not found. Run the one-time setup in this script's header." >&2
  exit 1
fi
if [ ! -e "$PORT" ]; then
  echo "Port $PORT not found. Is the Arduino connected and the udev rule in place?" >&2
  exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser "$PORT" >/dev/null 2>&1; then
  echo "WARNING: $PORT is in use (a ROS node?). Stop it before flashing." >&2
  exit 1
fi

echo "Compiling $SKETCH ..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH"

echo "Uploading to $PORT (auto-reset, no unplug needed) ..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"

echo "Done. The Arduino restarts into the new firmware."
