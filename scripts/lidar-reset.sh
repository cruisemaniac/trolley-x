#!/usr/bin/env bash
# Reset the RPLIDAR USB port to clear the "Can not start scan: 8000800x" state
# that sllidar_node leaves behind when it is hard-killed. After the reset the
# device re-enumerates, so this WAITS for it to come back and settle before you
# launch (a launch fired too early times out with 80008000).
#
#   ./scripts/lidar-reset.sh          # uses /dev/lidar
#   ./scripts/lidar-reset.sh /dev/ttyUSB0
#
# Needs sudo (it resets a USB device).
set -e
DEV="${1:-/dev/lidar}"
sudo python3 - "$DEV" << 'PY'
import os, sys, fcntl
USBDEVFS_RESET = (ord('U') << 8) | 20
dev = sys.argv[1]
tty = os.path.realpath(dev)
name = os.path.basename(tty)
p = os.path.realpath(f"/sys/class/tty/{name}/device")
usb = None
while p and p != "/":
    if os.path.exists(f"{p}/busnum") and os.path.exists(f"{p}/devnum"):
        bus = int(open(f"{p}/busnum").read()); num = int(open(f"{p}/devnum").read())
        usb = f"/dev/bus/usb/{bus:03d}/{num:03d}"; break
    p = os.path.dirname(p)
if not usb:
    print(f"could not resolve USB node for {dev} ({tty})"); sys.exit(1)
print(f"resetting {usb}  (for {dev} -> {tty})")
fd = os.open(usb, os.O_WRONLY)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0); print("USB reset OK")
finally:
    os.close(fd)
PY

# wait for the device node to re-appear after re-enumeration
echo "waiting for $DEV to come back..."
for i in $(seq 1 30); do
  [ -e "$DEV" ] && break
  sleep 0.5
done
if [ ! -e "$DEV" ]; then
  echo "WARNING: $DEV did not re-appear; check the udev rule / cable"; exit 1
fi
# let the motor controller settle before the first start_scan
sleep 4
echo "LiDAR ready - launch now"
