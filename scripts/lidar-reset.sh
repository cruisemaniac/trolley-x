#!/usr/bin/env bash
# Reset the RPLIDAR USB port to clear the "Can not start scan: 80008002" state
# that sllidar_node leaves behind when it is hard-killed (it ignores SIGINT and
# gets SIGTERM'd, so the scan/motor are never stopped cleanly).
#
# Run this before a launch that errored on the LiDAR:
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
tty = os.path.realpath(dev)                      # /dev/lidar -> /dev/ttyUSBx
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
echo "done - wait ~2 s, then launch"
