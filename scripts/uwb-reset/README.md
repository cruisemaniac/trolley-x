# UWB power-on reset

## Problem

Each REYAX RYUW122_Lite is silent on serial after boot until NRST gets a clean
low-to-high edge, and NRST must then be held high. The module has no reliable
internal pull-up, so a floating NRST does not work. This service drives that edge
for every anchor at boot and holds each line high, removing the manual step.

## Anchors and pins

Two anchors, one per front pillar, each on its own Pi UART, NRST on a GPIO the
service can pulse:

| Anchor | UART / device | NRST pin |
| --- | --- | --- |
| RHS | UART0 `/dev/ttyAMA0` (GPIO14/15) | GPIO17 = physical pin 11 |
| LHS | UART2 `/dev/ttyAMA2` (GPIO4/5) | GPIO27 = physical pin 13 |

The NRST BCM numbers live in `NRST_PINS` in `uwb_reset.py`; add or remove pins
there if the anchor count changes. Full pin tables: `hardware/wiring.md` section 6.

## Boot config

UART2 needs its Pi 5 overlay. In `/boot/firmware/config.txt`:

```
enable_uart=1
dtoverlay=uart2-pi5
```

`disable-bt` is not needed on the Pi 5. Do not use `uart3-pi5` (GPIO8/9) - it
clashes with SPI0.

## Dependencies

```
sudo apt install -y python3-gpiozero python3-lgpio
```

## Install (on the Pi)

```
sudo mkdir -p /opt/trolley-x
sudo cp uwb_reset.py /opt/trolley-x/
sudo cp uwb-reset.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now uwb-reset.service
```

## Test

1. Reboot, or `sudo systemctl restart uwb-reset.service`.
2. The RYUW122 is command-driven - it does not stream, so a passive `cat` shows
   nothing even when healthy. Test with an AT command:

```
python3 - <<'PY'
import serial, time
for dev in ('/dev/ttyAMA0', '/dev/ttyAMA2'):
    p = serial.Serial(dev, 115200, timeout=1)
    p.write(b'AT\r\n'); time.sleep(0.3)
    print(dev, '->', p.read(200)); p.close()
PY
```

   Each device should return `+OK`, with no manual NRST pulse.
3. If a module is mute, increase the hold time (`time.sleep(0.15)` -> `0.3`) and
   retest.

## Status

Verified on hardware (Pi 5, Ubuntu 24.04): both anchors answer `AT` -> `+OK` on
`ttyAMA0` and `ttyAMA2` with the service running.
