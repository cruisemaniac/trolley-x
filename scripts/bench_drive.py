#!/usr/bin/env python3
"""Drivetrain bench test (raw serial, no ROS).

Runs the cart through forward, reverse, both spins and both arcs, so you can
confirm wheel direction and that drive+turn blend (the differential 'M' command).
It also reads the encoder telemetry during the forward move to prove the encoders
count.

SAFETY: put the cart on a stand with the wheels OFF the ground. The motors run
off the LiPo through the L298Ns, so the LiPo must be on and the E-stop released.

Stop any ROS base node first (it holds /dev/arduino):
    pkill -f arduino_base

Usage:
    python3 scripts/bench_drive.py                 # default /dev/arduino
    python3 scripts/bench_drive.py --port /dev/ttyACM0 --pwm 130
"""

import argparse
import sys
import time

import serial


PHASES = [
    ("M {f} {f}",   "FORWARD  - all four wheels forward"),
    ("M -{f} -{f}", "REVERSE  - all four wheels backward"),
    ("M -{t} {t}",  "SPIN LEFT  (CCW) - left back, right forward"),
    ("M {t} -{t}",  "SPIN RIGHT (CW)  - left forward, right back"),
    ("M {f} {s}",   "ARC RIGHT - left wheels faster than right"),
    ("M {s} {f}",   "ARC LEFT  - right wheels faster than left"),
]


def parse_telemetry(line):
    parts = line.split(',')
    if len(parts) != 4:
        return None
    try:
        return tuple(float(p) for p in parts)   # left_m, right_m, accel_x, gyro_z
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/arduino')
    ap.add_argument('--baud', type=int, default=9600)
    ap.add_argument('--pwm', type=int, default=120, help='forward/reverse PWM')
    ap.add_argument('--turn', type=int, default=150, help='spin PWM (needs breakaway torque)')
    ap.add_argument('--slow', type=int, default=70, help='inner-wheel PWM for arcs')
    ap.add_argument('--secs', type=float, default=2.0, help='seconds per phase')
    args = ap.parse_args()

    try:
        s = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as e:
        print(f"cannot open {args.port}: {e}"); sys.exit(1)
    time.sleep(2.0)  # Arduino resets when the port opens

    print("WHEELS ON THE STAND, LiPo ON, E-stop RELEASED.")
    for n in (3, 2, 1):
        print(f"  starting in {n}...", end='\r'); time.sleep(1.0)
    print(" " * 30)

    last_tel = None

    def send(cmd):
        nonlocal last_tel
        s.write((cmd + "\n").encode())
        line = s.readline().decode(errors='ignore').strip()
        tel = parse_telemetry(line)
        if tel:
            last_tel = tel

    def drive(cmd, label):
        print(f">>> {label}: '{cmd}'")
        start_tel = last_tel
        end = time.time() + args.secs
        while time.time() < end:
            send(cmd); time.sleep(0.1)   # 10 Hz beats the 600 ms firmware watchdog
        for _ in range(6):               # stop and let it settle
            send("X"); time.sleep(0.05)
        time.sleep(0.8)
        return start_tel, last_tel

    try:
        for tmpl, label in PHASES:
            cmd = tmpl.format(f=args.pwm, t=args.turn, s=args.slow)
            a, b = drive(cmd, label)
            if label.startswith("FORWARD") and a and b:
                dl, dr = b[0] - a[0], b[1] - a[1]
                print(f"    encoders: left {dl:+.3f} m, right {dr:+.3f} m "
                      f"({'OK - both moved' if dl > 0.01 and dr > 0.01 else 'CHECK - a side did not count'})")
        print("done - all phases sent")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        for _ in range(6):
            s.write(b"X\n"); time.sleep(0.05)
        s.close()


if __name__ == '__main__':
    main()
