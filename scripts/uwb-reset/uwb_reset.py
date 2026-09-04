#!/usr/bin/env python3
# Hold both RYUW122 NRST lines high after power-on.
import os
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")   # Pi 5 backend
import signal, time
from gpiozero import OutputDevice

NRST_PINS = [17, 27]   # BCM numbers, one per UWB module

# Assert reset, let the rail settle, then release both lines.
lines = [OutputDevice(p, active_high=True, initial_value=False) for p in NRST_PINS]
time.sleep(0.15)                 # hold reset ~150 ms
for ln in lines:
    ln.on()                      # HIGH = release
signal.pause()                   # keep process alive so the lines stay HIGH
