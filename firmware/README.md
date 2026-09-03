# Firmware

Arduino Uno R3 low-level control. Pin map verified in
[../hardware/wiring.md](../hardware/wiring.md) (EN 9/10, IN 8/7/6/5; encoders
FL 2/4, FR 3/11 with phase A on the 2 & 3 interrupts). Serial is **9600 baud**.

## Sketches (progression)

| Sketch | Purpose |
| --- | --- |
| `initial-motor-run-test/` | Open-loop F/R/spin sequence - first drive test. |
| `encoder-test/` | Verifies tick counting + direction on both front encoders. |
| `pid-test/` | Closed-loop straight-line PID using asymmetric TPR (L 332 / R 325). |
| `teleoperation-wireless/` | Serial `W/A/S/D/X` listener (open loop). |
| `Trolley-X-unified-Spinal-setup/` | **Production firmware**: non-blocking teleop listener + context-aware PID + MPU6050 IMU + 20 Hz CSV telemetry. |

`Trolley-X-unified-Spinal-setup` is the one to flash for integrated runs. It
requires the Adafruit MPU6050 + Adafruit Unified Sensor libraries.

## !! PWM / 6 V-motor safety !!

The motors are 6 V; the L298N output rail is ~9.1 V. Forward PID is correctly
capped at `constrain(..., 0, 160)`. **But turns are not:** `TURN_SPEED = 200` in
the unified sketch, and `teleoperation-wireless` / `pid-test` write PWM up to
`255`. Sustained spins at 200-255 push ~7-9 V into 6 V motors and will overheat
them. Recommend capping every `analogWrite` (turns included) at **160** unless a
turn genuinely needs a brief torque burst - and even then, keep it short.

## Serial protocol
Single-char in (`W/A/S/D/X`), CSV telemetry out
(`left_m,right_m,accel_x,gyro_z`). See [../docs/interfaces.md](../docs/interfaces.md).
