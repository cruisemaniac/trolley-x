/*
 * Trolley-X: UNIFIED Spinal Cord (v3 - BTS7960 differential drive)
 * Architecture: Non-Blocking State Machine, line-based serial parser.
 *
 * WHAT CHANGED FROM v2 (L298N -> BTS7960)
 *  - Motor drivers are now 2x BTS7960, one per side. Each side's two motors
 *    are wired in PARALLEL on that module's M+/M-.
 *  - Each side is driven by two PWM lines, RPWM and LPWM (no EN/IN pins).
 *    RPWM drives one direction, LPWM the other. Only ONE is ever non-zero,
 *    so the H-bridge never shoot-throughs.
 *  - R_EN + L_EN on both modules are tied to 5 V (always enabled), so no
 *    Arduino enable pin is used. Motion still stops via PWM=0 (watchdog / X)
 *    and the hardware E-stop cuts the motor rail.
 *  - MAX_PWM dropped to 120. The BTS7960 drops ~0.1 V (vs the L298N's ~2-3 V),
 *    so the motor now sees nearly the full rail. 120/255 keeps a 6 V motor at
 *    about 6 V even on a full 3S pack. DO NOT raise this without checking the
 *    motor voltage, or you will cook the 6 V motors.
 *  - Pin map: LEFT  RPWM=9  LPWM=10 ; RIGHT RPWM=5 LPWM=6. Encoders unchanged.
 *
 * SERIAL PROTOCOL (Pi -> Arduino), one command per line, '\n' terminated:
 *   X                stop
 *   M <left> <right> signed per-wheel PWM, e.g. "M 120 -80" (arc/spin/drive)
 *   W <pwm>          forward straight (encoder PID keeps a straight line)
 *   S <pwm>          reverse both wheels
 *   A <pwm>          spin left in place
 *   D <pwm>          spin right in place
 *   (a bare W/S/A/D with no number uses the default speed)
 *
 * TELEMETRY (Arduino -> Pi, 20 Hz): CSV "left_m,right_m,accel_x,gyro_z"
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// --- MOTOR DRIVER PINS: 2x BTS7960 (RPWM/LPWM per side). Forward = RPWM. ---
const int L_RPWM = 9;   const int L_LPWM = 10;   // LEFT module
const int R_RPWM = 5;   const int R_LPWM = 6;    // RIGHT module
const int ENCODER_LEFT_A = 2;  const int ENCODER_LEFT_B = 4;
const int ENCODER_RIGHT_A = 3; const int ENCODER_RIGHT_B = 11;

// --- TUNING CONSTANTS ---
const int   DRIVE_SPEED = 100;   // default PWM for a bare "W"
const int   TURN_SPEED  = 120;   // default PWM for a bare "A"/"D" spin
const int   MAX_PWM     = 120;   // ceiling: BTS7960 ~0 drop, keeps 6 V motors near 6 V
const float METERS_PER_TICK_LEFT  = (3.14159 * 0.065) / 332.0;
const float METERS_PER_TICK_RIGHT = (3.14159 * 0.065) / 325.0;
const unsigned long CMD_TIMEOUT_MS = 600;   // watchdog: stop if no command in this window

// --- SYSTEM STATE ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

char currentCmd = 'X';
int  cmdLeftPwm = 0;             // used by 'M'
int  cmdRightPwm = 0;            // used by 'M'
int  cmdSpeed = 0;               // used by W/S/A/D (0 => use default)
unsigned long lastCmdTime = 0;

// Line buffer
const int BUF_LEN = 24;
char buf[BUF_LEN];
int  bufIdx = 0;

// Timing
unsigned long lastLoopTime = 0;
const int LOOP_INTERVAL_MS = 50; // 20 Hz control + telemetry

// Straight-line PID trackers (for 'W')
long leftTicksAtStart = 0;
long rightTicksAtStart = 0;

void setup() {
  Serial.begin(9600);

  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0);
  pinMode(ENCODER_LEFT_A, INPUT); pinMode(ENCODER_LEFT_B, INPUT);
  pinMode(ENCODER_RIGHT_A, INPUT); pinMode(ENCODER_RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countRight, RISING);

  if (!mpu.begin()) {
    Serial.println("ERR: IMU_NOT_FOUND");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  stopMotors();
  lastCmdTime = millis();
}

void loop() {
  // 1. NON-BLOCKING LISTENER: read a full line, then parse it.
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) { buf[bufIdx] = '\0'; parseLine(buf); bufIdx = 0; }
    } else if (bufIdx < BUF_LEN - 1) {
      buf[bufIdx++] = c;
    }
  }

  // 2. 20 Hz HEARTBEAT: watchdog, drive, telemetry.
  if (millis() - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime = millis();
    if (millis() - lastCmdTime > CMD_TIMEOUT_MS) { currentCmd = 'X'; }  // watchdog
    executeMotionState();
    transmitTelemetry();
  }
}

// --- COMMAND PARSER ---
void parseLine(char *line) {
  char cmd = line[0];
  if (cmd == 'X' || cmd == 'W' || cmd == 'S' || cmd == 'A' || cmd == 'D' || cmd == 'M') {
    if (cmd == 'M') {
      int l = 0, r = 0;
      if (sscanf(line + 1, "%d %d", &l, &r) == 2) {
        cmdLeftPwm = clampPwm(l);
        cmdRightPwm = clampPwm(r);
        currentCmd = 'M';
        lastCmdTime = millis();
      }
      return;
    }
    int v = 0;
    cmdSpeed = (sscanf(line + 1, "%d", &v) == 1) ? clampPwm(v) : 0;
    if (cmd == 'W' && currentCmd != 'W') {   // reset PID datum when entering forward
      leftTicksAtStart = leftTicks;
      rightTicksAtStart = rightTicks;
    }
    currentCmd = cmd;
    lastCmdTime = millis();
  }
}

int clampPwm(int v) {
  if (v > MAX_PWM) return MAX_PWM;
  if (v < -MAX_PWM) return -MAX_PWM;
  return v;
}

// --- MOTION ---
void executeMotionState() {
  if (currentCmd == 'X') {
    stopMotors();
  }
  else if (currentCmd == 'M') {
    setWheel(true, cmdLeftPwm);     // left
    setWheel(false, cmdRightPwm);   // right
  }
  else if (currentCmd == 'W') {
    int base = (cmdSpeed > 0) ? cmdSpeed : DRIVE_SPEED;
    float leftDist  = (leftTicks - leftTicksAtStart) * METERS_PER_TICK_LEFT;
    float rightDist = (rightTicks - rightTicksAtStart) * METERS_PER_TICK_RIGHT;
    float error = leftDist - rightDist;
    int leftPWM  = constrain(base - (int)(error * 4000.0), 0, MAX_PWM);
    int rightPWM = constrain(base + (int)(error * 4000.0), 0, MAX_PWM);
    setWheel(true, leftPWM);
    setWheel(false, rightPWM);
  }
  else if (currentCmd == 'S') {
    int s = (cmdSpeed > 0) ? cmdSpeed : DRIVE_SPEED;
    setWheel(true, -s); setWheel(false, -s);
  }
  else if (currentCmd == 'A') {         // spin left: left back, right fwd
    int s = (cmdSpeed > 0) ? cmdSpeed : TURN_SPEED;
    setWheel(true, -s); setWheel(false, +s);
  }
  else if (currentCmd == 'D') {         // spin right: left fwd, right back
    int s = (cmdSpeed > 0) ? cmdSpeed : TURN_SPEED;
    setWheel(true, +s); setWheel(false, -s);
  }
}

// Drive one side by a signed PWM. Positive = forward (RPWM). Only one of the
// two PWM inputs is ever non-zero, so the H-bridge never shoot-throughs.
void setWheel(bool left, int pwm) {
  int mag = abs(pwm);
  if (mag > MAX_PWM) mag = MAX_PWM;
  int rpwm = (pwm > 0) ? mag : 0;
  int lpwm = (pwm < 0) ? mag : 0;
  if (left) {
    analogWrite(L_RPWM, rpwm); analogWrite(L_LPWM, lpwm);
  } else {
    analogWrite(R_RPWM, rpwm); analogWrite(R_LPWM, lpwm);
  }
}

void stopMotors() {
  analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0);
}

void transmitTelemetry() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float leftTotalDist  = leftTicks * METERS_PER_TICK_LEFT;
  float rightTotalDist = rightTicks * METERS_PER_TICK_RIGHT;
  Serial.print(leftTotalDist, 4); Serial.print(",");
  Serial.print(rightTotalDist, 4); Serial.print(",");
  Serial.print(a.acceleration.x, 4); Serial.print(",");
  Serial.println(g.gyro.z, 4);
}

// --- ENCODER INTERRUPTS ---
void countLeft() {
  if (digitalRead(ENCODER_LEFT_B) == HIGH) { leftTicks++; } else { leftTicks--; }
}
void countRight() {
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) { rightTicks--; } else { rightTicks++; }
}
