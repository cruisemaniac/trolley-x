/*
 * Trolley-X: UNIFIED Spinal Cord (v2 - blended differential drive)
 * Architecture: Non-Blocking State Machine, line-based serial parser.
 *
 * WHAT CHANGED FROM v1
 *  - New command "M <leftPWM> <rightPWM>": signed per-wheel PWM. This lets the
 *    Pi command drive and turn AT THE SAME TIME (arcs), not one or the other.
 *    This is the fix for "moves back and forth but does not follow".
 *  - PWM values are now honoured (variable speed), so the safety SLOW zone works.
 *  - A watchdog stops the motors if no command arrives within CMD_TIMEOUT_MS.
 *  - W/S/A/D/X still work for teleop and manual tests (back-compat).
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

const int EN_LEFT = 9;   const int IN_LEFT_FWD = 8;   const int IN_LEFT_REV = 7;
const int ENCODER_LEFT_A = 2;  const int ENCODER_LEFT_B = 4;

const int EN_RIGHT = 10; const int IN_RIGHT_FWD = 6;  const int IN_RIGHT_REV = 5;
const int ENCODER_RIGHT_A = 3; const int ENCODER_RIGHT_B = 11;

// --- TUNING CONSTANTS ---
const int   DRIVE_SPEED = 120;   // default PWM for a bare "W"
const int   TURN_SPEED  = 200;   // default PWM for a bare "A"/"D" spin
const int   MAX_PWM     = 200;   // hard ceiling on any wheel PWM (protects 6 V motors)
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

  pinMode(EN_LEFT, OUTPUT); pinMode(IN_LEFT_FWD, OUTPUT); pinMode(IN_LEFT_REV, OUTPUT);
  pinMode(EN_RIGHT, OUTPUT); pinMode(IN_RIGHT_FWD, OUTPUT); pinMode(IN_RIGHT_REV, OUTPUT);
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
  // First non-space char is the command; the rest are integer arguments.
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
    // W/S/A/D optional speed argument
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
    // Forward with encoder straight-line PID.
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

// Drive one side by a signed PWM. Positive = forward.
void setWheel(bool left, int pwm) {
  int mag = abs(pwm);
  if (mag > MAX_PWM) mag = MAX_PWM;
  if (left) {
    if (pwm > 0)      { digitalWrite(IN_LEFT_FWD, HIGH); digitalWrite(IN_LEFT_REV, LOW); }
    else if (pwm < 0) { digitalWrite(IN_LEFT_FWD, LOW);  digitalWrite(IN_LEFT_REV, HIGH); }
    else              { digitalWrite(IN_LEFT_FWD, LOW);  digitalWrite(IN_LEFT_REV, LOW); }
    analogWrite(EN_LEFT, mag);
  } else {
    if (pwm > 0)      { digitalWrite(IN_RIGHT_FWD, HIGH); digitalWrite(IN_RIGHT_REV, LOW); }
    else if (pwm < 0) { digitalWrite(IN_RIGHT_FWD, LOW);  digitalWrite(IN_RIGHT_REV, HIGH); }
    else              { digitalWrite(IN_RIGHT_FWD, LOW);  digitalWrite(IN_RIGHT_REV, LOW); }
    analogWrite(EN_RIGHT, mag);
  }
}

void stopMotors() {
  digitalWrite(IN_LEFT_FWD, LOW); digitalWrite(IN_LEFT_REV, LOW);
  digitalWrite(IN_RIGHT_FWD, LOW); digitalWrite(IN_RIGHT_REV, LOW);
  analogWrite(EN_LEFT, 0); analogWrite(EN_RIGHT, 0);
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
