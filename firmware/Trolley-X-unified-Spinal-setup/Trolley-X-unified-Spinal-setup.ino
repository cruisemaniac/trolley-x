/* Trolley-X motor, encoder and IMU controller. */

 #include <Adafruit_MPU6050.h>
 #include <Adafruit_Sensor.h>
 #include <Wire.h>

 Adafruit_MPU6050 mpu;

 const int EN_LEFT = 9;       const int IN_LEFT_FWD = 8;    const int IN_LEFT_REV = 7;   
const int ENCODER_LEFT_A = 2;  const int ENCODER_LEFT_B = 4;  

const int EN_RIGHT = 10;     const int IN_RIGHT_FWD = 6;   const int IN_RIGHT_REV = 5;  
const int ENCODER_RIGHT_A = 3; const int ENCODER_RIGHT_B = 11; 

// --- TUNING CONSTANTS ---
const int DRIVE_SPEED = 120; // Base PWM for straight driving
const int TURN_SPEED = 200;  // Goldilocks torque for skid-steering
const float METERS_PER_TICK_LEFT = (3.14159 * 0.065) / 332.0;
const float METERS_PER_TICK_RIGHT = (3.14159 * 0.065) / 325.0;

// --- SYSTEM STATE VARIABLES ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;
char currentCmd = 'X';

// Timing 
unsigned long lastLoopTime = 0;
const int LOOP_INTERVAL_MS = 50; // 20Hz Control & Telemetry Loop

// PID Reset Trackers
long leftTicksAtStart = 0;
long rightTicksAtStart = 0;

void setup() {
  Serial.begin(9600);
  
  // Hardware Setup
  pinMode(EN_LEFT, OUTPUT); pinMode(IN_LEFT_FWD, OUTPUT); pinMode(IN_LEFT_REV, OUTPUT);
  pinMode(EN_RIGHT, OUTPUT); pinMode(IN_RIGHT_FWD, OUTPUT); pinMode(IN_RIGHT_REV, OUTPUT);
  pinMode(ENCODER_LEFT_A, INPUT); pinMode(ENCODER_LEFT_B, INPUT);
  pinMode(ENCODER_RIGHT_A, INPUT); pinMode(ENCODER_RIGHT_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countRight, RISING);

  // Sensor Initialization
  if (!mpu.begin()) {
    Serial.println("ERR: IMU_NOT_FOUND");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  stopMotors();
}

void loop() {
  // ---------------------------------------------------------
  // 1. NON-BLOCKING LISTENER: Always catch commands instantly
  // ---------------------------------------------------------
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    
    // Only process valid commands to prevent garbage data glitches
    if (incoming == 'W' || incoming == 'A' || incoming == 'S' || incoming == 'D' || incoming == 'X') {
      
      // If we are switching from Stopping/Turning to driving Forward, reset the PID target
      if (incoming == 'W' && currentCmd != 'W') {
        leftTicksAtStart = leftTicks;
        rightTicksAtStart = rightTicks;
      }
      
      currentCmd = incoming;
    }
  }

  // ---------------------------------------------------------
  // 2. THE 20Hz HEARTBEAT: Control Motors & Transmit Data
  // ---------------------------------------------------------
  if (millis() - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime = millis();
    
    executeMotionState();
    transmitTelemetry();
  }
}

// --- CORE FUNCTIONS ---

void executeMotionState() {
  if (currentCmd == 'X') {
    stopMotors();
  }
  else if (currentCmd == 'W') {
    // STATE: FORWARD (PID Engaged to maintain perfectly straight line)
    digitalWrite(IN_LEFT_FWD, HIGH); digitalWrite(IN_LEFT_REV, LOW);
    digitalWrite(IN_RIGHT_FWD, HIGH); digitalWrite(IN_RIGHT_REV, LOW);

    float leftDist = (leftTicks - leftTicksAtStart) * METERS_PER_TICK_LEFT;
    float rightDist = (rightTicks - rightTicksAtStart) * METERS_PER_TICK_RIGHT;
    float error = leftDist - rightDist;

    // Apply proportional correction
    int leftPWM = constrain(DRIVE_SPEED - (error * 4000.0), 0, 160);
    int rightPWM = constrain(DRIVE_SPEED + (error * 4000.0), 0, 160);

    analogWrite(EN_LEFT, leftPWM);
    analogWrite(EN_RIGHT, rightPWM);
  }
  else if (currentCmd == 'S') {
    // STATE: REVERSE (PID Bypassed for simple backing up)
    digitalWrite(IN_LEFT_FWD, LOW); digitalWrite(IN_LEFT_REV, HIGH);
    digitalWrite(IN_RIGHT_FWD, LOW); digitalWrite(IN_RIGHT_REV, HIGH);
    analogWrite(EN_LEFT, DRIVE_SPEED); analogWrite(EN_RIGHT, DRIVE_SPEED);
  }
  else if (currentCmd == 'A') {
    // STATE: SPIN LEFT (PID Bypassed, High Torque)
    digitalWrite(IN_LEFT_FWD, LOW); digitalWrite(IN_LEFT_REV, HIGH);
    digitalWrite(IN_RIGHT_FWD, HIGH); digitalWrite(IN_RIGHT_REV, LOW);
    analogWrite(EN_LEFT, TURN_SPEED); analogWrite(EN_RIGHT, TURN_SPEED);
  }
  else if (currentCmd == 'D') {
    // STATE: SPIN RIGHT (PID Bypassed, High Torque)
    digitalWrite(IN_LEFT_FWD, HIGH); digitalWrite(IN_LEFT_REV, LOW);
    digitalWrite(IN_RIGHT_FWD, LOW); digitalWrite(IN_RIGHT_REV, HIGH);
    analogWrite(EN_LEFT, TURN_SPEED); analogWrite(EN_RIGHT, TURN_SPEED);
  }
}

void stopMotors() {
  digitalWrite(IN_LEFT_FWD, LOW); digitalWrite(IN_LEFT_REV, LOW);
  digitalWrite(IN_RIGHT_FWD, LOW); digitalWrite(IN_RIGHT_REV, LOW);
  analogWrite(EN_LEFT, 0); analogWrite(EN_RIGHT, 0);
}

void transmitTelemetry() {
  // Read physics
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Read total absolute distance traveled
  float leftTotalDist = leftTicks * METERS_PER_TICK_LEFT;
  float rightTotalDist = rightTicks * METERS_PER_TICK_RIGHT;

  // Print CSV (Left_Odometry, Right_Odometry, Forward_Acceleration, Yaw_Rotation)
  Serial.print(leftTotalDist, 4); Serial.print(",");
  Serial.print(rightTotalDist, 4); Serial.print(",");
  Serial.print(a.acceleration.x, 4); Serial.print(",");
  Serial.println(g.gyro.z, 4);
}

// --- HARDWARE INTERRUPTS ---
void countLeft() {
  if (digitalRead(ENCODER_LEFT_B) == HIGH) { leftTicks++; } else { leftTicks--; }
}
void countRight() {
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) { rightTicks--; } else { rightTicks++; }
}