/* Trolley-X serial teleoperation listener. */

// --- PIN DEFINITIONS ---

// Left Side Motors (Top L298N)
const int EN_LEFT = 9;       // PWM Speed Control (ENA & ENB y-cabled)
const int IN_LEFT_FWD = 8;   // Forward Direction (IN1 & IN3 y-cabled)
const int IN_LEFT_REV = 7;   // Reverse Direction (IN2 & IN4 y-cabled)

// Right Side Motors (Bottom L298N) - CORRECTED TO USER SPECS
const int EN_RIGHT = 10;     // PWM Speed Control (ENA & ENB y-cabled)
const int IN_RIGHT_FWD = 6;  // Forward Direction (IN1 & IN3 y-cabled)
const int IN_RIGHT_REV = 5;  // Reverse Direction (IN2 & IN4 y-cabled)

// Encoders (Documented here so we don't forget the layout for the PID later)
const int ENCODER_LEFT_A = 2;   // Left Yellow
const int ENCODER_LEFT_B = 4;   // Left Green
const int ENCODER_RIGHT_A = 3;  // Right Yellow (Hardware Interrupt 1)
const int ENCODER_RIGHT_B = 11; // Right Green (Safely moved to Pin 11)

const int TEST_SPEED = 120;  // Safe cruising speed

void setup() {
  Serial.begin(9600);
  
  pinMode(EN_LEFT, OUTPUT);
  pinMode(IN_LEFT_FWD, OUTPUT);
  pinMode(IN_LEFT_REV, OUTPUT);
  
  pinMode(EN_RIGHT, OUTPUT);
  pinMode(IN_RIGHT_FWD, OUTPUT);
  pinMode(IN_RIGHT_REV, OUTPUT);

  stopMotors();
}

void loop() {
  // Listen for commands from the Raspberry Pi
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'W') driveForward(TEST_SPEED);
    else if (cmd == 'S') driveReverse(TEST_SPEED);
    else if (cmd == 'A') spinLeft(255);
    else if (cmd == 'D') spinRight(255);
    else if (cmd == 'X') stopMotors();
  }
}

// --- MOTOR CONTROL FUNCTIONS ---
void driveForward(int speed) {
  digitalWrite(IN_LEFT_FWD, HIGH); digitalWrite(IN_LEFT_REV, LOW);
  digitalWrite(IN_RIGHT_FWD, HIGH); digitalWrite(IN_RIGHT_REV, LOW);
  analogWrite(EN_LEFT, speed); analogWrite(EN_RIGHT, speed);
}

void driveReverse(int speed) {
  digitalWrite(IN_LEFT_FWD, LOW); digitalWrite(IN_LEFT_REV, HIGH);
  digitalWrite(IN_RIGHT_FWD, LOW); digitalWrite(IN_RIGHT_REV, HIGH);
  analogWrite(EN_LEFT, speed); analogWrite(EN_RIGHT, speed);
}

void spinLeft(int speed) {
  digitalWrite(IN_LEFT_FWD, LOW); digitalWrite(IN_LEFT_REV, HIGH);
  digitalWrite(IN_RIGHT_FWD, HIGH); digitalWrite(IN_RIGHT_REV, LOW);
  analogWrite(EN_LEFT, speed); analogWrite(EN_RIGHT, speed);
}

void spinRight(int speed) {
  digitalWrite(IN_LEFT_FWD, HIGH); digitalWrite(IN_LEFT_REV, LOW);
  digitalWrite(IN_RIGHT_FWD, LOW); digitalWrite(IN_RIGHT_REV, HIGH);
  analogWrite(EN_LEFT, speed); analogWrite(EN_RIGHT, speed);
}

void stopMotors() {
  analogWrite(EN_LEFT, 0); analogWrite(EN_RIGHT, 0);
  digitalWrite(IN_LEFT_FWD, LOW); digitalWrite(IN_LEFT_REV, LOW);
  digitalWrite(IN_RIGHT_FWD, LOW); digitalWrite(IN_RIGHT_REV, LOW);
}