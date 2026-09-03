/* Trolley-X closed-loop drive test. */

// --- PIN DEFINITIONS ---
const int EN_LEFT = 9;       const int IN_LEFT_FWD = 8;    const int IN_LEFT_REV = 7;   
const int ENCODER_LEFT_A = 2;  const int ENCODER_LEFT_B = 4;  

const int EN_RIGHT = 10;     const int IN_RIGHT_FWD = 6;   const int IN_RIGHT_REV = 5;  
const int ENCODER_RIGHT_A = 3; const int ENCODER_RIGHT_B = 11; 

// --- ODOMETRY CONSTANTS ---
const float WHEEL_DIAMETER_M = 0.065; 
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER_M;

const float TPR_LEFT = 332.0;
const float TPR_RIGHT = 325.0;

const float METERS_PER_TICK_LEFT = WHEEL_CIRCUMFERENCE / TPR_LEFT;
const float METERS_PER_TICK_RIGHT = WHEEL_CIRCUMFERENCE / TPR_RIGHT;

// --- VARIABLES ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

int baseSpeed = 120; // Cruising PWM speed

void setup() {
  Serial.begin(9600);
  
  pinMode(EN_LEFT, OUTPUT); pinMode(IN_LEFT_FWD, OUTPUT); pinMode(IN_LEFT_REV, OUTPUT);
  pinMode(EN_RIGHT, OUTPUT); pinMode(IN_RIGHT_FWD, OUTPUT); pinMode(IN_RIGHT_REV, OUTPUT);
  
  pinMode(ENCODER_LEFT_A, INPUT); pinMode(ENCODER_LEFT_B, INPUT);
  pinMode(ENCODER_RIGHT_A, INPUT); pinMode(ENCODER_RIGHT_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countRight, RISING);

  // Start driving forward
  digitalWrite(IN_LEFT_FWD, HIGH); digitalWrite(IN_LEFT_REV, LOW);
  digitalWrite(IN_RIGHT_FWD, HIGH); digitalWrite(IN_RIGHT_REV, LOW);
}

void loop() {
  // 1. Calculate physical distance traveled by each side
  float leftDistance = leftTicks * METERS_PER_TICK_LEFT;
  float rightDistance = rightTicks * METERS_PER_TICK_RIGHT;

  // 2. Calculate the physical error in METERS
  float distanceError = leftDistance - rightDistance;

  // 3. Proportional Gain (Kp)
  // Because our error is now a tiny decimal (e.g. 0.01 meters), 
  // our Kp multiplier must be very large to create a meaningful PWM adjustment (0-255).
  float kp = 4000.0; 
  
  // 4. Calculate PWM adjustments
  int leftAdjustment = baseSpeed - (distanceError * kp);
  int rightAdjustment = baseSpeed + (distanceError * kp);

  // 5. Constrain to safe limits so we don't feed negative numbers or overload the driver
  leftAdjustment = constrain(leftAdjustment, 0, 255);
  rightAdjustment = constrain(rightAdjustment, 0, 255);

  // 6. Apply power
  analogWrite(EN_LEFT, leftAdjustment);
  analogWrite(EN_RIGHT, rightAdjustment);

  // Debug Output
  Serial.print("L-Dist: "); Serial.print(leftDistance, 4);
  Serial.print("m | R-Dist: "); Serial.print(rightDistance, 4);
  Serial.print("m || Err: "); Serial.print(distanceError, 4);
  Serial.print("m || L-PWM: "); Serial.print(leftAdjustment);
  Serial.print(" | R-PWM: "); Serial.println(rightAdjustment);

  delay(50); // Run at 20Hz
}

// --- INTERRUPTS ---
void countLeft() {
  if (digitalRead(ENCODER_LEFT_B) == HIGH) { leftTicks++; } else { leftTicks--; }
}

void countRight() {
  // Logic inverted for mirrored right-side motor mounting
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) { rightTicks--; } else { rightTicks++; }
}