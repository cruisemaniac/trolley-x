/* Trolley-X open-loop drive test. */

// --- PIN DEFINITIONS ---

// Left Side Motors (Top L298N)
const int EN_LEFT = 9;       // PWM Speed Control (ENA & ENB y-cabled)
const int IN_LEFT_FWD = 8;   // Forward Direction (IN1 & IN3 y-cabled)
const int IN_LEFT_REV = 7;   // Reverse Direction (IN2 & IN4 y-cabled)

// Right Side Motors (Bottom L298N)
const int EN_RIGHT = 10;     // PWM Speed Control (ENA & ENB y-cabled)
const int IN_RIGHT_FWD = 6;  // Forward Direction (IN1 & IN3 y-cabled)
const int IN_RIGHT_REV = 5;  // Reverse Direction (IN2 & IN4 y-cabled)

// Test Speed (0 to 255)
// 120 is fast enough to overcome the heavy gearboxes, but slow enough to be safe
const int TEST_SPEED = 120; 

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Trolley-X Initial Hardware Test...");

  // Set all motor control pins as outputs
  pinMode(EN_LEFT, OUTPUT);
  pinMode(IN_LEFT_FWD, OUTPUT);
  pinMode(IN_LEFT_REV, OUTPUT);
  
  pinMode(EN_RIGHT, OUTPUT);
  pinMode(IN_RIGHT_FWD, OUTPUT);
  pinMode(IN_RIGHT_REV, OUTPUT);

  // Ensure motors are completely stopped at startup
  stopMotors();
  delay(2000); 
}

void loop() {
  // --- TEST SEQUENCE ---
  
  Serial.println("1. Driving FORWARD");
  driveForward(TEST_SPEED);
  delay(2000);
  stopMotors();
  delay(1000);

  Serial.println("2. Driving REVERSE");
  driveReverse(TEST_SPEED);
  delay(2000);
  stopMotors();
  delay(1000);

  Serial.println("3. Zero-Turn LEFT");
  spinLeft(TEST_SPEED);
  delay(2000);
  stopMotors();
  delay(1000);

  Serial.println("4. Zero-Turn RIGHT");
  spinRight(TEST_SPEED);
  delay(2000);
  stopMotors();

  Serial.println("Sequence complete. Pausing for 10 seconds...");
  delay(10000); // Wait 10 seconds before repeating the loop
}


// --- MOTOR CONTROL FUNCTIONS ---

void driveForward(int speed) {
  // Left side spins forward
  digitalWrite(IN_LEFT_FWD, HIGH);
  digitalWrite(IN_LEFT_REV, LOW);
  // Right side spins forward
  digitalWrite(IN_RIGHT_FWD, HIGH);
  digitalWrite(IN_RIGHT_REV, LOW);
  
  // Apply power
  analogWrite(EN_LEFT, speed);
  analogWrite(EN_RIGHT, speed);
}

void driveReverse(int speed) {
  // Left side spins backward
  digitalWrite(IN_LEFT_FWD, LOW);
  digitalWrite(IN_LEFT_REV, HIGH);
  // Right side spins backward
  digitalWrite(IN_RIGHT_FWD, LOW);
  digitalWrite(IN_RIGHT_REV, HIGH);
  
  // Apply power
  analogWrite(EN_LEFT, speed);
  analogWrite(EN_RIGHT, speed);
}

void spinLeft(int speed) {
  // Left side spins backward
  digitalWrite(IN_LEFT_FWD, LOW);
  digitalWrite(IN_LEFT_REV, HIGH);
  // Right side spins forward
  digitalWrite(IN_RIGHT_FWD, HIGH);
  digitalWrite(IN_RIGHT_REV, LOW);
  
  // Apply power
  analogWrite(EN_LEFT, speed);
  analogWrite(EN_RIGHT, speed);
}

void spinRight(int speed) {
  // Left side spins forward
  digitalWrite(IN_LEFT_FWD, HIGH);
  digitalWrite(IN_LEFT_REV, LOW);
  // Right side spins backward
  digitalWrite(IN_RIGHT_FWD, LOW);
  digitalWrite(IN_RIGHT_REV, HIGH);
  
  // Apply power
  analogWrite(EN_LEFT, speed);
  analogWrite(EN_RIGHT, speed);
}

void stopMotors() {
  // Cut PWM power immediately
  analogWrite(EN_LEFT, 0);
  analogWrite(EN_RIGHT, 0);
  
  // Pull all logic pins LOW for safety
  digitalWrite(IN_LEFT_FWD, LOW);
  digitalWrite(IN_LEFT_REV, LOW);
  digitalWrite(IN_RIGHT_FWD, LOW);
  digitalWrite(IN_RIGHT_REV, LOW);
}