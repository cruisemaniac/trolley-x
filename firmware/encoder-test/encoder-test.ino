/* Trolley-X encoder verification test. */

// --- PIN DEFINITIONS ---

// Front-Left Encoder
const int ENCODER_LEFT_A = 2;  // Hardware Interrupt 0 (Counts the ticks)
const int ENCODER_LEFT_B = 4;  // Standard Digital (Determines direction)

// Front-Right Encoder
const int ENCODER_RIGHT_A = 3; // Hardware Interrupt 1 (Counts the ticks)
const int ENCODER_RIGHT_B = 11; // Standard Digital (Determines direction)

// --- VARIABLES ---
// Volatile because they are changed inside interrupt routines
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Timer for serial output
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Trolley-X Encoder Test Starting...");

  // Set encoder pins as inputs
  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);
  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);

  // Attach Hardware Interrupts
  // "RISING" means the interrupt triggers when the signal goes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countRight, RISING);
}

void loop() {
  // Print the tick counts to the Serial Monitor every 500 milliseconds
  if (millis() - lastPrintTime >= 500) {
    Serial.print("Left Ticks: ");
    Serial.print(leftTicks);
    Serial.print("  |  Right Ticks: ");
    Serial.println(rightTicks);
    
    lastPrintTime = millis();
  }
}

// --- INTERRUPT SERVICE ROUTINES (ISRs) ---

void countLeft() {
  // Check the state of Pin B to determine direction
  if (digitalRead(ENCODER_LEFT_B) == HIGH) {
    leftTicks++;   // Spinning Forward
  } else {
    leftTicks--;   // Spinning Reverse
  }
}

void countRight() {
  // Check the state of Pin B to determine direction
  // (Right motor might need reversed logic depending on mounting orientation)
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) {
    rightTicks--;  // Spinning Forward
  } else {
    rightTicks++;  // Spinning Reverse
  }
}