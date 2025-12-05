#include <Servo.h>

// --------- USER SETTINGS ---------
const int trigPin          = 6;   // Ultrasonic TRIG pin
const int echoPin          = 7;   // Ultrasonic ECHO pin
const int servoPin         = 9;   // Main servo signal pin
const int smallservoPin    = 10;  // Small (second) servo signal pin
const int buttonPin        = 2;   // Button pin (wired to GND, using INPUT_PULLUP)

const int TRIGGER_DISTANCE_CM = 15;   // Distance threshold to trigger servo
const int SERVO_HOME_ANGLE    = 0;    // Starting/resting angle
const int SERVO_TRIGGER_ANGLE = 70;   // Angle when object is detected
int currentAngle = SERVO_HOME_ANGLE;

const int SMALL_SERVO_CENTER_ANGLE = 45;  // Resting position for small servo
const int SMALL_SERVO_LEFT_ANGLE   = 90;  // Left limit
const int SMALL_SERVO_RIGHT_ANGLE  = 45;  // Right limit

const unsigned long RESET_WAIT_MS = 3000;  // Wait time after returning home (ms)
// ---------------------------------

Servo myServo;
Servo smallServo;

enum State {
  WAIT_FOR_OBJECT,
  WAIT_FOR_BUTTON,
  WAIT_AFTER_RESET
};

State currentState = WAIT_FOR_OBJECT;
unsigned long resetStartTime = 0;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);  // Button to GND, internal pull-up enabled

  // Main servo
  myServo.attach(servoPin);
  myServo.write(SERVO_HOME_ANGLE);   // Move to home position at startup
  currentAngle = SERVO_HOME_ANGLE;

  // Small servo
  smallServo.attach(smallservoPin);
  smallServo.write(SMALL_SERVO_CENTER_ANGLE);  // Start centered
}

void loop() {
  switch (currentState) {
    case WAIT_FOR_OBJECT:
      handleWaitForObject();
      break;

    case WAIT_FOR_BUTTON:
      handleWaitForButton();
      break;

    case WAIT_AFTER_RESET:
      handleWaitAfterReset();
      break;
  }
}

// --------- SMOOTH MOVE FOR MAIN SERVO ---------
void moveServoSmooth(int targetAngle, int stepDelayMs = 15) {
  if (targetAngle == currentAngle) return;

  int step = (targetAngle > currentAngle) ? 1 : -1;

  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    myServo.write(angle);
    delay(stepDelayMs);  // Smaller = faster, larger = slower
  }

  myServo.write(targetAngle);  // Make sure we land exactly on it
  currentAngle = targetAngle;
}

// --------- BASE WIGGLE FUNCTION FOR SMALL SERVO (UNCHANGED MOTION) ---------
void wiggleSmallServo(int repeats = 4, int stepDelayMs = 300) {
  // Quick back-and-forth motion
  for (int i = 0; i < repeats; i++) {
    smallServo.write(SMALL_SERVO_LEFT_ANGLE);
    delay(stepDelayMs);
    smallServo.write(SMALL_SERVO_RIGHT_ANGLE);
    delay(stepDelayMs);
  }
  // Return to center at the end
  smallServo.write(SMALL_SERVO_CENTER_ANGLE);
}

// --------- AUDIO-TAGGED WIGGLE WRAPPERS ---------
// Use this after detection → MP3 #1
void wiggleSmallServoDetect(int repeats = 11, int stepDelayMs = 300) {
  Serial.println("AUDIO_DETECT_START");  // For PC script

  wiggleSmallServo(repeats, stepDelayMs);

  Serial.println("AUDIO_DETECT_STOP");
}

// Use this after button press → MP3 #2
void wiggleSmallServoButton(int repeats = 15, int stepDelayMs = 300) {
  Serial.println("AUDIO_BUTTON_START");  // For PC script

  wiggleSmallServo(repeats, stepDelayMs);

  Serial.println("AUDIO_BUTTON_STOP");
}

// ---------------- STATE HANDLERS ----------------

void handleWaitForObject() {
  long distance = readDistanceCm();

  if (distance > 0) {  // Valid reading
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  if (distance > 0 && distance <= TRIGGER_DISTANCE_CM) {
    // Object is close enough → move main servo to trigger position
    Serial.println("Object detected! Moving main servo to trigger position.");
    moveServoSmooth(SERVO_TRIGGER_ANGLE, 30);

    // After main servo finishes, wiggle the small servo quickly
    Serial.println("Wiggling small servo after detection.");
    // <-- behavior unchanged: still 4 repeats, 300 ms delay
    wiggleSmallServoDetect(11, 300);

    currentState = WAIT_FOR_BUTTON;
  }

  delay(50);  // Small delay to avoid spamming sensor
}

void handleWaitForButton() {
  // Button is wired to GND, so LOW means "pressed"
  if (isButtonPressed()) {
    Serial.println("Button pressed!");

    // First: wiggle the small servo quickly again
    Serial.println("Wiggling small servo before returning main servo home.");
    // <-- behavior unchanged: still 4 repeats, 300 ms delay
    wiggleSmallServoButton(15, 300);

    // Then: return main servo to home position
    Serial.println("Returning main servo to home position.");
    moveServoSmooth(SERVO_HOME_ANGLE, 15);

    // Start wait timer
    resetStartTime = millis();
    currentState   = WAIT_AFTER_RESET;
  }
}

void handleWaitAfterReset() {
  // Wait a few seconds before re-arming the ultrasonic trigger
  if (millis() - resetStartTime >= RESET_WAIT_MS) {
    Serial.println("Wait finished. Ready to detect object again.");
    currentState = WAIT_FOR_OBJECT;
  }
}

// ---------------- HELPER FUNCTIONS ----------------

// Read distance from ultrasonic sensor in cm
long readDistanceCm() {
  // Trigger the ultrasonic burst
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo time in microseconds
  long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout (~5 meters)

  if (duration == 0) {
    // No echo received within timeout
    return -1;
  }

  // Convert time to distance (cm)
  long distanceCm = duration / 58.0;
  return distanceCm;
}

// Simple button press detection with basic debounce
bool isButtonPressed() {
  if (digitalRead(buttonPin) == LOW) {  // Initial read: pressed
    delay(50);                          // Debounce delay
    if (digitalRead(buttonPin) == LOW) {
      // Wait until button is released so we only count one press
      while (digitalRead(buttonPin) == LOW) {
        delay(10);
      }
      return true;
    }
  }
  return false;
}
