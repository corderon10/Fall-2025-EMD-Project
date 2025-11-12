#include <Servo.h>
#include <IRremote.h>   // Include IR receiver library

// --- Pin definitions ---
#define TRIG_PIN        12
#define ECHO_PIN        13
#define SERVO_SENSOR_PIN 10   // Ultrasonic sensor servo
#define SERVO_STEER_PIN  9    // Steering servo (front wheels)

#define LEFT_MOTOR_EN   5
#define RIGHT_MOTOR_EN  6
#define LEFT_MOTOR_I1   4
#define LEFT_MOTOR_I2   2
#define RIGHT_MOTOR_I1  7
#define RIGHT_MOTOR_I2  8

#define IR_RECEIVE_PIN  11   // <-- IR receiver pin

// --- Constants ---
int safeDistance = 30;   // Distance threshold (cm)
int forwardAngle = 83;   // Facing forward
int leftAngle = 150;     // Look left
int rightAngle = 30;     // Look right
int steeringCenter = 90;
int steeringLeft = 60;
int steeringRight = 120;

// --- Timing control ---
unsigned long lastDistanceRead = 0;
int currentDistance = 0;
unsigned long readInterval = 100; // distance read every 100 ms

// --- Flags ---
bool robotStarted = false; // Robot will wait for IR signal

// --- Objects ---
Servo sensorServo;
Servo steeringServo;

// --- Function prototypes ---
int getDistance();
void scanAndAvoid();
void MoveForward();
void MoveBackward();
void StopMotors();
void TurnLeft();
void TurnRight();

// --- Setup ---
void setup() {
  Serial.begin(9600);

  // Attach servos
  sensorServo.attach(SERVO_SENSOR_PIN);
  steeringServo.attach(SERVO_STEER_PIN);
  sensorServo.write(forwardAngle);
  steeringServo.write(steeringCenter);

  // Set motor control pins
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_I1, OUTPUT);
  pinMode(LEFT_MOTOR_I2, OUTPUT);
  pinMode(RIGHT_MOTOR_I1, OUTPUT);
  pinMode(RIGHT_MOTOR_I2, OUTPUT);

  // Set ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Start IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  Serial.println("Waiting for IR button press to start...");
}

// --- Loop ---
void loop() {
  // Wait until a valid IR button is pressed
  if (!robotStarted) {
  if (IrReceiver.decode()) {
    unsigned long value = IrReceiver.decodedIRData.decodedRawData;

    // Debugging (optional): See what your remote sends
    Serial.print("IR Code: 0x");
    Serial.println(value, HEX);

    // Only start if the IR code matches a specific known button
    // Replace 0xBF40FF00 with your remote’s button code (you’ll find it using Serial Monitor)
    if (value == 0xBF40FF00) {  
      robotStarted = true;
      Serial.println("✅ IR button pressed — Robot starting!");
    }

    IrReceiver.resume();  // get ready for next code
  }

  StopMotors(); // stay still until started
  return;
}


  // --- Once started, continue as normal ---
  if (millis() - lastDistanceRead >= readInterval) {
    currentDistance = getDistance();
    lastDistanceRead = millis();
  }

  if (currentDistance < safeDistance && currentDistance > 0) {
    StopMotors();
    scanAndAvoid();
  } else {
    MoveForward();
  }
}

// --- Get ultrasonic distance (smoothed & faster) ---
int getDistance() {
  long sum = 0;
  int samples = 3;
  for (int i = 0; i < samples; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 8000);
    sum += duration * 0.034 / 2;
    delay(10);
  }
  return sum / samples;
}

// --- Scan left and right, decide safest path ---
void scanAndAvoid() {
  int rightDist, leftDist;

  MoveBackward();
  delay(2000);
  StopMotors();
  delay(150);

  sensorServo.write(rightAngle);
  delay(600);
  rightDist = getDistance();

  sensorServo.write(leftAngle);
  delay(600);
  leftDist = getDistance();

  sensorServo.write(forwardAngle);
  delay(600);

  if (leftDist > rightDist) {
    TurnLeft();
    delay(1500);
  } else {
    TurnRight();
    delay(1500);
  }

  MoveForward();
  delay(800);
  StopMotors();
  steeringServo.write(steeringCenter);
}

// --- Move forward ---
void MoveForward() {
  steeringServo.write(steeringCenter);
  digitalWrite(LEFT_MOTOR_I1, HIGH);
  digitalWrite(LEFT_MOTOR_I2, LOW);
  digitalWrite(RIGHT_MOTOR_I1, LOW);
  digitalWrite(RIGHT_MOTOR_I2, HIGH);
  analogWrite(LEFT_MOTOR_EN, 100);
  analogWrite(RIGHT_MOTOR_EN, 100);
}

// --- Move backward ---
void MoveBackward() {
  steeringServo.write(steeringCenter);
  digitalWrite(LEFT_MOTOR_I1, LOW);
  digitalWrite(LEFT_MOTOR_I2, HIGH);
  digitalWrite(RIGHT_MOTOR_I1, HIGH);
  digitalWrite(RIGHT_MOTOR_I2, LOW);
  analogWrite(LEFT_MOTOR_EN, 100);
  analogWrite(RIGHT_MOTOR_EN, 100);
}

// --- Turn left ---
void TurnLeft() {
  steeringServo.write(steeringLeft);
  digitalWrite(LEFT_MOTOR_I1, HIGH);
  digitalWrite(LEFT_MOTOR_I2, LOW);
  digitalWrite(RIGHT_MOTOR_I1, LOW);
  digitalWrite(RIGHT_MOTOR_I2, HIGH);
  analogWrite(LEFT_MOTOR_EN, 70);
  analogWrite(RIGHT_MOTOR_EN, 100);
}

// --- Turn right ---
void TurnRight() {
  steeringServo.write(steeringRight);
  digitalWrite(LEFT_MOTOR_I1, HIGH);
  digitalWrite(LEFT_MOTOR_I2, LOW);
  digitalWrite(RIGHT_MOTOR_I1, LOW);
  digitalWrite(RIGHT_MOTOR_I2, HIGH);
  analogWrite(LEFT_MOTOR_EN, 100);
  analogWrite(RIGHT_MOTOR_EN, 70);
}

// --- Stop all motors ---
void StopMotors() {
  digitalWrite(LEFT_MOTOR_I1, LOW);
  digitalWrite(LEFT_MOTOR_I2, LOW);
  digitalWrite(RIGHT_MOTOR_I1, LOW);
  digitalWrite(RIGHT_MOTOR_I2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}
