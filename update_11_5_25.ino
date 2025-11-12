#include <Servo.h>

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

// --- Constants ---
int safeDistance = 30;   // Distance threshold (cm)
int forwardAngle = 90;   // Facing forward
int leftAngle = 150;     // Look left
int rightAngle = 30;     // Look right
int steeringCenter = 90;
int steeringLeft = 60;
int steeringRight = 120;

// --- Objects ---
Servo sensorServo;
Servo steeringServo;

// --- Function prototypes ---
int getDistance();
void scanAndAvoid();
void MoveForward();
void StopMotors();
void TurnLeft();
void TurnRight();

// --- Setup ---
void setup() {
  Serial.begin(9600);

  // Attach servos
  sensorServo.attach(SERVO_SENSOR_PIN);
  steeringServo.attach(SERVO_STEER_PIN);
  sensorServo.write(forwardAngle);   // look straight
  steeringServo.write(steeringCenter); // center steering

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

  Serial.println("Obstacle Avoidance Vehicle Started");
}

// --- Loop ---
void loop() {
  int distance = getDistance();
  Serial.print("Front Distance: ");
  Serial.println(distance);

  if (distance < safeDistance && distance > 0) {
    StopMotors();
    scanAndAvoid();  // Decide where to turn and move
  } else {
    MoveForward();
  }

  delay(150);
}

// --- Get ultrasonic distance ---
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 15000);
  int distance = duration * 0.034 / 2;
  return distance;
}

// --- Scan left and right, decide safest path ---
void scanAndAvoid() {
  int rightDist, leftDist;

  // Look right
  sensorServo.write(rightAngle);
  delay(400);
  rightDist = getDistance();

  // Look left
  sensorServo.write(leftAngle);
  delay(400);
  leftDist = getDistance();

  // Return to forward
  sensorServo.write(forwardAngle);
  delay(200);

  // Decide direction
  if (leftDist > rightDist) {
    TurnLeft();
    delay(1500);  // turn duration
  } else {
    TurnRight();
    delay(1500);
  }

  // Move forward a bit after turning
  MoveForward();
  delay(1000);

  // Stop and recenter
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
