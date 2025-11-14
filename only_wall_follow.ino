/*
  Author: Team 7
  Date: 11/13/25
  About: Wall-Following Robot (Right-Side Following)
*/

#include <Servo.h>
#include <IRremote.h>

// ---------------- Pin Definitions ----------------
#define TRIG_PIN        12
#define ECHO_PIN        13
#define SERVO_SENSOR_PIN 10
#define SERVO_STEER_PIN  9

#define LEFT_MOTOR_EN   5
#define RIGHT_MOTOR_EN  6
#define LEFT_MOTOR_I1   4
#define LEFT_MOTOR_I2   2
#define RIGHT_MOTOR_I1  7
#define RIGHT_MOTOR_I2  8

#define IR_RECEIVE_PIN  11

// ---------------- Constants ----------------
int forwardAngle     = 83;    // Forward looking for servo
int rightScan        = 30;    // Right wall scan direction
int steeringCenter   = 90;    // Straight wheels
int targetDistance   = 20;    // Desired wall distance (cm)
int scanInterval     = 250;   // ms between distance checks

// ---------------- State ----------------
bool robotStarted = false;
unsigned long lastScanTime = 0;

// ---------------- Objects ----------------
Servo sensorServo;
Servo steeringServo;

// --------------- Distance Function ---------------
int getDistance() {
  long duration;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 8000);
  int distance = duration * 0.034 / 2;

  return distance;
}

// ---------------- Motor Control ----------------
void MoveForward() {
  digitalWrite(LEFT_MOTOR_I1, HIGH);
  digitalWrite(LEFT_MOTOR_I2, LOW);

  digitalWrite(RIGHT_MOTOR_I1, LOW);
  digitalWrite(RIGHT_MOTOR_I2, HIGH);

  analogWrite(LEFT_MOTOR_EN, 70);
  analogWrite(RIGHT_MOTOR_EN, 70);
}

void StopMotors() {
  digitalWrite(LEFT_MOTOR_I1, LOW);
  digitalWrite(LEFT_MOTOR_I2, LOW);

  digitalWrite(RIGHT_MOTOR_I1, LOW);
  digitalWrite(RIGHT_MOTOR_I2, LOW);

  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

// ---------------- Smooth Steering ----------------
void microSteerTowardWall() {
  steeringServo.write(steeringCenter + 10);  // Slight right
}

void microSteerAwayFromWall() {
  steeringServo.write(steeringCenter - 10);  // Slight left
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);

  sensorServo.attach(SERVO_SENSOR_PIN);
  steeringServo.attach(SERVO_STEER_PIN);

  sensorServo.write(rightScan);
  steeringServo.write(steeringCenter);

  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_I1, OUTPUT);
  pinMode(LEFT_MOTOR_I2, OUTPUT);
  pinMode(RIGHT_MOTOR_I1, OUTPUT);
  pinMode(RIGHT_MOTOR_I2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  Serial.println("Waiting for IR start...");
}

// ---------------- Loop ----------------
void loop() {

  // ----- WAIT FOR IR BUTTON -----
  if (!robotStarted) {

    if (IrReceiver.decode()) {
      unsigned long code = IrReceiver.decodedIRData.decodedRawData;

      // Replace with your REAL start code
      if (code == 0xBF40FF00) {
        robotStarted = true;
        Serial.println("Robot STARTED!");
      }

      IrReceiver.resume();
    }

    StopMotors();
    return;
  }

  // ----- ROBOT RUNNING -----
  MoveForward();

  // ---- WALL FOLLOWING ----
  if (millis() - lastScanTime >= scanInterval) {

    sensorServo.write(rightScan);
    delay(150);

    int dist = getDistance();
    lastScanTime = millis();

    if (dist == 0 || dist > 150) {
      // Wall disappeared → turn slightly right to reacquire
      microSteerTowardWall();
    }
    else if (dist < targetDistance - 5) {
      // Too close → steer left slightly
      microSteerAwayFromWall();
    }
    else if (dist > targetDistance + 5) {
      // Too far → steer right slightly
      microSteerTowardWall();
    }
    else {
      // Perfect = steer straight
      steeringServo.write(steeringCenter);
    }
  }
}
