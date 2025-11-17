/*
  Author: Team 7 (merged)
  Date:   11/15/25
  About:  Wall-Following + Obstacle Check
*/

#include <Servo.h>
#include <IRremote.h>

#define TRIG_PIN         12
#define ECHO_PIN         13

#define SERVO_SENSOR_PIN 10
#define SERVO_STEER_PIN  9

#define LEFT_MOTOR_EN    5
#define RIGHT_MOTOR_EN   6
#define LEFT_MOTOR_I1    4
#define LEFT_MOTOR_I2    2
#define RIGHT_MOTOR_I1   7
#define RIGHT_MOTOR_I2   8

#define IR_RECEIVE_PIN   11
#define LED              3

int rightScan       = 0;     // angle for wall following  
int forwardAngle    = 90;    // angle for obstacle avoidance

int steeringCenter  = 88;
int steeringLeft    = 60;
int steeringRight   = 120;

int targetDistance  = 20;    // distance from wall
int safeDistance    = 30;    // obstacle avoidance distance
int scanInterval    = 250;

// ---------------- State ----------------
bool robotStarted = false;
unsigned long lastScanTime = 0;
unsigned long wallFollowStart = 0;
const unsigned long wallFollowDuration = 5000; // 5 seconds to wall follow

// ----- NEW: Mission Timer -----
unsigned long missionStart = 0;
const unsigned long missionDuration = 5UL * 60UL * 1000UL; // 5 minute mission timer

// ---------------- Objects ----------------
Servo sensorServo;
Servo steeringServo;

// ---------------- Utility: Distance ----------------
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 12000); // up to ~200 cm
  if (duration == 0) return 0;
  return (int)(duration * 0.034 / 2.0);
}

// ---------------- Motor helper (from obstacle code) ----------------
void motors(int LI1, int LI2, int RI1, int RI2, int leftPWM, int rightPWM) {
  digitalWrite(LEFT_MOTOR_I1, LI1);
  digitalWrite(LEFT_MOTOR_I2, LI2);
  digitalWrite(RIGHT_MOTOR_I1, RI1);
  digitalWrite(RIGHT_MOTOR_I2, RI2);
  analogWrite(LEFT_MOTOR_EN, leftPWM);
  analogWrite(RIGHT_MOTOR_EN, rightPWM);
}

// ---------------- Movement functions ----------------
void MoveForward() {
  motors(1,0, 0,1, 100, 100);
}

void MoveBackward() {
  steeringServo.write(steeringCenter); // CENTER steering
  motors(0,1, 1,0, 100, 100);
}

void StopMotors() {
  motors(0,0, 0,0, 0,0);
}

// ----- Smooth Steering helpers -----
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

  sensorServo.write(rightScan);          // RIGHT (ultrasonic sensor servo)
  steeringServo.write(steeringCenter);   // CENTER (steering servo)

  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_I1, OUTPUT);
  pinMode(LEFT_MOTOR_I2, OUTPUT);
  pinMode(RIGHT_MOTOR_I1, OUTPUT);
  pinMode(RIGHT_MOTOR_I2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  Serial.println("Waiting for IR start...");
}

// ---------------- Main Loop ----------------
void loop() {

  // ----- WAIT FOR IR BUTTON -----
  if (!robotStarted) {
    if (IrReceiver.decode()) {
      unsigned long code = IrReceiver.decodedIRData.decodedRawData;
      Serial.print("IR code: 0x");
      Serial.println(code, HEX);

      if (code == 0xBF40FF00) {
        robotStarted = true;
        digitalWrite(LED, HIGH);

        wallFollowStart = millis();
        missionStart = millis();       // <------ START MISSION TIMER
        lastScanTime = 0;

        Serial.println("Robot STARTED!");
      }
      IrReceiver.resume();
    }
    StopMotors();
    return;
  }

  // ----- MISSION TIMER CHECK -----
  if (millis() - missionStart >= missionDuration) {
    Serial.println("MISSION COMPLETE: 5 minutes reached.");
    StopMotors();
    digitalWrite(LED, LOW);
    robotStarted = false;
    return;
  }

  // ----- MAIN BEHAVIOR: wall-following for 5s -----
  while (robotStarted && (millis() - wallFollowStart) < wallFollowDuration) {
    MoveForward();

    sensorServo.write(rightScan); // RIGHT (ultrasonic sensor servo)
    delay(150);

    if (millis() - lastScanTime >= scanInterval) {
      int dist = getDistance();
      lastScanTime = millis();

      Serial.print("Wall follow dist: ");
      Serial.print(dist);
      Serial.println(" cm");

      if (dist < targetDistance - 5) {
        microSteerAwayFromWall();
        Serial.println("Steering away from wall");
      }
      else if (dist > targetDistance + 5) {
        microSteerTowardWall();
        Serial.println("Steering toward the wall");
      }
      else {
        steeringServo.write(steeringCenter); // CENTER (steering servo)
      }
    }

    delay(100);
  }

  // ----- OBSTACLE CHECK -----
  sensorServo.write(forwardAngle); // FORWARD (90°)
  delay(200);

  int frontDist = getDistance();
  Serial.print("Front dist: ");
  Serial.print(frontDist);
  Serial.println(" cm");

  if (frontDist > 0 && frontDist < safeDistance) {
    Serial.println("Obstacle detected: backing up");
    MoveBackward();
    delay(1000);
    StopMotors();
  } else {
    Serial.println("No obstacle detected: returning to wall-follow");
  }

  wallFollowStart = millis();
  lastScanTime = 0;
}
