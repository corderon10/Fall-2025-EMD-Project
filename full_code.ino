/*
  Author: Team 7 (merged)
  Date:   11/15/25
  About:  Continuous Wall-Following + Obstacle Check
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

int steeringCenter  = 85;

int targetDistance  = 20;    // distance from wall
int safeDistance    = 15;    // obstacle avoidance distance
int scanInterval    = 250;

// ---------------- State ----------------
bool robotStarted = false;
unsigned long lastScanTime = 0;

// ----- Mission Timer -----
unsigned long missionStart = 0;
const unsigned long missionDuration = 5UL * 60UL * 1000UL; // 5 minutes

Servo sensorServo;
Servo steeringServo;

// ---------------- Distance ----------------
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 12000);
  if (duration == 0) return 0;

  return (int)(duration * 0.034 / 2.0);
}

// ---------------- Motor Helper ----------------
void motors(int LI1, int LI2, int RI1, int RI2, int leftPWM, int rightPWM) {
  digitalWrite(LEFT_MOTOR_I1, LI1);
  digitalWrite(LEFT_MOTOR_I2, LI2);
  digitalWrite(RIGHT_MOTOR_I1, RI1);
  digitalWrite(RIGHT_MOTOR_I2, RI2);
  analogWrite(LEFT_MOTOR_EN, leftPWM);
  analogWrite(RIGHT_MOTOR_EN, rightPWM);
}

// ---------------- Movement ----------------
void MoveForward() {
  motors(1,0, 0,1, 100, 100);
}

void MoveBackward() {
  steeringServo.write(steeringCenter);
  motors(0,1, 1,0, 100, 100);
}

void StopMotors() {
  motors(0,0, 0,0, 0,0);
}

// ---------------- Micro Steering ----------------
void microSteerTowardWall() {
  steeringServo.write(steeringCenter + 5);
}

void microSteerAwayFromWall() {
  steeringServo.write(steeringCenter - 5);
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

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  Serial.println("Waiting for IR start...");
}

// ---------------- Main Loop ----------------
void loop() {

  // ----- Wait for IR Start Button -----
  if (!robotStarted) {
    if (IrReceiver.decode()) {
      unsigned long code = IrReceiver.decodedIRData.decodedRawData;
      Serial.print("IR code: 0x");
      Serial.println(code, HEX);

      if (code == 0xBF40FF00) {
        robotStarted = true;
        digitalWrite(LED, HIGH);

        missionStart = millis();
        lastScanTime = 0;

        Serial.println("Robot STARTED!");
      }
      IrReceiver.resume();
    }

    StopMotors();
    return;
  }

  // ----- Mission Timer -----
  if (millis() - missionStart >= missionDuration) {
    Serial.println("MISSION COMPLETE (5 minutes)");
    StopMotors();
    digitalWrite(LED, LOW);
    robotStarted = false;
    return;
  }

  // ---------------------------------------
  //        CONTINUOUS WALL FOLLOWING
  // ---------------------------------------
  MoveForward();

  if (millis() - lastScanTime >= scanInterval) {

    // Look RIGHT
    sensorServo.write(rightScan);
    delay(250);
    delayMicroseconds(500);
    int dist = getDistance();
    lastScanTime = millis();

    Serial.print("Wall dist: ");
    Serial.println(dist);

    // Wall-follow logic
    if (dist == 0 || dist > 150) {
      microSteerTowardWall();
    }
    else if (dist < targetDistance - 5) {
      microSteerAwayFromWall();
    }
    else if (dist > targetDistance + 5) {
      microSteerTowardWall();
    }
    else {
      steeringServo.write(steeringCenter);
    }
  }

  // ---------------------------------------
  //        OBSTACLE CHECK (FORWARD)
  // ---------------------------------------
  sensorServo.write(forwardAngle);
  delay(100);

  int frontDist = getDistance();
  Serial.print("Front dist: ");
  Serial.println(frontDist);

  if (frontDist > 0 && frontDist < safeDistance) {
    StopMotors();
    delay(1000);
    Serial.println("Obstacle detected: BACKING UP!");
    delay(200);
    MoveBackward();
    delay(1000);
    StopMotors();
    delay(1000);
  }
}
