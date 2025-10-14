/*  
 Author: Nicholas Cordero 
 Date: 10/2/25
 About: Fall 2025 Project
        Obstruction Avoidance Vehicle 
*/

/* This project includes 2 servo motors controlling the ultrasonic sensor and 
   the steering shaft alongside 2 DC motors to control the front wheels */

#include <Servo.h>
#include <IRremote.h>

enum MISSION_PHASE {
  CONFIG, WAIT, GET_READINGS, STOP
};

#define GREEN_LED         12  // led connected to pin 12
#define IR_PIN            2   // ultrasonic sensor connected to pin 2

#define SERVO_SENSOR_PIN  11  // ultrasonic sensor servo connected to pin 11
#define SERVO_STEER_PIN   10  // steering servor connected to pin 10

#define LEFT_MOTOR_EN     5   // controls the speed of the motors no matter the direction 
#define RIGHT_MOTOR_EN    6   // controls the speed of the motors no matter the direction
#define LEFT_MOTOR_I1     7   // controls the forward direction of the left motor
#define LEFT_MOTOR_I2     8   // controls the backward direction of the left motor
#define RIGHT_MOTOR_I1    4   // controls the forward direction of the right motor
#define RIGHT_MOTOR_I2    3   // controls the backward direction of the right motor

#define TRIG               9      // sends a pulse or sound wave out (trigger signal)
#define ECHO               13     // recieves a pulse or sound wave from the trigger (echo signal)
#define DISTANCE_THRESHOLD 20     // 20 cm clearance
#define TIME_THRESHOLD     20000  // 20 seconds

Servo servoSensor;   // controls ultrasonic scan
Servo servoSteering; // controls front wheels direction
IRrecv irrecv(IR_PIN);
decode_results results;

int left_angle  = 45;
int front_angle = 90;
int right_angle = 135;

int steeringCenter = 90; // straight
int steeringLeft   = 60; // turn left
int steeringRight  = 120; // turn right

int token;
int IRPresence = 0;
long timeElapse, timeOrigin;

// ---------- Function Prototypes ----------
void SetUpLED();
void TurnOnGreenLED();
void TurnOffGreenLED();
void StartServo();
void StopServo();
void SetUpDCMotors();
void StopDCMotors();
void StartIR();
void StopIR();
void CheckIRPresence();
void UpdateMotorCommands();
long GetDistance();
long GetReadings(int);
void RotateLeft();
void RotateRight();
void MoveForward();
void SetTimeOrigin();
void GetTimeElapse();

// ---------- LED ----------
void SetUpLED() { pinMode(GREEN_LED, OUTPUT); }
void TurnOnGreenLED(){ digitalWrite(GREEN_LED, HIGH); }
void TurnOffGreenLED(){ digitalWrite(GREEN_LED, LOW); }

// ---------- Servo ----------
void StartServo() {
  servoSensor.attach(SERVO_SENSOR_PIN);
  servoSteering.attach(SERVO_STEER_PIN);
  servoSteering.write(steeringCenter); // center steering
}
void StopServo() {
  servoSensor.detach();
  servoSteering.detach();
}

// ---------- DC Motors ----------
void SetUpDCMotors() {
  pinMode(LEFT_MOTOR_I1, OUTPUT);
  pinMode(LEFT_MOTOR_I2, OUTPUT);
  pinMode(RIGHT_MOTOR_I1, OUTPUT);
  pinMode(RIGHT_MOTOR_I2, OUTPUT);
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

void StopDCMotors() {
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
  digitalWrite(LEFT_MOTOR_I1, LOW);
  digitalWrite(LEFT_MOTOR_I2, LOW);
  digitalWrite(RIGHT_MOTOR_I1, LOW);
  digitalWrite(RIGHT_MOTOR_I2, LOW);
}

// ---------- IR ----------
void StartIR() { irrecv.enableIRIn(); }
void StopIR() { irrecv.disableIRIn(); }

void CheckIRPresence() {
  if (irrecv.decode(&results)) {
    IRPresence = 1; // any signal starts the mission
    irrecv.resume(); 
  }
}

// ---------- Time ----------
void SetTimeOrigin() { timeOrigin = millis(); }
void GetTimeElapse() { timeElapse = millis() - timeOrigin; }

// ---------- Ultrasonic ----------
long GetDistance() {
  long duration;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH, 20000);
  long distance = duration * 0.034 / 2;
  delay(30);
  return distance;
}

long GetReadings(int angle) {
  servoSensor.write(angle);
  delay(250);
  long distance = GetDistance();
  return distance;
}

// ---------- Motion ----------
void MoveForward() {
  servoSteering.write(steeringCenter); // straight at 90 degrees
  digitalWrite(LEFT_MOTOR_I1, HIGH); // left motor rotate forward ON
  digitalWrite(LEFT_MOTOR_I2, LOW); // left motor rotate backward OFF
  digitalWrite(RIGHT_MOTOR_I1, HIGH); // right motor rotate forward ON
  digitalWrite(RIGHT_MOTOR_I2, LOW); //right motor rotate backward OFF
  analogWrite(LEFT_MOTOR_EN, 170);  
  analogWrite(RIGHT_MOTOR_EN, 170);
}

void RotateLeft() {
  servoSteering.write(steeringLeft); // turn left 60 degrees
  digitalWrite(LEFT_MOTOR_I1, HIGH); 
  digitalWrite(LEFT_MOTOR_I2, LOW);
  digitalWrite(RIGHT_MOTOR_I1, HIGH);
  digitalWrite(RIGHT_MOTOR_I2, LOW);
  analogWrite(LEFT_MOTOR_EN, 140);
  analogWrite(RIGHT_MOTOR_EN, 160);
}

void RotateRight() {
  servoSteering.write(steeringRight); // turn right at 120 degrees
  digitalWrite(LEFT_MOTOR_I1, HIGH);
  digitalWrite(LEFT_MOTOR_I2, LOW);
  digitalWrite(RIGHT_MOTOR_I1, HIGH);
  digitalWrite(RIGHT_MOTOR_I2, LOW);
  analogWrite(LEFT_MOTOR_EN, 160);
  analogWrite(RIGHT_MOTOR_EN, 140);
}

// ---------- Main Program ----------
void setup() {
  Serial.begin(9600);
  token = CONFIG;
}

void loop() {
  switch (token) {
    case CONFIG:                                          // this case "configures" the LEDs, motors, IR, servos to their starting positions
      SetUpLED();
      SetUpDCMotors();
      StartIR();
      StartServo();
      TurnOffGreenLED();
      pinMode(TRIG, OUTPUT);
      pinMode(ECHO, INPUT);
      token = WAIT;
      break;

    case WAIT:                                            // this case "waits" for the IR reciever to gather data before operation is started
      CheckIRPresence();
      if (IRPresence) {
        token = GET_READINGS;
        TurnOnGreenLED();
        StopIR();
        SetTimeOrigin();
      }
      break;

    case GET_READINGS: {                                  // this case gets the data and outputs the results in moving the different components (motors for wheels)
      long left  = GetReadings(left_angle);
      long front = GetReadings(front_angle);
      long right = GetReadings(right_angle);

      Serial.print("  Left: "); Serial.print(left);       // prints in the serial monitor so we can visually see how the data is gathered
      Serial.print("  Front: "); Serial.print(front);
      Serial.print("  Right: "); Serial.println(right);

      // Steering logic
      if (front > DISTANCE_THRESHOLD) {
        MoveForward();
      } else if (left > DISTANCE_THRESHOLD) {
        RotateLeft();
      } else {
        RotateRight();
      }

      GetTimeElapse();
      if (timeElapse > TIME_THRESHOLD) token = STOP;      // currently set to 20 seconds to test if time elapsed will obey the time threshold command
      delay(100);
      break;
    }

    case STOP:                                            // this case "stops" the motors and turns off the LED to show it is in the off state
      StopDCMotors();
      TurnOffGreenLED();
      StopServo();
      break;
  }

  delay(100);
}
