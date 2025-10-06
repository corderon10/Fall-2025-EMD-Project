/* 
 Author: Nicholas Cordero 
 Date: 10/2/25
 About: Fall 2025 Project
 Revision #1
 */

/* this porject includes 2 servo motors controlling the ultrasonic sensor and the steering shaft alongside 2 DC motors to control the rear wheels */
 
#include <Servo.h>
#include <IRremote.h>

enum MISSION_PHASE {                    // state definition
    CONFIG, WAIT, GET_READINGS, STOP
};

#define GREEN_LED       12
#define IR              2
#define SERVO           11

#define LEFT_MOTOR_EN   5          // must be a PWM compatible pin
#define RIGHT_MOTOR_EN  6         // must be a PWM compatible pin
#define LEFT_MOTOR_I1   7
#define LEFT_MOTOR_I2   8
#define RIGHT_MOTOR_I1  4
#define RIGHT_MOTOR_I2  3

#define TRIG                12            // trigger pin
#define ECHO                8            // echo pin
#define DISTANCE_THRESHOLD  20           // 20 cm clearance
#define TIME_THRESHOLD      20000      // 20 seconds 

Servo myservo;
IRrecv irrecv(IR);
decode_results results;

int left_angle  =  45;   // left view in degrees
int front_angle =  90;   // front view in degrees
int right_angle =  135;   // right view in degrees

int token;    // keep track of which part of the mission the vehicle is
int cmdLeftMotor, cmdRightMotor, cmdServo;  // in terms of duty cycles
int cmdDir;         // commanded direction of travel
int IRPresence;     //  1: yes, 2: no

long timeElapse, timeOrigin;                // in terms of time ticks

////////////////////// Function Prototyping //////////////////////

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

//////////////////////  Function Definitions //////////////////////

// LED Functions //
void SetUpLED() {
pinMode(GREEN_LED, OUTPUT);
}
void TurnOnGreenLED(){
  digitalWrite(GREEN_LED, HIGH);
}
void TurnOffGreenLED(){
  digitalWrite(GREEN_LED, LOW);
}

// Servo Functions //
void StartServo() {
    myservo.attach(SERVO);
}

void StopServo() {
    myservo.detach();
}


void SetUpDCMotors() {
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

void StartIR() {
    irrecv.enableIRIn(); // start the receiver
}

void StopIR() {
    irrecv.disableIRIn(); // stop the receiver
}

void StopDCMotors() {
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

long GetDistance() {
  
  long duration;
  digitalWrite(TRIG, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  long distance = duration/58;
  delay(50);
  return distance;
}

long GetReadings(int angle) {
  StartServo();
  myservo.write(angle);
  delay(50);
  long distance = GetDistance();
  delay(50);
  StopServo();
  
  return distance;
}

void SetTimeOrigin() {
  timeOrigin = millis();
}

void GetTimeElapse() {
  timeElapse = millis() - timeOrigin;
}

void CheckIRPresence() {}       // need to be built
void UpdateMotorCommands() {}   // need to be built

void RotateLeft() {}
void RotateRight() {}
void MoveForward(){}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  token = CONFIG; // the starting part is to configure all sensors/actuators
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (token) {
           
      case CONFIG:

          SetUpLED();
          SetUpDCMotors();
          StartIR();
          TurnOffGreenLED();
          pinMode(TRIG, OUTPUT);
          pinMode(ECHO, INPUT);
          token = WAIT;
          break;
        
     case WAIT:
      // do what is needed
      //test if the condition is met before breaking out of this case

          CheckIRPresence();
          if (IRPresence) {
            token = GET_READINGS;
            TurnOnGreenLED();
            StopIR();
          }
          SetTimeOrigin();
          break;

      case GET_READINGS:
        long left = GetReadings(left_angle);
        long front = GetReadings(front_angle);
        long right = GetReadings(right_angle);

        if (front > DISTANCE_THRESHOLD)
          MoveForward();
        else if (left > DISTANCE_THRESHOLD)
          RotateLeft();
        else
          RotateRight();
            
        GetTimeElapse();

        if (timeElapse > TIME_THRESHOLD)
          token = STOP;

        delay(100);
        break;
        
      case STOP:
       // do what is needed
      //test if the condition is met before breaking out of this case
          StopDCMotors();
          TurnOffGreenLED();
          break;
  }
  delay(100);
}
