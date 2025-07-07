#include <Arduino.h>
#include <math.h>
#include <AccelStepper.h>

#define STEP1 2
#define DIR1 3
#define LIMIT1 8

#define STEP2 4
#define DIR2 5
#define LIMIT2 9

#define STEP3 6
#define DIR3 7
#define LIMIT3 10

#define PUMP 12

#define MOTOR1_HOME  1592L
#define MOTOR2_HOME  1609L
#define MOTOR3_HOME  1578L

enum State {
  IDLE,
  RUNNING
};

State currentState = IDLE;
bool abortRequested = false;

AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);

#define MAX_WAYPOINTS 20
int positions[MAX_WAYPOINTS][3];
int waypoint_count = 0;
int current_index = 0;

String inputBuffer = "";
bool newMessage = false;

void homeMotor(AccelStepper &motor, int limitPin, long homePosition) {
  while (digitalRead(limitPin) == HIGH) {
    motor.move(10);
    motor.run();
  }
  motor.setCurrentPosition(homePosition);
  motor.moveTo(-100);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
}

void checkLimitSwitches() {
  if (digitalRead(LIMIT1) == LOW || digitalRead(LIMIT2) == LOW || digitalRead(LIMIT3) == LOW) {
    stopAllMotors();
    digitalWrite(PUMP, LOW); // Stop the pump if running
    homeMotor(motor1, LIMIT1, MOTOR1_HOME);
    homeMotor(motor2, LIMIT2, MOTOR2_HOME);
    homeMotor(motor3, LIMIT3, MOTOR3_HOME);
    Serial.println("LIMIT");
  }
}

bool motorsRunning() {
  return motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0;
}

void moveToPosition(int idx) {

  // More advanced motor control to ensure smooth movement in the x-y plane to be added here

  
  motor1.moveTo(positions[idx][0]);
  motor2.moveTo(positions[idx][1]);
  motor3.moveTo(positions[idx][2]);

  long d1 = abs(motor1.distanceToGo());
  long d2 = abs(motor2.distanceToGo());
  long d3 = abs(motor3.distanceToGo());

  long maxDist = max(d1, max(d2, d3));

  float baseSpeed = 2000.0; // max speed for longest-moving motor
  
  // Adjust speed to only move in x-y plane
  motor1.setMaxSpeed(baseSpeed * d1 / maxDist);
  motor2.setMaxSpeed(baseSpeed * d2 / maxDist);
  motor3.setMaxSpeed(baseSpeed * d3 / maxDist);
}

void stopAllMotors() {
  motor1.stop();
  motor2.stop();
  motor3.stop();
}

void readSerialMessage() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      newMessage = true;
      break;
    } else {
      inputBuffer += c;
    }
  }

  if (newMessage) {
    inputBuffer.trim();

    if (inputBuffer == "POSITION") {
      waypoint_count = 0;
      abortRequested = false;
    }
    else if (inputBuffer.startsWith("ANGLES") && currentState == IDLE) {
      int a1, a2, a3;
      if (sscanf(inputBuffer.c_str(), "ANGLES %d,%d,%d", &a1, &a2, &a3) == 3 && waypoint_count < MAX_WAYPOINTS) {
        positions[waypoint_count][0] = a1;
        positions[waypoint_count][1] = a2;
        positions[waypoint_count][2] = a3;
        waypoint_count++;
      }
    }
    else if (inputBuffer == "PUMP_ON") {
      digitalWrite(PUMP, HIGH);
    }
    else if (inputBuffer == "PUMP_OFF") {
      digitalWrite(PUMP, LOW);
    }
    else if (inputBuffer == "GO" && currentState == IDLE) {
      current_index = 0;
      currentState = RUNNING;
    }
    else if (inputBuffer == "ABORT") {
      abortRequested = true;
    }

    inputBuffer = "";
    newMessage = false;
  }
}

void goHome() {
  homeMotor(motor1, LIMIT1, MOTOR1_HOME);
  homeMotor(motor2, LIMIT2, MOTOR2_HOME);
  homeMotor(motor3, LIMIT3, MOTOR3_HOME);
}

void setup() {

  Serial.begin(57600);

  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);
  pinMode(LIMIT3, INPUT_PULLUP);
  pinMode(PUMP, OUTPUT);


  motor1.setMaxSpeed(4000);
  motor2.setMaxSpeed(4000);
  motor3.setMaxSpeed(4000);

  motor1.setAcceleration(2000);
  motor2.setAcceleration(2000);
  motor3.setAcceleration(2000);

  goHome();

  Serial.println("READY"); // signal Jetson
}

void loop() {
  // Read Serial
  readSerialMessage();

  switch (currentState) {
  case IDLE:
    break;

  case RUNNING:
    if (abortRequested) {
      stopAllMotors();
      currentState = IDLE;
      abortRequested = false;
      goHome();
      Serial.println("ABORTED");
      break;
      }
    
    if (motorsRunning()) {
      checkLimitSwitches();
      motor1.run();
      motor2.run();
      motor3.run();
      } else if (current_index < waypoint_count) {
        moveToPosition(current_index++);
      } else {
        currentState = IDLE;
        waypoint_count = 0;
        current_index = 0;

        for (int i = 0; i < MAX_WAYPOINTS; i++) {
          positions[i][0] = 0;
          positions[i][1] = 0;
          positions[i][2] = 0;
        }
        Serial.println("DONE");
      }
      break;
  }
}