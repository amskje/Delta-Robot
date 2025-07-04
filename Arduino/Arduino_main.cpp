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

AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);

#define MAX_WAYPOINTS 300
float theta1_list[MAX_WAYPOINTS];
float theta2_list[MAX_WAYPOINTS];
float theta3_list[MAX_WAYPOINTS];
int waypoint_count = 0;
int current_index = 0;

bool isExecuting = false;
bool pathReady = false;
String inputBuffer = "";


void homeMotor(AccelStepper &motor, int limitPin) {
  while (digitalRead(limitPin) == HIGH) {
    motor.move(10);
    motor.run();
  }
  motor.setCurrentPosition(0);  // theta = 0 at this point
  motor.moveTo(-100);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
}

void processLine(String line) {
  line.trim();

  if (line == "END") {
    pathReady = true;
    waypoint_count = current_index;
    current_index = 0;
    moveToIndex(0);
    isExecuting = true;
    Serial.println("START");  // Optional: signal Jetson
    return;
  }

  int i1 = line.indexOf(',');
  int i2 = line.indexOf(',', i1 + 1);
  if (i1 > 0 && i2 > i1 && current_index < MAX_WAYPOINTS) {
    float t1 = line.substring(0, i1).toFloat();
    float t2 = line.substring(i1 + 1, i2).toFloat();
    float t3 = line.substring(i2 + 1).toFloat();
    theta1_list[current_index] = t1;
    theta2_list[current_index] = t2;
    theta3_list[current_index] = t3;
    current_index++;
  }
}

void moveToIndex(int idx) {
  long s1 = theta1_list[idx];
  long s2 = theta2_list[idx];
  long s3 = theta3_list[idx];

  motor1.moveTo(s1);
  motor2.moveTo(s2);
  motor3.moveTo(s3);
}

void setup() {

    Serial.begin(57600);

  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);
  pinMode(LIMIT3, INPUT_PULLUP);


  motor1.setMaxSpeed(4000);
  motor2.setMaxSpeed(4000);
  motor3.setMaxSpeed(4000);

  motor1.setAcceleration(2000);
  motor2.setAcceleration(2000);
  motor3.setAcceleration(2000);

  homeMotor(motor1, LIMIT1);
  homeMotor(motor2, LIMIT2);
  homeMotor(motor3, LIMIT3);
}

void loop() {
  // Read Serial
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processLine(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Run motion
  if (isExecuting) {
    motor1.run();
    motor2.run();
    motor3.run();

    if (
      motor1.distanceToGo() == 0 &&
      motor2.distanceToGo() == 0 &&
      motor3.distanceToGo() == 0
    ) {
      current_index++;
      if (current_index < waypoint_count) {
        moveToIndex(current_index);
      } else {
        isExecuting = false;
        Serial.println("DONE");  // ACK to Jetson
      }
    }
  }
}