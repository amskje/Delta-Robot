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
#define LIMIT3 11

#define PUMP 12


enum State {
  IDLE,
  RUNNING
};

State currentState = IDLE;
bool abortRequested = false;

AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);

#define MAX_WAYPOINTS 10

int positions[MAX_WAYPOINTS][3];
int waypoint_count = 0;
int current_index = 0;

String inputBuffer = "";
bool newMessage = false;




// Debounce settings
const unsigned long debounceDelay = 30;  // ms
unsigned long lastLimit1Change = 0;
unsigned long lastLimit2Change = 0;
unsigned long lastLimit3Change = 0;
bool stableLimit1State = HIGH;
bool stableLimit2State = HIGH;
bool stableLimit3State = HIGH;

// Debounce function
bool debounceRead(int pin, unsigned long &lastChangeTime, bool &lastStableState) {
  bool reading = digitalRead(pin);
  if (reading != lastStableState) {
    lastChangeTime = millis();  // reset debounce timer
    lastStableState = reading;
  }
  return (lastStableState == LOW && (millis() - lastChangeTime >= debounceDelay));
}



void homeMotor(AccelStepper &motor, int limitPin) {
  
  while (digitalRead(limitPin) == HIGH) {
    motor.move(100);//Endre her hvis endre step på motor, var 10
    motor.run();
  }
  motor.setCurrentPosition(0);
  motor.moveTo(-1000);//Endre her hvis endre step på motor, var -100
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
}

// Debounced limit switch check
void checkLimitSwitches() {
  bool limit1Triggered = debounceRead(LIMIT1, lastLimit1Change, stableLimit1State);
  bool limit2Triggered = debounceRead(LIMIT2, lastLimit2Change, stableLimit2State);
  bool limit3Triggered = debounceRead(LIMIT3, lastLimit3Change, stableLimit3State);

  //if (limit1Triggered) Serial.println("[Arduino] LIMIT1 triggered");
  //if (limit2Triggered) Serial.println("[Arduino] LIMIT2 triggered");
  //if (limit3Triggered) Serial.println("[Arduino] LIMIT3 triggered");

  if (limit1Triggered || limit2Triggered || limit3Triggered) {
    //Serial.println("[Arduino] CheckLimitSwitches triggered");

    stopAllMotors();
    digitalWrite(PUMP, LOW); // Stop the pump if running

    homeMotor(motor1, LIMIT1);
    homeMotor(motor2, LIMIT2);
    homeMotor(motor3, LIMIT3);

    Serial.println("LIMIT");
  }
}

bool motorsRunning() {
  return motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0;
}

void moveToPosition(int idx) {

  // More advanced motor control to ensure smooth movement in the x-y plane to be added here

  Serial.print("[Arduino] Moving to: ");
  Serial.print(positions[idx][0]);
  Serial.print(", ");
  Serial.print(positions[idx][1]);
  Serial.print(", ");
  Serial.println(positions[idx][2]);

  
  motor1.moveTo(positions[idx][0]);
  motor2.moveTo(positions[idx][1]);
  motor3.moveTo(positions[idx][2]);

  long d1 = abs(motor1.distanceToGo());
  long d2 = abs(motor2.distanceToGo());
  long d3 = abs(motor3.distanceToGo());

  long maxDist = max(d1, max(d2, d3));

  float baseSpeed = 80000.0; // max speed for longest-moving motor
  
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

    //Serial.print("[Arduino] Received inputbuffer: ");
    //Serial.println(inputBuffer);

    if (inputBuffer == "POSITION") {
      waypoint_count = 0;
      abortRequested = false;
      Serial.println("READY");
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
  homeMotor(motor1, LIMIT1);
  homeMotor(motor2, LIMIT2);
  homeMotor(motor3, LIMIT3);


  Serial.print("M1 = ");
  Serial.print(motor1.currentPosition());
  Serial.print(", M2 = ");
  Serial.print(motor2.currentPosition());
  Serial.print(", M3 = ");
  Serial.println(motor3.currentPosition());
}

void setup() {

  Serial.begin(57600);

  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);
  pinMode(LIMIT3, INPUT_PULLUP);
  pinMode(PUMP, OUTPUT);


  motor1.setMaxSpeed(80000);
  motor2.setMaxSpeed(80000);
  motor3.setMaxSpeed(80000);

  motor1.setAcceleration(40000);
  motor2.setAcceleration(40000);
  motor3.setAcceleration(40000);

  goHome();

  Serial.println("Finished setup"); // signal Jetson
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