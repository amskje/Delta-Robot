#include <Arduino.h>
#include <math.h>
#include <AccelStepper.h>


//Pin setup
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
#define SENSOR A0


enum State {
  IDLE,
  RUNNING,
  PICKING_UP
};

State currentState = IDLE;
bool reset = false;
bool abortRequested = false;

String inputBuffer = "";
bool newMessage = false;

AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);

//Move to target setup
#define MAX_WAYPOINTS 10
int positions[MAX_WAYPOINTS][3];
int waypoint_count = 0;
int current_index = 0;

//Move down set up
#define MAX_PICKDOWN 5
int pickdown_positions[MAX_PICKDOWN][3];
int pickdown_count = 0;
int current_index_down = 0;
bool receivingPickdown = false;
int pickupPauseTime = 100;

//Pressure sensor set up
const float R = 250.0;
const float Vcc = 5.0; //Arduino ref voltage
const float pickupThreshold = 0.7;  // Pressure below this means candy is picked
const int maxPickupTries = 3;

// Debounce set up
const unsigned long debounceDelay = 30;  // ms
unsigned long lastLimit1Change = 0;
unsigned long lastLimit2Change = 0;
unsigned long lastLimit3Change = 0;
bool stableLimit1State = HIGH;
bool stableLimit2State = HIGH;
bool stableLimit3State = HIGH;

//Drop off set up
bool dropoffPlanned = false;




void clearWaypoints() {
  waypoint_count = 0;
  current_index = 0;
  pickdown_count = 0;
  current_index_down = 0;

  for (int i = 0; i < MAX_WAYPOINTS; i++) {
    positions[i][0] = 0;
    positions[i][1] = 0;
    positions[i][2] = 0;

    pickdown_positions[i][0] = 0;
    pickdown_positions[i][1] = 0;
    pickdown_positions[i][2] = 0;
  }
}


bool checkSensor() {
  int rawADC = analogRead(SENSOR);
  float voltage = rawADC * Vcc / 1023.0; // 1023 becuas 10 bit analog to digital converter on arduino
  float current_mA = voltage / R * 1000.0; 
  float pressure_bar = (current_mA - 4.0) * (4.0 / 16.0);  // Scale 4–20 mA to 0–4 bar
  // it is a round 0,64 bar when the candy is lifted

  if (pressure_bar > pickupThreshold){//Viktig!! flippe tegnet når du faktisk har pumpen og sensot koblet til
    return true;
  }else {
    return false;
  }
}


bool debounceRead(int pin, unsigned long &lastChangeTime, bool &lastStableState) {
  bool reading = digitalRead(pin);
  if (reading != lastStableState) {
    lastChangeTime = millis();  // reset debounce timer
    lastStableState = reading;
  }
  return (lastStableState == LOW && (millis() - lastChangeTime >= debounceDelay));
}


void checkLimitSwitches() {
  bool limit1Triggered = debounceRead(LIMIT1, lastLimit1Change, stableLimit1State);
  bool limit2Triggered = debounceRead(LIMIT2, lastLimit2Change, stableLimit2State);
  bool limit3Triggered = debounceRead(LIMIT3, lastLimit3Change, stableLimit3State);

  if (limit1Triggered || limit2Triggered || limit3Triggered) {
    stopAllMotors();
    digitalWrite(PUMP, LOW); // Stop the pump if running
    goHome3();

    Serial.println("LIMIT");
  }
}


void goHome() {
  homeMotor(motor1, LIMIT1);
  homeMotor(motor2, LIMIT2);
  homeMotor(motor3, LIMIT3);
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


void goHome3() {
  // Move all motors toward their limits simultaneously
  motor1.move(20000);  // move far enough to ensure hitting limit
  motor2.move(20000);
  motor3.move(20000);

  bool limit1Hit = false;
  bool limit2Hit = false;
  bool limit3Hit = false;

  while (!limit1Hit || !limit2Hit || !limit3Hit) {
    if (!limit1Hit && digitalRead(LIMIT1) == LOW) {
      motor1.stop();
      limit1Hit = true;
    } else if (!limit1Hit) {
      motor1.run();
    }

    if (!limit2Hit && digitalRead(LIMIT2) == LOW) {
      motor2.stop();
      limit2Hit = true;
    } else if (!limit2Hit) {
      motor2.run();
    }

    if (!limit3Hit && digitalRead(LIMIT3) == LOW) {
      motor3.stop();
      limit3Hit = true;
    } else if (!limit3Hit) {
      motor3.run();
    }
  }

  // Set all motors to position 0
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  motor3.setCurrentPosition(0);

  // Back off slightly from limit switches in sync
  motor1.moveTo(-1000);
  motor2.moveTo(-1000);
  motor3.moveTo(-1000);

  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
  }
  Serial.println("IDLE POSITION");//implementer i jetson code at den tar i mot dette og etter fått meling kan man velge twist
}


bool motorsRunning() {
  return motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0;
}

void moveToPosition(int idx, int positionArray[][3]) {
  // More advanced motor control to ensure smooth movement in the x-y plane to be added here  
  motor1.moveTo(positionArray[idx][0]);
  motor2.moveTo(positionArray[idx][1]);
  motor3.moveTo(positionArray[idx][2]);

  long d1 = abs(motor1.distanceToGo());
  long d2 = abs(motor2.distanceToGo());
  long d3 = abs(motor3.distanceToGo());

  long maxDist = max(d1, max(d2, d3));

  float baseSpeed = 80000.0; // max speed for longest-moving motor

  if (maxDist == 0) maxDist = 1;  // prevent division by zero

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
      pickdown_count = 0;
      receivingPickdown = false;
      abortRequested = false;
      Serial.println("READY");
    }
    else if (inputBuffer == "DOWN") {
      receivingPickdown = true;
      Serial.println("READY");
    }
    else if (inputBuffer == "DROPP") {
      dropoffPlanned = true;
    }
    else if (inputBuffer.startsWith("ANGLES") && currentState == IDLE) {
      int a1, a2, a3;
      if (sscanf(inputBuffer.c_str(), "ANGLES %d,%d,%d", &a1, &a2, &a3) == 3) {
        if (receivingPickdown) {
          if (pickdown_count < MAX_PICKDOWN) {
            pickdown_positions[pickdown_count][0] = a1;
            pickdown_positions[pickdown_count][1] = a2;
            pickdown_positions[pickdown_count][2] = a3;
            pickdown_count++;
          }
        } else {
          if (waypoint_count < MAX_WAYPOINTS) {
            positions[waypoint_count][0] = a1;
            positions[waypoint_count][1] = a2;
            positions[waypoint_count][2] = a3;
            waypoint_count++;
          }
        }
      }
    }
    else if (inputBuffer == "PUMP_ON") {
      digitalWrite(PUMP, HIGH);
    }
    else if (inputBuffer == "PUMP_OFF") {
      digitalWrite(PUMP, LOW);
    }
    else if (inputBuffer == "GO" && currentState == IDLE) {
      if (waypoint_count > 0) {
        moveToPosition(current_index++, positions);  // Start first move immediately
        currentState = RUNNING;
      } else {
        Serial.println("[Arduino] GO command received with 0 waypoints");
      }
    }
    else if (inputBuffer == "ABORT") {
      abortRequested = true;
    }

    inputBuffer = "";
    newMessage = false;
  }
}


void setup() {

  Serial.begin(57600);

  digitalWrite(PUMP, LOW);  

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

  goHome3();

  Serial.println("Finished setup"); // signal Jetson
}

void loop() {
  // Read Serial
  readSerialMessage();

  switch (currentState) {
  case IDLE:

    if (reset) {
      reset = false;
      clearWaypoints();
    }
    break;

  case RUNNING:
    if (abortRequested) {
      stopAllMotors();
      reset = true;
      currentState = IDLE;
      abortRequested = false;
      goHome3();
      Serial.println("ABORTED");
      break;
    }
    
    if (motorsRunning()) {
      checkLimitSwitches();
      motor1.run();
      motor2.run();
      motor3.run();
    } 
    else if (current_index < waypoint_count) {
      moveToPosition(current_index++, positions);
    } 
    else if ((current_index_down < pickdown_count) && (current_index >= waypoint_count)) {
      digitalWrite(PUMP, HIGH);

      if (checkSensor()){
        current_index_down = pickdown_count;
        Serial.println("PICKED_UP");
      } else {
          delay(pickupPauseTime);
          moveToPosition(current_index_down, pickdown_positions);
          current_index_down++;

          if (current_index_down == pickdown_count){
            Serial.println("NOT_PICKED_UP");
            //digitalWrite(PUMP, LOW);//av kommenter denne når du tester med sensor
            //legge til gohome her når du tester med sensor
            //goHome3();
          }
        } 
    }
    else {
      if (dropoffPlanned) {
        digitalWrite(PUMP, LOW);  
        //Serial.println("TWIST_FOUND");
        dropoffPlanned = false;   
      }

      Serial.println("DONE");
      reset = true;
      currentState = IDLE;
    }
    break;
  }
}