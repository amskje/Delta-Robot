#include <Arduino.h>
#include <math.h>
#include <AccelStepper.h>
#include <MultiStepper.h>



//Pin setup
#define STEP1 5
#define DIR1 10
#define LIMIT1 3

#define STEP2 7
#define DIR2 8
#define LIMIT2 2

#define STEP3 6
#define DIR3 9
#define LIMIT3 4

#define PUMP 12
#define SENSOR A0


enum State {
  IDLE,
  RUNNING,
  PICKING_UP
};

State currentState = IDLE;
bool abortRequested = false;

String inputBuffer = "";
bool newMessage = false;

//Motor set up
AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);

MultiStepper steppers;
                  
float pickupSpeed = 800.0; 
float minSpeed = 50.0;
float maxSpeed = 2000.0;
float maxAcceleration = 1500.0;

//Move to target setup
#define MAX_WAYPOINTS 100
int positions[MAX_WAYPOINTS][3];
int lookahead_threshold = 50;
int lookahead_threshold_pick = 5;
int waypoint_count = 0;
int current_index = 0;

//Move down set up
#define MAX_PICKDOWN 10
int pickdown_positions[MAX_PICKDOWN][3];
int pickupPauseTime = 400;
int num_waypoints_down = 0; 
int current_index_down = 0;
bool receivingPickdown = false;

//Pressure sensor set up
const float R = 250.0;
const float Vcc = 5.0; //Arduino ref voltage
const float pickupThreshold = 0.7;  // Pressure below this means candy is picked

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

bool pickupComplete = false;

long stepper_positions[3];  // target steps for each motor

void reset_variables() {
  waypoint_count = 0;
  current_index = 0;
  num_waypoints_down = 0;
  current_index_down = 0;
  dropoffPlanned = false;
  receivingPickdown = false;

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
  const int numSamples = 10;
  float totalPressure = 0.0;

  for (int i = 0; i < numSamples; i++) {
    int rawADC = analogRead(SENSOR);
    float voltage = rawADC * Vcc / 1023.0;
    float current_mA = voltage / R * 1000.0;
    float pressure_bar = (current_mA - 4.0) * (4.0 / 16.0);
    totalPressure += pressure_bar;
    delayMicroseconds(500); // total delay = 5ms, fast enough
  }

  float avgPressure = totalPressure / numSamples;
  return avgPressure < pickupThreshold;
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
    Serial.println("ABORTED");
    reset_variables();
    homeAllMotors();
  }
}

void homeAllMotors() {
  stopAllMotors();
  motor1.setMaxSpeed(maxSpeed);
  motor2.setMaxSpeed(maxSpeed);
  motor3.setMaxSpeed(maxSpeed);
  // Move all motors toward their limits simultaneously
  motor1.move(-2000);  // move far enough to ensure hitting limit
  motor2.move(-2000);
  motor3.move(-2000);

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
  motor1.moveTo(100);
  motor2.moveTo(100);
  motor3.moveTo(100);

  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
  }  
}

bool motorsRunning() {
  return motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0;
}


void moveToPosition(int idx, int positionArray[][3], float speed) {
  motor1.setMaxSpeed(speed);
  motor2.setMaxSpeed(speed);
  motor3.setMaxSpeed(speed);

  stepper_positions[0] = positionArray[idx][0];
  stepper_positions[1] = positionArray[idx][1];
  stepper_positions[2] = positionArray[idx][2];

  steppers.moveTo(stepper_positions);

  Serial.println("STATE SPEED: " + String(speed, 2) + " Actual speed: " + String(motor1.speed(), 2));
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
      Serial.println("READY");
    }
    else if (inputBuffer == "DOWN") {
      receivingPickdown = true;
      pickupComplete = false;
      Serial.println("READY");
    }
    else if (inputBuffer == "DROPP") {
      dropoffPlanned = true;
    }
    else if (inputBuffer.startsWith("ANGLES")) {
      int a1, a2, a3;
      if (sscanf(inputBuffer.c_str(), "ANGLES %d,%d,%d", &a1, &a2, &a3) == 3) {
        if (receivingPickdown) {
          if (num_waypoints_down < MAX_PICKDOWN) {
            pickdown_positions[num_waypoints_down][0] = a1;
            pickdown_positions[num_waypoints_down][1] = a2;
            pickdown_positions[num_waypoints_down][2] = a3;
            num_waypoints_down++;
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
        moveToPosition(current_index++, positions, maxSpeed);  // Start first move immediately
        currentState = RUNNING;
        Serial.println("STATE RUNNING");
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

  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);
  pinMode(LIMIT3, INPUT_PULLUP);
  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, LOW); 

  pinMode(13, OUTPUT); //LED onboard
  digitalWrite(13, LOW);  // Turn LED on

  motor1.setMaxSpeed(maxSpeed);
  motor2.setMaxSpeed(maxSpeed);
  motor3.setMaxSpeed(maxSpeed);

  motor1.setAcceleration(maxAcceleration);
  motor2.setAcceleration(maxAcceleration);
  motor3.setAcceleration(maxAcceleration);

  steppers.addStepper(motor1);
  steppers.addStepper(motor2);
  steppers.addStepper(motor3);

  homeAllMotors();

  homeAllMotors();

  Serial.println("Finished setup"); // signal Jetson
}

void loop() {
  // Read Serial
  readSerialMessage();

  if (abortRequested) {
    stopAllMotors();
    reset_variables();
    currentState = IDLE;
    Serial.println("STATE IDLE");
    abortRequested = false;
    homeAllMotors();
    Serial.println("ABORTED");
  }

  switch (currentState) {
    
  case IDLE:
    digitalWrite(13, LOW);  // Turn LED off

    break;
  

  case PICKING_UP:

    digitalWrite(PUMP, HIGH);
    if (checkSensor()){
      digitalWrite(13, HIGH);  // Turn LED on

      stopAllMotors();
      delay(pickupPauseTime);


      current_index_down = num_waypoints_down;

      if (waypoint_count > 0) {
        moveToPosition(waypoint_count - 1, positions, maxSpeed/3);
      }
      pickupComplete = true;
      currentState = RUNNING;
      Serial.println("STATE RUNNING");
      break;
    }
    
    if (current_index_down < num_waypoints_down) {
      if (motor1.distanceToGo() < lookahead_threshold_pick &&
          motor2.distanceToGo() < lookahead_threshold_pick &&
          motor3.distanceToGo() < lookahead_threshold_pick) {

          moveToPosition(current_index_down++, pickdown_positions, max(pickupSpeed - current_index_down*100, minSpeed));
      }
    } else if (current_index_down == num_waypoints_down) {
      if (waypoint_count > 0) {
        moveToPosition(waypoint_count - 1, positions, maxSpeed);
      }

      Serial.println("NOT_PICKED_UP");
      digitalWrite(PUMP, LOW);
      reset_variables();
      current_index_down++;  // prevents repeating this block
      currentState = RUNNING;
      Serial.println("STATE RUNNING");
    }
    break;


case RUNNING:
  motor1.setMaxSpeed(maxSpeed);
  motor2.setMaxSpeed(maxSpeed);
  motor3.setMaxSpeed(maxSpeed);


  // Check if the target is dropped
  if (dropoffPlanned && !checkSensor()) {
    Serial.println("DROPPED");
    reset_variables();
    digitalWrite(PUMP, LOW);
    currentState = IDLE;
    Serial.println("STATE IDLE");
  }

  // ───── LOOKAHEAD for all but the last waypoint ─────
  if (current_index < waypoint_count &&
      motor1.distanceToGo() < lookahead_threshold &&
      motor2.distanceToGo() < lookahead_threshold &&
      motor3.distanceToGo() < lookahead_threshold) {

    moveToPosition(current_index++, positions, maxSpeed);
  }

  // ───── All Waypoints Done ─────
  else if (current_index == waypoint_count &&
           motor1.distanceToGo() == 0 &&
           motor2.distanceToGo() == 0 &&
           motor3.distanceToGo() == 0) {

    if (!pickupComplete && current_index_down < num_waypoints_down) {
      currentState = PICKING_UP;
      Serial.println("STATE PICKING_UP");
      motor1.setMaxSpeed(pickupSpeed);
      motor2.setMaxSpeed(pickupSpeed);
      motor3.setMaxSpeed(pickupSpeed);
    }
    else {
      if (dropoffPlanned) {
        digitalWrite(PUMP, LOW);
        dropoffPlanned = false;
      }

      Serial.println("DONE");
      reset_variables();
      currentState = IDLE;
      Serial.println("STATE IDLE");
    }
  }

  break;


  // ───── UNKNOWN STATE ─────
  default:
    Serial.println("Unknown state!");
    currentState = IDLE;
    Serial.println("STATE IDLE");
    reset_variables();
    break;
  }


  if (motorsRunning()) {
    checkLimitSwitches();
  }

  steppers.run();
}