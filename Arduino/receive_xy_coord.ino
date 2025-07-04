String inputString = "";   // holds incoming data
float x_value = 0.0;
float y_value = 0.0;
bool newData = false;

void setup() {
  Serial.begin(57600);
  while (!Serial); // optional: wait for serial on boards like Leonardo

  Serial.println("Ready to receive x,y data...");
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    char inChar = Serial.read();

    // End of message
    if (inChar == '\n') {
      newData = true;
    } else {
      inputString += inChar;
    }
  }

  if (newData) {
    // Parse the string: expecting "x,y"
    int commaIndex = inputString.indexOf(',');

    if (commaIndex > 0) {
      String x_str = inputString.substring(0, commaIndex);
      String y_str = inputString.substring(commaIndex + 1);

      x_value = x_str.toFloat();
      y_value = y_str.toFloat();

      Serial.print("Received X: ");
      Serial.print(x_value);
      Serial.print(" cm, Y: ");
      Serial.print(y_value);
      Serial.println(" cm");

      // TODO: use x_value and y_value for motor control
    } else {
      Serial.println("⚠️ Invalid format (missing comma)");
    }

    // Reset for next input
    inputString = "";
    newData = false;
  }

  delay(10); // avoid flooding the serial buffer
}
