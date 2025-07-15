const int analogPin = A0;    // Sensor connected here (after 250Ω resistor)
const int pumpPin = 12;      // Digital pin to control the vacuum pump relay or transistor

const float R = 250.0;       // Shunt resistor in ohms (250Ω = 1V to 5V for 4–20mA)
const float Vcc = 5.0;       // Arduino analog reference voltage (usually 5V)

String inputBuffer = "";     // For reading serial input

void setup() {
  Serial.begin(9600);
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW);  // Start with pump OFF

  Serial.println("Vacuum Sensor Test Ready");
  Serial.println("Type ON or OFF in Serial Monitor to control the pump.");
}

void loop() {
  // --- Read sensor ---
  int rawADC = analogRead(analogPin);
  float voltage = rawADC * Vcc / 1023.0;
  float current_mA = voltage / R * 1000.0;
  float pressure_bar = (current_mA - 4.0) * (4.0 / 16.0);  // Scale 4–20 mA to 0–4 bar

  Serial.print("Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | Current: ");
  Serial.print(current_mA, 2);
  Serial.print(" mA | Pressure: ");
  Serial.print(pressure_bar, 2);
  Serial.println(" bar");

  // --- Handle serial input to toggle pump ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuffer.trim();

      if (inputBuffer.equalsIgnoreCase("ON")) {
        digitalWrite(pumpPin, HIGH);
        Serial.println("Pump turned ON");
      } else if (inputBuffer.equalsIgnoreCase("OFF")) {
        digitalWrite(pumpPin, LOW);
        Serial.println("Pump turned OFF");
      }

      inputBuffer = "";  // Reset buffer
    } else {
      inputBuffer += c;
    }
  }

  delay(500); // Slow down readings
}
