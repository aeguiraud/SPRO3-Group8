#include <Arduino.h>

void setup() {
  // Start the serial communication at a baud rate of 9600
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');

    if (commaIndex == -1 || commaIndex == 0 || commaIndex == data.length() - 1) {
      Serial.println("Error: Invalid data format");
      return;
    }

    String value1Str = data.substring(0, commaIndex);
    String value2Str = data.substring(commaIndex + 1);

    int value1 = value1Str.toInt();
    int value2 = value2Str.toInt();

    if (value1Str.toInt() == 0 && value1Str != "0" || value2Str.toInt() == 0 && value2Str != "0") {
      Serial.println("Error: Non-integer data received");
      return;
    }

    Serial.print("Received Value 1: ");
    Serial.print(value1);
    Serial.print(", Value 2: ");
    Serial.println(value2);

    // Send back an acknowledgement
    Serial.println("ACK: Data Received");
  }
}
