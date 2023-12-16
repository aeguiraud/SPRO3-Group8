#include <Arduino.h>

const int distancePin = A0; // Analog pin for distance value
const int speedPin = A1;    // Analog pin for speed value

void setup() {
  // Initialize USART communication
  Serial.begin(9600); // Set the baud rate to match the receiver (Arduino Uno)
}

void loop() {
  // Read the distance and speed values from analog pins
  int distanceValue = analogRead(distancePin);
  int speedValue = analogRead(speedPin);

  // Send the data as a binary packet (two 16-bit integers) using USART
  Serial.write((byte*)&distanceValue, sizeof(distanceValue));
  Serial.write((byte*)&speedValue, sizeof(speedValue));

  // Add a delay or any other necessary logic here
  delay(1000); // Delay for 1 second before sending the next packet
}
