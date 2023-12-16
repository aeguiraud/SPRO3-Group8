#include <Arduino.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <SPI.h>



char ssid[] = "ALS-328";
char pass[] = "TNNgcht5";
// char ssid[] = "Avi's iPhone";
// char pass[] = "12345678";
int status = WL_IDLE_STATUS;   // the Wifi radio's status

WiFiServer server(80);

int value1 = 0; // Initialize value 1
int value2 = 0; // Initialize value 2

void setup() {
  Serial.begin(9600);

  // Start the WiFi connection
  if (WiFi.begin(ssid,pass) != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println("Connected to WiFi");
  }

  server.begin();
}

void loop() {
  // Listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // HTTP headers
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // Web page content for status display
            client.println("<html><body>");
            client.println("<h1>Arduino MKR Status</h1>");
            client.print("<p>Last received Value 1: ");
            client.print(value1);
            client.print("</p><p>Last received Value 2: ");
            client.print(value2);
            client.println("</p></body></html>");

            // End of the response
            client.println();
            break;
          } else if (currentLine.startsWith("GET /send?")) {
            int value1Index = currentLine.indexOf("value1=") + 7;
            int value2Index = currentLine.indexOf("&", value1Index);
            value1 = currentLine.substring(value1Index, value2Index).toInt();
            value2 = currentLine.substring(currentLine.indexOf("value2=") + 7).toInt();

            // Send values to Uno
            Serial.print(value1);
            Serial.print(",");
            Serial.println(value2);

            // Wait for acknowledgement from Uno
            unsigned long startTime = millis();
            while (!Serial.available()) {
              // Timeout after 5 seconds
              if (millis() - startTime > 5000) {
                Serial.println("Error: Acknowledgement timeout");
                break;
              }
            }

            if (Serial.available()) {
              // Read the acknowledgement
              String ack = Serial.readStringUntil('\n');
              Serial.println("Received ACK: " + ack);
            }
          }
          currentLine = "";
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
  }
}
