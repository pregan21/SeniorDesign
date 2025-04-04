#define TX_PIN 16
#define RX_PIN 25

#include <Arduino.h>

void setup() {
    Serial.begin(115200);  // USB serial for debugging
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("ESP32 UART TEST: Sending to Jetson");
}

bool transmit = true;
unsigned long lastSendTime = 0;
unsigned long sendInterval = 1000;

bool detectionMode = false;

String msg = "Hello Jetson";

void loop() {
  unsigned long currentTime = millis();

  if(currentTime - lastSendTime >= sendInterval)
  {
    lastSendTime = currentTime;

    if(transmit)
    {
      Serial2.println(msg);  // Send message
      Serial.println("sent: "+ msg);
    }
    else
    {
      Serial.println("no transmission");
    }
  }


    unsigned long startTime = millis();
    bool receivedFlag = false;
    

    while (millis() - startTime < 50) {  // iterate every 50ms
        if (Serial2.available()) {
            String received = Serial2.readStringUntil('\n');  // Read until newline
            received.trim();
            Serial.print("Received from Jetson: ");
            Serial.println(received);
            if(received == "waiting to start")
            {
              msg = "ready to start";
            }
            else if(received == "trying to start process")
            {
              transmit = false;
            }
            else if ( received == "Test!")
            {
              detectionMode = true;
            }

            if(detectionMode)
            {
              if(received.length() > 0)
                handleDetectedObject(received);
            }
            receivedFlag = true;
            break;
        }
    }

    if (!receivedFlag) {
        //Serial.println("No response from Jetson.");
    }

   
}

void handleDetectedObject(String receivedDetection) {
    String label = "Unknown";      // Default value
    String distance = "N/A";       // Default value
    String centering = "Unknown";  // Default value

    receivedDetection.replace(" ", ""); 
    receivedDetection.toLowerCase();  // Normalize case (optional)
    
    // Look for keys and extract values
    int labelIndex = receivedDetection.indexOf("label:");
    if (labelIndex != -1) {
        int start = labelIndex + 6;  // Move past "label:"
        int end = receivedDetection.indexOf(',', start);
        label = receivedDetection.substring(start, (end == -1) ? receivedDetection.length() : end);
        label.trim(); // Remove spaces
    }

    int distanceIndex = receivedDetection.indexOf("distance:");
    if (distanceIndex != -1) {
        int start = distanceIndex + 9;  // Move past "distance:"
        int end = receivedDetection.indexOf(',', start);
        distance = receivedDetection.substring(start, (end == -1) ? receivedDetection.length() : end);
        distance.trim();
    }

    int centeringIndex = receivedDetection.indexOf("centering:");
    if (centeringIndex != -1) {
        int start = centeringIndex + 10;  // Move past "centering:"
        int end = receivedDetection.indexOf(',', start);
        centering = receivedDetection.substring(start, (end == -1) ? receivedDetection.length() : end);
        centering.trim();
    }

    // Debug output to Serial Monitor
    Serial.println("Parsed Data:");
    Serial.println("Label: " + label);
    Serial.println("Distance: " + distance);
    Serial.println("Centering: " + centering);
    
    double distanceDouble = atof(distance.c_str());

    if(centering == "left" || centering == "right")
    {
      Serial.println("Lane Departure!");
    }

    if (distance != "N/A" && distanceDouble > 0 && distanceDouble < 250) {
      Serial.println("Tailing Hazard!");
    }

}



