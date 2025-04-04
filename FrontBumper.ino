#define TX_PIN 16 // to 10 on jetson
#define RX_PIN 25 // to 8 on jetson

#include <Arduino.h>
#include <Wire.h>

#define TESSEO_I2C_ADDR 0X3A

void setup() {
    Serial.begin(115200);  // USB serial for debugging

    // UART connection with Jetson Nano
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("ESP32 UART TEST: Sending to Jetson");

    // I2C connection with GPS
    Wire.begin(18,19);
}

bool transmit = true;
unsigned long lastSendTime = 0;
unsigned long sendInterval = 1000;

bool detectionMode = false;

double speedMPS;

String msg = "Hello Jetson";

void loop() {

  HandleGPS(); // reads GPS and extracts speed in KPH

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

    if (isTailingTooClose(distanceDouble, speedMPS)) {
      Serial.println("Tailing Hazard!");
    }

    if(label == "person" && distanceDouble < 500 && speedMPS > 0)
    {
      Serial.println("Pedestrian in danger");
    }
    

}

String lastScentence = "";

void HandleGPS()
{

  Wire.requestFrom(TESSEO_I2C_ADDR, 32);
  String nmeaData = "";

  while(Wire.available())
  {
    char c = Wire.read();
    nmeaData += c;
  }

  if(nmeaData != lastScentence)
  {
    int start = nmeaData.indexOf("$GPRMC");
    if(start != -1)
    {
      int commas = 0, speedIdx= 0;
      for (int i = start; i < nmeaData.length(); i++)
      {
        if(nmeaData[i] == ',')
        {
          if(++commas == 7) { speedIdx = i + 1; break; }
        }
      }

      int endIdx = nmeaData.indexOf(',', speedIdx);
      speedMPS = nmeaData.substring(speedIdx, endIdx).toFloat() * 0.5144;
    }
  }
}

bool isTailingTooClose(double tailingDistance, double currentSpeed)
{
  // tailingDistance is in mm
  double safeDistance = currentSpeed * 2.0; // 2-second rule

  if(currentSpeed < 0)
  {
    if (tailingDistance < 250)
    {
      return true;
    }
  }
  else
  {
    return (tailingDistance / 1000) < safeDistance;
  }
}



