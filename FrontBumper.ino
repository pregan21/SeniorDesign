#define TX_PIN 16 // to 10 on jetson
#define RX_PIN 25 // to 8 on jetson

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <esp_now.h>
#include <WiFi.h>

#define TESSEO_I2C_ADDR 0X3A

Adafruit_VL53L1X sensor = Adafruit_VL53L1X();

//MAC Address for reciever/main PCB
uint8_t broadcastAddress[] = {0x30, 0xc6, 0xf7, 0x43, 0xec, 0x50};

typedef struct message {
  int id; 
  float dist; 
} message;

message dataSend;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status: \t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

TwoWire I2CGPS = TwoWire(1);

void setup() {
    Serial.begin(115200);  // USB serial for debugging
    dataSend.id = 2;

    //Esp-now setup
    WiFi.mode(WIFI_STA);
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if(esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }

    Wire.begin();
    //--------------------Setup Lidar----------------------------------------------------//
    // Init Lidar
    if(!sensor.begin(0x29, &Wire)) // set Lidar bus
    {
      Serial.printf("Error initializing Lidar: ");
      Serial.println(sensor.vl_status);
      while(1);
    }

    if(!sensor.startRanging())
    {
      Serial.printf("Couldn't start ranging: ");
      Serial.println(sensor.vl_status);
      while(1);
    }

    // Set ROI to maximum
    sensor.VL53L1X_SetROI(16,16);
    
    sensor.setTimingBudget(33);

    // Setup GPS --------------------------------------------------------------------------//
    I2CGPS.begin(18,19);

    // config LED
    pinMode(23, OUTPUT);

    // UART connection with Jetson Nano -------------------------------
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("ESP32 UART TEST: Sending to Jetson");
}

bool transmit = true;
unsigned long lastSendTime = 0;
unsigned long sendInterval = 1000;
bool detectionMode = false;
double speedMPS;
String msg = "Hello Jetson";

double LidarDistance;

void loop() {

  HandleGPS(); // reads GPS and extracts speed in KPH

  HandleLidar();

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

            handleDetectedObject(received);
            receivedFlag = true;
            break;
        }
    }

    if (!receivedFlag) {
        //Serial.println("No response from Jetson.");
    }

   
}

void HandleLidar()
{
  int16_t reading;
   
  if (sensor.dataReady())
  {
    reading = sensor.distance();

    if (reading == - 1)
    {
      // Error
      Serial.print("Couldn't get distance!: ");
      Serial.println(sensor.vl_status);
      LidarDistance = 2000;
      return;
    }

    LidarDistance = reading;
    Serial.print("Reading: ");
    Serial.print(reading);
    Serial.println(" mm");


    if(LidarDistance < 500)
    {
      Serial.println("Too close hazard");
      dataSend.dist = 0.5;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataSend, sizeof (dataSend));
    }

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
    double distanceDouble = 0.0;
    distanceDouble = atof(distance.c_str());

    if(centering == "left" || centering == "right")
    {
      // lane departure detected
      Serial.println("Lane Departure!");
      dataSend.dist = 0.5;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataSend, sizeof (dataSend));
    }

    if (isTailingTooClose(distanceDouble, speedMPS)) {
      Serial.println("Tailing Hazard!");
      dataSend.dist = 0.5;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataSend, sizeof (dataSend));
    }
    else
    {
      dataSend.dist = 2000;
    }

    if(label == "person" && distanceDouble < 500 && speedMPS > 0)
    {
      Serial.println("Pedestrian in danger");
      dataSend.dist = 0.5;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataSend, sizeof (dataSend));
    }
    else 
    {
      dataSend.dist = 2000;
    }



}

void HandleGPS()
{
  int bytesRead = I2CGPS.requestFrom(TESSEO_I2C_ADDR, 64);
  String nmeaData = "";

  while (I2CGPS.available())
  {
    char c = I2CGPS.read();
    
    // Only include printable ASCII (normal characters)
    if (isPrintable(c))
      nmeaData += c;
  }

  // Optional: trim whitespace or bad endings
  nmeaData.trim();

  // Skip if it's blank or malformed
  if (nmeaData.length() == 0 || !nmeaData.startsWith("$"))
  {
    //Serial.println("⚠️ Skipped garbage or empty data");
    return;
  }

  Serial.println("📡 Received NMEA: " + nmeaData);

  // Only process $GNRMC or $GPRMC
  if (nmeaData.startsWith("$GNRMC") || nmeaData.startsWith("$GPRMC"))
  {
    int commas = 0, speedIdx = 0;

    for (int i = 0; i < nmeaData.length(); i++)
    {
      if (nmeaData[i] == ',')
      {
        commas++;
        if (commas == 7)
        {
          speedIdx = i + 1;
          break;
        }
      }
    }

    int endIdx = nmeaData.indexOf(',', speedIdx);
    String speedStr = nmeaData.substring(speedIdx, endIdx);

    speedMPS = speedStr.toFloat() * 0.5144;

    Serial.print("🚗 Speed (knots): ");
    Serial.println(speedStr);
    Serial.print("💨 Speed (m/s): ");
    Serial.println(speedMPS);
  }
}


bool isTailingTooClose(double tailingDistance, double currentSpeed)
{
  // tailingDistance is in mm
  double safeDistance = currentSpeed * 2.0; // 2-second rule

  Serial.printf("speed: %lf\n", currentSpeed);

  if(tailingDistance < 1)
  {
    return false;
  }
    

  if(currentSpeed < 1)
  {
    Serial.printf("tailing distance: %lf\n", tailingDistance );
    return (tailingDistance < 500);
   
  }
  else
  {
    return (tailingDistance / 1000) < safeDistance;
  }
}
