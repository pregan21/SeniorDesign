#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <esp_now.h>
#include <WiFi.h>

Adafruit_VL53L1X sensor = Adafruit_VL53L1X();

//MAC Address for reciever/main PCB
uint8_t broadcastAddress[] = {0x30, 0xc6, 0xf7, 0x43, 0xec, 0x50};

typedef struct message {
  int id; 
  int dist; 
} message;

message dataSend;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status: \t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  //Initalize ESP-NOW
  if (esp_now_init() != ESP_OK) {
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

  //Initialize LiDAR
  Wire.begin();
  if(!sensor.begin(0x29, &Wire)) {
    Serial.printf("Error on sensor: ");
    Serial.println(sensor.vl_status);
    while(1);
  }

  if(!sensor.startRanging()) {
    Serial.print("Couldn't start ranging: ");
    Serial.println(sensor.vl_status);
    while(1);
  }

  // Set ROI to maximum (16x16) for full Field of View
  sensor.VL53L1X_SetROI(16, 16);

  // Set Long-Distance Mode to ensure full FoV and accuracy
  //sensor.VL53L1X_SetDistanceMode(VL53L1X::Long);

  sensor.setTimingBudget(100);

  //configure LED
  pinMode(23, OUTPUT);

}

void loop() {
  int16_t reading;

  //LiDAR Readings
  if(sensor.dataReady()) {
    reading = sensor.distance();

    if(reading == -1) {
      //Error
      Serial.print("Couldn't get distance: ");
      Serial.println(sensor.vl_status);
      delay(1000);
      return;
    }

    Serial.print("Reading: ");
    Serial.print(reading);
    Serial.println(" mm");

    //Transmit data wirelessly
    dataSend.id = 2;
    dataSend.dist = reading;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataSend, sizeof (dataSend));

    if(result == ESP_OK) {
      Serial. println("Sent successfully");
    } else {
      Serial.println("Error sending data");
    }
    sensor.clearInterrupt();
  }

  //Turn LED on/off
  digitalWrite(23, HIGH);
  delay(100);
  digitalWrite(23, LOW);
  delay(100);

}
