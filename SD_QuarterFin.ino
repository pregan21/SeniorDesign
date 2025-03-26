#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

const int trigPin[3] = {17, 4, 19};
const int echoPin[3] = {18, 16, 21};

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

long duration[3];
float distance[3];

//MAC Address for reciever/main PCB
uint8_t broadcastAddress[] = {0x30, 0xc6, 0xf7, 0x43, 0xec, 0x50};

typedef struct message {
  int id; 
  float dist[3]; 
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

  //Set pins for each ultrasonic sensor
  for(int i = 0; i < 3; i++) {
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }

  //configure LED
  pinMode(23, OUTPUT);

}

void loop() {

  dataSend.id = 1;

  for(int i = 0; i < 3; i++) {
    // Clears the trigPin
    digitalWrite(trigPin[i], LOW);
    delayMicroseconds(2);

    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[i], LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration[i] = pulseIn(echoPin[i], HIGH);
    distance[i] = (duration[i] * SOUND_SPEED/2) / 100;
  }

  for(int i = 0; i < 3; i++) {
    dataSend.dist[i] = distance[i];

    if(dataSend.dist[i] <= 0.5) {

      Serial.printf("Distance of sensor %d (m): ", i);
      Serial.println(distance[i]);

      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataSend, sizeof (dataSend));

      if(result == ESP_OK) {
        Serial. println("Sent successfully");
      } else {
        Serial.println("Error sending data");
      }
    }
  }

  /*esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataSend, sizeof (dataSend));

  if(result == ESP_OK) {
    Serial. println("Sent successfully");
  } else {
    Serial.println("Error sending data");
  }*/

  delay(100);

}
