#include <Wire.h>
#include <Arduino.h>

#define TESEO_I2C_ADDR 0x3A  // Teseo-LIV4FTR default address

void setup() {
    Serial.begin(115200);
    Wire.begin(18, 19);  // ESP32 I2C Pins
}

void loop() {
    Wire.requestFrom(TESEO_I2C_ADDR, 32);  // Request 32 bytes from GPS
    while (Wire.available()) {
        char c = Wire.read();
        Serial.print(c);  // Print raw GPS data
    }
    Serial.println();
    delay(1000);  // Wait a second before next read
}
