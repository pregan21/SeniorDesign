#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <math.h>

#define FREQUENCY 440         // Target frequency in Hz
#define SAMPLE_RATE 48000   // Sample rate in Hz
#define AMPLITUDE 127           // Maximum amplitude for 8-bit DAC
#define AMPLITUDE_SILENCE 0
#define OFFSET 128              // Offset to center the sine wave in the 8-bit range
#define PI 3.14159265358979323846

#define NUM_SAMPLES ((int)(SAMPLE_RATE / FREQUENCY) + 1)

uint8_t sine_values[NUM_SAMPLES];
uint8_t sine_values_silence[NUM_SAMPLES];

void task_play_sound(void * parameters);

#include "driver/i2s.h"

#define I2S_NUM         I2S_NUM_0  // I2S port number
#define I2S_BCLK_PIN    26         // Bit Clock
#define I2S_LRC_PIN     25         // Left-Right Clock (Word Select)
#define I2S_DATA_PIN    22         // Serial Data Out

TaskHandle_t xBuzzer = NULL;

typedef struct message {
int id; 
float dist[3]; 
} message;

message dataSend;

message board1; 
message board2;
message boardsStruct[2] = {board1, board2};

bool buzzerOn = false;

void OnDataRecV(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  //Serial.print("Packet recieved from: ");
  //snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //        mac_addr [0], mac_addr [1], mac_addr [2], mac_addr [3], mac_addr [4], mac_addr [5]);
          
  //Serial.println(macStr);
  memcpy(&dataSend, incomingData, sizeof (dataSend));
  
  for(int i = 0; i < 3; i++) {
    boardsStruct[dataSend.id - 1].dist[i] = dataSend.dist[i];

    if(boardsStruct[dataSend.id - 1].dist[i] <= 1) {
      //Serial.printf("Board ID %u: %u bytes\n", dataSend.id, len);

      //Serial.printf("Object within Field! %0.3f m", boardsStruct[dataSend.id - 1].dist[i]);
      //Serial.println();

      

    }
  }

    buzzerOn = true;
    if(xBuzzer != NULL)
    { // reset timer on current buzzer 
        xTaskNotifyGive(xBuzzer);
    }
    else
    { 
        // create new task if noe exists 
        xTaskCreate(task_play_sound, "play_sound", 7000, NULL, 1, &xBuzzer);
        Serial.println("Creating");
    }
  
}


void setup() {
  Serial.begin (115200);
  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK) {
    Serial.println("Error loading ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecV));

  // Configure I2S with chosen pins
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),  // Master, TX only
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,         // Use 16-bit audio
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,         // Stereo format
    .communication_format = I2S_COMM_FORMAT_I2S,          // I2S format
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,             // Interrupt level 1
    .dma_buf_count = 8,                                   // DMA buffer count
    .dma_buf_len = 64                                    // DMA buffer length
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRC_PIN,
    .data_out_num = I2S_DATA_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE   // Not used for output only
  };

  // Install and start I2S driver
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);

    for (int i = 0; i < NUM_SAMPLES; i++) {
      sine_values[i] = (uint8_t)(OFFSET + AMPLITUDE * sin(2.0 * PI * i / NUM_SAMPLES));
      sine_values_silence[i] = (uint8_t)(OFFSET + AMPLITUDE_SILENCE * sin(2.0 * PI * i / NUM_SAMPLES));
    }

  //LED
  pinMode(23, OUTPUT);

}



void loop() {

//   if(buzzerOn && (xBuzzer == NULL)) { // unly create one buzzer at a time
//   // Create FreeRTOS task for playing sound
//     Serial.println("Creating task");
//       xTaskCreate(
//         task_play_sound,     // Task function
//         "play_sound",        // Task name
//         7000,                // Stack size
//         NULL,                // Task parameters
//         1,                   // Task priority
//         &xBuzzer             // Task handle
//       );
//   } //else if (!buzzerOn && (xBuzzer != NULL)) {
// //     vTaskDelete(xBuzzer);
// //     xBuzzer = NULL;
// //   }
//   delay(100);
}

void task_play_sound(void * parameters) {
    size_t bytes_written;
    i2s_start(I2S_NUM);
    Serial.println("testing");
    while (buzzerOn) {
    
        i2s_write(I2S_NUM, sine_values, sizeof(sine_values), &bytes_written, portMAX_DELAY);
        //delay(500);

        if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) == 0){ // count down to turn off buzzer
                buzzerOn = false; 
        }


    }

    // Stop I2S before the task exits
    i2s_stop(I2S_NUM);
    i2s_zero_dma_buffer(I2S_NUM);
    xBuzzer = NULL;
    vTaskDelete(xBuzzer);
}
