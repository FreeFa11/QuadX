#include "Transmitter.h"


QueueHandle_t ControllerQueue = xQueueCreate(2, sizeof(ControllerData));
QueueHandle_t CalibrationQueue = xQueueCreate(2, sizeof(CalibrationData));
QueueHandle_t StateQueue = xQueueCreate(1, sizeof(States));


// Define Queue
void StartReciever()
{
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!!");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}


void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  static ControllerData ImuPacket;
    if (len == sizeof(ImuPacket)) {
        memcpy((void*)&ImuPacket, incomingData, sizeof(ImuPacket));
        xQueueSend(ControllerQueue, &ImuPacket, 0);
    }
}