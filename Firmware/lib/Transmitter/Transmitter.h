#pragma once

// Includes
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "esp_now.h"


// TypeDefines
enum States
{
    Control,
    Calibrate,
    Connect
};

typedef struct __attribute__((packed)){
    int16_t     Slider1=0, Slider2=0, Slider3=0, Slider4=0;
    int16_t     JoystickX1=0, JoystickX2=0, JoystickY1=0, JoystickY2=0;
    bool        Toggle1=false, Toggle2=false, Toggle3=false, Toggle4=false;
} ControllerData;

typedef struct {
    bool        save=false; 
    int16_t     motorA=0, motorB=0, motorC=0, motorD=0, maxthrottle=0;
    float       P=0, I=0, D=0;
    uint8_t     sensitivityS1=0, sensitivityS2=0, sensitivityS3=0, sensitivityS4=0;
    uint8_t     sensitivityX1=0, sensitivityX2=0, sensitivityY1=0, sensitivityY2=0;
} CalibrationData;


// Objects
extern QueueHandle_t ControllerQueue;
extern QueueHandle_t CalibrationQueue;
extern QueueHandle_t StateQueue;


// Functions
void StartReciever();
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
