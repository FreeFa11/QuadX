#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <USB.h>
#include <LittleFS.h>
#include <Preferences.h>

// Modular Codes
#include <Motor.h>
#include <Sensor.h>
#include <Webserver.h>
#include "Control.h"



// Task Handles & Functions
void SystemTask(void *param);
TaskHandle_t ControllerTask;
void Blinky(void *param);
TaskHandle_t BlinkyTask;

// Objects
Sensor<float> droneSensor;
Motor droneMotor;
Webserver droneWebserver;



// All the Initializations
void setup()
{
    Serial.begin(115200);
    LittleFS.begin();

//  Components
    // Wifi & Server
    droneWebserver.StartWiFi();
    droneWebserver.StartWebserver();
    // Motor
    droneMotor.InitializeMotor();
    droneMotor.StartMotor();
    // Sensor
    // droneSensor.InitializeMAG();
    droneSensor.InitializeIMU();
    droneSensor.StartSensors();


//  Tasks
    xTaskCreatePinnedToCore(
      Blinky,
      "Blinky",
      1000,
      NULL,
      3,
      &BlinkyTask,
      0
    );
    xTaskCreatePinnedToCore(
      SystemTask,
      "SystemTask",
      6000,
      NULL,
      3,
      &ControllerTask,
      0
    );
}
void loop(){vTaskDelete(NULL);}
void Blinky(void *param)
{
  uint8_t LEDS[3] = {0, 0, 0};
  uint8_t CountColor;
  uint16_t Delay = 300, CountDelay = 4;
  
  // Builtin NEOPIXEL LED 10
  pinMode(10, OUTPUT);
  
  while(true)
  {
    for (int i = 0; i < CountDelay; ++i)
    {
      for (int i = 0; i < 3; ++i)
      {
        LEDS[i] = 0;
      }

      LEDS[CountColor] = 64;
      neopixelWrite(10, LEDS[0], LEDS[1], LEDS[2]);
      vTaskDelay(Delay / portTICK_RATE_MS);

      neopixelWrite(10, 0, 0, 0);
      vTaskDelay(Delay / portTICK_RATE_MS);

      CountColor++;
      CountColor %= 3;
    }
   
    CountDelay += CountDelay;
    Delay -= (Delay)/2;
    if (Delay < 30){Delay = 500; CountDelay = 4;}
  }
}




// ************************************************ Flight System ******************************************* //

void SystemTask(void *param)
{
  System mySystem(State::Flight);
  mySystem.Start();
}


void System::FlightHandle()
{
    SensorData Attitude;
    MotorData Motor;
    ControllerData Input;
    CalibrationData Settings;

    // Read Saved CalibrationData
    ReadCalibration(Settings);

    while(xQueueIsQueueEmptyFromISR(StateQueue))
    {
      // New Input Data from Controller
      xQueueReceive(ControllerQueue, &Input, 0);

      if (Input.Toggle2 & Input.Toggle1){
        Motor.A  = Settings.motorA + Input.Slider1 + Input.JoystickX2;
        Motor.B  = Settings.motorB + Input.Slider1 - Input.JoystickX2;
        Motor.C  = Settings.motorC + Input.Slider1 + Input.JoystickY2;
        Motor.D  = Settings.motorD + Input.Slider1 - Input.JoystickY2;
      }
      else if (Input.Toggle1){
        Serial.printf("Sens1:%d\t", Settings.sensitivityS1);
        Serial.printf("X1:%d\tY1:%d\tX2:%d\tY2:%d\n", Input.JoystickX1, Input.JoystickY1, Input.JoystickX2, Input.JoystickY2);
      }
      else{
        Motor.A = 0;
        Motor.B = 0;
        Motor.C = 0;
        Motor.D = 0;
      }      

      Motor.A = Clamp(Motor.A, (int16_t)0, Settings.maxthrottle);
      Motor.B = Clamp(Motor.B, (int16_t)0, Settings.maxthrottle);
      Motor.C = Clamp(Motor.C, (int16_t)0, Settings.maxthrottle);
      Motor.D = Clamp(Motor.D, (int16_t)0, Settings.maxthrottle);
      xQueueSend(MotorQueue, &Motor, 1);

      vTaskDelay(2 / portTICK_PERIOD_MS);
    }

      // Disable the Motors
  Motor.A   = 0;
  Motor.B   = 0;
  Motor.C   = 0;
  Motor.D   = 0;
  xQueueSend(MotorQueue, &Motor, 1);

}

void System::CalibrationHandle()
{
  CalibrationData newData;
  MotorData Motor;

  while(xQueueIsQueueEmptyFromISR(StateQueue))
  {
    if (xQueueReceive(CalibrationQueue, &newData, 0))
    {
      if (newData.save)
            {
              SaveCalibration(newData);
            }
      else  {
              Motor.A   = newData.motorA;
              Motor.B   = newData.motorB;
              Motor.C   = newData.motorC;
              Motor.D   = newData.motorD;
              xQueueSend(MotorQueue, &Motor, 1);
            }      
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  // Disable the Motors
  Motor.A   = 0;
  Motor.B   = 0;
  Motor.C   = 0;
  Motor.D   = 0;
  xQueueSend(MotorQueue, &Motor, 1);
}

void System::ConnectionHandle()
{
  while(xQueueIsQueueEmptyFromISR(StateQueue))
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);   
  }
}

void System::UpdateState()
{
  xQueueReceive(StateQueue, &(this->currentState), 0);
}