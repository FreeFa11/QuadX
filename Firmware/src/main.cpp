#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <USB.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_now.h>

// Modular Codes
#include <Motor.h>
#include <Sensor.h>
#include <Webserver.h>
#include <Transmitter.h>
#include <Control.h>


// Task Handles & Functions
void SystemTask(void *param);
TaskHandle_t ControllerTask;
void Blinky(void *param);
TaskHandle_t BlinkyTask;
hw_timer_t *timer_sensor = NULL;
Webserver droneWebserver;

static void IRAM_ATTR onTimerSensor(){
    BaseType_t xHigherPriorityWoken = pdFALSE;
    vTaskNotifyGiveFromISR(ControllerTask, &xHigherPriorityWoken);
    if (xHigherPriorityWoken == pdTRUE) {portYIELD_FROM_ISR();}
}



// All the Initializations
void setup()
{
  USBSerial.begin(115200);
  LittleFS.begin();

// Wifi & Server
  droneWebserver.StartWiFi();
  droneWebserver.StartWebserver();

  // Transmitter
  StartReciever();

  // Test Sensor
  // Sensor<float> droneSensor;
  // droneSensor.InitializeIMU();
  // droneSensor.InitializeBARO();
  // droneSensor.StartSensors();
  // while(true){delay(1000);}


//  Tasks
  // xTaskCreatePinnedToCore(
  //   Blinky,
  //   "Blinky",
  //   1000,
  //   NULL,
  //   3,
  //   &BlinkyTask,
  //   0
  // );

  
  xTaskCreatePinnedToCore(
    SystemTask,
    "SystemTask",
    6000,
    NULL,
    3,
    &ControllerTask,
    1
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
  // Hardware Timer for accuracy
  timer_sensor = timerBegin(0, 80, true);       // 1 MHz
  timerAttachInterrupt(timer_sensor, &onTimerSensor, true);
  timerAlarmWrite(timer_sensor, TimerRateModeCount, true);

  System mySystem(State::Flight);
  mySystem.Start();
}


void System::FlightHandle()
{
  // Peripheral Objects
  Sensor<float> droneSensor;
  Quaternion<float> droneAttitude;
  Motor droneMotor;
  CalibrationData droneSettings;
  ControllerData remoteInput;         // remoteInput Buffer
  
  // Control System Objects
  uint8_t counter=0;
  uint16_t altitude_counter=0;
  float InputPitch=0, InputRoll=0, InputYaw=0;
  Quaternion<float> DesiredAngle, ErrorAngle;
  PID pid_roll, pid_pitch, pid_yaw;
  float DesiredGX=0, DesiredGY=0, DesiredGZ=0;
  Quaternion<float> DesiredRateBodyFrame;
  float ErrorGX=0, ErrorGY=0, ErrorGZ=0;
  PID pid_gx, pid_gy, pid_gz;
  float CorrectedGX=0, CorrectedGY=0, CorrectedGZ=0;
  
  // CalibrationData
  ReadCalibration(droneSettings);
  pid_roll.SetGain(1, 0.05, 0);
  pid_pitch.SetGain(1, 0.05, 0);
  pid_yaw.SetGain(1, 0.05, 0);
  pid_roll.SetIntegralLimit(.5);
  pid_pitch.SetIntegralLimit(.5);
  pid_yaw.SetIntegralLimit(.5);
  pid_roll.SetControlFrequency(TimerAngleModeFreq);
  pid_pitch.SetControlFrequency(TimerAngleModeFreq);
  pid_yaw.SetControlFrequency(TimerAngleModeFreq);
  pid_gx.SetGain(droneSettings.P, droneSettings.I, droneSettings.D);
  pid_gy.SetGain(droneSettings.P, droneSettings.I, droneSettings.D);
  pid_gz.SetGain(droneSettings.P, droneSettings.I, droneSettings.D);
  pid_gx.SetIntegralLimit(150);
  pid_gy.SetIntegralLimit(150);
  pid_gz.SetIntegralLimit(150);
  pid_gx.SetControlFrequency(TimerRateModeFreq);
  pid_gy.SetControlFrequency(TimerRateModeFreq);
  pid_gz.SetControlFrequency(TimerRateModeFreq);
  

  // Initialization
  droneSensor.InitializeIMU();
  #ifdef HAS_MAGNETOMETER
  droneSensor.InitializeMAG();
  #endif
  #ifdef HAS_BAROMETER
  droneSensor.InitializeBARO();
  #endif
  droneMotor.InitializeMotor();
  timerAlarmEnable(timer_sensor);

  

  while(xQueueIsQueueEmptyFromISR(StateQueue))
  {
    // Wait for Hardware Timer Notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Update Data and Counter
    droneSensor.UpdateIMUData();
    droneSensor.NormalizeVectors();
    counter += 1;


    // ** Angle Control **//
    if(counter >= TimerModeSwitchCount)
    {
      counter = 0;

      // Update Attitude
      droneAttitude.UpdateMahony(droneSensor);
      // droneAttitude.UpdateMadgwick(droneSensor);

      if (!xQueueIsQueueEmptyFromISR(ControllerQueue))
      {
        // New Data from Queues
        xQueueReceive(ControllerQueue, &remoteInput, 0);

        // Convert Input Command
        InputRoll=remoteInput.JoystickX2/1023.f          * MAX_ROLL_RADIAN;
        InputPitch=remoteInput.JoystickY2/1023.f         * MAX_PITCH_RADIAN;
        InputYaw -= 0.02f*remoteInput.JoystickX1/1023.f  * MAX_YAW_RAD_SEC;
        if (InputYaw >  M_PI) InputYaw -= 2.0f * M_PI;
        if (InputYaw < -M_PI) InputYaw += 2.0f * M_PI;
        
        // Angle in Quaternion
        float cr = cosf(InputRoll  * 0.5f);
        float sr = sinf(InputRoll  * 0.5f);
        float cp = cosf(InputPitch * 0.5f);
        float sp = sinf(InputPitch * 0.5f);
        float cy = cosf(InputYaw   * 0.5f);
        float sy = sinf(InputYaw   * 0.5f);
  
        // Euler --> quaternion
        DesiredAngle.w = cy*cp*cr + sy*sp*sr;
        DesiredAngle.x = cy*cp*sr - sy*sp*cr;
        DesiredAngle.y = cy*sp*cr + sy*cp*sr;
        DesiredAngle.z = sy*cp*cr - cy*sp*sr;
      }
      
      // Error Attitude
      ErrorAngle = DesiredAngle.inverse()*droneAttitude*2.0f;
      ErrorAngle = (ErrorAngle.w < 0)? ErrorAngle*(-1): ErrorAngle;
      
      // PID Angle Corrected Rates
      DesiredGX = - pid_roll.Update(ErrorAngle.x);  // World Frame
      DesiredGY = - pid_pitch.Update(ErrorAngle.y);
      DesiredGZ = - pid_yaw.Update(ErrorAngle.z);
      
      // USBSerial.printf("%.4f,%.4f,%.4f,%.4f\n", DesiredAngle.w, DesiredAngle.x, DesiredAngle.y, DesiredAngle.z);        
      // USBSerial.printf("%.4f,%.4f,%.4f,%.4f\n", droneAttitude.w, droneAttitude.x, droneAttitude.y, droneAttitude.z);     
      // USBSerial.printf(">X:%.4f\n>Y:%.4f\n>Z:%.4f\n", DesiredGX, DesiredGY, DesiredGZ);
    }

    // altitude_counter += 1;
    // if (altitude_counter >= TimerAltitudeCount) {
    //   altitude_counter = 0;
      // droneSensor.UpdateBAROData();
    // }
    
    USBSerial.printf(">X:%.2f\n>Y:%.2f\n>Z:%.2f\n", droneSensor.GX, droneSensor.GY, droneSensor.GZ);    
    
    //** Rate Control **//
    // Actuation is done in body frame. Desired World Rate -> Desired Body Rate
    DesiredRateBodyFrame = droneAttitude.inverse()*Quaternion<float>(0,DesiredGX,DesiredGY,DesiredGZ)*droneAttitude;
    // ErrorGX = DesiredRateBodyFrame.x - droneSensor.GX;
    // ErrorGY = DesiredRateBodyFrame.y - droneSensor.GY;
    // ErrorGZ = DesiredRateBodyFrame.z - droneSensor.GZ;
    ErrorGX = 0 - droneSensor.GX;
    ErrorGY = 0 - droneSensor.GY;
    ErrorGZ = 0 - droneSensor.GZ;
    CorrectedGX = pid_gx.Update(ErrorGX);
    CorrectedGY = pid_gx.Update(ErrorGY);
    CorrectedGZ = pid_gx.Update(ErrorGZ);


    if (remoteInput.Toggle1){
      // Mixer
      droneMotor.A  = Clamp(uint16_t(droneSettings.motorA + remoteInput.Slider1*5 + CorrectedGX + CorrectedGY - CorrectedGZ), uint16_t(0), uint16_t(droneSettings.maxthrottle));
      droneMotor.B  = Clamp(uint16_t(droneSettings.motorB + remoteInput.Slider1*5 + CorrectedGX - CorrectedGY + CorrectedGZ), uint16_t(0), uint16_t(droneSettings.maxthrottle));
      droneMotor.C  = Clamp(uint16_t(droneSettings.motorC + remoteInput.Slider1*5 - CorrectedGX - CorrectedGY - CorrectedGZ), uint16_t(0), uint16_t(droneSettings.maxthrottle));
      droneMotor.D  = Clamp(uint16_t(droneSettings.motorD + remoteInput.Slider1*5 - CorrectedGX + CorrectedGY + CorrectedGZ), uint16_t(0), uint16_t(droneSettings.maxthrottle));
      droneMotor.ActuateMotor();
    }
    else{
      droneMotor.UpdateMotor(0,0,0,0);
      droneMotor.ActuateMotor();
    }      
  }

  // Disable the Motors
  droneMotor.UpdateMotor(0,0,0,0);
  droneMotor.ActuateMotor();
  timerAlarmDisable(timer_sensor);
}

void System::CalibrationHandle()
{
  CalibrationData newData;
  Motor droneMotor;

  while(xQueueIsQueueEmptyFromISR(StateQueue))
  {
    if (xQueueReceive(CalibrationQueue, &newData, 0))
    {
      if (newData.save) {
          SaveCalibration(newData);
        }
      else {
          droneMotor.UpdateMotor(newData.motorA, newData.motorB, newData.motorC, newData.motorD);
          droneMotor.ActuateMotor();
        }      
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }

  // Disable the Motors
  droneMotor.UpdateMotor(0,0,0,0);
  droneMotor.ActuateMotor();
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