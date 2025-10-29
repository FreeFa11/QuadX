#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <USB.h>
#include <LittleFS.h>
#include <Preferences.h>

// Modular Codes
#include <Motor.h>
#include <Sensor.h>
#include <Webserver.h>
#include <Functions.h>



// Task Handles & Functions
void Controller(void *param);
TaskHandle_t ControllerTask;
void Blinky(void *param);
TaskHandle_t BlinkyTask;

// Objects
Sensor droneSensor;
Motor droneMotor;
Webserver droneWebserver;



// All the Initializations
void setup()
{
    Serial.begin(115200);


//  Components
    // Wifi & Server
    droneWebserver.StartWiFi();
    droneWebserver.StartWebserver();
    // Motor
    droneMotor.InitializeMotor();
    droneMotor.StartMotor();
    // Sensor
    // droneSensor.InitializeCompass();
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
      Controller,
      "Controller",
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
  static bool STATE=false;
  pinMode(BUILTIN_LED, OUTPUT);

  while(true)
  {
    digitalWrite(BUILTIN_LED, STATE);
    STATE = !STATE;

    vTaskDelay(500 / portTICK_RATE_MS);
  }
}



// ************************************************ Flight Controller ******************************************* //

void Controller(void *param)
{
    SensorData Attitude;
    MotorData Motor;
    ControllerData Input;
    float Roll, Pitch, Yaw, RollE, PitchE, YawE;                  // Error
    float RollR, PitchR, YawR, RollRE, PitchRE, YawRE;            // Rate, Rate Error
    float RollREP=0, PitchREP=0, YawREP=0;                              // Rate Error Previous
    float RollITP=0, PitchITP=0, YawITP=0;                              // Integral Term Previous

    // Deactivated
    while (Input.Toggle1 != true)
    {
      vTaskDelay(20 / portTICK_RATE_MS);
      xQueueReceive(InputQueue, &Input, 0);
    }

    while(true)
    {
        // Refresh Data
        if (xQueueReceive(SensorQueue, &Attitude, 0) == pdTRUE)
        {
          // Serial.print(">Time:");
          // Serial.println(Attitude.Time);
          xQueueReceive(InputQueue, &Input, 0);


      // Error  (External)
          // Angle Mode
          RollE  = Input.JoystickX2 * .2f - Attitude.Roll;
          PitchE = Input.JoystickY2 * .2f - Attitude.Pitch;
          YawE   = Input.JoystickX1 * .2f - Attitude.Yaw;

      // PID    (External)
          // Angle Mode
          Roll    = AngleModeP * RollE;
          Pitch   = AngleModeP * PitchE;
          Yaw     = AngleModeP * YawE;


      // Error  (Internal)
          // Angle Mode
          RollRE  = Roll    -   Attitude.RollRate * .007f;
          PitchRE = Pitch   -   Attitude.PitchRate * .007f;
          YawRE   = Yaw     -   Attitude.YawRate * .007f;
          // This has been scaled to get values near each other;
          // Angle Rate Mode
          // RollRE  = Input.JoystickX2*5.0f   -   Attitude.RollRate * .015f;
          // PitchRE = Input.JoystickY2*5.0f   -   Attitude.PitchRate * .015f;
          // YawRE   = Input.JoystickX1*5.0f   -   Attitude.YawRate * .015f;


      // PID    (Internal)
          RollR    = PID(RollRE, RollREP, RollITP, Attitude.Time, AngleRateP, AngleRateI, AngleRateD);
          PitchR   = PID(PitchRE, PitchREP, PitchITP, Attitude.Time, AngleRateP, AngleRateI, AngleRateD);
          YawR     = PID(YawRE, YawREP, YawITP, Attitude.Time, AngleRateP, AngleRateI, AngleRateD);
          RollREP = RollRE;
          PitchREP = PitchRE;
          YawREP = YawRE;


      // Motor Output
          Motor.One   = 1023 - 5 * Input.Slider1 - Input.Slider4     -     RollR  +  PitchR   +  YawR;
          Motor.Two   = 1023 - 5 * Input.Slider1 - Input.Slider4     +     RollR  +  PitchR   -  YawR;
          Motor.Three = 1023 - 5 * Input.Slider1 - Input.Slider4     +     RollR  -  PitchR   +  YawR;
          Motor.Four  = 1023 - 5 * Input.Slider1 - Input.Slider4     -     RollR  -  PitchR   -  YawR;
          Clamp(Motor.One, 500, 1023); Clamp(Motor.Two, 500, 1023);
          Clamp(Motor.Three, 500, 1023); Clamp(Motor.Four, 500, 1023);
          xQueueSend(MotorQueue, &Motor, 1);
      
        }

        // Deactivated
        if (Input.Toggle1 == false)
        {
          while (Input.Toggle1 == false)
          {
            Motor.One = 1023; Motor.Two = 1023; Motor.Three = 1023; Motor.Four = 1023;
            xQueueSend(MotorQueue, &Motor, 1);
            vTaskDelay(30 / portTICK_RATE_MS);
            xQueueReceive(InputQueue, &Input, 0);
          }
        }
    }
}