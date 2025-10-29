#include <Motor.h>


// Objects
QueueHandle_t MotorQueue;
TaskHandle_t MotorTaskHandle;


// Definitions
Motor::Motor(){}
Motor::~Motor(){}

void Motor::InitializeMotor()
{
    ledcSetup(MotorChannelOne, MotorFrequency, MotorResolution);
    ledcSetup(MotorChannelTwo, MotorFrequency, MotorResolution);
    ledcSetup(MotorChannelThree, MotorFrequency, MotorResolution);
    ledcSetup(MotorChannelFour, MotorFrequency, MotorResolution);

    ledcAttachPin(MotorPinOne, MotorChannelOne);
    ledcAttachPin(MotorPinTwo, MotorChannelTwo);
    ledcAttachPin(MotorPinThree, MotorChannelThree);
    ledcAttachPin(MotorPinFour, MotorChannelFour);

    ledcWrite(MotorChannelOne, 1023);
    ledcWrite(MotorChannelTwo, 1023);
    ledcWrite(MotorChannelThree, 1023);
    ledcWrite(MotorChannelFour, 1023);    
}

void Motor::StartMotor()
{
    MotorQueue = xQueueCreate(2, sizeof(MotorData));

    xTaskCreatePinnedToCore(
        MotorTask,
        "MotorTask",
        2048,
        NULL,
        3,
        &MotorTaskHandle,
        0
    );
}

void MotorTask(void *param)
{   
    MotorData newData;

    while (true)
    {
        if (xQueueReceive(MotorQueue, &newData, 0) == pdTRUE)
        {
            ledcWrite(MotorChannelOne, newData.One);
            ledcWrite(MotorChannelTwo, newData.Two);
            ledcWrite(MotorChannelThree, newData.Three);
            ledcWrite(MotorChannelFour, newData.Four);
        }

        // Serial.print(">One:");
        // Serial.println(newData.One);
        // Serial.print(">Two:");
        // Serial.println(newData.Two);
        // Serial.print(">Three:");
        // Serial.println(newData.Three);
        // Serial.print(">Four:");
        // Serial.println(newData.Four);

        vTaskDelay(5 / portTICK_RATE_MS);
    }
}