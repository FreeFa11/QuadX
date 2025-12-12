#include <Motor.h>


// Objects
QueueHandle_t MotorQueue;
TaskHandle_t MotorTaskHandle;


// Definitions
Motor::Motor(){}
Motor::~Motor(){}

void Motor::InitializeMotor()
{
    ledcSetup(MotorChannelA,  MotorFrequency, MotorResolution);
    ledcSetup(MotorChannelB,  MotorFrequency, MotorResolution);
    ledcSetup(MotorChannelC,  MotorFrequency, MotorResolution);
    ledcSetup(MotorChannelD,  MotorFrequency, MotorResolution);

    ledcAttachPin(MotorPinA,  MotorChannelA);
    ledcAttachPin(MotorPinB,  MotorChannelB);
    ledcAttachPin(MotorPinC,  MotorChannelC);
    ledcAttachPin(MotorPinD,  MotorChannelD);

    ledcWrite(MotorChannelA,  0);
    ledcWrite(MotorChannelB,  0);
    ledcWrite(MotorChannelC,  0);
    ledcWrite(MotorChannelD,  0);    
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
            ledcWrite(MotorChannelA, (uint16_t)newData.A);
            ledcWrite(MotorChannelB, (uint16_t)newData.B);
            ledcWrite(MotorChannelC, (uint16_t)newData.C);
            ledcWrite(MotorChannelD, (uint16_t)newData.D);
        }

        vTaskDelay(5 / portTICK_RATE_MS);
    }
}