// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// Definition
#define MotorFrequency          20000
#define MotorResolution         10
#define MotorPinOne             4
#define MotorPinTwo             5
#define MotorPinThree           3
#define MotorPinFour            2
#define MotorChannelOne         0
#define MotorChannelTwo         1
#define MotorChannelThree       2
#define MotorChannelFour        3

typedef struct {
    int One = 1023, Two = 1023, Three = 1023, Four = 1023;
} MotorData;




// Objects
extern QueueHandle_t MotorQueue;
extern TaskHandle_t MotorTaskHandle;


// Declarations
void MotorTask(void *param);

class Motor
{
public:
    Motor();
    ~Motor();

    void InitializeMotor();
    void StartMotor();
};
