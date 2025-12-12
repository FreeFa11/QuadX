// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// Definition
#define MotorFrequency          20000
#define MotorResolution         10
#define MotorPinA               2
#define MotorPinB               3
#define MotorPinC               4
#define MotorPinD               5
#define MotorChannelA           0
#define MotorChannelB           1
#define MotorChannelC           2
#define MotorChannelD           3

typedef struct {
    int16_t A = 1023, B = 1023, C = 1023, D = 1023;
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
