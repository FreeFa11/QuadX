// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// Definition
#define MotorFrequency          30000
#define MotorResolution         10
// #define MotorPinA               2
// #define MotorPinB               3
// #define MotorPinC               4
// #define MotorPinD               5
#define MotorPinA               4
#define MotorPinB               5
#define MotorPinC               2
#define MotorPinD               3
#define MotorChannelA           0
#define MotorChannelB           1
#define MotorChannelC           2
#define MotorChannelD           3




class Motor
{
public:
    uint16_t A, B, C, D;

    Motor();
    ~Motor();

    void InitializeMotor();
    void UpdateMotor(uint16_t A=0, uint16_t B=0, uint16_t C=0, uint16_t D=0);
    void ActuateMotor();
};
