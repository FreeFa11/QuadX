// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "Pinout.h"

// Definition
#define MotorFrequency          24000
#define MotorResolution         10
#define MotorPinA               PCB_M1
#define MotorPinB               PCB_M4
#define MotorPinC               PCB_M3
#define MotorPinD               PCB_M2
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
