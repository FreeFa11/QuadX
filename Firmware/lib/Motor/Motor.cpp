#include <Motor.h>


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

void Motor::UpdateMotor(uint16_t A, uint16_t B, uint16_t C, uint16_t D)
{
    this->A = A;
    this->B = B;
    this->C = C;
    this->D = D;
}

void Motor::ActuateMotor()
{
    ledcWrite(MotorChannelA, A);
    ledcWrite(MotorChannelB, B);
    ledcWrite(MotorChannelC, C);
    ledcWrite(MotorChannelD, D);
}