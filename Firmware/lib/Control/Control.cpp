#include "Control.h"


PID::PID(){}
PID::PID(float Pgain, float Igain, float Dgain, float IntegralLimit, float TimePeriod)
{
    P = Pgain;
    I = Igain;
    D = Dgain;
    this->IntegralLimit = IntegralLimit;
    this->ControlPeriod = TimePeriod;
    this->ControlFrequency = 1/ControlPeriod;
}

void PID::SetGain(float Pgain, float Igain, float Dgain)
{
    P = Pgain;
    I = Igain;
    D = Dgain;
}
void PID::SetControlPeriod(float Period)
{
    this->ControlPeriod = Period;
    this->ControlFrequency = 1/ControlPeriod;
}
void PID::SetControlFrequency(float Frequency)
{
    this->ControlFrequency = Frequency;
    this->ControlPeriod = 1/ControlFrequency;
}
void PID::SetIntegralLimit(float AbsoluteLimit)
{
    this->IntegralLimit = AbsoluteLimit;
}

float PID::Update(float Error)
{
    Iprevious = Clamp(Iprevious + I*(Error+Eprevious)*0.5f*ControlPeriod, IntegralLimit);
    float output = P*Error     +     Iprevious    +     D*(Error-Eprevious)*ControlFrequency;
    Eprevious = Error;                                      // F=1/T to reduce Division
    return output; 
}