#ifndef CONTROL
#define CONTROL

#include <type_traits>


// Universal Clamp
template<typename T>
inline T Clamp(T Data, T minValue, T maxValue)
{
    return (Data>maxValue)?maxValue:(Data<minValue)?minValue:Data;
}

template<typename T>
inline typename std::enable_if<std::is_signed<T>::value, T>::type
Clamp(T Data, T maxAbsolute)
{
    return (Data>maxAbsolute)?maxAbsolute:(Data<-maxAbsolute)?-maxAbsolute:Data;
}


// Controller
class PID
{
private:
    float P=0, I=0, D=0;
    float IntegralLimit=0;                  // Integral Limit
    float ControlPeriod=1;                  // Loop Period (T)
    float ControlFrequency=1;               // Precompute (1/T)
    float Eprevious=0, Iprevious=0;         // Previous Terms

public:
    PID();
    PID(float Pgain, float Igain, float Dgain, float IntegralLimit, float TimePeriod);
    void SetGain(float Pgain, float Igain, float Dgain);
    void SetControlPeriod(float Period);
    void SetControlFrequency(float Frequency);
    void SetIntegralLimit(float AbsoluteLimit);
    float Update(float Error);
};


// System State
enum State {Flight, Calibration, Connection};
class System
{
private:
    State currentState;

public:
    System(State DefaultState = State::Flight)  {currentState = DefaultState;}
    ~System()                                   {}

    void Start()
    {
        while (true)
        {
            switch (currentState)
            {
            case State::Flight:
                FlightHandle();
                UpdateState();
                break;

            case State::Calibration:
                CalibrationHandle();
                UpdateState();
                break;

            case State::Connection:
                ConnectionHandle();
                UpdateState();
                break;
            }
        }
    }

    void UpdateState();
    void FlightHandle();
    void CalibrationHandle();
    void ConnectionHandle();
};


#endif