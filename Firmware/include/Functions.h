#ifndef FUNCTIONS
#define FUNCTIONS

#define AngleRateP  .55
#define AngleRateI  .0001
#define AngleRateD  .35
#define AngleModeP  5
#define AngleModeI  1
#define AngleModeD  1



// Necessary to not exceed the PWMs resolution and range
template <typename T>
inline void Clamp(T &Data, T LowLimit = 1, T HighLimit = 1023)
{
    if (Data < LowLimit)
        {   Data = LowLimit;    }
    else if (Data > HighLimit)
        {   Data = HighLimit;   }
}
template <typename T>
inline T Clamp(T Data, T Limit)
{
    if (Data < -Limit)
        {   return -Limit;  }
    else if (Data > Limit)
        {   return Limit;   }
    else
        {   return Data;    }
}

// Inline for no Function call overhead or pass by value overhead
inline float PID(float Error, float PreviousError, float &PreviousIntegralTerm, float Time, float P, float I, float D)
{
  PreviousIntegralTerm = Clamp(PreviousIntegralTerm + I * (Error + PreviousError) / 2 * Time, 100.0f);
  return (P * Error     +     PreviousIntegralTerm    +     D * (Error - PreviousError) / Time);
}



#endif