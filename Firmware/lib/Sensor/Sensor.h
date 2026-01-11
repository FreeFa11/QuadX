#pragma once

// Definitions
#define SDAWire             1
#define SCLWire             0

// Filter Tuning
#define NumericType         float
#define Kp                  1           // Mahony
#define Ki                  .1
#define Kacc                1
#define Kmag                2
#define Beta                .8          // Madgwick

// Controller Tuning
#define MAX_ROLL_DEGREE     45
#define MAX_PITCH_DEGREE    45
#define MAX_YAW_DEG_SEC     90
#define TimerRateModeFreq   1000        // Must be integer multiple of Angle Mode Frequency
#define TimerAngleModeFreq  250

// Setup
// #define HAS_MAGNETOMETER




// Derived Definitions
#define MAX_ROLL_RADIAN         MAX_ROLL_DEGREE *0.0174532925
#define MAX_PITCH_RADIAN        MAX_PITCH_DEGREE*0.0174532925
#define MAX_YAW_RAD_SEC         MAX_YAW_DEG_SEC *0.0174532925
#define TimerRateModePeriod     float(1.0f/TimerRateModeFreq)
#define TimerAngleModePeriod    float(1.0f/TimerAngleModeFreq)
#define TimerRateModeCount      1000000/TimerRateModeFreq
#define TimerAngleModeCount     1000000/TimerAngleModeFreq
#define TimerModeSwitchCount    (TimerRateModeFreq/TimerAngleModeFreq)


// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "math.h"
#include "Wire.h"
#include "MPU6500.h"
#include "HMC5883L.h"
#include "Control.h"


// Objects
extern QueueHandle_t SensorQueue;
extern TaskHandle_t SensorTaskHandle;


// Declaration
void SensorTask(void* param);



template<typename T>
class Sensor
{
public:
    T AX, AY, AZ=1, GX, GY, GZ;                               // IMU
    MPU6500 IMU;

    #ifdef HAS_MAGNETOMETER
    T MX, MY, MZ=1;                                           // MAG
    HMC5883L MAG;
    #endif
    
    
    public:
    Sensor();
    ~Sensor();
    
    void InitializeIMU();
    void UpdateIMUData();
    void UpdateIMUData(T &AX, T &AY, T &AZ, T &GX, T &GY, T &GZ);
    void NormalizeVectors();
    void StartSensors();

    #ifdef HAS_MAGNETOMETER
    void InitializeMAG();
    void UpdateMAGData();
    void UpdateMAGData(T &MX, T &MY, T &MZ);    
    #endif
};
template<typename T>


class Quaternion
{
private:
public:
void Normalize();
T w=1, x=0, y=0, z=0;

    Quaternion();
    Quaternion(T w, T x, T y, T z);
    ~Quaternion();
    
    // Operations
    Quaternion<T> operator*(Quaternion<T> Other);
    Quaternion<T> & operator*=(Quaternion<T> Other);        // Return by reference and not pointer
    Quaternion<T> operator*(T Value);
    Quaternion<T> & operator*=(T Value);                    // Return by reference and not pointer
    Quaternion<T> operator+(Quaternion<T> Other);
    Quaternion<T> operator-(Quaternion<T> Other);
    Quaternion<T> inverse();

    // Filters
    void UpdateMahony(Sensor<T> &S);
    void UpdateMadgwick(Sensor<T> &S);
};


// Structures
typedef struct {
    Quaternion<NumericType> Q;
    NumericType GX, GY, GZ;
} SensorData;