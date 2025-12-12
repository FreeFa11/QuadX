#pragma once

// Definitions
#define SDAWire             1
#define SCLWire             0
#define IMUDelay            2
#define IMUFrequency        (1000 / IMUDelay)


// Filter Tuning
#define NumericType         float
#define Kp                  .5          // Mahony
#define Ki                  .003
#define Kacc                1
#define Kmag                2
#define Beta                0.5         // Madgwick






// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "math.h"
#include "Wire.h"
#include "MPU6500.h"
#include "Control.h"



// Structures
typedef struct {
    NumericType Pitch, Roll, Yaw, PitchRate, RollRate, YawRate;
} SensorData;


// Objects
extern QueueHandle_t SensorQueue;
extern TaskHandle_t SensorTaskHandle;


// Declaration
void SensorTask(void* param);



template<typename T>
class Sensor
{
// private:                                                 // Later
public:
    T AX, AY, AZ, GX, GY, GZ;                               // IMU
    T MX, MY, MZ;                                           // MAG
    
public:
    Sensor();
    ~Sensor();

    void InitializeIMU();
    void InitializeMAG();
    void UpdateIMUData();
    void UpdateIMUData(T &AX, T &AY, T &AZ, T &GX, T &GY, T &GZ);
    void UpdateMAGData();
    void UpdateMAGData(T &MX, T &MY, T &MZ);    
    void NormalizeVectors();
    void StartSensors();
};
template<typename T>



class Quaternion
{
private:
    void Normalize();
public:
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
    void UpdateMahony(T &GX, T &GY, T &GZ, T &AX, T &AY, T &AZ, T &MX, T &MY, T &MZ);
    void UpdateMadgwick(Sensor<T> &S);
    void UpdateMadgwick(T &GX, T &GY, T &GZ, T &AX, T &AY, T &AZ, T &MX, T &MY, T &MZ);    
};