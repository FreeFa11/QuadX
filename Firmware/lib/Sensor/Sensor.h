// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "math.h"
#include "Wire.h"
#include "MPU9250.h"
#include "HMC5883L.h"
#include "../../include/Functions.h"

// Definitions
#define SDAWire             8
#define SCLWire             7
#define SDAWire1            2
#define SCLWire1            15

#define IMUDelay            3
#define IMUFrequency        (1000 / IMUDelay)
#define AlphaCF             .98

typedef struct {
    float Pitch, Roll, Yaw, PitchRate, RollRate, YawRate;
    uint32_t Time = 1;
} SensorData;




// Objects
extern QueueHandle_t SensorQueue;
extern TaskHandle_t SensorTaskHandle;


// Declaration
void SensorTask(void* param);

class Sensor
{
private:
    float AX, AY, AZ, GX, GY, GZ;
    // float MX, MY, MZ;
    float AccPitch, AccRoll, GyroPitch, GyroRoll, GyroYaw;
    float GyroRollRate, GyroPitchRate, GyroYawRate;
    float Roll, Pitch, Yaw;
    uint32_t CurrentTime, TimeInterval;
    
public:
    Sensor();
    ~Sensor();

    void InitializeIMU();
    void InitializeCompass();
    void UpdateData();
    void UpdateData(int16_t &AX, int16_t &AY, int16_t &AZ, int16_t &GX, int16_t &GY, int16_t &GZ);
    void UpdateOrientation();
    void UpdateOrientation(float &Roll, float &Pitch, float &Yaw, float &RollRate, float &PitchRate, float &YawRate, uint32_t &Time);
    void StartSensors();
};