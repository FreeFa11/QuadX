#include <Sensor.h>
#include <ArduinoJson.h>
static float Gyro = 0;



// Objects
MPU9250 IMU(Wire, 0x68);
// HMC5883L Compass;
QueueHandle_t SensorQueue;
TaskHandle_t SensorTaskHandle;


// Definitions
Sensor::Sensor(){}
Sensor::~Sensor(){}

void Sensor::InitializeIMU()
{
  // Power (Not Final)
  pinMode(5, OUTPUT);pinMode(6, OUTPUT);digitalWrite(5, HIGH);digitalWrite(6, LOW);

  Wire.begin(SDAWire, SCLWire);
  IMU.begin();
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

}

void Sensor::InitializeCompass()
{
  // This is for Powering the Compass Module (1mA)
  // pinMode(32, OUTPUT); pinMode(33, OUTPUT);
  // digitalWrite(32, HIGH); digitalWrite(33, LOW);

  // Wire.begin(SDAWire, SCLWire);
  // Compass.initialize(); 
  // Compass.setDataRate(HMC5883L_RATE_75);
  // Compass.setSampleAveraging(HMC5883L_AVERAGING_8);
}

void Sensor::UpdateData()
{
// Calibration
  //            Accelerometer
  //  [ 546.656145  -2.7620121  1752.44544]     // Bias
  //  [ 0.60911795  0.04435091  0.00549269]     // Scaling & Cross Axis
  //  [-0.04806627  0.61362404  0.00194539]
  //  [-0.00647422 -0.00364235  0.60452002]
  //              Gyroscope 
  //  [241.0913333  598.598166 -149.491166]     // Bias



  // Sensor Data
  IMU.readSensor();

  AX =  0.60911795f   *   (IMU.getAccelX() - 547  );        // **Edited**
  AY =  0.61362404f   *   (IMU.getAccelY() + 3    ); 
  AZ =  0.60452002f   *   (IMU.getAccelZ() - 1752 );

  GX = IMU.getGyroX() - 241;
  GY = IMU.getGyroY() - 599;
  GZ = IMU.getGyroZ() + 149;

  Serial.print(">A:");
  Serial.println(AX);



  // Time for Calculus
  TimeInterval  =   xTaskGetTickCount() - CurrentTime;
  CurrentTime   +=  TimeInterval;
}

void Sensor::UpdateData(int16_t &AX, int16_t &AY, int16_t &AZ, int16_t &GX, int16_t &GY, int16_t &GZ)
{
  UpdateData();

  AX = this->AX;
  AY = this->AY;
  AY = this->AY;
  GX = this->GX;
  GY = this->GY;
  GZ = this->GZ;

}

void Sensor::UpdateOrientation()
{
  // Precalculation
  float sinRoll, cosRoll, cosPitch, tanPitch;
  float DynamicWeight;
  sinRoll = sinf(Roll * PI / 180.f);
  cosRoll = cosf(Roll * PI / 180.f);
  cosPitch = cosf(Pitch * PI / 180.f);
  tanPitch = tanf(Pitch * PI / 180.f);
  DynamicWeight = 1.0f - abs( sqrt(pow(AX,2) + pow(AY,2) + pow(AZ,2))  -  10000.0f) / 5000.0f;    // Linear Mapping
  Clamp(DynamicWeight, 0.0f, 1.0f);

  // Acce Attitude
  AccPitch = atanf(-AX/AZ);
  AccRoll = atanf(AY/AZ);

  // Gyro Attitude
  GyroRollRate  =   GX  +   GY * sinRoll * tanPitch   +   GZ * cosRoll * tanPitch;
  GyroPitchRate =           GY * cosRoll              -   GZ * sinRoll;
  GyroYawRate   =       -   GY * sinRoll / cosPitch   -   GZ * cosRoll / cosPitch; 

  GyroRoll  = Roll  + 0.0000069f * GyroRollRate * TimeInterval;
  GyroPitch = Pitch + 0.0000069f * GyroPitchRate * TimeInterval;
  GyroYaw   = Yaw   + 0.0000069f * GyroYawRate * TimeInterval;

  // Complementary Filter
  Pitch = AlphaCF * GyroPitch   +   DynamicWeight * (1 - AlphaCF) * AccPitch * 180 / PI;
  Roll  = AlphaCF * GyroRoll    +   DynamicWeight * (1 - AlphaCF) * AccRoll * 180 / PI;
  Yaw   = GyroYaw;


  // JsonDocument data;
  // Gyro = Gyro + 0.0000069f * GyroPitchRate * TimeInterval;
  // data["Accelerometer"] = AccPitch * 180 / PI;
  // data["Gyroscope"] = Gyro;
  // data["ComplementaryFilter"] = Pitch;
  // serializeJson(data, Serial);
  // Serial.println();

}

void Sensor::UpdateOrientation(float &Roll, float &Pitch, float &Yaw, float &RollRate, float &PitchRate, float &YawRate, uint32_t &Time)
{
  UpdateOrientation();

  Roll  = this->Roll;
  Pitch = this->Pitch;
  Yaw   = this->Yaw;
  Time  = this->TimeInterval;
  RollRate  = this->GyroRollRate;
  PitchRate = this->GyroPitchRate;
  YawRate   = this->GyroYawRate;
}

void Sensor::StartSensors()
{
  SensorQueue = xQueueCreate(1, sizeof(SensorData));

  xTaskCreatePinnedToCore(
    SensorTask,
    "SensorTask",
    2048,
    (void*)this,
    4,
    &SensorTaskHandle,
    0
  );
}

void SensorTask(void* param)
{
  SensorData newData;
  Sensor *thisSensor = (Sensor*)param;
  // Update data twice before Calculation to remove initial spike of value
  // This spike is due to the way Time is calculated in the UpdateData()
  thisSensor->UpdateData();
  vTaskDelay(IMUDelay / portTICK_RATE_MS);

  while(true)
  {
    thisSensor->UpdateData();
    thisSensor->UpdateOrientation(newData.Roll, newData.Pitch, newData.Yaw, newData.RollRate,  newData.PitchRate, newData.YawRate, newData.Time);
    xQueueSend(SensorQueue, &newData, 0);

    vTaskDelay(IMUDelay / portTICK_RATE_MS);
  }
}