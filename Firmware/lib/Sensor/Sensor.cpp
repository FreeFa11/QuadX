#include <Sensor.h>


// Objects
MPU6500 IMU;
// HMC5883L Compass;
QueueHandle_t SensorQueue;
TaskHandle_t SensorTaskHandle;
template class Sensor<NumericType>; 
template class Quaternion<NumericType>; 


// Definitions
template<typename T>
Sensor<T>::Sensor(){}
template<typename T>
Sensor<T>::~Sensor(){}


// Quaternion
template<typename T>
Quaternion<T>::Quaternion()
{
  w = 1; x = 0; y = 0; z = 0;
}
template<typename T>
Quaternion<T>::Quaternion(T w, T x, T y, T z)
{
    this->w = w; this->x = x; this->y = y; this->z = z;
}
template<typename T>
Quaternion<T>::~Quaternion(){}


template<typename T>
void Quaternion<T>::Normalize()
{
  T recp_magnitude = 1 / sqrt(w*w + x*x + y*y + z*z);
  w *= recp_magnitude;
  x *= recp_magnitude;
  y *= recp_magnitude;
  z *= recp_magnitude;
}

template<typename T>
Quaternion<T> Quaternion<T>::operator*(Quaternion<T> Other)
{
  Quaternion<T> result;
  
  // Multiplication
  result.w = w*Other.w - x*Other.x - y*Other.y - z*Other.z;
  result.x = x*Other.w + w*Other.x - z*Other.y + y*Other.z;
  result.y = y*Other.w + z*Other.x + w*Other.y - x*Other.z;
  result.z = z*Other.w - y*Other.x + x*Other.y + w*Other.z;

  return result;
}

template<typename T>
Quaternion<T> & Quaternion<T>::operator*=(Quaternion<T> Other)
{
  // Multiplication
  w = w*Other.w - x*Other.x - y*Other.y - z*Other.z;
  x = x*Other.w + w*Other.x - z*Other.y + y*Other.z;
  y = y*Other.w + z*Other.x + w*Other.y - x*Other.z;
  z = z*Other.w - y*Other.x + x*Other.y + w*Other.z;

  return *this;
}

template<typename T>
Quaternion<T> Quaternion<T>::operator*(T Value)
{
  Quaternion result;

  result.w = this->w * Value;
  result.x = this->x * Value;
  result.y = this->y * Value;
  result.z = this->z * Value;

  return result;
}

template<typename T>
Quaternion<T> & Quaternion<T>::operator*=(T Value)
{
  this->w *= Value;
  this->x *= Value;
  this->y *= Value;
  this->z *= Value;

  return *this;
}

template<typename T>
Quaternion<T> Quaternion<T>::operator+(Quaternion<T> Other)
{
  Other.w = this->w + Other.w;
  Other.x = this->x + Other.x;
  Other.y = this->y + Other.y;
  Other.z = this->z + Other.z;

  return Other;
}

template<typename T>
Quaternion<T> Quaternion<T>::operator-(Quaternion<T> Other)
{
  Other.w = this->w - Other.w;
  Other.x = this->x - Other.x;
  Other.y = this->y - Other.y;
  Other.z = this->z - Other.z;

  return Other;
}

template<typename T>
Quaternion<T> Quaternion<T>::inverse()
{
  Quaternion inv(this->w, -this->x, -this->y, -this->z);

  return inv;
}

template<typename T>
void Quaternion<T>::UpdateMahony(Sensor<T> &S)
{
  static T WX, WY, WZ;                                                  // Innovation Vector
  static T wxInt=0, wyInt=0, wzInt=0;
  static Quaternion<T> tempQuatA;             
  
  tempQuatA = (this->inverse())*Quaternion<T>(0,0,0,1)*(*this);         // Estimated  Vector

  WX = S.AY * tempQuatA.z - S.AZ * tempQuatA.y;
  WY = S.AZ * tempQuatA.x - S.AX * tempQuatA.z; 
  WZ = S.AX * tempQuatA.y - S.AY * tempQuatA.x;

  // wxInt += WX * 0.002;
  // wyInt += WY * 0.002;
  // wzInt += WZ * 0.002;
  // S.GX += Kp * WX + Ki * wxInt;                                         // PI Correction
  // S.GY += Kp * WY + Ki * wyInt;
  // S.GZ += Kp * WZ + Ki * wzInt;
  // It performed better without the Integral //
  S.GX += Kp * WX;                                                      // P Correction
  S.GY += Kp * WY;
  S.GZ += Kp * WZ;

  tempQuatA = ((*this) * Quaternion<T>(0,S.GX,S.GY,S.GZ)) * 0.5;        // Quat  Derivative
  (*this) = (*this) + tempQuatA * 0.002;                                // Euler Integration
  this->Normalize();
}

template<typename T>
void Quaternion<T>::UpdateMahony(T &GX, T &GY, T &GZ, T &AX, T &AY, T &AZ, T &MX, T &MY, T &MZ)
{
  static T WX, WY, WZ;                                                  // Innovation Vector
  static T wxInt=0, wyInt=0, wzInt=0;
  static Quaternion<T> tempQuatA, tempQuatM;             
  
  tempQuatM = (*this)*Quaternion<T>(0, MX, MY, MZ)*(this->inverse());   // Local to Global
  tempQuatA = tempQuatM;                                                // A is Measured
  tempQuatM.x = sqrt(pow(tempQuatM.x,2) + pow(tempQuatM.y,2));          // M is Reference
  tempQuatM.y = 0;
  WX = (tempQuatA.y * tempQuatM.z - tempQuatA.z * tempQuatM.y);         // Innovation in Global
  WY = (tempQuatA.z * tempQuatM.x - tempQuatA.x * tempQuatM.z);
  WZ = (tempQuatA.x * tempQuatM.y - tempQuatA.y * tempQuatM.x);
  WZ =  sqrt(pow(WX, 2)+pow(WY, 2)+pow(WZ, 2));                         // Innovation in Z

  tempQuatM = (this->inverse()) * Quaternion<T>(0, 0, 0, WZ) * (*this); // Magnetic Innovation
  tempQuatA = (this->inverse()) * Quaternion<T>(0, 0, 0, 1) * (*this);  // Estimated Gravity

  WX = (AY * tempQuatA.z - AZ * tempQuatA.y)*Kacc   +   tempQuatM.x*Kmag; 
  WY = (AZ * tempQuatA.x - AX * tempQuatA.z)*Kacc   +   tempQuatM.y*Kmag; 
  WZ = (AX * tempQuatA.y - AY * tempQuatA.x)*Kacc   +   tempQuatM.z*Kmag;
  // WX = AY * tempQuatA.z - AZ * tempQuatA.y;
  // WY = AZ * tempQuatA.x - AX * tempQuatA.z; 
  // WZ = AX * tempQuatA.y - AY * tempQuatA.x;

  GX += Kp * WX + Ki;                                                   // PI Correction
  GY += Kp * WY + Ki;
  GZ += Kp * WZ + Ki;  

  tempQuatA = ((*this) * Quaternion<T>(0, GX, GY, GZ)) * 0.5;           // Quat  Derivative
  (*this)   = (*this) + tempQuatA * 0.002;                              // Euler Integration
  this->Normalize();
}

template<typename T>
void Quaternion<T>::UpdateMadgwick(Sensor<T> &S)
{
  static T costX, costY, costZ;
  static Quaternion<T> tempQuatA;

  costX = -S.AX - 2*(w*y - x*z);                                        // q^-1*(0,0,0,1)*q - a
  costY = -S.AY + 2*(w*x + y*z);
  costZ = -S.AZ - 2*(x*x + y*y) + 1;

  tempQuatA.w = -2*y*costX + 2*x*costY;                                 // J^T * (q^-1*(0,0,0,1)*q - a)
  tempQuatA.x =  2*z*costX + 2*w*costY - 4*x*costZ;
  tempQuatA.y = -2*w*costX + 2*z*costY - 4*y*costZ;
  tempQuatA.z =  2*x*costX + 2*y*costY;
  tempQuatA.Normalize();

  tempQuatA = ((*this)*Quaternion<T>(0,S.GX,S.GY,S.GZ))*0.5 - tempQuatA*Beta;
  (*this)   = (*this) + tempQuatA * 0.002;
  this->Normalize();
}

template<typename T>
void Quaternion<T>::UpdateMadgwick(T &GX, T &GY, T &GZ, T &AX, T &AY, T &AZ, T &MX, T &MY, T &MZ)
{
  static T costAX, costAY, costAZ;
  static T costMX, costMY, costMZ;
  static Quaternion<T> tempQuatA, tempQuatM;
  
  costAX = -AX - 2*(w*y - x*z);                                         // q^-1*(0,0,0,1)*q - a
  costAY = -AY + 2*(w*x + y*z);
  costAZ = -AZ - 2*(x*x + y*y) + 1;

  tempQuatM = (*this)*Quaternion<T>(0, MX, MY, MZ)*(this->inverse());   // q*(0,x,y,z)*q^-1
  tempQuatM.x = sqrt(pow(tempQuatM.x,2) + pow(tempQuatM.y,2));
  tempQuatM.y = 0;                                                      // q^-1*(0,x,0,z)*q - m
  costMX = 2*tempQuatM.x*(0.5-y*y-z*z) + 2*tempQuatM.z*(x*z-w*y)     - MX; 
  costMY = 2*tempQuatM.x*(x*y-w*z)     + 2*tempQuatM.z*(w*x+y*z)     - MY;
  costMZ = 2*tempQuatM.x*(w*y+x*z)     + 2*tempQuatM.z*(0.5-x*x-y*y) - MZ;
  
  tempQuatA.w = -2*y*costAX + 2*x*costAY                - 2*tempQuatM.z*y*costMX                   + 2*(tempQuatM.z*x-tempQuatM.x*z)*costMY + 2*tempQuatM.x*y*costMZ;      
  tempQuatA.x =  2*z*costAX + 2*w*costAY - 4*x*costAZ   + 2*tempQuatM.z*z*costMX                   + 2*(tempQuatM.x*y+tempQuatM.z*w)*costMY + 2*(tempQuatM.x*z-2*tempQuatM.z*x)*costMZ;
  tempQuatA.y = -2*w*costAX + 2*z*costAY - 4*y*costAZ   - 2*(2*tempQuatM.x*y+tempQuatM.z*w)*costMX + 2*(tempQuatM.x*x+tempQuatM.z*z)*costMY + 2*(tempQuatM.x*w-2*tempQuatM.z*y)*costMZ;
  tempQuatA.z =  2*x*costAX + 2*y*costAY                + 2*(tempQuatM.z*x-2*tempQuatM.x*z)*costMX + 2*(tempQuatM.z*y-tempQuatM.x*w)*costMY + 2*tempQuatM.x*x*costMZ;
  tempQuatA.Normalize();
  
  tempQuatA = ((*this)*Quaternion<T>(0, GX, GY, GZ))*0.5 - tempQuatA*Beta;
  (*this)   = (*this) + tempQuatA * 0.002;
  this->Normalize();  
}




// Sensors
template<typename T>
void Sensor<T>::InitializeIMU()
{
  TwoWire *myI2C = new TwoWire(0);
  myI2C->begin(SDAWire, SCLWire);
  
  IMU.Config(myI2C, MPU6500::I2C_ADDR_PRIM);
  IMU.Begin();
  IMU.ConfigAccelRange(MPU6500::ACCEL_RANGE_2G);
  IMU.ConfigGyroRange(MPU6500::GYRO_RANGE_1000DPS);
  IMU.ConfigDLPFBandwidth(MPU6500::DLPF_BANDWIDTH_92HZ);
  
  // Calibration
  IMU.ConfigAccelBiasX(547);
  IMU.ConfigAccelBiasY(-3);
  IMU.ConfigAccelBiasZ(1752);
  IMU.ConfigGyroBiasX(68);
  IMU.ConfigGyroBiasY(93);
  IMU.ConfigGyroBiasZ(-35);
 
//           Accelerometer
//  [ 546.65f  -2.7620f   1752.4f]

//  [ 1.0005f  -0.0005f   0.0252f]
//  [-0.0065f   1.0076f   0.0123f]
//  [-0.0271f  -0.0146f   0.9899f]

//             Gyroscope 
//  [68.00f   93.00f  -35.00f]
}

template<typename T>
void Sensor<T>::InitializeMAG()
{
  // Wire.begin(SDAWire, SCLWire);
  // Compass.initialize(); 
  // Compass.setGain(HMC5883L_GAIN_220);
  // Compass.setDataRate(HMC5883L_RATE_75);
  // Compass.setSampleAveraging(HMC5883L_AVERAGING_8);
}

template<typename T>
void Sensor<T>::UpdateIMUData()
{
// Sensor Data
  IMU.Read();
  
  AX =  IMU.accel_x();
  AY =  IMU.accel_y(); 
  AZ =  IMU.accel_z();
  
  GX = IMU.gyro_x_radps();
  GY = IMU.gyro_y_radps();
  GZ = IMU.gyro_z_radps();
}

template<typename T>
void Sensor<T>::UpdateIMUData(T &AX, T &AY, T &AZ, T &GX, T &GY, T &GZ)
{
  UpdateIMUData();

  AX = this->AX;
  AY = this->AY;
  AY = this->AY;
  GX = this->GX;
  GY = this->GY;
  GZ = this->GZ;

}

template<typename T>
void Sensor<T>::UpdateMAGData()
{
  int16_t X, Y, Z;

  // Compass.getHeading(&X, &Y, &Z);
  MX = X; MY = Y; MZ = Z;
}

template<typename T>
void Sensor<T>::UpdateMAGData(T &MX, T &MY, T &MZ)
{
  UpdateMAGData();
  MX = this->MX;
  MY = this->MY;
  MZ = this->MZ;
}  

template<typename T>
void Sensor<T>::NormalizeVectors()
{
  float recp_magnitude = 1/sqrt(AX*AX + AY*AY + AZ*AZ);
  AX *= recp_magnitude;
  AY *= recp_magnitude;
  AZ *= recp_magnitude;
  
  // recp_magnitude = 1/sqrt(MX*MX + MY*MY + MZ*MZ);
  // MX *= recp_magnitude;
  // MY *= recp_magnitude;
  // MZ *= recp_magnitude;
}



// Sensor Task
template<typename T>
void Sensor<T>::StartSensors()
{
  SensorQueue = xQueueCreate(1, sizeof(SensorData));

  xTaskCreatePinnedToCore(
    SensorTask,
    "SensorTask",
    3000,
    (void*)this,
    4,
    &SensorTaskHandle,
    0
  );
}

void SensorTask(void* param)
{
  SensorData newData;
  Quaternion<NumericType> myQuaternion;
  Sensor<NumericType> *thisSensor = (Sensor<NumericType>*)param;
  
  thisSensor->UpdateIMUData();
  vTaskDelay(IMUDelay / portTICK_PERIOD_MS);


  while(true)
  {
    thisSensor->UpdateIMUData();
    thisSensor->NormalizeVectors();

    // myQuaternion.UpdateMahony(*thisSensor);
    // myQuaternion.UpdateMadgwick(*thisSensor);
    // myQuaternion.UpdateMahony(thisSensor->GX, thisSensor->GY, thisSensor->GZ, thisSensor->AX, thisSensor->AY, thisSensor->AZ, thisSensor->MX, thisSensor->MY, thisSensor->MZ);
    // myQuaternion.UpdateMadgwick(thisSensor->GX, thisSensor->GY, thisSensor->GZ, thisSensor->AX, thisSensor->AY, thisSensor->AZ, thisSensor->MX, thisSensor->MY, thisSensor->MZ);
    // Serial.printf("W:%.2f\tX:%.2f\tY:%.2f\tZ:%.2f\n", myQuaternion.w, myQuaternion.x, myQuaternion.y, myQuaternion.z);
    // Serial.printf("X:%.2f\tY:%.2f\tZ:%.2f\n", thisSensor->AX, thisSensor->AY, thisSensor->AZ);
    
    xQueueSend(SensorQueue, &newData, 0);
    vTaskDelay(IMUDelay / portTICK_PERIOD_MS);
  }
}