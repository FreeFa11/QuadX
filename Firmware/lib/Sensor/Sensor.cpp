#include <Sensor.h>


// Objects
TaskHandle_t SensorTaskHandle;
template class Sensor<NumericType>; 
template class Quaternion<NumericType>;
TwoWire *myI2C = new TwoWire(0);


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
  #ifndef HAS_MAGNETOMETER
  
  
  static T WX, WY, WZ;                                                  // Innovation Vector
  static T wxInt=0, wyInt=0, wzInt=0;
  static Quaternion<T> tempQuatA;             
  
  tempQuatA = (this->inverse())*Quaternion<T>(0,0,0,1)*(*this);         // Estimated  Vector

  WX = S.AY * tempQuatA.z - S.AZ * tempQuatA.y;
  WY = S.AZ * tempQuatA.x - S.AX * tempQuatA.z; 
  WZ = S.AX * tempQuatA.y - S.AY * tempQuatA.x;

  wxInt += WX * 0.004;
  wyInt += WY * 0.004;
  wzInt += WZ * 0.004;
  S.GX += Kp * WX + Ki * wxInt;                                         // PI Correction
  S.GY += Kp * WY + Ki * wyInt;
  S.GZ += Kp * WZ + Ki * wzInt;
  
  tempQuatA = ((*this) * Quaternion<T>(0,S.GX,S.GY,S.GZ))*0.5;          // Quat  Derivative
  (*this) = (*this) + tempQuatA * TimerAngleModePeriod;                 // Euler Integration
  this->Normalize();
  

  #else


  static T WX, WY, WZ;                                                  // Innovation Vector
  static T wxInt=0, wyInt=0, wzInt=0;
  static Quaternion<T> tempQuatA, tempQuatM;             
  
  tempQuatM = (*this)*Quaternion<T>(0, S.MX, S.MY, S.MZ)*(this->inverse());   // Local to Global
  tempQuatA = tempQuatM;                                                // A is Measured
  tempQuatM.x = sqrt(pow(tempQuatM.x,2) + pow(tempQuatM.y,2));          // M is Estimated
  tempQuatM.y = 0;
  WX = (tempQuatA.y * tempQuatM.z - tempQuatA.z * tempQuatM.y);         // Innovation Global
  WY = (tempQuatA.z * tempQuatM.x - tempQuatA.x * tempQuatM.z);
  WZ = (tempQuatA.x * tempQuatM.y - tempQuatA.y * tempQuatM.x);
  // WZ = sqrt(WX*WX + WY*WY + WZ*WZ)
  tempQuatM = (this->inverse()) * Quaternion<T>(0,WX,WY,WZ) * (*this);  // Magnetic Innovation


  tempQuatA = (this->inverse()) * Quaternion<T>(0, 0, 0, 1) * (*this);  // Estimated Gravity

  WX = (S.AY * tempQuatA.z - S.AZ * tempQuatA.y)*Kacc   +   tempQuatM.x*Kmag; 
  WY = (S.AZ * tempQuatA.x - S.AX * tempQuatA.z)*Kacc   +   tempQuatM.y*Kmag; 
  WZ = (S.AX * tempQuatA.y - S.AY * tempQuatA.x)*Kacc   +   tempQuatM.z*Kmag;
  
  wxInt += WX * 0.004;
  wyInt += WY * 0.004;
  wzInt += WZ * 0.004;
  S.GX += Kp * WX + Ki * wxInt;                                           // PI Correction
  S.GY += Kp * WY + Ki * wyInt;
  S.GZ += Kp * WZ + Ki * wzInt;

  tempQuatA = ((*this) * Quaternion<T>(0, S.GX, S.GY, S.GZ))*0.5;       // Quat  Derivative
  (*this)   = (*this) + tempQuatA * TimerAngleModePeriod;               // Euler Integration
  this->Normalize();


  #endif
}

template<typename T>
void Quaternion<T>::UpdateMadgwick(Sensor<T> &S)
{
  #ifndef HAS_MAGNETOMETER


  static T costX, costY, costZ;
  static Quaternion<T> tempQuatA;

  costX = -S.AX - 2*(w*y - x*z);                                        // q^-1*(0,0,0,1)*q - a
  costY = -S.AY + 2*(w*x + y*z);
  costZ = -S.AZ - 2*(x*x + y*y) + 1;

  tempQuatA.w = -2*y*costX + 2*x*costY;                                 // J^T * (q^-1*(0,0,0,1)*q - a)
  tempQuatA.x =  2*z*costX + 2*w*costY - 4*x*costZ;
  tempQuatA.y = -2*w*costX + 2*z*costY - 4*y*costZ;
  tempQuatA.z =  2*x*costX + 2*y*costY;

  tempQuatA = ((*this)*Quaternion<T>(0,S.GX,S.GY,S.GZ))*0.5 - tempQuatA*Beta;
  (*this)   = (*this) + tempQuatA * TimerAngleModePeriod;
  this->Normalize();


  #else


  static T costAX, costAY, costAZ;
  static T costMX, costMY, costMZ;
  static Quaternion<T> tempQuatA, tempQuatM;
  
  costAX = -S.AX - 2*(w*y - x*z);                                       // q^-1*(0,0,0,1)*q - a
  costAY = -S.AY + 2*(w*x + y*z);
  costAZ = -S.AZ - 2*(x*x + y*y) + 1;
                                                                        // q*(0,x,y,z)*q^-1
  tempQuatM = (*this)*Quaternion<T>(0, S.MX, S.MY, S.MZ)*(this->inverse());
  tempQuatM.x = sqrt(pow(tempQuatM.x,2) + pow(tempQuatM.y,2));
  tempQuatM.y = 0;                                                      // q^-1*(0,x,0,z)*q - m
  costMX = 2*tempQuatM.x*(0.5-y*y-z*z) + 2*tempQuatM.z*(x*z-w*y)     - S.MX; 
  costMY = 2*tempQuatM.x*(x*y-w*z)     + 2*tempQuatM.z*(w*x+y*z)     - S.MY;
  costMZ = 2*tempQuatM.x*(w*y+x*z)     + 2*tempQuatM.z*(0.5-x*x-y*y) - S.MZ;
  
  tempQuatA.w = -2*y*costAX + 2*x*costAY                - 2*tempQuatM.z*y*costMX                   + 2*(tempQuatM.z*x-tempQuatM.x*z)*costMY + 2*tempQuatM.x*y*costMZ;      
  tempQuatA.x =  2*z*costAX + 2*w*costAY - 4*x*costAZ   + 2*tempQuatM.z*z*costMX                   + 2*(tempQuatM.x*y+tempQuatM.z*w)*costMY + 2*(tempQuatM.x*z-2*tempQuatM.z*x)*costMZ;
  tempQuatA.y = -2*w*costAX + 2*z*costAY - 4*y*costAZ   - 2*(2*tempQuatM.x*y+tempQuatM.z*w)*costMX + 2*(tempQuatM.x*x+tempQuatM.z*z)*costMY + 2*(tempQuatM.x*w-2*tempQuatM.z*y)*costMZ;
  tempQuatA.z =  2*x*costAX + 2*y*costAY                + 2*(tempQuatM.z*x-2*tempQuatM.x*z)*costMX + 2*(tempQuatM.z*y-tempQuatM.x*w)*costMY + 2*tempQuatM.x*x*costMZ;
  
  tempQuatA = ((*this)*Quaternion<T>(0, S.GX, S.GY, S.GZ))*0.5 - tempQuatA*Beta;
  (*this)   = (*this) + tempQuatA * TimerAngleModePeriod;
  this->Normalize();  


  #endif
}




// Sensors
template<typename T>
void Sensor<T>::InitializeIMU()
{
  myI2C->begin(SDAWire, SCLWire);
  myI2C->setClock(400000L);
  
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

#ifdef HAS_MAGNETOMETER
template<typename T>
void Sensor<T>::InitializeMAG()
{
  myI2C->begin(SDAWire, SCLWire);
  myI2C->setClock(400000L);

  MAG.Config(myI2C, HMC5883L_ADDRESS);
  MAG.initialize();
  MAG.setMode(HMC5883L_MODE_CONTINUOUS);
  MAG.setGain(HMC5883L_GAIN_220);
  MAG.setDataRate(HMC5883L_RATE_75);
  MAG.setSampleAveraging(HMC5883L_AVERAGING_8);
}

template<typename T>
void Sensor<T>::UpdateMAGData()
{
  int16_t X, Y, Z;

  MAG.getHeading(&X, &Y, &Z);
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
#endif

template<typename T>
void Sensor<T>::NormalizeVectors()
{
  float recp_magnitude = 1/sqrt(AX*AX + AY*AY + AZ*AZ);
  AX *= recp_magnitude;
  AY *= recp_magnitude;
  AZ *= recp_magnitude;
  
  #ifdef HAS_MAGNETOMETER
  recp_magnitude = 1/sqrt(MX*MX + MY*MY + MZ*MZ);
  MX *= recp_magnitude;
  MY *= recp_magnitude;
  MZ *= recp_magnitude;
  #endif
}



// Sensor Task
template<typename T>
void Sensor<T>::StartSensors()
{
  xTaskCreatePinnedToCore(
    SensorTask,
    "SensorTask",
    3000,
    (void*)this,
    6,
    &SensorTaskHandle,
    0
  );
}

void SensorTask(void* param)
{
  SensorData newData;
  Quaternion<NumericType> myQuaternion;
  Sensor<NumericType> *thisSensor = (Sensor<NumericType>*)param;
  

  while(true)
  {
    // if (!thisSensor->MAG.getReadyStatus())
    // {
    //   thisSensor->UpdateIMUData();
    //   thisSensor->NormalizeVectors();
    //   // myQuaternion.UpdateMahony(*thisSensor);
    //   myQuaternion.UpdateMadgwick(*thisSensor);
    // }
    // else
    // {
    //   thisSensor->UpdateIMUData();
    //   thisSensor->UpdateMAGData();
    //   thisSensor->NormalizeVectors();
    //   // myQuaternion.UpdateMahony(thisSensor->GX, thisSensor->GY, thisSensor->GZ, thisSensor->AX, thisSensor->AY, thisSensor->AZ, thisSensor->MX, thisSensor->MY, thisSensor->MZ);
    //   myQuaternion.UpdateMadgwick(thisSensor->GX, thisSensor->GY, thisSensor->GZ, thisSensor->AX, thisSensor->AY, thisSensor->AZ, thisSensor->MX, thisSensor->MY, thisSensor->MZ);
    // }

    thisSensor->UpdateIMUData();
    thisSensor->NormalizeVectors();
    myQuaternion.UpdateMahony(*thisSensor);
    // myQuaternion.UpdateMadgwick(*thisSensor);


    // Serial.printf("W:%.2f\tX:%.2f\tY:%.2f\tZ:%.2f\n", myQuaternion.w, myQuaternion.x, myQuaternion.y, myQuaternion.z);
    // Serial.printf("%.4f,%.4f,%.4f,%.4f\n", myQuaternion.w, myQuaternion.x, myQuaternion.y, myQuaternion.z);
    // Serial.printf("X:%.2f\tY:%.2f\tZ:%.2f\n", thisSensor->AX, thisSensor->AY, thisSensor->AZ);
    // Serial.printf("X:%.2f\tY:%.2f\tZ:%.2f\n", thisSensor->MX, thisSensor->MY, thisSensor->MZ);
    // Serial.printf(">X:%.2f\n", thisSensor->AX);
  }
}