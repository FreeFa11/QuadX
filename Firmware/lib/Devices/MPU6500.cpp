/*
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "MPU6500.h"
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include "core/core.h"
#endif


void MPU6500::Config(TwoWire *i2c, const I2cAddr addr) {
  i2c_ = i2c;
  dev_ = addr;
  iface_ = I2C;
}
void MPU6500::Config(SPIClass *spi, const uint8_t cs) {
  spi_ = spi;
  dev_ = cs;
  iface_ = SPI;
}
bool MPU6500::Begin() {
  if (iface_ == SPI) {
    pinMode(dev_, OUTPUT);
    digitalWrite(dev_, LOW);
    delay(1);
    digitalWrite(dev_, HIGH);
    delay(1);
  }

  spi_clock_ = SPI_CFG_CLOCK_;

  /* Select clock source to gyro */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
    return false;
  }
  /* Check the WHO AM I byte */
  if (!ReadRegisters(WHOAMI_, sizeof(bytes_rx_), &bytes_rx_)) {
    return false;
  }
  if (bytes_rx_ != WHOAMI_MPU6500_) {
    return false;
  }
  // Default Configuration //
  if (!ConfigAccelRange(ACCEL_RANGE_16G)) {
    return false;
  }
  if (!ConfigGyroRange(GYRO_RANGE_2000DPS)) {
    return false;
  }
  if (!ConfigDLPFBandwidth(DLPF_BANDWIDTH_184HZ)) {
    return false;
  }
  if (!ConfigSRD(0)) {
    return false;
  }
  return true;
}
bool MPU6500::EnableDrdyInt() {
  spi_clock_ = SPI_CFG_CLOCK_;
  if (!WriteRegister(INT_PIN_CFG_, INT_PULSE_50US_)) {
    return false;
  }
  if (!WriteRegister(INT_ENABLE_, INT_RAW_RDY_EN_)) {
    return false;
  }
  return true;
}
bool MPU6500::DisableDrdyInt() {
  spi_clock_ = SPI_CFG_CLOCK_;
  if (!WriteRegister(INT_ENABLE_, INT_DISABLE_)) {
    return false;
  }
  return true;
}
bool MPU6500::ConfigAccelRange(const AccelRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;
  AccelRange requested_accel_range_;
  float requested_accel_scale_;
  switch (range) {
    case ACCEL_RANGE_2G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 2.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_4G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 4.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_8G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 8.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_16G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 16.0f / 32767.5f;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try the requested range */
  if (!WriteRegister(ACCEL_CONFIG_, requested_accel_range_)) {
    return false;
  }
  /* Update stored range and scale */
  accel_range_ = requested_accel_range_;
  accel_scale_ = requested_accel_scale_;
  return true;
}
bool MPU6500::ConfigGyroRange(const GyroRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;
  GyroRange requested_gyro_range_;
  float requested_gyro_scale_;
  switch (range) {
    case GYRO_RANGE_250DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 250.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_500DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 500.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_1000DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 1000.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_2000DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 2000.0f / 32767.5f;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the requested range */
  if (!WriteRegister(GYRO_CONFIG_, requested_gyro_range_)) {
    return false;
  }
  /* Update stored range and scale */
  gyro_range_ = requested_gyro_range_;
  gyro_scale_ = requested_gyro_scale_;
  return true;
}
bool MPU6500::ConfigSRD(const uint8_t srd) {
  spi_clock_ = SPI_CFG_CLOCK_;
  if (!WriteRegister(SMPLRT_DIV_, srd)) {
    return false;
  }
  srd_ = srd;
  return true;
}
bool MPU6500::ConfigDLPFBandwidth(const DlpfBandwidth dlpf) {
  spi_clock_ = SPI_CFG_CLOCK_;
  DlpfBandwidth requested_dlpf_;
  switch (dlpf) {
    case DLPF_BANDWIDTH_184HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_92HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_41HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_20HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the DLPF */
  if (!WriteRegister(ACCEL_CONFIG2_, requested_dlpf_)) {
    return false;
  }
  if (!WriteRegister(CONFIG_, requested_dlpf_)) {
    return false;
  }
  /* Update stored DLPF */
  dlpf_bandwidth_ = requested_dlpf_;
  return true;
}
void MPU6500::ConfigAccelBiasX(int16_t Bias){
  axb_ = Bias;
}
void MPU6500::ConfigAccelBiasY(int16_t Bias){
  ayb_ = Bias;
}
void MPU6500::ConfigAccelBiasZ(int16_t Bias){
  azb_ = Bias;
}
void MPU6500::ConfigGyroBiasX(int16_t Bias){
  gxb_ = Bias;
}
void MPU6500::ConfigGyroBiasY(int16_t Bias){
  gyb_ = Bias;
}
void MPU6500::ConfigGyroBiasZ(int16_t Bias){
  gzb_ = Bias;
}
bool MPU6500::Read() {
  spi_clock_ = SPI_READ_CLOCK_;
  new_imu_data_ = false;
  if (!ReadRegisters(INT_STATUS_, sizeof(data_buf_), data_buf_)) {
    return false;
  }
  // The whole sequence is read at once //
  // Therefore, read -> check if new    //
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) {
    return false;
  }
  accel_cnts_[0] = static_cast<int16_t>(data_buf_[1])  << 8 | data_buf_[2];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[3])  << 8 | data_buf_[4];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[5])  << 8 | data_buf_[6];
  temp_cnts_ =     static_cast<int16_t>(data_buf_[7])  << 8 | data_buf_[8];
  gyro_cnts_[0] =  static_cast<int16_t>(data_buf_[9])  << 8 | data_buf_[10];
  gyro_cnts_[1] =  static_cast<int16_t>(data_buf_[11]) << 8 | data_buf_[12];
  gyro_cnts_[2] =  static_cast<int16_t>(data_buf_[13]) << 8 | data_buf_[14];
  // Bias, Scaling and Polarity Corrections //
  accel_[0] = static_cast<float>(accel_cnts_[0]-axb_) * accel_scale_;
  accel_[1] = static_cast<float>(accel_cnts_[1]-ayb_) * accel_scale_;
  accel_[2] = static_cast<float>(accel_cnts_[2]-azb_) * accel_scale_;
  temp_ =    (static_cast<float>(temp_cnts_) - 21.0f) / TEMP_SCALE_ + 21.0f;
  gyro_[0] =  static_cast<float>(gyro_cnts_[0]-gxb_) * gyro_scale_ * DEG2RAD_;
  gyro_[1] =  static_cast<float>(gyro_cnts_[1]-gyb_) * gyro_scale_ * DEG2RAD_;
  gyro_[2] =  static_cast<float>(gyro_cnts_[2]-gzb_) * gyro_scale_ * DEG2RAD_;
  return true;
}
bool MPU6500::WriteRegister(const uint8_t reg, const uint8_t data) {
  uint8_t ret_val;
  if (iface_ == I2C) {
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->write(data);
    i2c_->endTransmission();
  } else {
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWrite(dev_, LOW);
    spi_->transfer(reg);
    spi_->transfer(data);
    digitalWrite(dev_, HIGH);
    spi_->endTransaction();
  }
  delay(10);
  // Checking if the value has been correctly written //
  ReadRegisters(reg, sizeof(ret_val), &ret_val);
  if (data == ret_val) {
    return true;
  } else {
    return false;
  }
}
bool MPU6500::ReadRegisters(const uint8_t reg, const uint8_t count, uint8_t * const data) {
  if (!data) {return false;}
  if (iface_ == I2C) {
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->endTransmission(false);
    bytes_rx_ = i2c_->requestFrom(static_cast<uint8_t>(dev_), count);
    if (bytes_rx_ == count) {
      for (size_t i = 0; i < count; i++) {
        data[i] = i2c_->read();
      }
      return true;
    } else {
      return false;
    }
  } else {
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWrite(dev_, LOW);
    spi_->transfer(reg | SPI_READ_);
    spi_->transfer(data, count);
    digitalWrite(dev_, HIGH);
    spi_->endTransaction();
    return true;
  }
}
