#include "RPI_MPU6050.h"

#include <Arduino.h>



RPI_MPU6050::RPI_MPU6050()
{
    _found = 0;

    _gx0 = 0;
    _gy0 = 0;
    _gz0 = 0;

    _gyroResolution = GyroResolution::GYRO_250_DPS;
    _accelResolution = AccelResolution::ACCEL_2G;
    _i2c = new Adafruit_I2CDevice(RPI_MPU6050::I2C_ADDR);
}

bool RPI_MPU6050::init()
{
    Wire.beginTransmission(RPI_MPU6050::I2C_ADDR);
    _found = (Wire.endTransmission(true) == 0);

    if (_found)
    {
      uint8_t zero = 0x00;
      uint8_t one  = 0x01;

      int writeRes = i2cWrite(RPI_MPU6050::Register::SMPLRT_DIV, &zero, 1);
      if (writeRes != 0)
      {
          Serial.println("[NOK] <IMU Init> Set sample rate");
          return false;
      }

      /** Reset all sensors **/
      writeRes = i2cWrite(RPI_MPU6050::Register::PWR_MGMT_1, &zero, 1);
      if (writeRes != 0)
      {
          Serial.println("[NOK] <IMU Init> Reset all sensors");
          return false;
      }

      /** Power management and crystal settings **/
      writeRes = i2cWrite(RPI_MPU6050::Register::PWR_MGMT_1, &one, 1);
      if (writeRes != 0)
      {
          Serial.println("[NOK] <IMU Init> Power management");
          return false;
      }
            
      /** Write to Configuration register **/
      writeRes = i2cWrite(RPI_MPU6050::Register::CONFIG, &zero, 1);
      if (writeRes != 0)
      {
          Serial.println("[NOK] <IMU Init> Configuration register");
          return false;
      }

      if (!setGyroResolution(RPI_MPU6050::GyroResolution::GYRO_250_DPS))
      {
          Serial.println("[NOK] <IMU Init> Set gyro resolution 250 DPS");
          return false;
      }

      if (!setAccelResolution(RPI_MPU6050::AccelResolution::ACCEL_2G))
      {
          Serial.println("[NOK] <IMU Init> Set accelerometer resolution 2G");
          return false;
      }

      writeRes = i2cWrite(RPI_MPU6050::Register::INT_ENABLE, &one, 1);
      if (writeRes != 0)
      {
          Serial.println("[NOK] <IMU Init> Interrupt enable");
      }
      else
      {
          return true;
      }
   }
   else
   {
      Serial.println("[NOK] <IMU Init> Not found on i2c bus");
      return false;
   }

   return true;
}


bool RPI_MPU6050::setGyroResolution(RPI_MPU6050::GyroResolution gyroResolution)
{
    _gyroResolution = gyroResolution;

    uint8_t byte = static_cast<uint8_t>(gyroResolution);

    return i2cWrite(RPI_MPU6050::Register::GYRO_CONFIG, &byte, 1) == 0;
}


bool RPI_MPU6050::setAccelResolution(RPI_MPU6050::AccelResolution accelResolution)
{
    _accelResolution = accelResolution;

    uint8_t byte = static_cast<uint8_t>(accelResolution);

    return i2cWrite(RPI_MPU6050::Register::ACCEL_CONFIG, &byte, 1) == 0;
}


void RPI_MPU6050::readGyro(float* gx, float* gy, float* gz)
{
    int16_t rawGyroX = 0;
    i2cRead(RPI_MPU6050::Register::GYRO_XOUT_H, (uint8_t*) &rawGyroX, 2);
    int16_t rawGyroY = 0;
    i2cRead(RPI_MPU6050::Register::GYRO_YOUT_H, (uint8_t*) &rawGyroY, 2);
    int16_t rawGyroZ = 0;
    i2cRead(RPI_MPU6050::Register::GYRO_ZOUT_H, (uint8_t*) &rawGyroZ, 2);

    *gx = toDps(rawGyroX) - _gx0;
    *gy = toDps(rawGyroY) - _gy0;
    *gz = -toDps(rawGyroZ) - _gz0;
}


void RPI_MPU6050::readAccel(float* ax, float* ay, float* az)
{
    int16_t rawAccelX = 0;
    i2cRead(RPI_MPU6050::Register::ACCEL_XOUT_H, (uint8_t*) &rawAccelX, 2);
    int16_t rawAccelY = 0;
    i2cRead(RPI_MPU6050::Register::ACCEL_YOUT_H, (uint8_t*) &rawAccelY, 2);
    int16_t rawAccelZ = 0;
    i2cRead(RPI_MPU6050::Register::ACCEL_ZOUT_H, (uint8_t*) &rawAccelZ, 2);

    *ax = toG(rawAccelX);
    *ay = toG(rawAccelY);
    *az = -toG(rawAccelZ);
}


void RPI_MPU6050::gyroByas(uint8_t loops)
{
    const int N = loops;

    _gx0 = 0.0f;
    _gy0 = 0.0f;
    _gz0 = 0.0f;

    float gx0 = 0.0f;
    float gy0 = 0.0f;
    float gz0 = 0.0f;
    for (int i = 0; i < N; i++)
    {
        float gx, gy, gz;
        gx = 0.0f;
        gy = 0.0f;
        gz = 0.0f;

        readGyro(&gx, &gy, &gz);
        gx0 += gx;
        gy0 += gy;
        gz0 += gz;

        Serial.print("*");
    }

    _gx0 = gx0 / (float)N;
    _gy0 = gy0 / (float)N;
    _gz0 = gz0 / (float)N;

    Serial.printf("[INFO] <gyro0> gx0(%f)\tgy0(%f)\tgz0(%f)", _gx0, _gy0, _gz0);
}


int RPI_MPU6050::i2cWrite(RPI_MPU6050::Register reg, uint8_t* data, uint32_t data_len)
{
  return _i2c->write(data, data_len, true, (uint8_t*)&reg, 1) ? 0 : 1;
}



int RPI_MPU6050::i2cRead(RPI_MPU6050::Register reg, uint8_t* buffer, uint32_t data_len)
{
  return _i2c->write_then_read((uint8_t*)&reg, 1, buffer, data_len) ? 0 : 1;
}


float RPI_MPU6050::toDps(int16_t gyroRaw)
{
    gyroRaw = big_endian(gyroRaw);
    float gyroResolution = gyroResolutionValue(_gyroResolution);
    return (static_cast<float>(gyroRaw) / static_cast<float>(std::numeric_limits<int16_t>::max())) * gyroResolution;
}


float RPI_MPU6050::toG(int16_t accelRaw)
{
    accelRaw = big_endian(accelRaw);
    float accelResolution = accelResolutionValue(_accelResolution);
    return (static_cast<float>(accelRaw) / static_cast<float>(std::numeric_limits<int16_t>::max())) * accelResolution;
}


float RPI_MPU6050::gyroResolutionValue(RPI_MPU6050::GyroResolution res)
{
    float _ret = 0.0f;
    switch (res)
    {
        case RPI_MPU6050::GyroResolution::GYRO_250_DPS:  _ret = 250.f;  break;
        case RPI_MPU6050::GyroResolution::GYRO_500_DPS:  _ret = 500.f;  break;
        case RPI_MPU6050::GyroResolution::GYRO_1000_DPS: _ret = 1000.f; break;
        case RPI_MPU6050::GyroResolution::GYRO_2000_DPS: _ret = 2000.f; break;
    }

    return _ret;
}


float RPI_MPU6050::accelResolutionValue(RPI_MPU6050::AccelResolution res)
{
    float _ret = 0.0f;
    switch (res)
    {
        case RPI_MPU6050::AccelResolution::ACCEL_2G:  _ret = 2.f;  break;
        case RPI_MPU6050::AccelResolution::ACCEL_4G:  _ret = 4.f;  break;
        case RPI_MPU6050::AccelResolution::ACCEL_8G:  _ret = 8.f;  break;
        case RPI_MPU6050::AccelResolution::ACCEL_16G: _ret = 16.f; break;
    }

    return _ret;
}


int16_t RPI_MPU6050::big_endian(int16_t little_endian)
{
    uint8_t* bytes = (uint8_t*) &little_endian;
    return ((bytes[0] << 8) | bytes[1]);
}