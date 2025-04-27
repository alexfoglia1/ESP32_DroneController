#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>


class RPI_MPU6050
{
public:
    const uint8_t I2C_ADDR     = 0x68;

    // Registers
    enum class Register
    {
        PWR_MGMT_1   = 0x6B,
        SMPLRT_DIV   = 0x19,
        CONFIG       = 0x1A,
        GYRO_CONFIG  = 0x1B,
        ACCEL_CONFIG = 0x1C,
        INT_ENABLE   = 0x38,
        ACCEL_XOUT_H = 0x3B,
        ACCEL_YOUT_H = 0x3D,
        ACCEL_ZOUT_H = 0x3F,
        TEMP_OUT_H   = 0x41,
        GYRO_XOUT_H  = 0x43,
        GYRO_YOUT_H  = 0x45,
        GYRO_ZOUT_H  = 0x47
    };

    enum class GyroResolution : uint8_t
    {
        GYRO_250_DPS  = 0x00,
        GYRO_500_DPS  = 0x08,
        GYRO_1000_DPS = 0x10,
        GYRO_2000_DPS = 0x18
    };

    enum class AccelResolution : uint8_t
    {
        ACCEL_2G  = 0x00,
        ACCEL_4G  = 0x08,
        ACCEL_8G  = 0x10,
        ACCEL_16G = 0x18
    };

    RPI_MPU6050();

    bool init();
    void gyroByas(int loops = 100);
    bool setGyroResolution(RPI_MPU6050::GyroResolution gyroResolution);
    bool setAccelResolution(RPI_MPU6050::AccelResolution accelResolution);
    void readGyro(float* gx, float* gy, float* gz);
    void readAccel(float* ax, float* ay, float* az);

private:

    int _found;
    GyroResolution _gyroResolution;
    AccelResolution _accelResolution;
    Adafruit_I2CDevice* _i2c;
    float _gx0;
    float _gy0;
    float _gz0;

    int i2cWrite(RPI_MPU6050::Register reg, uint8_t* data, uint32_t data_len);
    int i2cRead(RPI_MPU6050::Register reg, uint8_t* data, uint32_t data_len);

    float toDps(int16_t gyroRaw);
    float toG(int16_t accelRaw);
    float gyroResolutionValue(RPI_MPU6050::GyroResolution res);
    float accelResolutionValue(RPI_MPU6050::AccelResolution res);
    int16_t big_endian(int16_t little_endian);

};

#endif