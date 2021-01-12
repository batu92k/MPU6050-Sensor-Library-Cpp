/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  Ali Batuhan KINDAN
  * @date    20.12.2020
  * @brief   This file constains MPU6050 driver declarations.
  ******************************************************************************
  */

#ifndef MPU6050_H
#define MPU6050_H

#include "i2c_interface.h"

#define AD0 1

/* MPU6050 I2C Device Address */
#define MPU6050_ADDRESS_AD0 0x68 // AD0 pin low
#define MPU6050_ADDRESS_AD1 0x69 // AD0 pin high

/* Define default MPU6050 address as 0x68 (AD0 pin low) */
#if AD0
#define MPU6050_ADDRESS MPU6050_ADDRESS_AD0
#else
#define MPU6050_ADDRESS MPU6050_ADDRESS_AD1
#endif

#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C

/* Accelerometer read registers */
#define REG_ACCEL_X_OUT_H 0x3B
#define REG_ACCEL_X_OUT_L 0x3C
#define REG_ACCEL_Y_OUT_H 0x3D
#define REG_ACCEL_Y_OUT_L 0x3E
#define REG_ACCEL_Z_OUT_H 0x3F
#define REG_ACCEL_Z_OUT_L 0x40

/* Temperature read registers */
#define REG_TEMP_OUT_H 0x41
#define REG_TEMP_OUT_L 0x42

/* Gyroscope read registers */
#define REG_GYRO_X_OUT_H 0x43
#define REG_GYRO_X_OUT_L 0x44
#define REG_GYRO_Y_OUT_H 0x45
#define REG_GYRO_Y_OUT_L 0x46
#define REG_GYRO_Z_OUT_H 0x47
#define REG_GYRO_Z_OUT_L 0x48

/* Gyroscope offset registers */
#define REG_XG_OFFS_USR_H 0x13
#define REG_XG_OFFS_USR_L 0x14
#define REG_YG_OFFS_USR_H 0x15
#define REG_YG_OFFS_USR_L 0x16
#define REG_ZG_OFFS_USR_H 0x17
#define REG_ZG_OFFS_USR_L 0x18

/* Gyro offset register constant to compensate 1 DPS (degree per second) offset.
 * Check sensor datasheet for more info about the offset procedure! */
#define GYRO_OFFSET_1DPS 32.8f

enum regbits_pwr_mgmt_1_t 
{
  BIT_CLKSEL_0 = 0,
  BIT_CLKSEL_1 = 1,
  BIT_CLKSEL_2 = 2,
  BIT_TEMP_DIS = 3,
  BIT_CYCLE = 5,
  BIT_SLEEP = 6,
  BIT_DEVICE_RESET = 7
};

enum regbits_gyro_config_t
{
  XG_ST = 7,
  YG_ST = 6,
  ZG_ST = 5
  /* other bits related with gyro full scale config! */
};

enum regbits_accel_config_t
{
  XA_ST = 7,
  YA_ST = 6,
  ZA_ST = 5
  /* other bits related with accelerometer full scale config! */
};

/* Gyroscope full scale ranges in degrees per second */
enum gyro_full_scale_range_t
{
  GYRO_SCALE_250 = 0,
  GYRO_SCALE_500 = 1,
  GYRO_SCALE_1000 = 2,
  GYRO_SCALE_2000 = 3
};

/* Accelerometer full scale ranges in G's */
enum accel_full_scale_range_t
{
  ACCEL_SCALE_2G = 0,
  ACCEL_SCALE_4G = 1,
  ACCEL_SCALE_8G = 2,
  ACCEL_SCALE_16G = 3
};

class MPU6050 
{
public:

  /**
  * @brief  Class constructor. In order to make the class communicate with sensor
  * user should pass a valid I2C_Interface class instance!
  * @param  comInterface I2C interface pointer
  * @retval none
  */
  MPU6050(I2C_Interface* comInterface);

  /**
  * @brief  This method wakes up the sensor and configures the accelerometer and
  * gyroscope full scale renges with given parameters. It returns the result of
  * the process.
  * @param  gyroScale Gyroscope scale value to be set
  * @param  accelScale Accelerometer scale value to be set
  * @retval i2c_status_t Success rate
  */
  i2c_status_t InitializeSensor(
      gyro_full_scale_range_t gyroScale = GYRO_SCALE_250,
      accel_full_scale_range_t accelScale = ACCEL_SCALE_2G);

  /**
  * @brief  This method wakes the sensor up by cleraing the REG_PWR_MGMT_1
  * BIT_SLEEP. Power management 1 sensors default values is 0x40 so it will
  * be in sleep mode when it's powered up.
  * @param  none
  * @retval i2c_status_t
  */
  i2c_status_t WakeUpSensor(void);

  /**
  * @brief  This method resets the sensor by simply setting the REG_PWR_MGMT_1
  * Device_Reset bit. After the sensor reset this bit will be cleared automatically.
  * @param  none
  * @retval i2c_status_t
  */
  i2c_status_t ResetSensor(void);

  /**
  * @brief  This method used for configuring the gyroscope full scale range.
  * Check gyro_full_scale_range_t for available scales.
  * @param  gyroScale Gyroscope scale value to be set
  * @retval i2c_status_t
  */
  i2c_status_t SetGyroFullScale(gyro_full_scale_range_t gyroScale);

  /**
  * @brief  This method used for getting the gyroscope full scale range.
  * Check gyro_full_scale_range_t for available scales. It basically reads the
  * Gyro configuration register and returns the full scale range.
  * @param  error Result of the sensor reading process
  * @retval gyro_full_scale_range_t
  */
  gyro_full_scale_range_t GetGyroFullScale(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest gyroscope X axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and gyroscope full
  * scale range is set to desired range before reading the values.
  * @param  error Error state of process
  * @retval int16_t X axis RAW gyroscope value
  */
  int16_t GetGyro_X_Raw(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest gyroscope Y axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and gyroscope full
  * scale range is set to desired range before reading the values.
  * @param  error Error state of process
  * @retval int16_t Y axis RAW gyroscope value
  */
  int16_t GetGyro_Y_Raw(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest gyroscope Z axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and gyroscope full
  * scale range is set to desired range before reading the values.
  * @param  error Error state of process
  * @retval int16_t Z axis RAW gyroscope value
  */
  int16_t GetGyro_Z_Raw(i2c_status_t* error);

  /**
  * @brief  This method used for configuring the accelerometer full scale range.
  * Check accel_full_scale_range_t for available scales.
  * @param  accelScale Accelerometer scale value to be set
  * @retval i2c_status_t
  */
  i2c_status_t SetAccelFullScale(accel_full_scale_range_t accelScale);

  /**
  * @brief  This method used for getting the acceleromteter full scale range.
  * Check accel_full_scale_range_t for available scales. It basically reads the
  * Accel configuration register and returns the full scale range.
  * @param  error Result of the sensor reading process
  * @retval accel_full_scale_range_t
  */
  accel_full_scale_range_t GetAccelFullScale(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest accelerometer X axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t X axis RAW acceleration value
  */
  int16_t GetAccel_X_Raw(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest accelerometer Y axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t Y axis RAW acceleration value
  */
  int16_t GetAccel_Y_Raw(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest accelerometer Z axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t Z axis RAW acceleration value
  */
  int16_t GetAccel_Z_Raw(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest temperature value from the sensor.
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval float Temperature in celcius-degrees
  */
  float GetTemperature_Celcius(i2c_status_t* error);

  /**
  * @brief  This method used for setting the gyroscope X axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
  i2c_status_t SetGyro_X_Offset(int16_t offset);

  /**
  * @brief  This method used for setting the gyroscope Y axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
  i2c_status_t SetGyro_Y_Offset(int16_t offset);

  /**
  * @brief  This method used for setting the gyroscope Z axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
  i2c_status_t SetGyro_Z_Offset(int16_t offset);

  /**
  * @brief  This method used for calibrating the gyroscope registers to given target values.
  * @param targetX target value for gyroscope X axis register
  * @param targetY target value for gyroscope Y axis register
  * @param targetZ target value for gyroscope Z axis register
  * @retval i2c_status_t
  */
  i2c_status_t Calibrate_Gyro_Registers(int16_t targetX = 0, int16_t targetY = 0, int16_t targetZ = 0);

private:
  I2C_Interface* i2c = nullptr;
  /* DPS constant to convert raw register value to the degree per seconds (angular velocity).
   * The index of the values are adjusted to have corresponding values with the gyro_full_scale_range_t
   * enum. So, we can just get the DPS value by "dpsConstantArr[GYRO_SCALE_250]"" for GYRO_SCALE_250. */
  const float dpsConstantArr[4] = {250.0f / 32767.0f, 500.0f / 32767.0f, 1000.0f / 32767.0f, 2000.0f / 32767.0f};

};

#endif /* include guard */
