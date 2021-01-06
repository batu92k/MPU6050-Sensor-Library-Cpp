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
  * @brief  This method wakes the sensor up by cleraing the REG_PWR_MGMT_1
  * BIT_SLEEP. Power management 1 sensors default values is 0x40 so it will
  * be in sleep mode when it's powered up.
  * @param  none
  * @retval i2c_status_t
  */
  i2c_status_t WakeUpSensor(void);

  /**
  * @brief  This method used for configuring the gyroscope full scale range.
  * Check gyro_full_scale_range_t for available scales.
  * @param  gyroScale Gyroscope scale value to be set
  * @retval i2c_status_t
  */
  i2c_status_t SetGyroFullScale(gyro_full_scale_range_t gyroScale);

  /**
  * @brief  This method used for configuring the accelerometer full scale range.
  * Check accel_full_scale_range_t for available scales.
  * @param  accelScale Accelerometer scale value to be set
  * @retval i2c_status_t
  */
  i2c_status_t SetAccelFullScale(accel_full_scale_range_t accelScale);

  /**
  * @brief  This method used for getting the latest accelerometer X axis value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t X axis acceleration value
  */
  int16_t GetAccel_X(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest accelerometer Y axis value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t Y axis acceleration value
  */
  int16_t GetAccel_Y(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest accelerometer Z axis value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t Z axis acceleration value
  */
  int16_t GetAccel_Z(i2c_status_t* error);

  /**
  * @brief  This method used for getting the latest temperature value from the sensor.
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t Temperature in celcius-degrees
  */
  int16_t GetTemperature(i2c_status_t* error);

private:
  I2C_Interface* i2c = nullptr;

};

#endif /* include guard */
