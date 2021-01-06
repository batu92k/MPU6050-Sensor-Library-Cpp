/**
  ******************************************************************************
  * @file    mpu6050.cpp
  * @author  Ali Batuhan KINDAN
  * @date    20.12.2020
  * @brief   This file constains MPU6050 driver implementation.
  ******************************************************************************
  */

#include "mpu6050.h"

/**
  * @brief  Class constructor. In order to make the class communicate with sensor
  * user should pass a valid I2C_Interface class instance!
  * @param  comInterface I2C interface pointer
  * @retval none
  */
MPU6050::MPU6050(I2C_Interface *comInterface)
{
  /* assign internal interface pointer if given is not null! */
  if (comInterface)
  {
    this->i2c = comInterface;
  }
}

/**
  * @brief  This method wakes the sensor up by cleraing the REG_PWR_MGMT_1
  * BIT_SLEEP. Power management 1 sensors default values is 0x40 so it will
  * be in sleep mode when it's powered up.
  * @param  none
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::WakeUpSensor(void)
{
  return i2c->WriteRegisterBit(MPU6050_ADDRESS, REG_PWR_MGMT_1, BIT_SLEEP, false);
}

/**
  * @brief  This method used for configuring the gyroscope full scale range.
  * Check gyro_full_scale_range_t for available scales.
  * @param  gyroScale Gyroscope scale value to be set
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetGyroFullScale(gyro_full_scale_range_t gyroScale)
{
  return i2c->WriteRegister(MPU6050_ADDRESS, REG_GYRO_CONFIG, ((uint8_t)gyroScale << 3));
}

/**
  * @brief  This method used for configuring the accelerometer full scale range.
  * Check accel_full_scale_range_t for available scales.
  * @param  accelScale Accelerometer scale value to be set
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetAccelFullScale(accel_full_scale_range_t accelScale)
{
  return i2c->WriteRegister(MPU6050_ADDRESS, REG_ACCEL_CONFIG, ((uint8_t)accelScale << 3));
}

/**
  * @brief  This method used for getting the latest accelerometer X axis value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval uint16_t X axis acceleration value
  */
int16_t MPU6050::GetAccel_X(i2c_status_t *error)
{
  int16_t accelXVal = i2c->ReadRegister(MPU6050_ADDRESS, REG_ACCEL_X_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelXVal = (accelXVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, REG_ACCEL_X_OUT_L, error); // assemble higher and lower bytes
    return accelXVal;
  }

  return 0x00;
}

/**
  * @brief  This method used for getting the latest accelerometer Y axis value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval uint16_t Y axis acceleration value
  */
int16_t MPU6050::GetAccel_Y(i2c_status_t *error)
{
  int16_t accelYVal = i2c->ReadRegister(MPU6050_ADDRESS, REG_ACCEL_Y_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelYVal = (accelYVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, REG_ACCEL_Y_OUT_L, error); // assemble higher and lower bytes
    return accelYVal;
  }

  return 0x00;
}

/**
  * @brief  This method used for getting the latest accelerometer Z axis value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval uint16_t Z axis acceleration value
  */
int16_t MPU6050::GetAccel_Z(i2c_status_t *error)
{
  int16_t accelZVal = i2c->ReadRegister(MPU6050_ADDRESS, REG_ACCEL_Z_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelZVal = (accelZVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, REG_ACCEL_Z_OUT_L, error); // assemble higher and lower bytes
    return accelZVal;
  }

  return 0x00;
}

/**
  * @brief  This method used for getting the latest temperature value from the sensor.
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t Temperature in celcius-degrees
  */
int16_t MPU6050::GetTemperature(i2c_status_t *error)
{
  int16_t sensorTemp = i2c->ReadRegister(MPU6050_ADDRESS, REG_TEMP_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    sensorTemp = (sensorTemp << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, REG_TEMP_OUT_L, error); // assemble higher and lower bytes
    return sensorTemp;
  }

  return 0x00;
}
