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
  * @brief  This method wakes up the sensor and configures the accelerometer and
  * gyroscope full scale renges with given parameters. It returns the result of
  * the process.
  * @param  gyroScale Gyroscope scale value to be set
  * @param  accelScale Accelerometer scale value to be set
  * @retval i2c_status_t Success rate
  */
i2c_status_t MPU6050::InitializeSensor(
    gyro_full_scale_range_t gyroScale,
    accel_full_scale_range_t accelScale)
{
  i2c_status_t result = WakeUpSensor();
  if(result == I2C_STATUS_SUCCESS)
    result = SetGyroFullScale(gyroScale);
  if(result == I2C_STATUS_SUCCESS)
    result = SetAccelFullScale(accelScale);

  return result;
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
  * @brief  This method resets the sensor by simply setting the REG_PWR_MGMT_1
  * Device_Reset bit. After the sensor reset this bit will be cleared automatically.
  * TODO: Can be modify later to check the Device_Reset bit is clear after the reset
  * in order to make it safer (for this we probably need an interface for platform
  * delay function).
  * @param  none
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::ResetSensor(void)
{
  return i2c->WriteRegisterBit(MPU6050_ADDRESS, REG_PWR_MGMT_1, BIT_DEVICE_RESET, true);
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
  * @brief  This method used for getting the gyroscope full scale range.
  * Check gyro_full_scale_range_t for available scales. It basically reads the
  * Gyro configuration register and returns the full scale range.
  * @param  error Result of the sensor reading process
  * @retval gyro_full_scale_range_t
  */
gyro_full_scale_range_t MPU6050::GetGyroFullScale(i2c_status_t *error)
{
  uint8_t gyroConfig = i2c->ReadRegister(MPU6050_ADDRESS, REG_GYRO_CONFIG, error);
  return (gyro_full_scale_range_t)((gyroConfig >> 3) & 0x03);
}

/**
  * @brief  This method used for getting the latest gyroscope X axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and gyroscope full
  * scale range is set to desired range before reading the values.
  * @param  error Error state of process
  * @retval int16_t X axis RAW gyroscope value
  */
int16_t MPU6050::GetGyro_X_Raw(i2c_status_t *error)
{
  int16_t gyroXVal = i2c->ReadRegister(MPU6050_ADDRESS, REG_GYRO_X_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroXVal = (gyroXVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, REG_GYRO_X_OUT_L, error); // assemble higher and lower bytes
    return gyroXVal;
  }

  return 0x00; 
}

/**
  * @brief  This method used for getting the latest gyroscope Y axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and gyroscope full
  * scale range is set to desired range before reading the values.
  * @param  error Error state of process
  * @retval int16_t Y axis RAW gyroscope value
  */
int16_t MPU6050::GetGyro_Y_Raw(i2c_status_t *error)
{
  int16_t gyroYVal = i2c->ReadRegister(MPU6050_ADDRESS, REG_GYRO_Y_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroYVal = (gyroYVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, REG_GYRO_Y_OUT_L, error); // assemble higher and lower bytes
    return gyroYVal;
  }

  return 0x00;  
}

/**
  * @brief  This method used for getting the latest gyroscope Z axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and gyroscope full
  * scale range is set to desired range before reading the values.
  * @param  error Error state of process
  * @retval int16_t Z axis RAW gyroscope value
  */
int16_t MPU6050::GetGyro_Z_Raw(i2c_status_t *error)
{
  int16_t gyroZVal = i2c->ReadRegister(MPU6050_ADDRESS, REG_GYRO_Z_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroZVal = (gyroZVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, REG_GYRO_Z_OUT_L, error); // assemble higher and lower bytes
    return gyroZVal;
  }

  return 0x00;
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
  * @brief  This method used for getting the acceleromteter full scale range.
  * Check accel_full_scale_range_t for available scales. It basically reads the
  * Accel configuration register and returns the full scale range.
  * @param  error Result of the sensor reading process
  * @retval accel_full_scale_range_t
  */
accel_full_scale_range_t MPU6050::GetAccelFullScale(i2c_status_t *error)
{
  uint8_t accelConfig = i2c->ReadRegister(MPU6050_ADDRESS, REG_ACCEL_CONFIG, error);
  return (accel_full_scale_range_t)((accelConfig >> 3) & 0x03);  
}

/**
  * @brief  This method used for getting the latest accelerometer X axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t X axis RAW acceleration value
  */
int16_t MPU6050::GetAccel_X_Raw(i2c_status_t *error)
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
  * @brief  This method used for getting the latest accelerometer Y axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t Y axis RAW acceleration value
  */
int16_t MPU6050::GetAccel_Y_Raw(i2c_status_t *error)
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
  * @brief  This method used for getting the latest accelerometer Z axis RAW value from
  * the sensor. Make sure that sensor is not in sleeping mode and accelerometer full
  * scale range is set to desired range, before reading the values.
  * @param  error Error state of process
  * @retval int16_t Z axis RAW acceleration value
  */
int16_t MPU6050::GetAccel_Z_Raw(i2c_status_t *error)
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
  * @retval float Temperature in celcius-degrees
  */
float MPU6050::GetTemperature_Celcius(i2c_status_t *error)
{
  int16_t sensorTemp = i2c->ReadRegister(MPU6050_ADDRESS, REG_TEMP_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    sensorTemp = (sensorTemp << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, REG_TEMP_OUT_L, error); // assemble higher and lower bytes
    return (sensorTemp / 340.0 + 36.53f);
  }

  return 0x00;
}

/**
  * @brief  This method used for setting the gyroscope X axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetGyro_X_Offset(int16_t offset)
{
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, REG_XG_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, REG_XG_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;
}

/**
  * @brief  This method used for getting the gyroscope X axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
int16_t MPU6050::GetGyro_X_Offset(i2c_status_t *error)
{
  int16_t gyroXOffset = i2c->ReadRegister(MPU6050_ADDRESS, REG_XG_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroXOffset = (gyroXOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, REG_XG_OFFS_USR_L, error); // assemble higher and lower bytes
    return gyroXOffset;
  }

  return 0x00;
}

/**
  * @brief  This method used for setting the gyroscope Y axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetGyro_Y_Offset(int16_t offset)
{
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, REG_YG_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, REG_YG_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;
}

/**
  * @brief  This method used for getting the gyroscope Y axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
int16_t MPU6050::GetGyro_Y_Offset(i2c_status_t *error)
{
  int16_t gyroYOffset = i2c->ReadRegister(MPU6050_ADDRESS, REG_YG_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroYOffset = (gyroYOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, REG_YG_OFFS_USR_L, error); // assemble higher and lower bytes
    return gyroYOffset;
  }

  return 0x00;
}

/**
  * @brief  This method used for setting the gyroscope Z axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetGyro_Z_Offset(int16_t offset)
{
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, REG_ZG_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, REG_ZG_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;
}

/**
  * @brief  This method used for getting the gyroscope Z axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
int16_t MPU6050::GetGyro_Z_Offset(i2c_status_t *error)
{
  int16_t gyroZOffset = i2c->ReadRegister(MPU6050_ADDRESS, REG_ZG_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroZOffset = (gyroZOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, REG_ZG_OFFS_USR_L, error); // assemble higher and lower bytes
    return gyroZOffset;
  }

  return 0x00;
}

/**
  * @brief  This method used for calibrating the gyroscope registers to given target values.
  * Its VERY important to mention that when you call this method make sure that the sensor is
  * standing statically (no vibrations, no rotations, no movement etc.) otherwise you will end
  * up with wrong calibration values!
  * TODO: Use more detailed return type about the calibration status to inform user about the
  * failure (which step it failed and why etc.).
  * @param targetX target value for gyroscope X axis register
  * @param targetY target value for gyroscope Y axis register
  * @param targetZ target value for gyroscope Z axis register
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::Calibrate_Gyro_Registers(int16_t targetX, int16_t targetY, int16_t targetZ)
{
  i2c_status_t result = I2C_STATUS_NONE;
  gyro_full_scale_range_t gyroRange = GetGyroFullScale(&result);
  if(result != I2C_STATUS_SUCCESS)
    return result;

  /* DPS constant to convert raw register value to the 
   * degree per seconds (angular velocity). */
  const float dpsConstant = dpsConstantArr[gyroRange];
  float sumOfSamples = 0;
  int16_t offsetVal = 0;

  /*
   * Gyro X axis calibration
   */
  for(uint16_t i = 0; i < 1000 && result == I2C_STATUS_SUCCESS; i++)
  {
    sumOfSamples += GetGyro_X_Raw(&result);
  }
  sumOfSamples *= 0.001f; // get mean value of 1000 readings

  if(result != I2C_STATUS_SUCCESS)
    return result;

  offsetVal = (int16_t)(((targetX - sumOfSamples) * dpsConstant) * GYRO_OFFSET_1DPS);
  result = SetGyro_X_Offset(offsetVal);

  if(result != I2C_STATUS_SUCCESS)
    return result;

  /*
   * Gyro Y axis calibration
   */
  sumOfSamples = 0;
  for(uint16_t i = 0; i < 1000 && result == I2C_STATUS_SUCCESS; i++)
  {
    sumOfSamples += GetGyro_Y_Raw(&result);
  }
  sumOfSamples *= 0.001f; // get mean value of 1000 readings

  if(result != I2C_STATUS_SUCCESS)
    return result;

  offsetVal = (int16_t)(((targetY - sumOfSamples) * dpsConstant) * GYRO_OFFSET_1DPS);
  result = SetGyro_Y_Offset(offsetVal);

  if(result != I2C_STATUS_SUCCESS)
    return result;

  /*
   * Gyro Z axis calibration
   */
  sumOfSamples = 0;
  for(uint16_t i = 0; i < 1000 && result == I2C_STATUS_SUCCESS; i++)
  {
    sumOfSamples += GetGyro_Z_Raw(&result);
  }
  sumOfSamples *= 0.001f; // get mean value of 1000 readings

  if(result != I2C_STATUS_SUCCESS)
    return result;

  offsetVal = (int16_t)(((targetZ - sumOfSamples) * dpsConstant) * GYRO_OFFSET_1DPS);
  result = SetGyro_Z_Offset(offsetVal);

  return result;
}

/**
  * @brief  This method returns the DPS (Degree Per Second) coversion value depending on
  * the gyroscope full scale range. DPS value is used to convert raw sensor value to angular
  * velocity for orientation related calculations.
  * @param gyroRange Configured gyro full scale range
  * @retval float
  */
float MPU6050::GetGyro_DPS_Constant(gyro_full_scale_range_t gyroRange)
{
  return dpsConstantArr[gyroRange];
}

/**
  * @brief  This method used for setting the accelerometer X axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetAccel_X_Offset(int16_t offset)
{
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, REG_XA_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, REG_XA_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;   
}

/**
  * @brief  This method used for setting the accelerometer Y axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetAccel_Y_Offset(int16_t offset)
{
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, REG_YA_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, REG_YA_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;  
}

/**
  * @brief  This method used for setting the accelerometer Z axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetAccel_Z_Offset(int16_t offset)
{
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, REG_ZA_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, REG_ZA_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;  
}
