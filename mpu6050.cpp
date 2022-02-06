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
  * @brief  This method wakes the sensor up by cleraing the MPU6050_Regs::PWR_MGMT_1
  * BIT_SLEEP. Power management 1 sensors default values is 0x40 so it will
  * be in sleep mode when it's powered up.
  * @param  none
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::WakeUpSensor(void)
{
  return i2c->WriteRegisterBit(MPU6050_ADDRESS, MPU6050_Regs::PWR_MGMT_1, BIT_SLEEP, false);
}

/**
  * @brief  This method resets the sensor by simply setting the MPU6050_Regs::PWR_MGMT_1
  * Device_Reset bit. After the sensor reset this bit will be cleared automatically.
  * TODO: Can be modify later to check the Device_Reset bit is clear after the reset
  * in order to make it safer (for this we probably need an interface for platform
  * delay function).
  * @param  none
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::ResetSensor(void)
{
  return i2c->WriteRegisterBit(MPU6050_ADDRESS, MPU6050_Regs::PWR_MGMT_1, BIT_DEVICE_RESET, true);
}

/**
  * @brief  This method used for configuring the gyroscope full scale range.
  * Check gyro_full_scale_range_t for available scales.
  * @param  gyroScale Gyroscope scale value to be set
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetGyroFullScale(gyro_full_scale_range_t gyroScale)
{
  return i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::GYRO_CONFIG, ((uint8_t)gyroScale << 3));
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
  uint8_t gyroConfig = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::GYRO_CONFIG, error);
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
  int16_t gyroXVal = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::GYRO_X_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroXVal = (gyroXVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::GYRO_X_OUT_L, error); // assemble higher and lower bytes
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
  int16_t gyroYVal = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::GYRO_Y_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroYVal = (gyroYVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::GYRO_Y_OUT_L, error); // assemble higher and lower bytes
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
  int16_t gyroZVal = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::GYRO_Z_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroZVal = (gyroZVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::GYRO_Z_OUT_L, error); // assemble higher and lower bytes
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
  return i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::ACCEL_CONFIG, ((uint8_t)accelScale << 3));
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
  uint8_t accelConfig = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ACCEL_CONFIG, error);
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
  int16_t accelXVal = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ACCEL_X_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelXVal = (accelXVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ACCEL_X_OUT_L, error); // assemble higher and lower bytes
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
  int16_t accelYVal = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ACCEL_Y_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelYVal = (accelYVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ACCEL_Y_OUT_L, error); // assemble higher and lower bytes
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
  int16_t accelZVal = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ACCEL_Z_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelZVal = (accelZVal << 8) | i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ACCEL_Z_OUT_L, error); // assemble higher and lower bytes
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
  int16_t sensorTemp = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::TEMP_OUT_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    sensorTemp = (sensorTemp << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::TEMP_OUT_L, error); // assemble higher and lower bytes
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
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::XG_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::XG_OFFS_USR_L, (offset & 0x00FF));
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
  int16_t gyroXOffset = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::XG_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroXOffset = (gyroXOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::XG_OFFS_USR_L, error); // assemble higher and lower bytes
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
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::YG_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::YG_OFFS_USR_L, (offset & 0x00FF));
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
  int16_t gyroYOffset = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::YG_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroYOffset = (gyroYOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::YG_OFFS_USR_L, error); // assemble higher and lower bytes
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
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::ZG_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::ZG_OFFS_USR_L, (offset & 0x00FF));
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
  int16_t gyroZOffset = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ZG_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    gyroZOffset = (gyroZOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ZG_OFFS_USR_L, error); // assemble higher and lower bytes
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
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::XA_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::XA_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;   
}

/**
  * @brief  This method used for getting the accelerometer X axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
int16_t MPU6050::GetAccel_X_Offset(i2c_status_t *error)
{
  int16_t accelXOffset = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::XA_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelXOffset = (accelXOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::XA_OFFS_USR_L, error); // assemble higher and lower bytes
    return accelXOffset;
  }

  return 0x00;
}

/**
  * @brief  This method used for setting the accelerometer Y axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetAccel_Y_Offset(int16_t offset)
{
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::YA_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::YA_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;  
}

/**
  * @brief  This method used for getting the accelerometer Y axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
int16_t MPU6050::GetAccel_Y_Offset(i2c_status_t *error)
{
  int16_t accelYOffset = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::YA_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelYOffset = (accelYOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::YA_OFFS_USR_L, error); // assemble higher and lower bytes
    return accelYOffset;
  }

  return 0x00; 
}

/**
  * @brief  This method used for setting the accelerometer Z axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetAccel_Z_Offset(int16_t offset)
{
  i2c_status_t result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::ZA_OFFS_USR_H, (offset >> 8));
  if(result == I2C_STATUS_SUCCESS)
  {
    result = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::ZA_OFFS_USR_L, (offset & 0x00FF));
  }
  return result;  
}

/**
  * @brief  This method used for getting the accelerometer Z axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
int16_t MPU6050::GetAccel_Z_Offset(i2c_status_t *error)
{
  int16_t accelZOffset = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ZA_OFFS_USR_H, error); // higher 8 bits
  if(*error == I2C_STATUS_SUCCESS)
  {
    accelZOffset = (accelZOffset << 8) |  i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::ZA_OFFS_USR_L, error); // assemble higher and lower bytes
    return accelZOffset;
  }

  return 0x00;   
}

/**
  * @brief  This method used for calibrating the accelerometer registers to given target values. Even if
  * the official calibration method in the invensense application notes are tried, it didnt work as expected.
  * So there is another method implemented to calibrate accelerometer registers automatically. It works with the
  * similar concept of binary search algorithm (setting a range and narrowing on each step).
  * @param targetX target value for accelerometer X axis register in MG so 1.0f means 1G
  * @param targetY target value for accelerometer Y axis register in MG
  * @param targetZ target value for accelerometer Z axis register in MG
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::Calibrate_Accel_Registers(float targetX_MG, float targetY_MG, float targetZ_MG)
{
  i2c_status_t result = I2C_STATUS_NONE;
  accel_full_scale_range_t accelRange = GetAccelFullScale(&result);
  if(result != I2C_STATUS_SUCCESS)
    return result;

  /* MG constant to convert raw register value to the 
   * gravity (9.81 m/s2). */
  const float mgConstant = mgCostantArr[accelRange];

  /* Some constants for our calibration routine to modify easily when needed. */
  const int16_t calibrationRangeHigh = 4096;
  const int16_t calibrationRangeLow = -calibrationRangeHigh;
  const uint8_t calibrationSteps = 13; // if the calibrationRangeHigh = 2^n so this will be (n +1)
  const int tolerance = 5; // acceptable tolerance between target and mean value of samples

  float meanOfNSamples = 0;
  int16_t regExpected = 0;
  int16_t diff = 0;
  int16_t high = calibrationRangeHigh;
  int16_t low = calibrationRangeLow;
  int16_t currentOffsetVal = 0;

  result = SetAccel_X_Offset(0); // set the offset to 0 first

  if(result != I2C_STATUS_SUCCESS)
    return result;

  result = SetAccel_Y_Offset(0); // set the offset to 0 first

  if(result != I2C_STATUS_SUCCESS)
    return result;

  result = SetAccel_Z_Offset(0); // set the offset to 0 first

  if(result != I2C_STATUS_SUCCESS)
    return result;

  /*
   * Accel X axis calibration
   */
  regExpected = targetX_MG / mgConstant;

  /* Get the initial deviation from our target value after reseting the offset register */
  meanOfNSamples = 0;
  for (uint8_t i = 0; i < 100 && result == I2C_STATUS_SUCCESS; i++)
  {
    meanOfNSamples += GetAccel_X_Raw(&result);
  }

  if(result != I2C_STATUS_SUCCESS)
    return result;

  meanOfNSamples *= 0.01;
  diff = regExpected - meanOfNSamples;

  /* Limit our ranges depending on the initial results. So we
   * are either work on negative or positive range during the 
   * calibration steps. */
  if (diff < 0)
    high = 0;
  else
    low = 0;

  /* Start N steps of calibration. This method is very similar to binary search
   * algorightm in sorted list. On every iteration we are setting the offset register
   * to a middle value of our high and low range then we read 100 samples from 
   * Accelerometer and compare our target and mean value of the samples. Depending on the
   * result we are narrowing our high and low limits and repeat... */
  for (uint8_t step = 0; step < calibrationSteps; step++)
  {
    /* Update offset register! */
    currentOffsetVal = (int16_t)((high + low) /2.0f);
    result = SetAccel_X_Offset(currentOffsetVal);
    if (result != I2C_STATUS_SUCCESS)
      return result;

    /* Take 100 samples and compare with target */
    meanOfNSamples = 0;
    for (uint8_t i = 0; i < 100 && result == I2C_STATUS_SUCCESS; i++)
    {
      meanOfNSamples += GetAccel_X_Raw(&result);
    }

    if (result != I2C_STATUS_SUCCESS)
      return result;

    meanOfNSamples *= 0.01;
    diff = regExpected - meanOfNSamples;

    /* Quick math.abs to check if difference is in tolerance level.
     * If yes abort calibration for this axis its done already! */
    if((diff & 0x8000 ? -diff : diff) < tolerance)
      break;

    /* Update ranges! */
    if(diff < 0)
      high = currentOffsetVal;
    else
      low = currentOffsetVal;
  } // for calibrationSteps

  /*
   * Accel Y axis calibration (TODO: unfortunately bad practice of code reusing but keep it for now!)
   */
  high = calibrationRangeHigh;
  low = calibrationRangeLow;
  currentOffsetVal = 0;

  regExpected = targetY_MG / mgConstant;

  /* Get the initial deviation from our target value after reseting the offset register */
  meanOfNSamples = 0;
  for (uint8_t i = 0; i < 100 && result == I2C_STATUS_SUCCESS; i++)
  {
    meanOfNSamples += GetAccel_Y_Raw(&result);
  }

  if (result != I2C_STATUS_SUCCESS)
    return result;

  meanOfNSamples *= 0.01;
  diff = regExpected - meanOfNSamples;

  /* Limit our ranges depending on the initial results. So we
   * are either work on negative or positive range during the 
   * calibration steps. */
  if (diff < 0)
    high = 0;
  else
    low = 0;

  /* Start N steps of calibration. This method is very similar to binary search
   * algorightm in sorted list. On every iteration we are setting the offset register
   * to a middle value of our high and low range then we read 100 samples from 
   * Accelerometer and compare our target and mean value of the samples. Depending on the
   * result we are narrowing our high and low limits and repeat... */
  for (uint8_t step = 0; step < calibrationSteps; step++)
  {
    /* Update offset register! */
    currentOffsetVal = (int16_t)((high + low) /2.0f);
    result = SetAccel_Y_Offset(currentOffsetVal);
    if (result != I2C_STATUS_SUCCESS)
      return result;

    /* Take 100 samples and compare with target */
    meanOfNSamples = 0;
    for (uint8_t i = 0; i < 100 && result == I2C_STATUS_SUCCESS; i++)
    {
      meanOfNSamples += GetAccel_Y_Raw(&result);
    }

    if (result != I2C_STATUS_SUCCESS)
      return result;

    meanOfNSamples *= 0.01;
    diff = regExpected - meanOfNSamples;

    /* Quick math.abs to check if difference is in tolerance level.
     * If yes abort calibration for this axis its done already! */
    if((diff & 0x8000 ? -diff : diff) < tolerance)
      break;

    /* Update ranges! */
    if(diff < 0)
      high = currentOffsetVal;
    else
      low = currentOffsetVal;
  } // for calibrationSteps

  /*
   * Accel Z axis calibration (TODO: unfortunately bad practice of code reusing but keep it for now!)
   */
  high = calibrationRangeHigh;
  low = calibrationRangeLow;
  currentOffsetVal = 0;

  regExpected = targetZ_MG / mgConstant;

  /* Get the initial deviation from our target value after reseting the offset register */
  meanOfNSamples = 0;
  for (uint8_t i = 0; i < 100 && result == I2C_STATUS_SUCCESS; i++)
  {
    meanOfNSamples += GetAccel_Z_Raw(&result);
  }

  if (result != I2C_STATUS_SUCCESS)
    return result;

  meanOfNSamples *= 0.01;
  diff = regExpected - meanOfNSamples;

  /* Limit our ranges depending on the initial results. So we
   * are either work on negative or positive range during the 
   * calibration steps. */
  if (diff < 0)
    high = 0;
  else
    low = 0;

  /* Start N steps of calibration. This method is very similar to binary search
   * algorightm in sorted list. On every iteration we are setting the offset register
   * to a middle value of our high and low range then we read 100 samples from 
   * Accelerometer and compare our target and mean value of the samples. Depending on the
   * result we are narrowing our high and low limits and repeat... */
  for (uint8_t step = 0; step < calibrationSteps; step++)
  {
    /* Update offset register! */
    currentOffsetVal = (int16_t)((high + low) /2.0f);
    result = SetAccel_Z_Offset(currentOffsetVal);
    if (result != I2C_STATUS_SUCCESS)
      return result;

    /* Take 100 samples and compare with target */
    meanOfNSamples = 0;
    for (uint8_t i = 0; i < 100 && result == I2C_STATUS_SUCCESS; i++)
    {
      meanOfNSamples += GetAccel_Z_Raw(&result);
    }

    if (result != I2C_STATUS_SUCCESS)
      return result;

    meanOfNSamples *= 0.01;
    diff = regExpected - meanOfNSamples;

    /* Quick math.abs to check if difference is in tolerance level.
     * If yes abort calibration for this axis its done already! */
    if((diff & 0x8000 ? -diff : diff) < tolerance)
      break;

    /* Update ranges! */
    if(diff < 0)
      high = currentOffsetVal;
    else
      low = currentOffsetVal;
  } // for calibrationSteps

  return result;
}

/**
  * @brief  This method returns the MG (Gravity) coversion value depending on
  * the accelerometer full scale range. MG value is used to convert raw sensor value to Gravity
  * for acceleration related calculations.
  * @param accelRange Configured accelerometer full scale range
  * @retval float
  */
float MPU6050::GetAccel_MG_Constant(accel_full_scale_range_t accelRange)
{
  return mgCostantArr[accelRange];
}

/**
  * @brief This function sets the gyroscope sample rate divider. Once the sample rate divider set, actual sample rate
  *        can be found with this formula:
  *        Actual sample rate = Gyroscope Output Rate / (1 + sampleRate)
  *        Keep in mind that Gyroscope Output Rate = 8kHz when the DLPF (digital low pass filter) is disabled
  *        (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled. Accel sample rate is constantly 1 kHz.
  * @param sampleRate Gyroscope sample rate divider value
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetGyro_SampleRateDivider(uint8_t sampleRate)
{
  return i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::SMPRT_DIV, sampleRate);
}

/**
  * @brief This function gets the gyroscope sample rate divider.
  *        Actual sample rate = Gyroscope Output Rate / (1 + sampleRate)
  *        Keep in mind that Gyroscope Output Rate = 8kHz when the DLPF (digital low pass filter) is disabled
  *        (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled. Accel sample rate is constantly 1 kHz.
  * @param error Result of the operation
  * @retval uint8_t
  */
uint8_t MPU6050::GetGyro_SampleRateDivider(i2c_status_t *error)
{
  return i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::SMPRT_DIV, error);
}

/**
  * @brief This function sets the sensor digital low pass filter values. Tighter bandwitdh configs will
  *        generate more delay on the sensor outputs (check sensor datasheet). Keep in mind that default
  *        Gyroscope sample rate is 8 kHz but if we set DLPF config different than 0 it will be 1 kHz by default
  *        unless if we make an extra configuration to Sample Rate Divider.
  * @param dlpfConfig Digital low pass filter configuration value
  * @retval i2c_status_t
  */
i2c_status_t MPU6050::SetSensor_DLPF_Config(dlpf_config_t dlpfConfig)
{
  i2c_status_t error = I2C_STATUS_NONE;
  /* This register also have EXT_SYNC config, so only set the DLPF part! */
  uint8_t currentRegVal = i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::CONFIG, &error);
  if (error == I2C_STATUS_SUCCESS) {
    currentRegVal &= (~0x07); // clear the DLPF section from the current register value
    error = i2c->WriteRegister(MPU6050_ADDRESS, MPU6050_Regs::CONFIG, (uint8_t)dlpfConfig | currentRegVal);
  }
  
  return error;
}

/**
  * @brief This function gets the current sensor digital low pass filter configuration.
  * @param error Result of the operation
  * @retval dlpf_config_t
  */
dlpf_config_t MPU6050::GetSensor_DLPF_Config(i2c_status_t *error)
{
  /* get only the first 3 bit of the register */
  return (dlpf_config_t) (i2c->ReadRegister(MPU6050_ADDRESS, MPU6050_Regs::CONFIG, error) & 0x07);
}

/**
  * @brief This function gets the current sensor sample rate. In order to do this, method
  *        reads Sample Rate Divider (0x19) and DLPF Config (0x1A) registers.
  * @param error Result of the operation
  * @retval float Current sample rate in Hz
  */
float MPU6050::GetSensor_CurrentSampleRate_Hz(i2c_status_t *error)
{
  uint8_t sampleRateDivider = GetGyro_SampleRateDivider(error);
  if(*error != I2C_STATUS_SUCCESS)
    return 0x00;

  dlpf_config_t dlpfConfig = GetSensor_DLPF_Config(error);
  if(*error != I2C_STATUS_SUCCESS)
    return 0x00;
  
  /* if dlpf config is disabled (0 or 7) then sample rate is 8 kHz otherwise 1 kHz */
  const float gyroDefaultOutRateHz = (dlpfConfig == DLPF_BW_260Hz || dlpfConfig == DLPF_RESERVED) ? 8000 : 1000;
  return gyroDefaultOutRateHz / (1 + sampleRateDivider);
}
