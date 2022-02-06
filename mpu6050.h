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

/* TODO: Make a setter for device address. Use enum for 2 possible sensor address! */
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

/* define sensor register type here to write it easily */
typedef const uint8_t REGType;
#define REG static REGType

struct MPU6050_Regs {
  REG PWR_MGMT_1 = 0x6B;
  REG GYRO_CONFIG = 0x1B;
  REG ACCEL_CONFIG = 0x1C;
  /* Accelerometer read registers */
  REG ACCEL_X_OUT_L = 0x3C;
  REG ACCEL_X_OUT_H = 0x3B;
  REG ACCEL_Y_OUT_H = 0x3D;
  REG ACCEL_Y_OUT_L = 0x3E;
  REG ACCEL_Z_OUT_H = 0x3F;
  REG ACCEL_Z_OUT_L = 0x40;
  /* Temperature read registers */
  REG TEMP_OUT_H = 0x41;
  REG TEMP_OUT_L = 0x42;
  /* Gyroscope read registers */
  REG GYRO_X_OUT_H = 0x43;
  REG GYRO_X_OUT_L = 0x44;
  REG GYRO_Y_OUT_H = 0x45;
  REG GYRO_Y_OUT_L = 0x46;
  REG GYRO_Z_OUT_H = 0x47;
  REG GYRO_Z_OUT_L = 0x48;
  /* Gyroscope offset registers */
  REG XG_OFFS_USR_H = 0x13;
  REG XG_OFFS_USR_L = 0x14;
  REG YG_OFFS_USR_H = 0x15;
  REG YG_OFFS_USR_L = 0x16;
  REG ZG_OFFS_USR_H = 0x17;
  REG ZG_OFFS_USR_L = 0x18;
  /* Accellerometer offset registers */
  REG XA_OFFS_USR_H = 0x06;
  REG XA_OFFS_USR_L = 0x07;
  REG YA_OFFS_USR_H = 0x08;
  REG YA_OFFS_USR_L = 0x09;
  REG ZA_OFFS_USR_H = 0x0A;
  REG ZA_OFFS_USR_L = 0x0B;

  REG SMPRT_DIV = 0x19; // sample rate divider
  REG CONFIG = 0x1A;    // digital low passand extra sync configutation
};

/* Gyro offset register constant to compensate 1 DPS (degree per second) offset.
 * Check sensor datasheet for more info about the offset procedure! */
#define GYRO_OFFSET_1DPS 32.8f

/* TODO: This value is not using currently. After some experiments and tests somehow
 * offset procedure didnt work as expected in the application notes. So another method
 * applied for auto-calibration, keep the value for future consideration!
 * Accel offset register constant to compensate 1 MG (Gravity - 9.81 m/s2) offset.
 * Check sensor datasheet for more info about the offset procedure! */
#define ACCEL_OFFSET_1MG 4096.0f

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

/* Digital low pass filter config bandwidth values in Hz*/
enum dlpf_config_t 
{
  DLPF_BW_260Hz = 0,
  DLPF_BW_184Hz = 1,
  DLPF_BW_94Hz = 2,
  DLPF_BW_44Hz = 3,
  DLPF_BW_21Hz = 4,
  DLPF_BW_10Hz = 5,
  DLPF_BW_5Hz = 6,
  DLPF_RESERVED = 7
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
  * @brief  This method used for getting the gyroscope X axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
  int16_t GetGyro_X_Offset(i2c_status_t* error);

  /**
  * @brief  This method used for setting the gyroscope Y axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
  i2c_status_t SetGyro_Y_Offset(int16_t offset);

  /**
  * @brief  This method used for getting the gyroscope Y axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
  int16_t GetGyro_Y_Offset(i2c_status_t* error);

  /**
  * @brief  This method used for setting the gyroscope Z axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
  i2c_status_t SetGyro_Z_Offset(int16_t offset);

  /**
  * @brief  This method used for getting the gyroscope Z axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
  int16_t GetGyro_Z_Offset(i2c_status_t* error);

  /**
  * @brief  This method used for calibrating the gyroscope registers to given target values.
  * @param targetX target value for gyroscope X axis register
  * @param targetY target value for gyroscope Y axis register
  * @param targetZ target value for gyroscope Z axis register
  * @retval i2c_status_t
  */
  i2c_status_t Calibrate_Gyro_Registers(int16_t targetX = 0, int16_t targetY = 0, int16_t targetZ = 0);

  /**
  * @brief  This method returns the DPS (Degree Per Second) coversion value depending on
  * the gyroscope full scale range. DPS value is used to convert raw sensor value to angular
  * velocity for orientation related calculations.
  * @param gyroRange Configured gyro full scale range
  * @retval float
  */
  float GetGyro_DPS_Constant(gyro_full_scale_range_t gyroRange);

  /**
  * @brief  This method used for setting the accelerometer X axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
  i2c_status_t SetAccel_X_Offset(int16_t offset);

  /**
  * @brief  This method used for getting the accelerometer X axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
  int16_t GetAccel_X_Offset(i2c_status_t* error);

  /**
  * @brief  This method used for setting the accelerometer Y axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
  i2c_status_t SetAccel_Y_Offset(int16_t offset);

  /**
  * @brief  This method used for getting the accelerometer Y axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
  int16_t GetAccel_Y_Offset(i2c_status_t* error);

  /**
  * @brief  This method used for setting the accelerometer Z axis offset value. Offset is
  * using in the sensor calibration routine.
  * @param offset
  * @retval i2c_status_t
  */
  i2c_status_t SetAccel_Z_Offset(int16_t offset);

  /**
  * @brief  This method used for getting the accelerometer Z axis offset value.
  * @param error Result of the operation
  * @retval int16_t
  */
  int16_t GetAccel_Z_Offset(i2c_status_t* error);

  /**
  * @brief  This method used for calibrating the accelerometer registers to given target values.
  * @param targetX target value for accelerometer X axis register in MG so 1.0f means 1G
  * @param targetY target value for accelerometer Y axis register in MG
  * @param targetZ target value for accelerometer Z axis register in MG
  * @retval i2c_status_t
  */
  i2c_status_t Calibrate_Accel_Registers(float targetX_MG = 0.0f, float targetY_MG = 0.0f, float targetZ_MG = 1.0f);

  /**
  * @brief  This method returns the MG (Gravity) coversion value depending on
  * the accelerometer full scale range. MG value is used to convert raw sensor value to Gravity
  * for acceleration related calculations.
  * @param accelRange Configured accelerometer full scale range
  * @retval float
  */
  float GetAccel_MG_Constant(accel_full_scale_range_t accelRange);

  /**
  * @brief This function sets the gyroscope sample rate divider. Once the sample rate divider set, actual sample rate
  *        can be found with this formula:
  *        Actual sample rate = Gyroscope Output Rate / (1 + sampleRate)
  *        Keep in mind that Gyroscope Output Rate = 8kHz when the DLPF (digital low pass filter) is disabled
  *        (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled. Accel sample rate is constantly 1 kHz.
  * @param sampleRate Gyroscope sample rate divider.     
  * @retval i2c_status_t
  */
  i2c_status_t SetGyro_SampleRateDivider(uint8_t sampleRate);

  /**
  * @brief This function gets the gyroscope sample rate divider.
  *        Actual sample rate = Gyroscope Output Rate / (1 + sampleRate)
  *        Keep in mind that Gyroscope Output Rate = 8kHz when the DLPF (digital low pass filter) is disabled
  *        (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled. Accel sample rate is constantly 1 kHz.
  * @param error Result of the operation
  * @retval uint8_t
  */
  uint8_t GetGyro_SampleRateDivider(i2c_status_t* error);

  /**
  * @brief This function sets the sensor digital low pass filter values. Tighter bandwitdh configs will
  *        generate more delay on the sensor outputs (check sensor datasheet). Keep in mind that default
  *        Gyroscope sample rate is 8 kHz but if we set DLPF config different than 0 it will be 1 kHz by default
  *        unless if we make an extra configuration to Sample Rate Divider.
  * @param dlpfConfig Digital low pass filter configuration value
  * @retval i2c_status_t
  */
  i2c_status_t SetSensor_DLPF_Config(dlpf_config_t dlpfConfig);

  /**
  * @brief This function gets the current sensor digital low pass filter configuration.
  * @param error Result of the operation
  * @retval dlpf_config_t
  */
  dlpf_config_t GetSensor_DLPF_Config(i2c_status_t* error);

  /**
  * @brief This function gets the current sensor sample rate. In order to do this, method
  *        reads Sample Rate Divider (0x19) and DLPF Config (0x1A) registers.
  * @param error Result of the operation
  * @retval float Current sample rate in Hz
  */
  float GetSensor_CurrentSampleRate_Hz (i2c_status_t* error);

private:
  I2C_Interface* i2c = nullptr;
  /* DPS constant to convert raw register value to the degree per seconds (angular velocity).
   * The index of the values are adjusted to have corresponding values with the gyro_full_scale_range_t
   * enum. So, we can just get the DPS value by "dpsConstantArr[GYRO_SCALE_250]"" for GYRO_SCALE_250. */
  const float dpsConstantArr[4] = {250.0f / 32767.0f, 500.0f / 32767.0f, 1000.0f / 32767.0f, 2000.0f / 32767.0f};

  /* MG constant to convert raw register value to gravity (9.81 m/s2). The index of the values are
   * adjusted to have corresponding values with the accel_full_scale_range_t enum. So, we can just get
   * the MG value by "mgConstantArr[ACCEL_SCALE_2G]"" for ACCEL_SCALE_2G. */
  const float mgCostantArr[4] = {2.0f / 32767.0f, 4.0f / 32767.0f, 8.0f / 32767.0f, 16.0f / 32767.0f};

};

#endif /* include guard */
