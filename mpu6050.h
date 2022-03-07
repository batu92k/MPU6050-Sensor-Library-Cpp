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

namespace MPU6050_Driver {

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
  #define SensorConst static constexpr uint8_t
  /* bit masks */
  #define BIT_0 (1 << 0)
  #define BIT_1 (1 << 1)
  #define BIT_2 (1 << 2)
  #define BIT_3 (1 << 3)
  #define BIT_4 (1 << 4)
  #define BIT_5 (1 << 5)
  #define BIT_6 (1 << 6)
  #define BIT_7 (1 << 7)

  namespace Sensor_Regs {
    SensorConst USER_CTRL = 0x6A;
    SensorConst PWR_MGMT_1 = 0x6B;
    SensorConst GYRO_CONFIG = 0x1B;
    SensorConst ACCEL_CONFIG = 0x1C;
    SensorConst INT_ENABLE = 0x38;
    SensorConst INT_PIN_CFG = 0x37;
    SensorConst INT_STATUS = 0x3A;

    /* FIFO registers */
    SensorConst FIFO_EN = 0x23;
    SensorConst FIFO_COUNT_L = 0x73;
    SensorConst FIFO_COUNT_H = 0x72;
    SensorConst FIFO_R_W = 0x74;

    /* Accelerometer read registers */
    SensorConst ACCEL_X_OUT_L = 0x3C;
    SensorConst ACCEL_X_OUT_H = 0x3B;
    SensorConst ACCEL_Y_OUT_H = 0x3D;
    SensorConst ACCEL_Y_OUT_L = 0x3E;
    SensorConst ACCEL_Z_OUT_H = 0x3F;
    SensorConst ACCEL_Z_OUT_L = 0x40;
    /* Temperature read SensorConstisters */
    SensorConst TEMP_OUT_H = 0x41;
    SensorConst TEMP_OUT_L = 0x42;
    /* Gyroscope read SensorConstisters */
    SensorConst GYRO_X_OUT_H = 0x43;
    SensorConst GYRO_X_OUT_L = 0x44;
    SensorConst GYRO_Y_OUT_H = 0x45;
    SensorConst GYRO_Y_OUT_L = 0x46;
    SensorConst GYRO_Z_OUT_H = 0x47;
    SensorConst GYRO_Z_OUT_L = 0x48;
    /* Gyroscope offset SensorConstisters */
    SensorConst XG_OFFS_USR_H = 0x13;
    SensorConst XG_OFFS_USR_L = 0x14;
    SensorConst YG_OFFS_USR_H = 0x15;
    SensorConst YG_OFFS_USR_L = 0x16;
    SensorConst ZG_OFFS_USR_H = 0x17;
    SensorConst ZG_OFFS_USR_L = 0x18;
    /* Accellerometer offset SensorConstisters */
    SensorConst XA_OFFS_USR_H = 0x06;
    SensorConst XA_OFFS_USR_L = 0x07;
    SensorConst YA_OFFS_USR_H = 0x08;
    SensorConst YA_OFFS_USR_L = 0x09;
    SensorConst ZA_OFFS_USR_H = 0x0A;
    SensorConst ZA_OFFS_USR_L = 0x0B;

    SensorConst SMPRT_DIV = 0x19; // sample rate divider
    SensorConst CONFIG = 0x1A;    // digital low passand extra sync configutation
  };

  namespace Regbits_USER_CTRL {
    SensorConst BIT_SIG_CONF_RESET = BIT_0;
    SensorConst BIT_I2C_MST_RESET = BIT_1;
    SensorConst BIT_FIFO_RESET = BIT_2;
    SensorConst BIT_I2C_IF_DIS = BIT_4;
    SensorConst BIT_I2C_MST_EN = BIT_5;
    SensorConst BIT_FIFO_EN = BIT_6;
  }

  namespace Regbits_INT_ENABLE {
    SensorConst BIT_DATA_RDY_EN = BIT_0;
    SensorConst BIT_I2C_MST_INT_EN = BIT_3;
    SensorConst BIT_FIFO_OFLOW_EN = BIT_4;
  }

  namespace Regbits_PWR_MGMT_1 {
    SensorConst BIT_CLKSEL_0 = BIT_0;
    SensorConst BIT_CLKSEL_1 = BIT_1;
    SensorConst BIT_CLKSEL_2 = BIT_2;
    SensorConst BIT_TEMP_DIS = BIT_3;
    SensorConst BIT_CYCLE = BIT_5;
    SensorConst BIT_SLEEP = BIT_6;
    SensorConst BIT_DEVICE_RESET = BIT_7;
  };

  namespace Regbits_FIFO_EN {
    SensorConst BIT_SLV0_FIFO_EN = BIT_0;
    SensorConst BIT_SLV1_FIFO_EN = BIT_1;
    SensorConst BIT_SLV2_FIFO_EN = BIT_2;
    SensorConst BIT_ACCEL_FIFO_EN = BIT_3;
    SensorConst BIT_ZG_FIFO_EN = BIT_4;
    SensorConst BIT_YG_FIFO_EN = BIT_5;
    SensorConst BIT_XG_FIFO_EN = BIT_6;
    SensorConst BIT_TEMP_FIFO_EN = BIT_7;
  };

  namespace Regbits_INT_PIN_CFG {
    SensorConst BIT_I2C_BYPASS_EN = BIT_1;
    SensorConst BIT_FSYNC_INT_EN = BIT_2;
    SensorConst BIT_FSYNC_INT_LEVEL = BIT_3;
    SensorConst BIT_INT_RD_CLEAR = BIT_4;
    SensorConst BIT_LATCH_INT_EN = BIT_5;
    SensorConst BIT_INT_OPEN = BIT_6;
    SensorConst BIT_INT_LEVEL = BIT_7;
  }

  /* Gyroscope full scale ranges in degrees per second */
  enum class Gyro_FS_t
  {
    FS_250_DPS = 0,
    FS_500_DPS = 1,
    FS_1000_DPS = 2,
    FS_2000_DPS = 3
  };

  /* Accelerometer full scale ranges in G's */
  enum class Accel_FS_t
  {
    FS_2G = 0,
    FS_4G = 1,
    FS_8G = 2,
    FS_16G = 3
  };

  /* Digital low pass filter config bandwidth values in Hz*/
  enum class DLPF_t 
  {
    BW_260Hz = 0,
    BW_184Hz = 1,
    BW_94Hz = 2,
    BW_44Hz = 3,
    BW_21Hz = 4,
    BW_10Hz = 5,
    BW_5Hz = 6,
    RESERVED = 7
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
        Gyro_FS_t gyroScale = Gyro_FS_t::FS_250_DPS,
        Accel_FS_t accelScale = Accel_FS_t::FS_2G);

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
    i2c_status_t SetGyroFullScale(Gyro_FS_t gyroScale);

    /**
    * @brief  This method used for getting the gyroscope full scale range.
    * Check gyro_full_scale_range_t for available scales. It basically reads the
    * Gyro configuration register and returns the full scale range.
    * @param  error Result of the sensor reading process
    * @retval gyro_full_scale_range_t
    */
    Gyro_FS_t GetGyroFullScale(i2c_status_t* error);

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
    i2c_status_t SetAccelFullScale(Accel_FS_t accelScale);

    /**
    * @brief  This method used for getting the acceleromteter full scale range.
    * Check accel_full_scale_range_t for available scales. It basically reads the
    * Accel configuration register and returns the full scale range.
    * @param  error Result of the sensor reading process
    * @retval accel_full_scale_range_t
    */
    Accel_FS_t GetAccelFullScale(i2c_status_t* error);

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
    float GetGyro_DPS_Constant(Gyro_FS_t gyroRange);

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
    float GetAccel_MG_Constant(Accel_FS_t accelRange);

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
    i2c_status_t SetSensor_DLPF_Config(DLPF_t dlpfConfig);

    /**
    * @brief This function gets the current sensor digital low pass filter configuration.
    * @param error Result of the operation
    * @retval dlpf_config_t
    */
    DLPF_t GetSensor_DLPF_Config(i2c_status_t* error);

    /**
    * @brief This function gets the current sensor sample rate. In order to do this, method
    *        reads Sample Rate Divider (0x19) and DLPF Config (0x1A) registers.
    * @param error Result of the operation
    * @retval float Current sample rate in Hz
    */
    float GetSensor_CurrentSampleRate_Hz (i2c_status_t* error);

    /**
    * @brief This function gets the number of bytes written in the sensor FIFO buffers.
    * @param error Result of the operation
    * @retval uint16_t Number of samples in the FIFO buffer in bytes
    */
    uint16_t GetSensor_FIFOCount(i2c_status_t* error);

    /**
    * @brief This function gets the INT_ENABLE register value.
    * @param error Result of the operation
    * @retval uint8_t Enabled/Disabled sensor interrupts. Use Regbits_INT_ENABLE namespace as bitmask to check enabled interrupts.
    */
    uint8_t GetSensor_InterruptEnable(i2c_status_t* error);

    /**
    * @brief This function sets the sensor INT_ENABLE register with given value.
    * @param enabledInterrupts Enabled/Disabled sensor interrupts. Use Regbits_INT_ENABLE namespace.
    * @retval i2c_status_t
    */
    i2c_status_t SetSensor_InterruptEnable(uint8_t enabledInterrupts);

    /**
    * @brief This function gets the sensor FIFO configuration. Use Regbits_FIFO_EN as bitmask to check which
    *        samples enabled in the FIFO reading.
    * @param error Result of the operation
    * @retval uint8_t Sensor fifo configuration value, use Regbits_FIFO_EN to check fifo config.
    */
    uint8_t GetSensor_FIFO_Config(i2c_status_t* error);

    /**
    * @brief This function sets the sensor FIFO configuration.
    * @param fifoConfigVal FIFO config value, use Regbits_FIFO_EN as bitmask to configure.
    * @retval i2c_status_t
    */
    i2c_status_t SetSensor_FIFO_Config(uint8_t fifoConfigVal);

    /**
    * @brief This function gets the sensor FIFO enable bit in USER_CTRL register.
    * @param error Result of the operation
    * @retval bool True if FIFO enabled
    */
    bool GetSensor_FIFO_Enable(i2c_status_t* error);

    /**
    * @brief This function sets the sensor FIFO enable bit in USER_CTRL register.
    * @param state State of the FIFO to be set. True if it will be enabled.
    * @retval i2c_status_t
    */
    i2c_status_t SetSensor_FIFO_Enable(bool state);

    /**
    * @brief This function resets the sensor FIFO.
    * @param none
    * @retval i2c_status_t
    */
    i2c_status_t Reset_Sensor_FIFO(void);

    /**
    * @brief This function gets the sensor interrput status (INT_STATUS) register.
    * @param error Result of the operation
    * @retval uint8_t Register value.
    */
    uint8_t GetSensor_InterruptStatus(i2c_status_t* error);

    /**
    * @brief This function reads 1 byte from sensor FIFO data register.
    * @param error Result of the operation.
    * @retval uint8_t FIFO data.
    */
    uint8_t GetSensor_FIFO_Data(i2c_status_t* error);

    /**
    * @brief This function returns sensor interrupt pin config register value.
    * @param error Result of the operation.
    * @retval uint8_t Interrupt pin config register value.
    */
    uint8_t GetSensor_InterruptPinConfig(i2c_status_t* error);

    /**
    * @brief This function sets the sensor interrupt pin config register.
    * @param intPinConfig interrput pin config value to set
    * @retval i2c_status_t
    */
    i2c_status_t SetSensor_InterruptPinConfig(uint8_t intPinConfig);


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

    /* Gyro offset register constant to compensate 1 DPS (degree per second) offset.
  * Check sensor datasheet for more info about the offset procedure! */
    const float gyro_offset_1dps = 32.8f;
  };

} // namespace MPU6050_Driver

#endif /* include guard */
