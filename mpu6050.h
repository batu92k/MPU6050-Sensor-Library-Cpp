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

enum pwr_mgmt_1_bits_t 
{
  BIT_CLKSEL_0 = 0,
  BIT_CLKSEL_1 = 1,
  BIT_CLKSEL_2 = 2,
  BIT_TEMP_DIS = 3,
  BIT_CYCLE = 5,
  BIT_SLEEP = 6,
  BIT_DEVICE_RESET = 7
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

private:
  I2C_Interface* i2c = nullptr;

};

#endif /* include guard */
