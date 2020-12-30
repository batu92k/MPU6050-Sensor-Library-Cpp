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
