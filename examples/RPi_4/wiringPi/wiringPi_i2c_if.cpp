/**
  ******************************************************************************
  * @file    wiringPi_i2c_if.cpp
  * @author  Ali Batuhan KINDAN
  * @date    24.11.2021
  * @brief   This file constains the imlementation I2C interface for wiringPi library.
  ******************************************************************************
  */
#include "wiringPi_i2c_if.h"
#include <wiringPiI2C.h>

  /**
  * @brief  I2C peripheral initialization method.
  * @param  slaveAddress adress of the device that will be communicated
  * @retval i2c_status_t
  */
i2c_status_t WIRINGPI_I2C_IF::Init_I2C(uint8_t slaveAddress)
{
  i2cInterfaceNum = wiringPiI2CSetup(slaveAddress);
  if (i2cInterfaceNum == -1) {
    return I2C_STATUS_ERROR;
  }

  return I2C_STATUS_SUCCESS;
}

/**
  * @brief  This method will be used for reading the data of the given register from
  * the slave with given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address to be read
  * @param  status Pointer for operation status
  * @retval uint8_t Read register value
  */
uint8_t WIRINGPI_I2C_IF::ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status)
{
  int result = wiringPiI2CReadReg8(i2cInterfaceNum, regAddress);

  if(result == -1) {
    *status = I2C_STATUS_ERROR;
  }

  *status = I2C_STATUS_SUCCESS;
  return result;
}

/**
  * @brief  This method will be used for writing gven data to the given register of the slave device 
  * with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  data Data to be written
  * @retval i2c_status_t
  */
i2c_status_t WIRINGPI_I2C_IF::WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data)
{
  return wiringPiI2CWriteReg8(i2cInterfaceNum, regAddress, data) == -1 ? I2C_STATUS_ERROR : I2C_STATUS_SUCCESS;
}

/**
  * @brief  Class destructor.
  * @param  none
  * @retval none
  */
WIRINGPI_I2C_IF::~WIRINGPI_I2C_IF()
{
  /* empty destructor */
}
