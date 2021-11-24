/**
  ******************************************************************************
  * @file    wiringPi_i2c_if.h
  * @author  Ali Batuhan KINDAN
  * @date    24.11.2021
  * @brief   This file constains the definition I2C interface for wiringPi library.
  ******************************************************************************
  */

#include "i2c_interface.h"

#ifndef WIRINGPI_I2C_IF_H
#define WIRINGPI_I2C_IF_H

/* I2C interface for WiringPi library. */
class WIRINGPI_I2C_IF : public I2C_Interface
{
public:
  /**
  * @brief  I2C peripheral initialization method.
  * @param  slaveAddress adress of the device that will be communicated
  * @retval i2c_status_t
  */
  i2c_status_t Init_I2C(uint8_t slaveAddress) override;

  /**
  * @brief  This method will be used for reading the data of the given register from
  * the slave with given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address to be read
  * @param  status Pointer for operation status
  * @retval uint8_t Read register value
  */
  uint8_t ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status) override;

  /**
  * @brief  This method will be used for writing gven data to the given register of the slave device 
  * with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  data Data to be written
  * @retval i2c_status_t
  */
  i2c_status_t WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data) override;

  /**
  * @brief  Class destructor.
  * @param  none
  * @retval none
  */  
  ~WIRINGPI_I2C_IF() override;

private:
  int i2cInterfaceNum = -1;

};

#endif
