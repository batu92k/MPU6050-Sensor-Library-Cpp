/**
  ******************************************************************************
  * @file    esp32_i2c_if.cpp
  * @author  Ali Batuhan KINDAN
  * @date    24.01.2022
  * @brief   This file constains the imlementation I2C interface for esp32-wroom microcontroller.
  ******************************************************************************
  */

#include "esp32_i2c_if.h"

  /**
  * @brief  I2C peripheral initialization method.
  * @param  baudrate I2C clock frequency (default 100 kHz)
  * @retval i2c_status_t
  */
  i2c_status_t ESP32_I2C_IF::Init_I2C(uint32_t baudrate) {
      /* TODO: implement */
      return I2C_STATUS_NONE;
  }

  /**
  * @brief  This method will be used for reading the data of the given register from
  * the slave with given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address to be read
  * @param  status Pointer for operation status
  * @retval uint8_t Read register value
  */
  uint8_t ESP32_I2C_IF::ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status) {
      /* TODO: implement */
      return 1;
  }

  /**
  * @brief  This method will be used for writing gven data to the given register of the slave device 
  * with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  data Data to be written
  * @retval i2c_status_t
  */
  i2c_status_t ESP32_I2C_IF::WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data) {
      /* TODO: implement */
      return I2C_STATUS_NONE;
  }

  /**
  * @brief  Class destructor.
  * @param  none
  * @retval none
  */  
  ESP32_I2C_IF::~ESP32_I2C_IF() {
      /* TODO: implement if needed */
  }
