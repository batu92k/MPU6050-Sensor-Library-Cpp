/**
  ******************************************************************************
  * @file    i2c_interface.cpp
  * @author  Ali Batuhan KINDAN
  * @date    28.12.2020
  * @brief   This file constains I2C Interface class implementation.
  ******************************************************************************
  */
#include "i2c_interface.h"

/**
  * @brief  This method will be used for reading a bit value of the given register
  * from the slace device with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be read
  * @param  bitNo Bit number to be read
  * @param  status Pointer for operation status
  * @retval bool Bit value
  */
bool I2C_Interface::ReadRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitNo, i2c_status_t *status)
{
    return (ReadRegister(slaveAddress, regAddress, status) & (0x01 << bitNo));
}

  /**
  * @brief  This method will be used for writing a bit value of the given register
  * from the slace device with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  bitNo Bit number to be set/reset
  * @param  bitVal Bit value to be written
  * @retval i2c_status_t
  */
i2c_status_t I2C_Interface::WriteRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitNo, bool bitVal)
{
    i2c_status_t status = I2C_STATUS_NONE;
    uint8_t data = ReadRegister(slaveAddress, regAddress, &status);
    if(status == I2C_STATUS_SUCCESS)
    {
        status = WriteRegister(slaveAddress, regAddress, (data & ~(0x01 << bitNo)) | ((bitVal ? 0x01 : 0x00) << bitNo));
    }

    return status;
}
