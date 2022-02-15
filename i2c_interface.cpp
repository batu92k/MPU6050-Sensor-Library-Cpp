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
  * @param  bitMask Bit mask to be read
  * @param  status Pointer for operation status
  * @retval bool Bit value
  */
bool I2C_Interface::ReadRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitMask, i2c_status_t *status)
{
    return (ReadRegister(slaveAddress, regAddress, status) & bitMask);
}

  /**
  * @brief  This method will be used for writing a bit value of the given register
  * from the slace device with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  bitNo Bit number to be set/reset
  * @param  bitMask Bit mask to be set/reset
  * @retval i2c_status_t
  */
i2c_status_t I2C_Interface::WriteRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitMask, bool bitVal)
{
    i2c_status_t status = I2C_STATUS_NONE;
    uint8_t data = ReadRegister(slaveAddress, regAddress, &status);
    if(status == I2C_STATUS_SUCCESS)
    {
        status = WriteRegister(slaveAddress, regAddress, (data & ~bitMask) | (bitVal ? bitMask : 0x00));
    }

    return status;
}
