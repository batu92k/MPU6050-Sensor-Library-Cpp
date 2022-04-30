/**
  ******************************************************************************
  * @file    i2c_interface.cpp
  * @author  Ali Batuhan KINDAN
  * @date    28.12.2020
  * @brief   This file constains I2C Interface class implementation.
  *
  * MIT License
  *
  * Copyright (c) 2022 Ali Batuhan KINDAN
  * 
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  * 
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  * SOFTWARE.
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
