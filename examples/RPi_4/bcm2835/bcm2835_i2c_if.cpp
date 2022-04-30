/**
  ******************************************************************************
  * @file    bcm2835_i2c_if.cpp
  * @author  Ali Batuhan KINDAN
  * @date    02.01.2021
  * @brief   This file constains the imlementation I2C interface for bcm2835 library.
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
#include "bcm2835_i2c_if.h"
#include <bcm2835.h>

/**
  * @brief  I2C peripheral initialization method.
  * @param  clock I2C clock frequency (default 100 kHz)
  * @retval i2c_status_t
  */
i2c_status_t BCM2835_I2C_IF::Init_I2C(i2c_clockspeed_t clock)
{
    /* Initalize library. We have to call this before we call any
     * bcm2835 driver method, otherwise we will get memory related
     * errors! (Segmentation Fault) */
    if(!bcm2835_init()) 
    {
        return I2C_STATUS_ERROR;
    }

    if(!bcm2835_i2c_begin())
    {
        return I2C_STATUS_ERROR;
    }
    
    bcm2835_i2c_set_baudrate(static_cast<uint32_t>(clock));
    bcm2835_gpio_set_pud(RPI_GPIO_P1_03, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(RPI_GPIO_P1_05, BCM2835_GPIO_PUD_UP);
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
uint8_t BCM2835_I2C_IF::ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status)
{
  char buf[2] = {regAddress, 0x00};
  bcm2835_i2c_setSlaveAddress(slaveAddress);
  if(bcm2835_i2c_write((const char *)buf, 1) != BCM2835_I2C_REASON_OK)
  {
    *status = I2C_STATUS_ERROR;
    return 0x00;
  }

  if(bcm2835_i2c_read((buf + 1), 1) != BCM2835_I2C_REASON_OK)
  {
    *status = I2C_STATUS_ERROR;
    return 0x00;
  }

  *status = I2C_STATUS_SUCCESS;
  return buf[1];
}

/**
  * @brief  This method will be used for writing gven data to the given register of the slave device 
  * with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  data Data to be written
  * @retval i2c_status_t
  */
i2c_status_t BCM2835_I2C_IF::WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data)
{
  const char buf[2] = {regAddress, data};
  bcm2835_i2c_setSlaveAddress(slaveAddress);
  return (bcm2835_i2c_write(buf, 2) == BCM2835_I2C_REASON_OK) ? I2C_STATUS_SUCCESS : I2C_STATUS_ERROR;
}

/**
  * @brief  Class destructor.
  * @param  none
  * @retval none
  */
BCM2835_I2C_IF::~BCM2835_I2C_IF()
{
  bcm2835_close(); /* close library and deallocate memory used by the library. */
}
