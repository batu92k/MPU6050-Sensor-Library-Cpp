/**
  ******************************************************************************
  * @file    bcm2835_i2c_if.cpp
  * @author  Ali Batuhan KINDAN
  * @date    02.01.2021
  * @brief   This file constains the imlementation I2C interface for bcm2835 library.
  ******************************************************************************
  */
#include "bcm2835_i2c_if.h"
#include <bcm2835.h>

/**
  * @brief  I2C peripheral initialization method.
  * @param  baudrate I2C clock frequency (default 100 kHz)
  * @retval i2c_status_t
  */
i2c_status_t BCM2835_I2C_IF::Init_I2C(uint32_t baudrate = 100000)
{
    if(!bcm2835_i2c_begin())
    {
        return I2C_STATUS_ERROR;
    }
    bcm2835_i2c_set_baudrate(baudrate);
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
