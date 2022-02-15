/**
  ******************************************************************************
  * @file    i2c_interface.h
  * @author  Ali Batuhan KINDAN
  * @date    28.12.2020
  * @brief   This file constains I2C Interface class definition.
  ******************************************************************************
  */
#include <stdint.h>

#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

/* I2C status types. 
 * TODO: will be extended for different I2C error types! Keep it as enum for now. */
enum i2c_status_t 
{
  I2C_STATUS_SUCCESS = 0x00,
  I2C_STATUS_ERROR = 0x01,
  I2C_STATUS_NONE = 0x02
};

/* I2C clock speed types */
enum class i2c_clockspeed_t
{
  CLK_NONE = 0,
  CLK_100KHz = 100000,
  CLK_200KHz = 200000,
  CLK_400KHz = 400000
};

/* I2C Interface class to make sensor driver work with
 * other MCU architectures by simply overriding this virtual
 * methods according to current architecture I2C driver methods. */
class I2C_Interface
{
public:
  /**
  * @brief  Class constructor.
  * @param  none
  * @retval none
  */
  I2C_Interface() {/* empty constructor */};

  /**
  * @brief  Class destructor
  * @param  none
  * @retval none
  */
  virtual ~I2C_Interface() {/* empty virtual destructor */};

  /* TODO: Init_I2C methods should be differenciated by compiler without the need of casting! */

  /**
  * @brief  I2C peripheral initialization method.
  * @param  clock I2C clock frequency (default 100 kHz)
  * @retval i2c_status_t
  */
  virtual i2c_status_t Init_I2C(i2c_clockspeed_t clock = i2c_clockspeed_t::CLK_100KHz) {return I2C_STATUS_NONE;};

  /**
  * @brief  I2C peripheral initialization method.
  * @param  slaveAddress adress of the device that will be communicated
  * @retval i2c_status_t
  */
  virtual i2c_status_t Init_I2C(uint8_t slaveAddress) {return I2C_STATUS_NONE;};

  /**
  * @brief  This method will be used for reading the data of the given register from
  * the slave with given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address to be read
  * @param  status Pointer for operation status
  * @retval uint8_t Read register value
  */
  virtual uint8_t ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status) = 0;

  /**
  * @brief  This method will be used for writing gven data to the given register of the slave device 
  * with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  data Data to be written
  * @retval i2c_status_t
  */
  virtual i2c_status_t WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data) = 0;

  /**
  * @brief  This method will be used for reading a bit value of the given register
  * from the slace device with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be read
  * @param  bitMask Bit mask to be read
  * @param  status Pointer for operation status
  * @retval bool Bit value
  */
  bool ReadRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitMask, i2c_status_t *status);

  /**
  * @brief  This method will be used for writing a bit value of the given register
  * from the slace device with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  bitMask Bit mask to be set/reset
  * @param  bitVal Bit value to be written
  * @retval i2c_status_t
  */
  i2c_status_t WriteRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitMask, bool bitVal);
};

#endif /* include guard */
