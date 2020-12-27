/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  Ali Batuhan KINDAN
  * @date    20.12.2020
  * @brief   This file constains MPU6050 driver declarations.
  ******************************************************************************
  */
#include <stdint.h>

/* I2C status types. 
 * TODO: will be extended for different I2C error types! */
enum i2c_status_t {
  I2C_STATUS_SUCCESS = 0x00,
  I2C_STATUS_ERROR = 0x01,
  I2C_STATUS_NONE = 0x02
};

/* I2C Interface class to make sensor driver work with
 * other MCU architectures by simply overriding this virtual
 * methods according to current architecture I2C driver methods. */
class I2C_Interface
{
public:
  virtual i2c_status_t Init_I2C(uint32_t baudrate) = 0;
  virtual void SetBaudRate(uint32_t baudrate) = 0;
  virtual uint8_t ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t* status) = 0;
  virtual i2c_status_t WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data) = 0;
  virtual bool ReadRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitNo, i2c_status_t* status) = 0;
  virtual i2c_status_t WriteRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitNo, bool bitVal) = 0;
};
