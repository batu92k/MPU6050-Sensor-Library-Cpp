/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  Ali Batuhan KINDAN
  * @date    20.12.2020
  * @brief   This file constains MPU6050 driver declarations.
  ******************************************************************************
  */
 #include <stdint.h>

/* I2C Interface class to make sensor driver work with
 * other MCU architectures by simply overriding this virtual
 * methods according to current architecture I2C driver methods. */
class I2C_Interface
{
public:
  virtual void Init_I2C(uint32_t baudrate) = 0;
  virtual void SetBaudRate(uint32_t baudrate) = 0;
  virtual uint8_t ReadRegister(uint8_t slaveAddress, uint8_t regAddress) = 0;
  virtual void WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data) = 0;
  virtual bool ReadRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitNo) = 0;
  virtual void WriteRegisterBit(uint8_t slaveAddress, uint8_t regAddress, uint8_t bitNo, bool bitVal) = 0;
};
