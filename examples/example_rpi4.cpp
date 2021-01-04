/**
  ******************************************************************************
  * @file    example_rpi4.cpp
  * @author  Ali Batuhan KINDAN
  * @date    19.12.2020
  * @brief   Example application for Rpi4 to show MPU6050 driver features.
  ******************************************************************************
  */
#include <iostream>
#include "../mpu6050.h"
#include "bcm2835_i2c_if.h"

int main() 
{
  I2C_Interface* i2c_if = new BCM2835_I2C_IF();
  /* initialize fast I2C 400 kHz */
  if(i2c_if->Init_I2C(400000) != I2C_STATUS_SUCCESS)
  {
    std::cout << "I2C initialization failed!\n";
    delete i2c_if;
    return EXIT_FAILURE;
  }
  MPU6050 sensor(i2c_if);
  /* wakeup sensor from sleep mode */
  if(sensor.WakeUpSensor() != I2C_STATUS_SUCCESS)
  {
    std::cout << "Sensor wakeup failed!\n";
    return EXIT_FAILURE;
  }

  /* configure gyroscope full scale range as 500 degree per second */
  if(sensor.SetGyroFullScale(GYRO_SCALE_500) != I2C_STATUS_SUCCESS)
  {
    std::cout << "Gyroscope configuration failed!\n";
    return EXIT_FAILURE;
  }

  std::cout << "Sensor awake!\n";
  return EXIT_SUCCESS;
}
