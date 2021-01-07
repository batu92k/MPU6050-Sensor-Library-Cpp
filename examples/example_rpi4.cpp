/**
  ******************************************************************************
  * @file    example_rpi4.cpp
  * @author  Ali Batuhan KINDAN
  * @date    19.12.2020
  * @brief   Example application for Rpi4 to show MPU6050 driver features.
  ******************************************************************************
  */
#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::milliseconds
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

  /* configure accelerometer full scale range as +-8G */
  if(sensor.SetAccelFullScale(ACCEL_SCALE_8G) != I2C_STATUS_SUCCESS)
  {
    std::cout << "Accelerometer configuration failed!\n";
    return EXIT_FAILURE;
  }

  std::cout << "Sensor configuration completed!\n";

  /* Read sensor conversion registers for test! */
  uint8_t totalRes = I2C_STATUS_SUCCESS;
  i2c_status_t currentRes = I2C_STATUS_SUCCESS;
  while(totalRes == I2C_STATUS_SUCCESS)
  {
    std::cout << "Acc_X: " << sensor.GetAccel_X(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Acc_Y: " << sensor.GetAccel_Y(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Acc_Z: " << sensor.GetAccel_Z(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Temp: " << sensor.GetTemperature(&currentRes) << "\n";
    totalRes |= currentRes;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return EXIT_SUCCESS;
}
