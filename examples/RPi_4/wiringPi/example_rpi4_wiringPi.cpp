/**
  ******************************************************************************
  * @file    example_rpi4.cpp
  * @author  Ali Batuhan KINDAN
  * @date    19.12.2020
  * @brief   Example application for Rpi4 to show MPU6050 driver features by using wiringPi library.
  ******************************************************************************
  */
#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::milliseconds
#include <iostream>
#include "mpu6050.h"
#include "wiringPi_i2c_if.h"


int main() 
{
  I2C_Interface* i2c_if = new WIRINGPI_I2C_IF();
  /* initialize fast I2C 400 kHz */
  if(i2c_if->Init_I2C((uint8_t)MPU6050_ADDRESS) != I2C_STATUS_SUCCESS)
  {
    std::cout << "I2C initialization failed!\n";
    delete i2c_if;
    return EXIT_FAILURE;
  }

  MPU6050 sensor(i2c_if);
  if(sensor.ResetSensor() != I2C_STATUS_SUCCESS)
  {
    std::cout << "Sensor reset failed!\n";
    return EXIT_FAILURE;    
  }
  /* Simple safety delay after sensor reset */
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  /* Wakeup sensor and set full scale ranges */
  if(sensor.InitializeSensor(GYRO_SCALE_1000, ACCEL_SCALE_8G) != I2C_STATUS_SUCCESS)
  {
    std::cout << "Sensor initialization failed!\n";
    return EXIT_FAILURE;
  }

  /* Auto-Calibrate gyroscope registers to target value 0 (default) */
  if(sensor.Calibrate_Gyro_Registers() != I2C_STATUS_SUCCESS)
  {
    std::cout << "Gyro calibration failed!\n";
    return EXIT_FAILURE;
  }

  /* Auto-Calibrate accelerometer registers to target values by default: 
   * X = 0 MG, Y = 0 MG, Z = 1 MG */
  if(sensor.Calibrate_Accel_Registers() != I2C_STATUS_SUCCESS)
  {
    std::cout << "Accel calibration failed!\n";
    return EXIT_FAILURE;
  }

  std::cout << "Sensor configuration completed!\n";

  /* Read sensor conversion registers for test! */
  uint8_t totalRes = I2C_STATUS_SUCCESS;
  i2c_status_t currentRes = I2C_STATUS_SUCCESS;
  while(totalRes == I2C_STATUS_SUCCESS)
  {
    std::cout << "Acc_X: " << sensor.GetAccel_X_Raw(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Acc_Y: " << sensor.GetAccel_Y_Raw(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Acc_Z: " << sensor.GetAccel_Z_Raw(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Temp: " << sensor.GetTemperature_Celcius(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Gyro_X: " << sensor.GetGyro_X_Raw(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Gyro_Y: " << sensor.GetGyro_Y_Raw(&currentRes) << " ";
    totalRes |= currentRes;
    std::cout << "Gyro_Z: " << sensor.GetGyro_Z_Raw(&currentRes) << "\n";
    totalRes |= currentRes;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Sensor read error program terminated!\n";

  return EXIT_FAILURE;
}
