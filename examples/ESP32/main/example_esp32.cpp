/**
  ******************************************************************************
  * @file    example_esp32.cpp
  * @author  Ali Batuhan KINDAN
  * @date    24.01.2022
  * @brief   Example application for ESP32-Wroom to show MPU6050 driver features.
  ******************************************************************************
  */
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "../../../mpu6050.h"
#include "esp32_i2c_if.h"

extern "C" void app_main(void)
{
  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
         CONFIG_IDF_TARGET,
         chip_info.cores,
         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
  printf("silicon revision %d, ", chip_info.revision);
  printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
  printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

  I2C_Interface *i2c_if = new ESP32_I2C_IF();
  /* initialize fast I2C 400 kHz */
  if (i2c_if->Init_I2C((uint32_t)400000) != I2C_STATUS_SUCCESS) {
    printf("I2C initialization failed!\n");
    delete i2c_if;
    esp_restart();
  }

  MPU6050 sensor(i2c_if);
  if(sensor.ResetSensor() != I2C_STATUS_SUCCESS)
  {
    printf("Sensor reset failed!\n");
    esp_restart();
  }
  /* Simple safety delay after sensor reset */
  vTaskDelay(100 / portTICK_RATE_MS);
  
  /* Wakeup sensor and set full scale ranges */
  if(sensor.InitializeSensor(GYRO_SCALE_1000, ACCEL_SCALE_8G) != I2C_STATUS_SUCCESS)
  {
    printf("Sensor initialization failed!\n");
    esp_restart();
  }

  /* Auto-Calibrate gyroscope registers to target value 0 (default) */
  if(sensor.Calibrate_Gyro_Registers() != I2C_STATUS_SUCCESS)
  {
    printf("Gyro calibration failed!\n");
    esp_restart();
  }

  /* Auto-Calibrate accelerometer registers to target values by default: 
   * X = 0 MG, Y = 0 MG, Z = 1 MG */
  if(sensor.Calibrate_Accel_Registers() != I2C_STATUS_SUCCESS)
  {
    printf("Accel calibration failed!\n");
    esp_restart();
  }

  printf("Sensor configuration completed!\n");

  /* Read sensor conversion registers for test! */
  uint8_t totalRes = I2C_STATUS_SUCCESS;
  i2c_status_t currentRes = I2C_STATUS_SUCCESS;
  while(totalRes == I2C_STATUS_SUCCESS)
  {
    printf("Acc_X: %d ", sensor.GetAccel_X_Raw(&currentRes));
    totalRes |= currentRes;
    printf("Acc_Y: %d ", sensor.GetAccel_Y_Raw(&currentRes));
    totalRes |= currentRes;
    printf("Acc_Z: %d ", sensor.GetAccel_Z_Raw(&currentRes));
    totalRes |= currentRes;
    printf("Temp: %.2f ", sensor.GetTemperature_Celcius(&currentRes));
    totalRes |= currentRes;
    printf("Gyro_X: %d ", sensor.GetGyro_X_Raw(&currentRes));
    totalRes |= currentRes;
    printf("Gyro_Y: %d ", sensor.GetGyro_Y_Raw(&currentRes));
    totalRes |= currentRes;
    printf("Gyro_Z: %d \n", sensor.GetGyro_Z_Raw(&currentRes));
    totalRes |= currentRes;
    vTaskDelay(50 / portTICK_RATE_MS);
  }

  printf("Sensor read error! Program terminated!\n");
  esp_restart();  
}
