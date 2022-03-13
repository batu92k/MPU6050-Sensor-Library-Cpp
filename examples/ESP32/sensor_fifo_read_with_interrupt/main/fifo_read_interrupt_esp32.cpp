/**
  ******************************************************************************
  * @file    fifo_read_interrupt_esp32.cpp
  * @author  Ali Batuhan KINDAN
  * @date    08.03.2022
  * @brief   Example application for ESP32-Wroom to show MPU6050 fifo reading sequence.
  *          IDF version used: ESP-IDF v5.0-dev-1295-gfaf0f61cdb
  ******************************************************************************
  */
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#include "../../../../mpu6050.h"
#include "esp32_i2c_if.h"

/* structure to keep latest sensor data */
struct SensorFrame {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

static xQueueHandle qSensorReadEvent = NULL;
static const gpio_num_t EXTI_Pin = GPIO_NUM_18;

extern "C" void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == GPIO_NUM_18) {
      uint8_t qSignal = 1; // dummy signal value for the queue
      xQueueSendFromISR(qSensorReadEvent, &qSignal, NULL);
    }
}

extern "C" void Sensor_Read_Task(void* arg)
{
  /* create i2c and sensor class instances and configure the sensor */
  I2C_Interface *i2c_if = new ESP32_I2C_IF();
  /* initialize fast I2C 400 kHz */
  if (i2c_if->Init_I2C(i2c_clockspeed_t::CLK_400KHz) != I2C_STATUS_SUCCESS) {
    printf("I2C initialization failed!\n");
    delete i2c_if;
    esp_restart();
  }

  MPU6050_Driver::MPU6050 sensor(i2c_if);
  if(sensor.ResetSensor() != I2C_STATUS_SUCCESS)
  {
    printf("Sensor reset failed!\n");
    esp_restart();
  }
  /* Simple safety delay after sensor reset */
  vTaskDelay(100 / portTICK_RATE_MS);
  
  /* Wakeup sensor and set full scale ranges */
  if(sensor.InitializeSensor(MPU6050_Driver::Gyro_FS_t::FS_1000_DPS, MPU6050_Driver::Accel_FS_t::FS_8G) != I2C_STATUS_SUCCESS)
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

  /* set digital low pass to BW_184Hz (sample rate reduced to 1KHz)
   * (just to show the feature it already has default value in startup) */
  if(sensor.SetSensor_DLPF_Config(MPU6050_Driver::DLPF_t::BW_184Hz) != I2C_STATUS_SUCCESS) {
    printf("DLPF configuration failed!\n");
    esp_restart();  
  }

  /* set sample rate divider for 20 Hz sample rate.
   * 1000 / (1 + 49) = 20 Hz */
  if(sensor.SetGyro_SampleRateDivider(49) != I2C_STATUS_SUCCESS) {
    printf("Sample rate config failed!\n");
    esp_restart();  
  }

  /* set sensor interrput coinfig to default value! */
  if(sensor.SetSensor_InterruptPinConfig(0x00) != I2C_STATUS_SUCCESS) {
    printf("Interrupt pin configuration failed!\n");
    esp_restart();
  }

  /* Enable sensor data ready interrupt. */
  uint8_t interruptConfigVal = MPU6050_Driver::Regbits_INT_ENABLE::BIT_DATA_RDY_EN;
  if(sensor.SetSensor_InterruptEnable(interruptConfigVal) != I2C_STATUS_SUCCESS) {
    printf("Interrput enable failed!\n");
    esp_restart();
  }

  /* Make sure sensor fifo is disabledbefore reseting the buffer. */
  if(sensor.SetSensor_FIFO_Enable(false) != I2C_STATUS_SUCCESS) {
    printf("Sensor fifo enable failed!\n");
    esp_restart();
  }

  /* Reset sensro fifo buffer. */
  if(sensor.Reset_Sensor_FIFO() != I2C_STATUS_SUCCESS) {
    printf("Sensor fifo reset failed!\n");
    esp_restart();
  }

  /* Enable sensor fifo. */
  if(sensor.SetSensor_FIFO_Enable(true) != I2C_STATUS_SUCCESS) {
    printf("Sensor fifo enable failed!\n");
    esp_restart();
  }

  /* configure sensor fifo to be able to read accelerometer x, y, z and
   * groscope x, y ,z values. */
  uint8_t fifoConfigVal = 
  MPU6050_Driver::Regbits_FIFO_EN::BIT_ACCEL_FIFO_EN |
  MPU6050_Driver::Regbits_FIFO_EN::BIT_XG_FIFO_EN |
  MPU6050_Driver::Regbits_FIFO_EN::BIT_YG_FIFO_EN |
  MPU6050_Driver::Regbits_FIFO_EN::BIT_ZG_FIFO_EN;
  if(sensor.SetSensor_FIFO_Config(fifoConfigVal) != I2C_STATUS_SUCCESS) {
    printf("Sensor fifo congfiguration failed!\n");
    esp_restart();
  }

  i2c_status_t error = I2C_STATUS_NONE;
  float currentSampleRateHz = sensor.GetSensor_CurrentSampleRate_Hz(&error);
  if(error != I2C_STATUS_SUCCESS) {
    printf("Sample rate reading failed!\n");
    esp_restart();  
  }

  printf("Sensor sample rate: %.2f Hz \n", currentSampleRateHz);
  printf("Sensor configuration completed!\n");

  uint8_t intStatus = 0;
  uint16_t fifoCount = 0;
  uint8_t fifoData[12]; // axH, axL, ayH, ayL, azH, azL, gxH, gxL, gyH, gyL, gzH, gzL
  SensorFrame currentFrame; // structure to keep current frame in constructed int16_t form
  constexpr uint16_t FIFO_FRAME_LEN = 12;

  uint8_t qData;
  while (error == I2C_STATUS_SUCCESS)
  {
    /* read sensor fifo on every data ready interrupt */
    if (xQueueReceive(qSensorReadEvent, &qData, portMAX_DELAY) == pdPASS)
    {
      intStatus = sensor.GetSensor_InterruptStatus(&error);

      if(error != I2C_STATUS_SUCCESS) {
        printf("Interrupt status read failed!\n");
        break;
      }

      /* Check if there is any overflow. If yes then abort! */
      if(intStatus & MPU6050_Driver::Regbits_INT_ENABLE::BIT_FIFO_OFLOW_EN) {
        printf("FIFO overflow detected!\n");
        break;
      }

      fifoCount = sensor.GetSensor_FIFOCount(&error);

      if(error != I2C_STATUS_SUCCESS) {
        printf("FIFO count read failed!\n");
        break;
      }

      /* If sensor fifo count is smaller than our frame size or if the data ready bit is not set,
       * do not execute the further sequence where we read and process FIFO data. */
      if(fifoCount < FIFO_FRAME_LEN || !(intStatus & MPU6050_Driver::Regbits_INT_ENABLE::BIT_DATA_RDY_EN)) {
        continue;
      }

      /* Read sensor frame amount of bytes (12 bytes) from FIFO buffer. */
      for(uint8_t i = 0; i < FIFO_FRAME_LEN && (error == I2C_STATUS_SUCCESS); i++) {
        fifoData[i] = sensor.GetSensor_FIFO_Data(&error);
      }

      if(error != I2C_STATUS_SUCCESS) {
        printf("FIFO data read failed!\n");
        break;
      }

      /* Set sensor frame structure values from latest sensor FIFO data. */
      currentFrame.ax = (int16_t)fifoData[0] << 8 | (int16_t)fifoData[1];
      currentFrame.ay = (int16_t)fifoData[2] << 8 | (int16_t)fifoData[3];
      currentFrame.az = (int16_t)fifoData[4] << 8 | (int16_t)fifoData[5];
      currentFrame.gx = (int16_t)fifoData[6] << 8 | (int16_t)fifoData[7];
      currentFrame.gy = (int16_t)fifoData[8] << 8 | (int16_t)fifoData[9];
      currentFrame.gz = (int16_t)fifoData[10] << 8 | (int16_t)fifoData[11];

      /* print the constructed sensor FIFO data! */
      printf("aX: %d aY: %d aZ: %d gX: %d gY: %d gZ: %d\n",
      currentFrame.ax,
      currentFrame.ay,
      currentFrame.az,
      currentFrame.gx,
      currentFrame.gy,
      currentFrame.gz);
    }
  }

  esp_restart(); 
}

static void Configure_GPIO_Interrupt(void)
{
    gpio_config_t io_conf = {};
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pin GPIO18
    io_conf.pin_bit_mask = (uint64_t) (1UL << EXTI_Pin);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(EXTI_Pin, gpio_isr_handler, (void*) EXTI_Pin);    
}

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

  // create a queue to handle gpio event from isr
  qSensorReadEvent = xQueueCreate(10, sizeof(uint8_t));
  Configure_GPIO_Interrupt();
  //start gpio task
  xTaskCreate(Sensor_Read_Task, "Sensor_Read_Task", 2048, NULL, 10, NULL);
}
