/**
  ******************************************************************************
  * @file    esp32_i2c_if.cpp
  * @author  Ali Batuhan KINDAN
  * @date    24.01.2022
  * @brief   This file constains the imlementation I2C interface for esp32-wroom microcontroller.
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

#include "esp32_i2c_if.h"
#include "driver/i2c.h"

  /**
  * @brief  I2C peripheral initialization method.
  * @param  clock I2C clock frequency (default 100 kHz)
  * @retval i2c_status_t
  */
  i2c_status_t ESP32_I2C_IF::Init_I2C(i2c_clockspeed_t clock) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.scl_io_num = 22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = static_cast<uint32_t>(clock);
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_Master_Port, &conf);

    return i2c_driver_install(I2C_Master_Port, conf.mode, 0 /* rxBuf */, 0 /* txBuf */, 0) == ESP_OK ? I2C_STATUS_SUCCESS : I2C_STATUS_ERROR;
  }

  /**
  * @brief  This method will be used for reading the data of the given register from
  * the slave with given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address to be read
  * @param  status Pointer for operation status
  * @retval uint8_t Read register value
  */
  uint8_t ESP32_I2C_IF::ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status) {
    uint8_t data = 0;
    esp_err_t error = i2c_master_write_read_device(I2C_Master_Port, slaveAddress, &regAddress, 1, &data, 1, I2C_Timeout_ms / portTICK_RATE_MS);
    *status = error == ESP_OK ? I2C_STATUS_SUCCESS : I2C_STATUS_ERROR;
    return data;
  }

  /**
  * @brief  This method will be used for writing gven data to the given register of the slave device 
  * with the given address.
  * @param  slaveAddress Slave chip I2C bus address
  * @param  regAddress Register address that the data to be written
  * @param  data Data to be written
  * @retval i2c_status_t
  */
  i2c_status_t ESP32_I2C_IF::WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data) {
    uint8_t write_buf[2] = {regAddress, data};
    esp_err_t error = i2c_master_write_to_device(I2C_Master_Port, slaveAddress, write_buf, sizeof(write_buf), I2C_Timeout_ms / portTICK_RATE_MS);

    /* TODO: implement */
    return error == ESP_OK ? I2C_STATUS_SUCCESS : I2C_STATUS_ERROR;
  }

  /**
  * @brief  Class destructor.
  * @param  none
  * @retval none
  */  
  ESP32_I2C_IF::~ESP32_I2C_IF() {
      i2c_driver_delete(I2C_Master_Port);
  }
