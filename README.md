# **MPU6050 Sensor Library Cpp**

## **About the Library**

This library is made to create a simple interface in C++ for the MPU6050 sensor over I2C. The library initially made for Raspberry Pi 4 but it allows users to port the library to other architectures easily. Additionally, the library currently contains example applications made for RaspberryPi 4 and ESP32 Wroom boards.

## **How to Use**
Since the I2C related methods (such as initialization, read, write, etc.) may differ for every architecture, the library is using an interface class called ```I2C_Interface``` (see i2c_interface.h source file) in order to keep the library flexible to port to other architectures. The user should create its own class from the I2C_Interface base class and implement only 3 methods to make it work for its architecture. Methods to be implemented by the user are the following:
 * ```virtual i2c_status_t Init_I2C(uint32_t baudrate = 100000)```
 * ```virtual uint8_t ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status)```
 * ```virtual i2c_status_t WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data)```

## **Requirements**
### **For RaspberryPi 4 Examples**
 * bcm2835 library (For RaspberryPi 4 example application. Information about install can bu found here https://www.airspayce.com/mikem/bcm2835/)
 * make
 * gcc/g++

### **For ESP32 Examples**
 * ESP-IDF (applications made with *ESP-IDF v5.0-dev-1295-gfaf0f61cdb*)


## **About the Example Applications**
### **1) Raspberry Pi 4**
There is currently 2 example applications for RPi4. Both example applications doing the same basic reading from the sensor to show the features of the driver and how to port the driver to other libraries if needed. One of the example is using *bcm2835 library* and other one is using the *wiringPi library* to access the I2C port.

#### **Connection**
 * SDA -> GPIO2
 * SCL -> GPIO3
 * VCC -> 3V3 Power
 * ADD -> GND
 * GND -> GND :)

#### Important: Pin connections are same for both applications! Check this link for RaspberryPi pinout: https://www.raspberrypi.org/documentation/usage/gpio/

#### **Build and Run**

In order to build the example application, make sure that the requirements are fulfilled (check requirements above) and move into one of the examples folder by using terminal then type ```make all```. If everything is ok, it will create the executable inside the example applications ```/build``` folder.

After the build process you can run the application by moving into ```/build``` folder with terminal and type ```sudo ./example_rpi4_bcm2835``` or ```sudo ./example_rpi4_wiringPi```.

**Important:** You should use sudo (super user permissions) keyword to run the *bcm2835 library* example application!

### **2) ESP32 - Wroom**

#### **Connection**
 * SDA -> IO21
 * SCL -> IO22
 * VCC -> 3V3 Power
 * ADD -> GND
 * GND -> GND :)

#### **Build and Run**
In order to build and run the ESP32 application, just use the ESP-IDF toolchain.

Build the application with: 
 * ```idf.py build``` 
 
After the build flash the application to ESP32 with:
 * ```idf.py -p <portNum> -b 115200 flash```

Once the application is running on the MCU, use following command to monitor the sensor readings:
 * ```idf.py -p <portNum> monitor```