# **MPU6050 Sensor Library Cpp**

## **About the Library**

This library is made to create a simple interface in C++ for the MPU6050 sensor over I2C. The library initially made for Raspberry Pi 4 but it allows users to port the library to other architectures easily. Additionally, the library currently includes an example application the is made for RaspberryPi 4.

## **How to Use**
Since the I2C related methods (such as initialization, read, write, etc.) may differ for every architecture, the library is using an interface class called ```I2C_Interface``` (see i2c_interface.h source file) in order to keep the library flexible to port to other architectures. The user should create its own class from the I2C_Interface base class and implement only 3 methods to make it work for its architecture. Methods to be implemented by the user are the following:
 * ```virtual i2c_status_t Init_I2C(uint32_t baudrate = 100000)```
 * ```virtual uint8_t ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status)```
 * ```virtual i2c_status_t WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data)```

## **Requirements**
 * bcm2835 library (For RaspberryPi 4 example application. Information about install can bu found here https://www.airspayce.com/mikem/bcm2835/)
 * make

## **About the Example Application**
### **1) Source Files**
The library has an example application that made for RaspberryPi 4. The exampe application can be found under the ```examples``` folder. Here is a brief information about source files:
 * ```example_rpi4.cpp``` : This file is the source file that performs library features by using bcm2835 GPIO library.
 * ```bcm2835_i2c_if.cpp``` : This file contains the ```BCM2835_I2C_IF``` class that inherited from the MPU6050 Sensor Library's ```I2C_Interface``` class in order to interface bcm2835 I2C communication functionality to the example application.

### **2) Sensor Connection to RaspberryPi 4**
 * SDA -> GPIO2
 * SCL -> GPIO3
 * VCC -> 3V3 Power
 * ADD -> GND
 * GND -> GND :)

Check this link for RaspberryPi pinout: https://www.raspberrypi.org/documentation/usage/gpio/

### **3) Build and Run**

In order to build the example application, make sure that the requirements are fullfilled (check requirements above) and move into the examples folder by using terminal then type ```make all```. If everything is ok, it will create the executable inside the ```examples/build``` folder.

After the build process you can run the application by moving into ```examples/build``` folder with terminal and type ```sudo ./example_rpi4```.

**Important:** You should use sudo (super user permissions) keyword to run the application!




