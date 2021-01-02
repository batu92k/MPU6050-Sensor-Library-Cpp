# Project Makefile 20201219 - Ali Batuhan KINDAN
all: main

main: create_build_folder i2c_interface.o mpu6050.o bcm2835_i2c_if.o example_rpi4.o
	$(info -> Creating executable)
	@g++ -o build/example_rpi4 build/example_rpi4.o

create_build_folder:
	$(info -> Creating build folder if not exist)
	@mkdir -p build

example_rpi4.o: example_rpi4.cpp
	$(info -> Compiling example.cpp)
	@g++ -c -Wall example_rpi4.cpp -o build/example_rpi4.o

bcm2835_i2c_if.o: bcm2835_i2c_if.cpp
	$(info -> Compiling bcm2835_i2c_if.cpp)
	@g++ -c -Wall bcm2835_i2c_if.cpp -o build/bcm2835_i2c_if.o

i2c_interface.o: ../i2c_interface.cpp
	$(info -> Compiling i2c_interface.cpp)
	@g++ -c -Wall ../i2c_interface.cpp -o build/i2c_interface.o

mpu6050.o: ../mpu6050.cpp
	$(info -> Compiling mpu6050.cpp)
	@g++ -c -Wall ../mpu6050.cpp -o build/mpu6050.o

clean:
	$(info -> Cleaning build files and executable)
	@rm -rf build
