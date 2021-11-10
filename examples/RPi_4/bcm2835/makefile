# Project Makefile 20201219 - Ali Batuhan KINDAN
DRIVER_DIR = ../../../
OBJ = $(wildcard build/*.o)

all: main

main: create_build_folder compile_all
#	$(info -> Creating executable)
#	@g++ -o build/example_rpi4 build/example_rpi4.o build/bcm2835_i2c_if.o build/mpu6050.o build/i2c_interface.o -l bcm2835 -I $(DRIVER_DIR)

create_build_folder:
	$(info -> Creating build folder if not exist)
	@mkdir -p build

compile_all: example_rpi4.o bcm2835_i2c_if.o mpu6050.o i2c_interface.o
	$(info -> Creating executable)
	@g++ -o build/example_rpi4 $(OBJ) -l bcm2835 -I $(DRIVER_DIR)

example_rpi4.o: example_rpi4.cpp
	$(info -> Compiling example.cpp)
	@g++ -c -Wall $^ -o build/$@ -I $(DRIVER_DIR)

bcm2835_i2c_if.o: bcm2835_i2c_if.cpp
	$(info -> Compiling bcm2835_i2c_if.cpp)
	@g++ -c -Wall $^ -l bcm2835 -o build/$@ -I $(DRIVER_DIR)

i2c_interface.o: $(DRIVER_DIR)i2c_interface.cpp
	$(info -> Compiling i2c_interface.cpp)
	@g++ -c -Wall $^ -o build/$@ -I $(DRIVER_DIR)

mpu6050.o: $(DRIVER_DIR)mpu6050.cpp
	$(info -> Compiling mpu6050.cpp)
	@g++ -c -Wall $^ -o build/$@ -I $(DRIVER_DIR)

clean:
	$(info -> Cleaning build files and executable)
	@rm -rf build
