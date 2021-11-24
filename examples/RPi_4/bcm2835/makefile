# Project Makefile 20201219 - Ali Batuhan KINDAN
#TODO: more automation!

DRIVER_DIR = ../../../
OBJS = $(wildcard build/*.o)
# LFLAGS included libraries
LFLAGS = -lbcm2835
# IFLAGS included folders for header files
IFLAGS = -I $(DRIVER_DIR)
# WFLAGS warning definitions
WFLAGS = -Wall
# CPPSTD compiler standard
CPPSTD = -std=c++11
# CC Compiler
CC= g++
# EXE Executable name
EXE = example_rpi4

all: main

main: create_build_folder compile_all

create_build_folder:
	$(info -> Creating build folder if not exist)
	@mkdir -p build

compile_all: example_rpi4.o bcm2835_i2c_if.o mpu6050.o i2c_interface.o
	$(info -> Creating executable)
	@$(CC) -o build/$(EXE) $(CPPSTD) $(OBJS) $(LFLAGS) $(IFLAGS)

example_rpi4.o: example_rpi4.cpp
	$(info -> Compiling example.cpp)
	@$(CC) -c $(WFLAGS) $^ -o build/$@ $(CPPSTD) $(LFLAGS) $(IFLAGS)

bcm2835_i2c_if.o: bcm2835_i2c_if.cpp
	$(info -> Compiling bcm2835_i2c_if.cpp)
	@$(CC) -c $(WFLAGS) $^ -o build/$@ $(CPPSTD) $(LFLAGS) $(IFLAGS)

i2c_interface.o: $(DRIVER_DIR)i2c_interface.cpp
	$(info -> Compiling i2c_interface.cpp)
	@$(CC) -c $(WFLAGS) $^ -o build/$@ $(CPPSTD) $(LFLAGS) $(IFLAGS)

mpu6050.o: $(DRIVER_DIR)mpu6050.cpp
	$(info -> Compiling mpu6050.cpp)
	@$(CC) -c $(WFLAGS) $^ -o build/$@ $(CPPSTD) $(LFLAGS) $(IFLAGS)

clean:
	$(info -> Cleaning build files and executable)
	@rm -rf build
