# Project Makefile 20201219 - Ali Batuhan KINDAN
all: main

main: create_build_folder example.o mpu6050.o
	$(info -> Creating executable)
	@g++ -o build/example build/example.o

create_build_folder:
	$(info -> Creating build folder if not exist)
	@mkdir -p build

example.o: example.cpp
	$(info -> Compiling example.cpp)
	@g++ -c -Wall example.cpp -o build/example.o

mpu6050.o: mpu6050.cpp
	$(info -> Compiling mpu6050.cpp)
	@g++ -c -Wall mpu6050.cpp -o build/mpu6050.o

clean:
	$(info -> Cleaning build files and executable)
	@rm -rf build
