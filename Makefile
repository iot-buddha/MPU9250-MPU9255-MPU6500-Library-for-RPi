all: MPU9255_Main.cpp MPU9255.cpp
	g++ -o app MPU9255_Main.cpp MPU9255.cpp -I . -lwiringPi