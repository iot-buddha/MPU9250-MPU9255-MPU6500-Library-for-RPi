all: MPU9255_Main.cpp MPU9255.cpp tb_core.cpp
	g++ -g -o app MPU9255_Main.cpp MPU9255.cpp tb_core.cpp -I . -lwiringPi
