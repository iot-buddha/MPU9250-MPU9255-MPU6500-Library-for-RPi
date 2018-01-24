# MPU9250-MPU9255-MPU6500-Library-for-RPi

## Summary
Motion sensor library for RPi.<br/>
Requires wiringPi, wiringPiI2C

I named the library <MPU9255.h> because I'm using it.<br/>
I'm not certain if my library can support MPU9250 and MPU6500, but i think it could because register addresses are identical.<br/>
In fact, I made this library reading MPU9250's datasheet.

## List of public variables/instances
* MPU9255 Mpu
>Default motion sensor.
* float angles[2]
>Stores measurement result. { x, y }

## List of public methods
* uint8_t MPU9255::initialize(void)
>Initializes motion sensor. You must call this method before using any other methods.
* void MPU9255::initDT(void)
>Call this method just before entering loop
* void MPU9255::calcDT(void)
>Call this method just before the end of loop.
* void MPU9255::getResult(void)
>Call this method to get resulst(float angle[2]) updated.
