#include <iostream>
#include "MPU9255.h"

using namespace std;

int main(void) {
	Mpu.initialize();
	Mpu.setAlpha(0.98);  // Default is 0.96
	
	Mpu.initDT();  // Place this before entering loop
	while (1) {
		float acc_measurement[6] = {0.0};
		float gyr_measurement[6] = {0.0};
		float filtered_angles[3] = {0.0};

		Mpu.getResult(acc_measurement,6,gyr_measurement,6,filtered_angles,3);

		cout<<"Accelerometer raw x:"<<acc_measurement[0]<<endl;
		cout<<"Accelerometer raw y:"<<acc_measurement[1]<<endl;
		cout<<"Accelerometer raw z:"<<acc_measurement[2]<<endl;

		cout<<"Accelerometer angle x:"<<acc_measurement[3]<<endl;
		cout<<"Accelerometer angle y:"<<acc_measurement[4]<<endl;
		cout<<"Accelerometer angle z:"<<acc_measurement[5]<<endl;

		cout<<"Gyroscope raw x:"<<gyr_measurement[0]<<endl;
		cout<<"Gyroscope raw y:"<<gyr_measurement[1]<<endl;
		cout<<"Gyroscope raw z:"<<gyr_measurement[2]<<endl;

		cout<<"Gyroscope angle x:"<<gyr_measurement[3]<<endl;
		cout<<"Gyroscope angle y:"<<gyr_measurement[4]<<endl;
		cout<<"Gyroscope angle z:"<<gyr_measurement[5]<<endl;

		cout<<"Filtered angle x:"<<filtered_angles[0]<<endl;
		cout<<"Filtered angle y:"<<filtered_angles[1]<<endl;
		cout<<"Filtered angle z:"<<filtered_angles[2]<<endl;

		Mpu.calcDT();  // MUST place this at the end of loop
	}

	return 0;
}

