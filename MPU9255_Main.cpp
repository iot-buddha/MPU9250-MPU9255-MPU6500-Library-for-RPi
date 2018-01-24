#include <iostream>
#include "MPU9255.h"

using namespace std;

int main(void) {
	Mpu.initialize();
	Mpu.setAlpha(0.98);  // Default is 0.96
	
	Mpu.initDT();  // Place this before entering loop
	while (1) {
		Mpu.getResult();
		cout << fixed << Mpu.angles[0] << ", " << fixed << Mpu.angles[1] << endl;
		Mpu.calcDT();  // MUST place this at the end of loop
	}

	return 0;
}

