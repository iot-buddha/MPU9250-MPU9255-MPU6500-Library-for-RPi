#include <iostream>
#include <chrono>
#include"imu_ukf.h"

using namespace std;

int main(void) {

	const float Q[6][6] = {{100, 0, 0, 0, 0, 0},
						   {0, 100, 0, 0, 0, 0},
						   {0, 0, 100, 0, 0, 0},
						   {0, 0, 0, 0.100000000000000, 0, 0},
						   {0, 0, 0, 0, 0.100000000000000, 0},
						   {0, 0, 0, 0, 0, 0.100000000000000}};

	const float R[9][9] = {{0.5, 0, 0, 0, 0, 0},
						   {0, 0.5, 0, 0, 0, 0},
						   {0, 0, 0.5, 0, 0, 0},
						   {0, 0, 0, 0.5, 0, 0},
						   {0, 0, 0, 0, 0.5, 0},
						   {0, 0, 0, 0, 0, 0.5}};

	float state[7] = {1, 0, 0, 0, 0, 0, 0};

	float P[6][6] = {{0.0100000000000000, 0, 0, 0, 0, 0},
					 {0, 0.0100000000000000, 0, 0, 0, 0},
					 {0, 0, 0.0100000000000000, 0, 0, 0},
					 {0, 0, 0, 0.0100000000000000, 0, 0},
					 {0, 0, 0, 0, 0.0100000000000000, 0},
					 {0, 0, 0, 0, 0, 0.0100000000000000}};

	int count = 0;

	cout<<"Starting to Initialize"<<endl;
	Mpu.initialize();

	cout<<"Setting alpha value"<<endl;
	Mpu.setAlpha(0.98);  // Default is 0.96
	
	cout<<"Setting alpha value"<<endl;
	Mpu.initDT();  // Place this before entering loop
	while (1) {
		cout<< "Running Loop"<<endl;
		float acc_measurement[6] = {0.0};
		float gyr_measurement[6] = {0.0};
		float filtered_angles[3] = {0.0};
		double timedifference = 0;

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

		float data[6] = [acc_measurement[0],acc_measurement[1],acc_measurement[2],gyr_measurement[0],gyr_measurement[1],gyr_measurement[2]];
		

		Mpu.calcDT(timedifference);  // MUST place this at the end of loop

		cout<<"Time diff in sec"<<timedifference<<endl;

		float P_u[6][6];
		float state_u[7];

		float d_t;
		if (count == 0){
			d_t = 0.01;
		}
            
    	else
            d_t = timedifference;
			for (int i = 0; i < 7; i++)
				state[i] = state_u[i];

			for (int i = 0; i < 6; i++)
				for (int j = 0; j < 6; j++)
					P[i][j] = P_u[i][j];

            if d_t < 0.010
                d_t = 0.01;
            end
    	end

		uk_filter_ag(P,state,data,d_t,P_u,state_u);

		
		const auto p1 = std::chrono::system_clock::now();
		std::cout << "time now us: "<< std::chrono::duration_cast<std::chrono::microseconds>(p1.time_since_epoch()).count()<<endl;
	}
	cout<< "Exiting the loop"<<endl;
	return 0;
}

