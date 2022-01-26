#include <math.h>
#include <stdio.h>

int quat_inv(float a[4], float b[4])
{
	float factor = 1 / (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
	b[0] = a[0] * factor;
	b[1] = -a[1] * factor;
	b[2] = -a[2] * factor;
	b[3] = -a[3] * factor;

	return 0;
}

int quat_mul(float a[4], float b[4], float ab[4])
{
	ab[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	ab[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	ab[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	ab[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
	return 0;
}

int quat_norm(float a[4], float b[4])
{
	float factor = 1 / sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
	b[0] = a[0] * factor;
	b[1] = a[1] * factor;
	b[2] = a[2] * factor;
	b[3] = a[3] * factor;

	return 0;
}

int quat2vec(float a[4], float b[3])
{
	float norm = sqrt(a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
	if (norm == 0)
	{

		b[0] = 0;
		b[1] = 0;
		b[2] = 0;
	}
	else
	{
		float theta;
		if (a[0] > 1)
			a[0] = 1;
		float theta_ = 2 * acos(a[0]);
		if (theta_ == 0)
			theta = 2.788630173671217e-04;
		else
			theta = theta_;

		float factor = theta / sin(theta / 2);

		b[0] = a[1] * factor;
		b[1] = a[2] * factor;
		b[2] = a[3] * factor;
	}

	return 0;
}

int vec2quat(float a[3], float b[4])
{
	float theta = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	if (theta == 0)
	{
		b[0] = cos(theta / 2);
		b[1] = 0;
		b[2] = 0;
		b[3] = 0;
	}
	else
	{
		float factor = sin(theta / 2) / theta;
		b[0] = cos(theta / 2);
		b[1] = a[0] * factor;
		b[2] = a[1] * factor;
		b[3] = a[2] * factor;
	}

	return 0;
}

int cholesky(float in[6][6], float out[6][6])
{

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			out[i][j] = 0;

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			float sum = 0;

			if (j == i) // summation for diagonals
			{
				for (int k = 0; k < j; k++)
					sum += (out[j][k] * out[j][k]);
				out[j][j] = sqrt(in[j][j] - sum) * sqrt(12);
			}
			else
			{

				// Evaluating L(i, j) using L(j, j)
				for (int k = 0; k < j; k++)
					sum += (out[i][k] * out[j][k]);
				out[i][j] = ((in[i][j] - sum) / out[j][j]) * sqrt(12);
			}
		}
	}

	return 0;
}

int inverse(float in[9][9], float out[9][9])
{
	int eye[9][9] = {{1, 0, 0, 0, 0, 0, 0, 0, 0},
					 {0, 1, 0, 0, 0, 0, 0, 0, 0},
					 {0, 0, 1, 0, 0, 0, 0, 0, 0},
					 {0, 0, 0, 1, 0, 0, 0, 0, 0},
					 {0, 0, 0, 0, 1, 0, 0, 0, 0},
					 {0, 0, 0, 0, 0, 1, 0, 0, 0},
					 {0, 0, 0, 0, 0, 0, 1, 0, 0},
					 {0, 0, 0, 0, 0, 0, 0, 1, 0},
					 {0, 0, 0, 0, 0, 0, 0, 0, 1}};

	float A[9][18];

	for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 9; j++)
		{

			// To store elements
			// of matrix A
			A[i][j] = in[i][j];

			// To store elements
			// of matrix B
			A[i][j + 9] = eye[i][j];
		}
	}

	float T[9][18];
	for (int i = 0; i < 9; i++)
		for (int j = 0; j < 18; j++)
			T[i][j] = 0;

	for (int i = 0; i < 9; i++)
	{
		for (int j = i; j <= i + 9; j++)
		{
			float sum = 0;

			if (j == i) // summation for diagonals
			{
				for (int k = 0; k < i; k++)
					sum += (T[k][i] * T[k][i]);
				T[i][i] = sqrt(A[i][i] - sum);
			}
			else
			{

				// Evaluating L(i, j) using L(j, j)
				for (int k = 0; k < i; k++)
					sum += (T[k][i] * T[k][j]);
				T[i][j] = (A[i][j] - sum) / T[i][i];
			}
		}
	}

	float T1[9][9];
	for (int i = 0; i < 9; i++)
		for (int j = 0; j < 9; j++)
			T1[i][j] = T[i][j + 9];

	float T1_transpose[9][9];
	for (int i = 0; i < 9; i++)
		for (int j = 0; j < 9; j++)
			T1_transpose[i][j] = T1[j][i];

	for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 9; j++)
		{

			float sum = 0;
			for (int k = 0; k < 9; k++)
			{
				sum += T1_transpose[i][k] * T1[k][j];
			}
			out[i][j] = sum;
		}
	}
	return 0;
}


int uk_filter_ag()
{

	const float Q[6][6] = {{100, 0, 0, 0, 0, 0},
						   {0, 100, 0, 0, 0, 0},
						   {0, 0, 100, 0, 0, 0},
						   {0, 0, 0, 0.100000000000000, 0, 0},
						   {0, 0, 0, 0, 0.100000000000000, 0},
						   {0, 0, 0, 0, 0, 0.100000000000000}};

	const float R[9][9] = {{0.5, 0, 0, 0, 0, 0, 0, 0, 0},
						   {0, 0.5, 0, 0, 0, 0, 0, 0, 0},
						   {0, 0, 0.5, 0, 0, 0, 0, 0, 0},
						   {0, 0, 0, 0.5, 0, 0, 0, 0, 0},
						   {0, 0, 0, 0, 0.5, 0, 0, 0, 0},
						   {0, 0, 0, 0, 0, 0.5, 0, 0, 0},
						   {0, 0, 0, 0, 0, 0, 0.01, 0, 0},
						   {0, 0, 0, 0, 0, 0, 0, 0.01, 0},
						   {0, 0, 0, 0, 0, 0, 0, 0, 0.01}};

	float state[7] = {1, 0, 0, 0, 0, 0, 0};

	float P[6][6] = {{0.0100000000000000, 0, 0, 0, 0, 0},
					 {0, 0.0100000000000000, 0, 0, 0, 0},
					 {0, 0, 0.0100000000000000, 0, 0, 0},
					 {0, 0, 0, 0.0100000000000000, 0, 0},
					 {0, 0, 0, 0, 0.0100000000000000, 0},
					 {0, 0, 0, 0, 0, 0.0100000000000000}};

	printf("[");

	for (int s = 0; s < 100; s++)
	{

		float data[9];

		for (int i = 0; i < 9; i++)
		{
			data[i] = data_set[s][i];
			//		printf("%f, ",data[i]);
		}

		float d_t = dt_set[s];
		//	printf("%f, \n",d_t);

		float q[4] = {state[0], state[1], state[2], state[3]};
		float omega[3] = {state[4], state[5], state[6]};

		// ****************         sigma = generate_sigma_points(state,P,Q,n);        *****************

		float matrix_sig[6][6];
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				matrix_sig[i][j] = P[i][j] + Q[i][j];

		float lower[6][6];
		cholesky(matrix_sig, lower);

		float lower_neg[6][6];
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				lower_neg[i][j] = -lower[i][j];

		float W_sig[6][12]; //append 2 matrices
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				W_sig[i][j] = lower[i][j];
				W_sig[i][j + 6] = lower_neg[i][j];
			}
		}

		float sigma[7][13];
		for (int i = 0; i < 12; i++)
		{

			float vec[3] = {W_sig[0][i], W_sig[1][i], W_sig[2][i]};
			float q_[4];
			vec2quat(vec, q_);

			float p[4];
			for (int k = 0; k < 4; k++)
				p[k] = q[k];

			float Qmul[4];
			quat_mul(p, q_, Qmul);

			float Qnorm[4];
			quat_norm(Qmul, Qnorm);

			for (int j = 0; j < 4; j++)
				sigma[j][i] = Qnorm[j];
		}

		for (int i = 0; i < 4; i++)
			sigma[i][12] = q[i];

		for (int i = 0; i < 12; i++)
		{
			sigma[4][i] = omega[0] + W_sig[3][i];
			sigma[5][i] = omega[1] + W_sig[4][i];
			sigma[6][i] = omega[2] + W_sig[5][i];
		}

		sigma[4][12] = omega[0];
		sigma[5][12] = omega[1];
		sigma[6][12] = omega[2];

		//   %%%%%%%%%%%%%%%%%%%%%%%%%    Y = Process_model_A(sigma,dt,M);  %%%%%%%%%%%%%%%%%%%%%%%%%

		float Y[7][13];

		for (int i = 0; i < 13; i++)
		{

			float normW = sqrt(sigma[4][i] * sigma[4][i] + sigma[5][i] * sigma[5][i] + sigma[6][i] * sigma[6][i]);

			if (normW == 0)
			{
				Y[0][i] = 1;
				Y[1][i] = 0;
				Y[2][i] = 0;
				Y[3][i] = 0;
			}

			else
			{

				float vec[3] = {sigma[4][i] * d_t, sigma[5][i] * d_t, sigma[6][i] * d_t};
				float q[4];
				vec2quat(vec, q);

				float p[4];
				for (int j = 0; j < 4; j++)
					p[j] = sigma[j][i];

				float Qmul[4];
				quat_mul(p, q, Qmul);

				float Qnorm[4];
				quat_norm(Qmul, Qnorm);

				for (int j = 0; j < 4; j++)
				{
					Y[0][i] = Qnorm[0];
					Y[1][i] = Qnorm[1];
					Y[2][i] = Qnorm[2];
					Y[3][i] = Qnorm[3];
				}
			}
		}
		for (int i = 0; i < 13; i++)
		{
			Y[4][i] = sigma[4][i];
			Y[5][i] = sigma[5][i];
			Y[6][i] = sigma[6][i];
		}

		//  %%%%%%%%%%%%%%%%    [x_bar, Pxx, Wdash] = cal_mean_and_covariance(Y,state,M);     %%%%%%%%%%%%%%%%%%%%

		float x_bar[7];
		float Pxx[6][6];
		float Wdash[6][13];

		float q_inv[4];
		quat_inv(q, q_inv);

		float condition = 1;
		float e_avg[3];

		for (int j = 0; j < 100 & condition >= 0.01; j++)
		{

			float e_vec[3][13];

			for (int i = 0; i < 13; i++)
			{

				float p[4];
				for (int k = 0; k < 4; k++)
					p[k] = Y[k][i];

				float Qmul[4];
				quat_mul(p, q_inv, Qmul);

				float e_norm[4];
				quat_norm(Qmul, e_norm);

				float e_ve[3];
				quat2vec(e_norm, e_ve);

				float norm = sqrt(pow(e_ve[0], 2) + pow(e_ve[1], 2) + pow(e_ve[2], 2));
				if (norm == 0)
				{

					e_vec[0][i] = 0;
					e_vec[1][i] = 0;
					e_vec[2][i] = 0;
				}
				else
				{
					float factor = (-3.141592653589793 + fmod((norm + 3.141592653589793), (2 * 3.141592653589793))) / norm;
					e_vec[0][i] = e_ve[0] * factor;
					e_vec[1][i] = e_ve[1] * factor;
					e_vec[2][i] = e_ve[2] * factor;
				}
			}

			float e_sum[3];
			for (int i = 0; i < 3; i++)
				e_sum[i] = 0;

			for (int i = 0; i < 13; i++)
			{
				e_sum[0] += e_vec[0][i];
				e_sum[1] += e_vec[1][i];
				e_sum[2] += e_vec[2][i];
			}

			for (int i = 0; i < 3; i++)
				e_avg[i] = e_sum[i] / 13;

			condition = sqrt(e_avg[0] * e_avg[0] + e_avg[1] * e_avg[1] + e_avg[2] * e_avg[2]);
		}

		float p_0[4];
		vec2quat(e_avg, p_0);

		float Qmul_0[4];
		quat_mul(p_0, q, Qmul_0);

		float mean_q[4];
		quat_norm(Qmul_0, mean_q);

		float mean_q_[4];
		quat_norm(mean_q, mean_q_);

		for (int i = 0; i < 4; i++)
			x_bar[i] = mean_q_[i];

		float w_sum[3];
		for (int i = 0; i < 3; i++)
			w_sum[i] = 0;

		for (int i = 0; i < 13; i++)
		{
			w_sum[0] += Y[4][i];
			w_sum[1] += Y[5][i];
			w_sum[2] += Y[6][i];
		}

		for (int i = 0; i < 3; i++)
			x_bar[i + 4] = w_sum[i] / 13;

		float p_1[4];
		float f_in[4] = {x_bar[0], x_bar[1], x_bar[2], x_bar[3]};
		quat_inv(f_in, p_1);

		float YMeanCentered[7][13];
		for (int i = 0; i < 13; i++)
		{
			float q[4];
			for (int j = 0; j < 4; j++)
				q[j] = Y[j][i];

			float Qmul[4];
			quat_mul(p_1, q, Qmul);

			for (int j = 0; j < 4; j++)
				YMeanCentered[j][i] = Qmul[j];

			for (int j = 4; j < 7; j++)
				YMeanCentered[j][i] = Y[j][i] - x_bar[j];
		}

		for (int i = 0; i < 13; i++)
		{

			float norm = sqrt(pow(YMeanCentered[1][i], 2) + pow(YMeanCentered[2][i], 2) + pow(YMeanCentered[3][i], 2));
			if (norm == 0)
			{

				Wdash[0][i] = 0;
				Wdash[1][i] = 0;
				Wdash[2][i] = 0;
			}
			else
			{
				float theta;
				if (YMeanCentered[0][i] > 1)
					YMeanCentered[0][i] = 1;
				float theta_ = 2 * acos(YMeanCentered[0][i]);
				if (theta_ == 0)
					theta = 2.788630173671217e-04;
				else
					theta = theta_;

				float factor = theta / sin(theta / 2);
				Wdash[0][i] = YMeanCentered[1][i] * factor;
				Wdash[1][i] = YMeanCentered[2][i] * factor;
				Wdash[2][i] = YMeanCentered[3][i] * factor;
			}

			for (int j = 3; j < 6; j++)
				Wdash[j][i] = YMeanCentered[j + 1][i];
		}

		float Wdash_trans[13][6];
		for (int i = 0; i < 13; i++)
			for (int j = 0; j < 6; j++)
				Wdash_trans[i][j] = Wdash[j][i];

		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				float sum = 0;

				for (int k = 0; k < 13; k++)
				{
					sum += Wdash[i][k] * Wdash_trans[k][j];
				}
				Pxx[i][j] = sum / 12;
			}
		}

		//%%%%%%%%%%%%%%%%%%%%%%%%%  Z = Measurement_model_H(Y, M);    %%%%%%%%%%%%%%%%%%%%%%

		float Z[9][13];
		float quat_g[4] = {0, 0, 0, 1};
		float quat_m[4] = {0, 0.3907, 0, -0.9205}; // according to braunschweig. formula [0;cos(Inclination);0;-sin(Inclination)]

		for (int i = 0; i < 13; i++)
		{

			float q_inv[4];
			float f_in0[4] = {Y[0][i], Y[1][i], Y[2][i], Y[3][i]};
			quat_inv(f_in0, q_inv);

			float Qmul[4];
			quat_mul(q_inv, quat_g, Qmul);

			float prod[4];
			quat_mul(Qmul, f_in0, prod);

			float vec_0[3];
			quat2vec(prod, vec_0);

			for (int j = 0; j < 3; j++)
				Z[j][i] = vec_0[j];

			float Qmul1[4];
			quat_mul(q_inv, quat_m, Qmul1);

			float prod1[4];
			quat_mul(Qmul1, f_in0, prod1);

			float vec_1[3];
			quat2vec(prod1, vec_1);

			for (int j = 0; j < 3; j++)
				Z[j + 3][i] = vec_1[j];

			Z[6][i] = Y[4][i];
			Z[7][i] = Y[5][i];
			Z[8][i] = Y[6][i];
		}

		// #############        [mean_Zk, K, Pvv] = cal_mean_and_KalmanGain(Z,R,Wdash);       #################

		float mean_Zk[9];
		float K[6][9];
		float Pvv[9][9];

		float Z_sum[9];
		for (int i = 0; i < 9; i++)
			Z_sum[i] = 0;

		for (int i = 0; i < 13; i++)
			for (int j = 0; j < 9; j++)
				Z_sum[j] += Z[j][i];

		for (int i = 0; i < 9; i++)
			mean_Zk[i] = Z_sum[i] / 13;

		float ZMeanCentered[9][13];
		for (int i = 0; i < 13; i++)
			for (int j = 0; j < 9; j++)
				ZMeanCentered[j][i] = Z[j][i] - mean_Zk[j];

		float ZMeanCentered_trans[13][9];
		for (int i = 0; i < 13; i++)
			for (int j = 0; j < 9; j++)
				ZMeanCentered_trans[i][j] = ZMeanCentered[j][i];

		float Pzz[9][9];
		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				float sum = 0;

				for (int k = 0; k < 13; k++)
				{
					sum += ZMeanCentered[i][k] * ZMeanCentered_trans[k][j];
				}
				Pzz[i][j] = sum / 12;
			}
		}

		for (int i = 0; i < 9; i++)
			for (int j = 0; j < 9; j++)
				Pvv[i][j] = Pzz[i][j] + R[i][j];

		float Pxz[6][9];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				float sum = 0;

				for (int k = 0; k < 13; k++)
				{
					sum += Wdash[i][k] * ZMeanCentered_trans[k][j];
				}
				Pxz[i][j] = sum / 12;
			}
		}

		float Pvv_inv[9][9];
		inverse(Pvv, Pvv_inv);

		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 9; j++)
			{

				float sum = 0;
				for (int k = 0; k < 9; k++)
				{
					sum += Pxz[i][k] * Pvv_inv[k][j];
				}
				K[i][j] = sum;
			}
		}

		// ################  [state_u, P_u] = update(x_bar, Pxx, Pvv, K, data, mean_Zk);    ########################

		float state_u[7];
		float P_u[6][6];

		float vk[9];
		for (int i = 0; i < 6; i++)
			vk[i] = data[i] - mean_Zk[i];

		float Mul[6];
		for (int i = 0; i < 6; i++)
		{

			float sum = 0;
			for (int k = 0; k < 9; k++)
			{
				sum += K[i][k] * vk[k];
			}
			Mul[i] = sum;
		}

		float q_u[4];
		float f_in1[3] = {Mul[0], Mul[1], Mul[2]};
		vec2quat(f_in1, q_u);

		float p_u[4];
		for (int i = 0; i < 4; i++)
			p_u[i] = x_bar[i];

		float Qmul_u[4];
		quat_mul(p_u, q_u, Qmul_u);

		float mean_Q[4];
		quat_norm(Qmul_u, mean_Q);

		for (int i = 0; i < 4; i++)
			state_u[i] = mean_Q[i];

		state_u[4] = x_bar[4] + Mul[3];
		state_u[5] = x_bar[5] + Mul[4];
		state_u[6] = x_bar[6] + Mul[5];

		float Mat_mul[6][9];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 9; j++)
			{

				float sum = 0;
				for (int k = 0; k < 9; k++)
				{
					sum += K[i][k] * Pvv[k][j];
				}
				Mat_mul[i][j] = sum;
			}
		}

		float K_trans[9][6];
		for (int i = 0; i < 9; i++)
			for (int j = 0; j < 6; j++)
				K_trans[i][j] = K[j][i];

		float Mat_mul_1[6][9];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{

				float sum = 0;
				for (int k = 0; k < 9; k++)
				{
					sum += Mat_mul[i][k] * K_trans[k][j];
				}
				Mat_mul_1[i][j] = sum;
			}
		}

		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				P_u[i][j] = Pxx[i][j] - Mat_mul_1[i][j];

		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				P[i][j] = P_u[i][j];
				//			printf("%f, ",P[i][j]);
			}
			//		printf("\n");
		}

		for (int i = 0; i < 7; i++)
		{
			state[i] = state_u[i];
			printf("%f, ", state[i]); // display the current element out of the array
		}
		printf(";\n");
	}
	printf("]");
	return 0;
}


int uk_filter_mag()
{

	const float Q[6][6] = {{100, 0, 0, 0, 0, 0},
						   {0, 100, 0, 0, 0, 0},
						   {0, 0, 100, 0, 0, 0},
						   {0, 0, 0, 0.100000000000000, 0, 0},
						   {0, 0, 0, 0, 0.100000000000000, 0},
						   {0, 0, 0, 0, 0, 0.100000000000000}};

	const float R[9][9] = {{0.5, 0, 0, 0, 0, 0, 0, 0, 0},
						   {0, 0.5, 0, 0, 0, 0, 0, 0, 0},
						   {0, 0, 0.5, 0, 0, 0, 0, 0, 0},
						   {0, 0, 0, 0.5, 0, 0, 0, 0, 0},
						   {0, 0, 0, 0, 0.5, 0, 0, 0, 0},
						   {0, 0, 0, 0, 0, 0.5, 0, 0, 0},
						   {0, 0, 0, 0, 0, 0, 0.01, 0, 0},
						   {0, 0, 0, 0, 0, 0, 0, 0.01, 0},
						   {0, 0, 0, 0, 0, 0, 0, 0, 0.01}};

	float state[7] = {1, 0, 0, 0, 0, 0, 0};

	float P[6][6] = {{0.0100000000000000, 0, 0, 0, 0, 0},
					 {0, 0.0100000000000000, 0, 0, 0, 0},
					 {0, 0, 0.0100000000000000, 0, 0, 0},
					 {0, 0, 0, 0.0100000000000000, 0, 0},
					 {0, 0, 0, 0, 0.0100000000000000, 0},
					 {0, 0, 0, 0, 0, 0.0100000000000000}};

	printf("[");

	for (int s = 0; s < 100; s++)
	{

		float data[9];

		for (int i = 0; i < 9; i++)
		{
			data[i] = data_set[s][i];
			//		printf("%f, ",data[i]);
		}

		float d_t = dt_set[s];
		//	printf("%f, \n",d_t);

		float q[4] = {state[0], state[1], state[2], state[3]};
		float omega[3] = {state[4], state[5], state[6]};

		// ****************         sigma = generate_sigma_points(state,P,Q,n);        *****************

		float matrix_sig[6][6];
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				matrix_sig[i][j] = P[i][j] + Q[i][j];

		float lower[6][6];
		cholesky(matrix_sig, lower);

		float lower_neg[6][6];
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				lower_neg[i][j] = -lower[i][j];

		float W_sig[6][12]; //append 2 matrices
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				W_sig[i][j] = lower[i][j];
				W_sig[i][j + 6] = lower_neg[i][j];
			}
		}

		float sigma[7][13];
		for (int i = 0; i < 12; i++)
		{

			float vec[3] = {W_sig[0][i], W_sig[1][i], W_sig[2][i]};
			float q_[4];
			vec2quat(vec, q_);

			float p[4];
			for (int k = 0; k < 4; k++)
				p[k] = q[k];

			float Qmul[4];
			quat_mul(p, q_, Qmul);

			float Qnorm[4];
			quat_norm(Qmul, Qnorm);

			for (int j = 0; j < 4; j++)
				sigma[j][i] = Qnorm[j];
		}

		for (int i = 0; i < 4; i++)
			sigma[i][12] = q[i];

		for (int i = 0; i < 12; i++)
		{
			sigma[4][i] = omega[0] + W_sig[3][i];
			sigma[5][i] = omega[1] + W_sig[4][i];
			sigma[6][i] = omega[2] + W_sig[5][i];
		}

		sigma[4][12] = omega[0];
		sigma[5][12] = omega[1];
		sigma[6][12] = omega[2];

		//   %%%%%%%%%%%%%%%%%%%%%%%%%    Y = Process_model_A(sigma,dt,M);  %%%%%%%%%%%%%%%%%%%%%%%%%

		float Y[7][13];

		for (int i = 0; i < 13; i++)
		{

			float normW = sqrt(sigma[4][i] * sigma[4][i] + sigma[5][i] * sigma[5][i] + sigma[6][i] * sigma[6][i]);

			if (normW == 0)
			{
				Y[0][i] = 1;
				Y[1][i] = 0;
				Y[2][i] = 0;
				Y[3][i] = 0;
			}

			else
			{

				float vec[3] = {sigma[4][i] * d_t, sigma[5][i] * d_t, sigma[6][i] * d_t};
				float q[4];
				vec2quat(vec, q);

				float p[4];
				for (int j = 0; j < 4; j++)
					p[j] = sigma[j][i];

				float Qmul[4];
				quat_mul(p, q, Qmul);

				float Qnorm[4];
				quat_norm(Qmul, Qnorm);

				for (int j = 0; j < 4; j++)
				{
					Y[0][i] = Qnorm[0];
					Y[1][i] = Qnorm[1];
					Y[2][i] = Qnorm[2];
					Y[3][i] = Qnorm[3];
				}
			}
		}
		for (int i = 0; i < 13; i++)
		{
			Y[4][i] = sigma[4][i];
			Y[5][i] = sigma[5][i];
			Y[6][i] = sigma[6][i];
		}

		//  %%%%%%%%%%%%%%%%    [x_bar, Pxx, Wdash] = cal_mean_and_covariance(Y,state,M);     %%%%%%%%%%%%%%%%%%%%

		float x_bar[7];
		float Pxx[6][6];
		float Wdash[6][13];

		float q_inv[4];
		quat_inv(q, q_inv);

		float condition = 1;
		float e_avg[3];

		for (int j = 0; j < 100 & condition >= 0.01; j++)
		{

			float e_vec[3][13];

			for (int i = 0; i < 13; i++)
			{

				float p[4];
				for (int k = 0; k < 4; k++)
					p[k] = Y[k][i];

				float Qmul[4];
				quat_mul(p, q_inv, Qmul);

				float e_norm[4];
				quat_norm(Qmul, e_norm);

				float e_ve[3];
				quat2vec(e_norm, e_ve);

				float norm = sqrt(pow(e_ve[0], 2) + pow(e_ve[1], 2) + pow(e_ve[2], 2));
				if (norm == 0)
				{

					e_vec[0][i] = 0;
					e_vec[1][i] = 0;
					e_vec[2][i] = 0;
				}
				else
				{
					float factor = (-3.141592653589793 + fmod((norm + 3.141592653589793), (2 * 3.141592653589793))) / norm;
					e_vec[0][i] = e_ve[0] * factor;
					e_vec[1][i] = e_ve[1] * factor;
					e_vec[2][i] = e_ve[2] * factor;
				}
			}

			float e_sum[3];
			for (int i = 0; i < 3; i++)
				e_sum[i] = 0;

			for (int i = 0; i < 13; i++)
			{
				e_sum[0] += e_vec[0][i];
				e_sum[1] += e_vec[1][i];
				e_sum[2] += e_vec[2][i];
			}

			for (int i = 0; i < 3; i++)
				e_avg[i] = e_sum[i] / 13;

			condition = sqrt(e_avg[0] * e_avg[0] + e_avg[1] * e_avg[1] + e_avg[2] * e_avg[2]);
		}

		float p_0[4];
		vec2quat(e_avg, p_0);

		float Qmul_0[4];
		quat_mul(p_0, q, Qmul_0);

		float mean_q[4];
		quat_norm(Qmul_0, mean_q);

		float mean_q_[4];
		quat_norm(mean_q, mean_q_);

		for (int i = 0; i < 4; i++)
			x_bar[i] = mean_q_[i];

		float w_sum[3];
		for (int i = 0; i < 3; i++)
			w_sum[i] = 0;

		for (int i = 0; i < 13; i++)
		{
			w_sum[0] += Y[4][i];
			w_sum[1] += Y[5][i];
			w_sum[2] += Y[6][i];
		}

		for (int i = 0; i < 3; i++)
			x_bar[i + 4] = w_sum[i] / 13;

		float p_1[4];
		float f_in[4] = {x_bar[0], x_bar[1], x_bar[2], x_bar[3]};
		quat_inv(f_in, p_1);

		float YMeanCentered[7][13];
		for (int i = 0; i < 13; i++)
		{
			float q[4];
			for (int j = 0; j < 4; j++)
				q[j] = Y[j][i];

			float Qmul[4];
			quat_mul(p_1, q, Qmul);

			for (int j = 0; j < 4; j++)
				YMeanCentered[j][i] = Qmul[j];

			for (int j = 4; j < 7; j++)
				YMeanCentered[j][i] = Y[j][i] - x_bar[j];
		}

		for (int i = 0; i < 13; i++)
		{

			float norm = sqrt(pow(YMeanCentered[1][i], 2) + pow(YMeanCentered[2][i], 2) + pow(YMeanCentered[3][i], 2));
			if (norm == 0)
			{

				Wdash[0][i] = 0;
				Wdash[1][i] = 0;
				Wdash[2][i] = 0;
			}
			else
			{
				float theta;
				if (YMeanCentered[0][i] > 1)
					YMeanCentered[0][i] = 1;
				float theta_ = 2 * acos(YMeanCentered[0][i]);
				if (theta_ == 0)
					theta = 2.788630173671217e-04;
				else
					theta = theta_;

				float factor = theta / sin(theta / 2);
				Wdash[0][i] = YMeanCentered[1][i] * factor;
				Wdash[1][i] = YMeanCentered[2][i] * factor;
				Wdash[2][i] = YMeanCentered[3][i] * factor;
			}

			for (int j = 3; j < 6; j++)
				Wdash[j][i] = YMeanCentered[j + 1][i];
		}

		float Wdash_trans[13][6];
		for (int i = 0; i < 13; i++)
			for (int j = 0; j < 6; j++)
				Wdash_trans[i][j] = Wdash[j][i];

		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				float sum = 0;

				for (int k = 0; k < 13; k++)
				{
					sum += Wdash[i][k] * Wdash_trans[k][j];
				}
				Pxx[i][j] = sum / 12;
			}
		}

		//%%%%%%%%%%%%%%%%%%%%%%%%%  Z = Measurement_model_H(Y, M);    %%%%%%%%%%%%%%%%%%%%%%

		float Z[9][13];
		float quat_g[4] = {0, 0, 0, 1};
		float quat_m[4] = {0, 0.3907, 0, -0.9205}; // according to braunschweig. formula [0;cos(Inclination);0;-sin(Inclination)]

		for (int i = 0; i < 13; i++)
		{

			float q_inv[4];
			float f_in0[4] = {Y[0][i], Y[1][i], Y[2][i], Y[3][i]};
			quat_inv(f_in0, q_inv);

			float Qmul[4];
			quat_mul(q_inv, quat_g, Qmul);

			float prod[4];
			quat_mul(Qmul, f_in0, prod);

			float vec_0[3];
			quat2vec(prod, vec_0);

			for (int j = 0; j < 3; j++)
				Z[j][i] = vec_0[j];

			float Qmul1[4];
			quat_mul(q_inv, quat_m, Qmul1);

			float prod1[4];
			quat_mul(Qmul1, f_in0, prod1);

			float vec_1[3];
			quat2vec(prod1, vec_1);

			for (int j = 0; j < 3; j++)
				Z[j + 3][i] = vec_1[j];

			Z[6][i] = Y[4][i];
			Z[7][i] = Y[5][i];
			Z[8][i] = Y[6][i];
		}

		// #############        [mean_Zk, K, Pvv] = cal_mean_and_KalmanGain(Z,R,Wdash);       #################

		float mean_Zk[9];
		float K[6][9];
		float Pvv[9][9];

		float Z_sum[9];
		for (int i = 0; i < 9; i++)
			Z_sum[i] = 0;

		for (int i = 0; i < 13; i++)
			for (int j = 0; j < 9; j++)
				Z_sum[j] += Z[j][i];

		for (int i = 0; i < 9; i++)
			mean_Zk[i] = Z_sum[i] / 13;

		float ZMeanCentered[9][13];
		for (int i = 0; i < 13; i++)
			for (int j = 0; j < 9; j++)
				ZMeanCentered[j][i] = Z[j][i] - mean_Zk[j];

		float ZMeanCentered_trans[13][9];
		for (int i = 0; i < 13; i++)
			for (int j = 0; j < 9; j++)
				ZMeanCentered_trans[i][j] = ZMeanCentered[j][i];

		float Pzz[9][9];
		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				float sum = 0;

				for (int k = 0; k < 13; k++)
				{
					sum += ZMeanCentered[i][k] * ZMeanCentered_trans[k][j];
				}
				Pzz[i][j] = sum / 12;
			}
		}

		for (int i = 0; i < 9; i++)
			for (int j = 0; j < 9; j++)
				Pvv[i][j] = Pzz[i][j] + R[i][j];

		float Pxz[6][9];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				float sum = 0;

				for (int k = 0; k < 13; k++)
				{
					sum += Wdash[i][k] * ZMeanCentered_trans[k][j];
				}
				Pxz[i][j] = sum / 12;
			}
		}

		float Pvv_inv[9][9];
		inverse(Pvv, Pvv_inv);

		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 9; j++)
			{

				float sum = 0;
				for (int k = 0; k < 9; k++)
				{
					sum += Pxz[i][k] * Pvv_inv[k][j];
				}
				K[i][j] = sum;
			}
		}

		// ################  [state_u, P_u] = update(x_bar, Pxx, Pvv, K, data, mean_Zk);    ########################

		float state_u[7];
		float P_u[6][6];

		float vk[9];
		for (int i = 0; i < 6; i++)
			vk[i] = data[i] - mean_Zk[i];

		float Mul[6];
		for (int i = 0; i < 6; i++)
		{

			float sum = 0;
			for (int k = 0; k < 9; k++)
			{
				sum += K[i][k] * vk[k];
			}
			Mul[i] = sum;
		}

		float q_u[4];
		float f_in1[3] = {Mul[0], Mul[1], Mul[2]};
		vec2quat(f_in1, q_u);

		float p_u[4];
		for (int i = 0; i < 4; i++)
			p_u[i] = x_bar[i];

		float Qmul_u[4];
		quat_mul(p_u, q_u, Qmul_u);

		float mean_Q[4];
		quat_norm(Qmul_u, mean_Q);

		for (int i = 0; i < 4; i++)
			state_u[i] = mean_Q[i];

		state_u[4] = x_bar[4] + Mul[3];
		state_u[5] = x_bar[5] + Mul[4];
		state_u[6] = x_bar[6] + Mul[5];

		float Mat_mul[6][9];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 9; j++)
			{

				float sum = 0;
				for (int k = 0; k < 9; k++)
				{
					sum += K[i][k] * Pvv[k][j];
				}
				Mat_mul[i][j] = sum;
			}
		}

		float K_trans[9][6];
		for (int i = 0; i < 9; i++)
			for (int j = 0; j < 6; j++)
				K_trans[i][j] = K[j][i];

		float Mat_mul_1[6][9];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{

				float sum = 0;
				for (int k = 0; k < 9; k++)
				{
					sum += Mat_mul[i][k] * K_trans[k][j];
				}
				Mat_mul_1[i][j] = sum;
			}
		}

		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				P_u[i][j] = Pxx[i][j] - Mat_mul_1[i][j];

		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				P[i][j] = P_u[i][j];
				//			printf("%f, ",P[i][j]);
			}
			//		printf("\n");
		}

		for (int i = 0; i < 7; i++)
		{
			state[i] = state_u[i];
			printf("%f, ", state[i]); // display the current element out of the array
		}
		printf(";\n");
	}
	printf("]");
	return 0;
}
