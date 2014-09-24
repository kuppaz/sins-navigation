	NullingOfArray (iM*iM, KalmanMatrixA);

	KalmanMatrixA[1]  =  U*sin(Latitude);		KalmanMatrixA[2]    = -U*cos(Latitude);
	KalmanMatrixA[iM] = -U*sin(Latitude);		KalmanMatrixA[2*iM] =  U*cos(Latitude);

	if (iM == 12)
	{
		KalmanMatrixA[9*iM+1]  =  -Gravity;		KalmanMatrixA[10*iM+0]    = Gravity;

		KalmanMatrixA[iM*9+3]      =  A_x0s[0];
		KalmanMatrixA[iM*9+4] =  A_x0s[1];
		KalmanMatrixA[iM*9+5] =  A_x0s[2];
		KalmanMatrixA[iM*10+3] =  A_x0s[3];
		KalmanMatrixA[iM*10+4] =  A_x0s[4];
		KalmanMatrixA[iM*10+5] =  A_x0s[5];
		KalmanMatrixA[iM*11+3] =  A_x0s[6];
		KalmanMatrixA[iM*11+4] =  A_x0s[7];
		KalmanMatrixA[iM*11+5] =  A_x0s[8];

		KalmanMatrixH[9]      =  1.0;
		KalmanMatrixH[iM*1+10] =  1.0;
		KalmanMatrixH[iM*2+11] =  1.0;
	}
	else
	{
		KalmanMatrixH[1]  = -Gravity;				KalmanMatrixH[iM]    =  Gravity;
		//A_x0s
		A_x0s[0] = KalmanMatrixH[3]      =  A_sx0[0];
		A_x0s[3] = KalmanMatrixH[iM*1+3] =  A_sx0[1];
		A_x0s[6] = KalmanMatrixH[iM*2+3] =  A_sx0[2];
		A_x0s[1] = KalmanMatrixH[4]      =  A_sx0[3];
		A_x0s[4] = KalmanMatrixH[iM*1+4] =  A_sx0[4];
		A_x0s[7] = KalmanMatrixH[iM*2+4] =  A_sx0[5];
		A_x0s[2] = KalmanMatrixH[5]      =  A_sx0[6];
		A_x0s[5] = KalmanMatrixH[iM*1+5] =  A_sx0[7];
		A_x0s[8] = KalmanMatrixH[iM*2+5] =  A_sx0[8];
	}
	

	A_x0s[0] = KalmanMatrixA[6]      =  A_sx0[0];
    A_x0s[3] = KalmanMatrixA[iM*1+6] =  A_sx0[1];
    A_x0s[6] = KalmanMatrixA[iM*2+6] =  A_sx0[2];
    A_x0s[1] = KalmanMatrixA[7]      =  A_sx0[3];
    A_x0s[4] = KalmanMatrixA[iM*1+7] =  A_sx0[4];
    A_x0s[7] = KalmanMatrixA[iM*2+7] =  A_sx0[5];
    A_x0s[2] = KalmanMatrixA[8]      =  A_sx0[6];
    A_x0s[5] = KalmanMatrixA[iM*1+8] =  A_sx0[7];
    A_x0s[8] = KalmanMatrixA[iM*2+8] =  A_sx0[8];

	//Реинициализация ковариационной матрицы шумов на каждой итерации
	NullingOfArray(iM*iMq, CovarianceMatrixNoise);
	CovarianceMatrixNoise[    0] = NoiseFactor*3.0;
	CovarianceMatrixNoise[  iMq+1] = NoiseFactor;
	CovarianceMatrixNoise[2*iMq+2] = NoiseFactor;

	if (iMq == 6)
	{
		CovarianceMatrixNoise[9*iM+3] = 0.01;
		CovarianceMatrixNoise[10*iM+4] = 0.01;
		CovarianceMatrixNoise[11*iM+5] = 0.01;
	}

	


