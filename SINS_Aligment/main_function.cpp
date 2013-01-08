#include <math.h>
#include <conio.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "function_lib.h"

void RoughAligment	(double ALignCNT, double Gravity, double *timePlus, double *MeasureS, double *W_s, double *MeasureS_Average, double *N_Count, double *W_s_Summ, double *MeasureS_Summ, double *Heading, double *Roll, double *Pitch, double *Latitude, FILE    *EntranceParameters, double RoughT, FILE *ExitParameters, double *W_s_avg, FILE *ForHelp)
{
	double  *W_x_Estimate, *MatrixXS, Pi = 4.0*atan(1.0), Count = 0, Time = 0.0;
	char DataString[1024];
	int N_sigm=0;
	double     U				 = 0.000072921157;
	double *temp = new double[3];

	W_x_Estimate = new double[3];
	MatrixXS	 = new double[9];

	double alpha_x = 0.1 * 3.141592 / 180.0;
	double alpha_y = 0.08  * 3.141592 / 180.0;
	double alpha_z = 0.46  * 3.141592 / 180.0;
	

	*N_Count = 0.0;
	NullingOfArray (3, W_s_Summ);
	NullingOfArray (3, MeasureS_Average);

	for (int i = 0; i<10; i++)
		fgets  (DataString, 1024, EntranceParameters); 

	while (Count < ALignCNT)
	{
		fgets  (DataString, 1024, EntranceParameters); 
		//sscanf (DataString, "%lf %lf %lf %lf %lf %lf %lf", &(*timePlus), &MeasureS[1], &MeasureS[2], &MeasureS[0], &W_s[1], &W_s[2], &W_s[0]); 
		sscanf (DataString, "%lf %lf %lf %lf %lf %lf %lf %lf", &Time, &Count, &MeasureS[1], &MeasureS[2], &MeasureS[0], &W_s[1], &W_s[2], &W_s[0]); 
		*timePlus = Count*0.02048;

		//temp[1] = MeasureS[1] + alpha_z * MeasureS[2] + alpha_y * MeasureS[0];//-0.02;
  //      temp[2] = MeasureS[2] - alpha_z * MeasureS[1] + alpha_x * MeasureS[0];//+0.037;
  //      temp[0] = MeasureS[0] - alpha_y * MeasureS[1] - alpha_x * MeasureS[2];

		//EquatingOfVectors(3, temp, MeasureS);

  //      temp[1] = W_s[1] + alpha_z * W_s[2] + alpha_y * W_s[0];
  //      temp[2] = W_s[2] - alpha_z * W_s[1] + alpha_x * W_s[0];
  //      temp[0] = W_s[0] - alpha_y * W_s[1] - alpha_x * W_s[2];

		//EquatingOfVectors(3, temp, W_s);

		//for (int i=0; i<3; i++)
			//W_s[i] = W_s[i]*Pi/180.0;
		/*for (int i=0; i<3; i++)
			MeasureS[i] = MeasureS[i]*9.81;*/

		*N_Count = *N_Count +1.0;
		for (int i=0; i<3; i++)
		{
			W_s_Summ[i] = W_s_Summ[i] + W_s[i];
			W_s_avg[i] = W_s_Summ[i]/(*N_Count);
			W_s[i] = W_s_Summ[i]/(*N_Count);
		}

		for (int i=0; i<3; i++)
			MeasureS_Summ[i] = MeasureS_Summ[i] + MeasureS[i];

		for (int i=0; i<3; i++)
			MeasureS_Average[i] = MeasureS_Summ[i]/(*N_Count);

		Gravity = sqrt(MeasureS_Average[0]*MeasureS_Average[0] + MeasureS_Average[1]*MeasureS_Average[1] + MeasureS_Average[2]*MeasureS_Average[2]);

		*Pitch =  atan (MeasureS_Average[1]/sqrt(MeasureS_Average[0]*MeasureS_Average[0] + MeasureS_Average[2]*MeasureS_Average[2]));
		*Roll  = -atan (MeasureS_Average[0]/MeasureS_Average[2]);

		MatrixXS[0] = cos(*Roll);
		MatrixXS[1] = 0.0;
		MatrixXS[2] = sin(*Roll);
		MatrixXS[3] = sin(*Pitch)*sin(*Roll);
		MatrixXS[4] = cos(*Pitch);
		MatrixXS[5] = -sin(*Pitch)*cos(*Roll);
		MatrixXS[6] = -cos(*Pitch)*sin(*Roll);
		MatrixXS[7] = sin(*Pitch);
		MatrixXS[8] = cos(*Pitch)*cos(*Roll);

		MatrixOnVector (3,3, MatrixXS, W_s_Summ, W_x_Estimate);

		//*Heading = asin(W_x_Estimate[0]/(*N_Count)/U/cos(*Latitude));
		*Heading = -atan(W_x_Estimate[0]/W_x_Estimate[1]);

		printf ("Time = %.4f,  %.4f,  %.4f,  %.4f   %.3f\n", Count, MeasureS_Average[0], MeasureS_Average[1], MeasureS_Average[2], *N_Count);

		fprintf (ExitParameters, "%f \t %.9f\t%.9f\t%.9f\t  %.11f\t%.11f\t%.11f\t  %.11f\t%.11f\t%.11f\t%.11f\n", Count, MeasureS_Average[0], MeasureS_Average[1], MeasureS_Average[2], W_s[0], W_s[1], W_s[2],
				*Latitude, *Heading, *Roll, *Pitch);  
		//fprintf(ForHelp, "%f %f %f %f %f %f %f\n", Count, MeasureS_Average[0], MeasureS_Average[1], MeasureS_Average[2], W_s_avg[0], W_s_avg[1], W_s_avg[2]);
	}
	//*Heading = *Heading - Pi;
	//system ("pause");


	free(temp);
	free(W_x_Estimate);
	free(MatrixXS);
}



void InitialOfMatrix	(double *A_sx0, double *A_sKsi, double Heading, double Roll, double Pitch, double Latitude)
{
	double *A_etaX0;
	A_etaX0 = new double[9];

        A_sx0[0] =  cos(Heading)* cos(Roll)+ sin(Heading)*sin(Pitch)*sin(Roll); 
        A_sx0[1] = -sin(Heading)* cos(Roll)+ cos(Heading)*sin(Pitch)*sin(Roll);
        A_sx0[2] = -cos(Pitch)  * sin(Roll);
        A_sx0[3] =  sin(Heading)* cos(Pitch);
        A_sx0[4] =  cos(Heading)* cos(Pitch);
        A_sx0[5] =  sin(Pitch);
        A_sx0[6] =  cos(Heading)* sin(Roll)- sin(Heading)*sin(Pitch)*cos(Roll);
        A_sx0[7] = -sin(Heading)* sin(Roll)- cos(Heading)*sin(Pitch)*cos(Roll);
        A_sx0[8] =  cos(Pitch)  * cos(Roll);

		A_etaX0[0] = 0.0;
		A_etaX0[1] = 1.0;
		A_etaX0[2] = 0.0;
		A_etaX0[3] = -sin(Latitude);
		A_etaX0[4] = 0.0;
		A_etaX0[5] = cos(Latitude);
		A_etaX0[6] = cos(Latitude);
		A_etaX0[7] = 0.0;
		A_etaX0[8] = sin(Latitude);

		MatrixOnMatrix(3,3,3, A_sx0, A_etaX0, A_sKsi);

	free(A_etaX0);
}


void IntegrationOfOrientationMatrix (double *A_sx0, double *W_s, double *W_s_prev, double DeltaTime, double Latitude, double U)
{
	int i=0;
	double *a, *DeltaW_s, *U_x, *U_s_x, Pi = 4.0*atan(1.0), *tempV;

	a		 = new double[9];
	DeltaW_s = new double[3];
	U_x		 = new double[3];
	U_s_x	 = new double[3];
	tempV	 = new double[3];

	U_x[0] = 0.0;
	U_x[1] = U*cos(Latitude);
	U_x[2] = U*sin(Latitude);

	MatrixOnVector (3,3, A_sx0, U_x, U_s_x);

	for (i=0; i<3; i++)
		DeltaW_s[i] = (W_s[i] + W_s_prev[i])/2.0 - U_s_x[i];

	a[0] = ( DeltaW_s[2]*A_sx0[3] - DeltaW_s[1]*A_sx0[6])*DeltaTime + A_sx0[0];
	a[1] = ( DeltaW_s[2]*A_sx0[4] - DeltaW_s[1]*A_sx0[7])*DeltaTime + A_sx0[1];
	a[2] = ( DeltaW_s[2]*A_sx0[5] - DeltaW_s[1]*A_sx0[8])*DeltaTime + A_sx0[2];
	a[3] = (-DeltaW_s[2]*A_sx0[0] + DeltaW_s[0]*A_sx0[6])*DeltaTime + A_sx0[3];
	a[4] = (-DeltaW_s[2]*A_sx0[1] + DeltaW_s[0]*A_sx0[7])*DeltaTime + A_sx0[4];
	a[5] = (-DeltaW_s[2]*A_sx0[2] + DeltaW_s[0]*A_sx0[8])*DeltaTime + A_sx0[5];
	a[6] = ( DeltaW_s[1]*A_sx0[0] - DeltaW_s[0]*A_sx0[3])*DeltaTime + A_sx0[6];
	a[7] = ( DeltaW_s[1]*A_sx0[1] - DeltaW_s[0]*A_sx0[4])*DeltaTime + A_sx0[7];
	a[8] = ( DeltaW_s[1]*A_sx0[2] - DeltaW_s[0]*A_sx0[5])*DeltaTime + A_sx0[8];

	for (i = 0; i < 3; i++)
                tempV[i] = sqrt(a[i*3+0] * a[i*3+0] + a[i*3+1] * a[i*3+1] + a[i*3+2] * a[i*3+2]);

	for (i=0; i<3; i++)
		for (int j=0; j<3; j++)
			A_sx0[i*3+j] = a[i*3+j]/tempV[i];

	free(a);
	free(DeltaW_s);
	free(U_x);
	free(U_s_x);
	free(tempV);
}

void 	KalmanFilter (int iMq, int iM, int  iMz, double timePlus, double *KalmanMatrixA, double *KalmanMatrixH, double *ConditionVector_minus, double *ConditionVector_plus, double *CovarianceMatrixS_minus, double *CovarianceMatrixS_plus, 
					 double *CovarianceMatrixNoise, double Gravity, double U, double *W_s, double *W_s_prev, double *MeasureS, double *MeasureX, double *TransitionMatrixF, double DeltaTime, 
					 double *StringOfMeasureH, double r_noise, double Latitude, double *A_x0s, FILE *ForHelp, double *MeasureS_prev, double N_Count)
{
	int j,b,i;
	double *KalmFactor = new double[iM];
	double *temp = new double[iM];
	double measure;

	NullingOfArray (iM, KalmFactor);

	//Формирование переходной матрицы Ф
    func (DeltaTime, KalmanMatrixA, TransitionMatrixF, iM);

	
		for (j=0; j< 3; j++)    
		{
			for (b=0 ; b<iM ; b++) 
				StringOfMeasureH[b] = KalmanMatrixH[j*iM+b];
			
			//сама функция коррекции
			if (iM == 9)
			{
				f0b (MeasureX[j], ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  r_noise*r_noise, 
								ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);
			}
			else if (iM == 12)
			{
				f0b (MeasureX[j], ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  0.1*0.1, 
								ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);
				//fprintf (ForHelp, "%.10f %.10f\n", MeasureX[1], MeasureX[2]);
			}
                    
			EquatingOfVectors (iM,	  ConditionVector_plus,   ConditionVector_minus);
			EquatingOfVectors (iM*iM, CovarianceMatrixS_plus, CovarianceMatrixS_minus);
		}


	//for (b=0 ; b<iM ; b++) 
	//		StringOfMeasureH[b] = 0.0;
	//for (i=0 ; i<3 ; i++) 
	//{
	//	temp[i] =  (W_s[i] + W_s_prev[i])/2.0;
	//	StringOfMeasureH[6 + i] = 2.0 * temp[i];
	//}
	//measure = (temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] - U*U);
	////fprintf (ForHelp, "%.10f\n", measure);
	//double r = 0.3*3.141592/180.0/3600.0;//2.0*3.141592/180.0/3600;
	//f0b (measure, ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  r, 
	//						ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);               
	//	EquatingOfVectors (iM,	  ConditionVector_plus,   ConditionVector_minus);
	//	EquatingOfVectors (iM*iM, CovarianceMatrixS_plus, CovarianceMatrixS_minus);



	//for (b=0 ; b<iM ; b++) 
	//		StringOfMeasureH[b] = 0.0;
	//for (i=0 ; i<3 ; i++) 
	//{
	//	temp[i] = (MeasureS[i] + MeasureS_prev[i])/2.0;
	//	StringOfMeasureH[3 + i] =  2.0 * temp[i];
	//}
	//measure = (temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] - Gravity*Gravity);
	////fprintf (ForHelp, "%.10f  %.10f\n", MeasureS[2], MeasureS_prev[2]);
	//r = 0.05;
	//f0b (measure, ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  r, 
	//						ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);               
	//	EquatingOfVectors (iM,	  ConditionVector_plus,   ConditionVector_minus);
	//	EquatingOfVectors (iM*iM, CovarianceMatrixS_plus, CovarianceMatrixS_minus);



	//for (b=0 ; b<iM ; b++) 
	//		StringOfMeasureH[b] = 0.0;
	//for (i=0 ; i<3 ; i++) 
	//{
	//	temp[i] = (W_s[i] + W_s_prev[i])/2.0;
	//	StringOfMeasureH[3 + i] =  temp[i];
	//	temp[i] = (MeasureS[i] + MeasureS_prev[i])/2.0;
	//	StringOfMeasureH[6 + i] =  temp[i];
	//}
	//for(i=0; i<3; i++)
	//	temp[i] = (W_s[i]*MeasureS[i] + W_s_prev[i]*MeasureS_prev[i])/2.0;
	//measure = (temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] - Gravity*U*sin(Latitude));
	////fprintf (ForHelp, "%.10f\n", measure);
	//r = 0.005;
	//f0b (measure, ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  r, 
	//						ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);               
	//	EquatingOfVectors (iM,	  ConditionVector_plus,   ConditionVector_minus);
	//	EquatingOfVectors (iM*iM, CovarianceMatrixS_plus, CovarianceMatrixS_minus);
			 




	dgq0b (ConditionVector_plus,  CovarianceMatrixS_plus,  TransitionMatrixF, CovarianceMatrixNoise, ConditionVector_minus, CovarianceMatrixS_minus, iM, iMq);
		EquatingOfVectors (iM,	  ConditionVector_minus,   ConditionVector_plus);
		EquatingOfVectors (iM*iM, CovarianceMatrixS_minus, CovarianceMatrixS_plus);

	free(KalmFactor);
	free(temp);
}

void 	KalmanFilter_2 (int iMq, int iM, int  iMz, double timePlus, double *KalmanMatrixA, double *KalmanMatrixH, double *ConditionVector_minus, double *ConditionVector_plus, double *CovarianceMatrixS_minus, double *CovarianceMatrixS_plus, 
					 double *CovarianceMatrixNoise, double Gravity, double U, double *W_s, double *W_s_prev, double *MeasureS, double *MeasureX, double *TransitionMatrixF, double DeltaTime, double *StringOfMeasureH, double r_noise, 
					 double Latitude, double *A_x0s, FILE *ForHelp, double *MeasureS_prev, double N_Count)
{
	int j,b,i;
	double *KalmFactor = new double[iM];
	double *temp = new double[iM];
	double measure;

	NullingOfArray (iM, KalmFactor);

	//Формирование переходной матрицы Ф
    func (DeltaTime, KalmanMatrixA, TransitionMatrixF, iM);

	if (iM == 9)
	{
		for (j=0; j< 3; j++)    
		{
			for (b=0 ; b<iM ; b++) 
				StringOfMeasureH[b] = KalmanMatrixH[j*iM+b];
			
			//сама функция коррекции
			if (iM == 9)
				{
					f0b (MeasureX[j], ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  r_noise*r_noise/N_Count, 
									ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);
				}
				else if (iM == 12)
				{
					f0b (MeasureX[j], ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  0.1*0.1, 
									ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);
				}
                    
			EquatingOfVectors (iM,	  ConditionVector_plus,   ConditionVector_minus);
			EquatingOfVectors (iM*iM, CovarianceMatrixS_plus, CovarianceMatrixS_minus);
		}
	}



	for (b=0 ; b<iM ; b++) 
			StringOfMeasureH[b] = 0.0;
	for (i=0 ; i<3 ; i++) 
	{
		temp[i] =  (W_s[i] + W_s_prev[i])/2.0;
		StringOfMeasureH[6 + i] = 2.0 * temp[i];
	}
	measure = (temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] - U*U);
	//fprintf (ForHelp, "%.10f\n", measure);
	double r = 0.03*3.141592/180.0/3600.0/N_Count;//2.0*3.141592/180.0/3600;
	f0b (measure, ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  r, 
							ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);               
		EquatingOfVectors (iM,	  ConditionVector_plus,   ConditionVector_minus);
		EquatingOfVectors (iM*iM, CovarianceMatrixS_plus, CovarianceMatrixS_minus);



	for (b=0 ; b<iM ; b++) 
			StringOfMeasureH[b] = 0.0;
	for (i=0 ; i<3 ; i++) 
	{
		temp[i] = (MeasureS[i] + MeasureS_prev[i])/2.0;
		StringOfMeasureH[3 + i] =  2.0 * temp[i];
	}
	measure = (temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] - Gravity*Gravity);
	//fprintf (ForHelp, "%.10f  %.10f\n", MeasureS[2], MeasureS_prev[2]);
	r = 0.05/N_Count;
	f0b (measure, ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  r, 
							ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);               
		EquatingOfVectors (iM,	  ConditionVector_plus,   ConditionVector_minus);
		EquatingOfVectors (iM*iM, CovarianceMatrixS_plus, CovarianceMatrixS_minus);



	for (b=0 ; b<iM ; b++) 
			StringOfMeasureH[b] = 0.0;
	for (i=0 ; i<3 ; i++) 
	{
		temp[i] = (W_s[i] + W_s_prev[i])/2.0;
		StringOfMeasureH[3 + i] =  temp[i];
		temp[i] = (MeasureS[i] + MeasureS_prev[i])/2.0;
		StringOfMeasureH[6 + i] =  temp[i];
	}
	for(i=0; i<3; i++)
		temp[i] = (W_s[i]*MeasureS[i] + W_s_prev[i]*MeasureS_prev[i])/2.0;
	measure = (temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] - Gravity*U*sin(Latitude));
	//fprintf (ForHelp, "%.10f\n", measure);
	r = 0.005/N_Count;
	f0b (measure, ConditionVector_minus,          CovarianceMatrixS_minus,          StringOfMeasureH,  r, 
							ConditionVector_plus,			  CovarianceMatrixS_plus,			KalmFactor, iM);               
		EquatingOfVectors (iM,	  ConditionVector_plus,   ConditionVector_minus);
		EquatingOfVectors (iM*iM, CovarianceMatrixS_plus, CovarianceMatrixS_minus);
			 




	dgq0b (ConditionVector_plus,  CovarianceMatrixS_plus,  TransitionMatrixF, CovarianceMatrixNoise, ConditionVector_minus, CovarianceMatrixS_minus, iM, iMq);
		EquatingOfVectors (iM,	  ConditionVector_minus,   ConditionVector_plus);
		EquatingOfVectors (iM*iM, CovarianceMatrixS_minus, CovarianceMatrixS_plus);

	free(KalmFactor);
	free(temp);
}

