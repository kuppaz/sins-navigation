#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "function_lib.h"


#define RoughTime 20.0 //Задается длительность грубой выставки

int main ()
{	
	int i,j,b, Count=0, iM, iMz, iMq;
	double timePlus=0.0, timeMinus=0.0, *MeasureS, *MeasureX, Heading=.0, Roll=.0, Pitch=.0, *CovarianceMatrixNoise, *KalmanMatrixA, *KalmanMatrixH,
		   *StringOfMeasureH, *TransitionMatrixF, *ConditionVector_minus, *CovarianceMatrixS_minus, *ConditionVector_plus, *CovarianceMatrixS_plus, *KalmFactor,
		   r_noise, Latitude=.0, Gravity, U, Radian, Pi, *A_x0s, *W_s, LatitudeTRUE, GravityTRUE, *MeasureS_Average, *A_etaX0, *A_sKsi, *Temp3, HeadingCor = .0, *W_s_avg,
		   NoiseFactor, DeltaPitch=.0, DeltaRoll=.0, DeltaHeading=.0, *A_xs, *Temp1, DeltaPhi=.0, *A_sx0, *Temp2, N_Count=0.0, *W_s_Summ, *MeasureS_Summ, nCount =0, timeStep, *W_s_prev, *MeasureS_prev;

	double alpha_x = 0.1 * 3.141592 / 180.0;
	double alpha_y = 0.08  * 3.141592 / 180.0;
	double alpha_z = 0.46  * 3.141592 / 180.0;
	double *V = new double[3];
	NullingOfArray(3, V);

	double *temp = new double[3];
	W_s_avg = new double[3];

	FILE * many_Runs = fopen("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//many_Runs.txt", "w");
	FILE * deltaV_alignment = fopen("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//deltaV_alignment.txt", "w");
	FILE * many_Runs_by_500 = fopen("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//many_Runs_by_500.txt", "w");

	fprintf (deltaV_alignment, "Time  Betta1  Betta2  Betta3  DeltaMeasure1  DeltaMeasure2  DeltaMeasure3  Drift1  Drift2  Drift3 dV1 dV2 dV3 Heading HeadingCor RollCor PitchCor\n");

	char DataString[1024];

				/*************************************/
				/*************************************/
				/**/	 iM				   = 9;    /**/
				/**/	 iMz			   = 3;	   /**/
				/**/	 iMq			   = 3;	   /**/ //размерность ветора шумов динамической системы q
				/*************************************/
				/*************************************/

	NoiseFactor		 = 1e-7;			//Сам шум динамической системы 
    Pi				 = 4.0*atan(1.0);
    U				 = 0.000072921157;	//радиан в секунду
    Radian			 = Pi/180.0;
	r_noise			 = 0.003;			//Шум измерений
	Gravity = GravityTRUE		 = 9.816719;
	Latitude = LatitudeTRUE	 = 1.021;
	timeStep = 0.02048;
	double tt = 25000, time = 0.0;
	int o = 0;

//кусок кода инициализирующий переменные
#include "DeclarationOfVariable.h"

	//for (o=1; o < 3; o++)
	//{
	/*for (int t=1; t<250; t++)
	{ 
		timePlus=0.0; timeMinus=0.0; N_Count=0.0; nCount =0;
		tt = 490.0 + t*10.0;*/

		NullingOfArray(3, MeasureS_Summ);
		NullingOfArray(3, W_s_Summ);
		NullingOfArray(iM*iM, KalmanMatrixA);		       
	NullingOfArray(iM*iM, CovarianceMatrixS_plus);
    NullingOfArray(iM*iM, CovarianceMatrixS_minus);   
    NullingOfArray(iM*iM, TransitionMatrixF);	       
	NullingOfArray(iM, ConditionVector_plus);
    NullingOfArray(iM*iMz, KalmanMatrixH);
	NullingOfArray(iM, ConditionVector_minus);
	NullingOfArray(iM, KalmFactor);
	NullingOfArray(iM, StringOfMeasureH);
	NullingOfArray(3, MeasureS);
	NullingOfArray(3, MeasureX);
	NullingOfArray(3*3, A_x0s);
	NullingOfArray(3*3, A_sx0);
	NullingOfArray(3*3, A_xs);
	NullingOfArray(3, Temp1);
	NullingOfArray(3, Temp2);
	NullingOfArray(3, MeasureS_Summ);
	NullingOfArray(3, W_s_Summ);
	NullingOfArray(3, W_s);
	NullingOfArray(iM*iMq, CovarianceMatrixNoise);
	NullingOfArray(3, V);

//кусок кода, задающий ковариационные матрицы
#include "CovarianceMatrix.h"  

	
		RoughAligment (tt , Gravity ,&timePlus, MeasureS, W_s, MeasureS_Average, &N_Count, W_s_Summ, MeasureS_Summ, &Heading, &Roll, &Pitch, &Latitude, EntranceParameters, RoughTime, ExitParameters, W_s_avg, ForHelp);

		InitialOfMatrix (A_sx0, A_sKsi, Heading, Roll, Pitch, Latitude);

		Temp1[0] = 0.0;
		Temp1[1] = U*cos(Latitude);
		Temp1[2] = U*sin(Latitude);
		MatrixOnVector(3,3, A_sx0, Temp1, Temp2);

		printf ("%f   %f   %f\n", Temp2[0], Temp2[1], Temp2[2]);
		printf ("%f   %f   %f\n", W_s[0], W_s[1], W_s[2]);

		//тут небольшая проверка на то, не вышел ли курс за пределы 180 градусов
		if (fabs(Temp2[0] - W_s[0]) < 0.00001){}
		else
		{			
			Heading = Heading - Pi;
			InitialOfMatrix (A_sx0, A_sKsi, Heading, Roll, Pitch, Latitude);
		}

		MatrixOnVector(3,3, A_sx0, Temp1, Temp2);
		//вывод на экран угловых скоростей для определения, насколько правильно все получилось с грубой выставкой
		printf ("%f   %f   %f\n", Temp2[0], Temp2[1], Temp2[2]);
		printf ("%f   %f   %f\n", W_s[0], W_s[1], W_s[2]);

        A_x0s[0] =  A_sx0[0]; 
        A_x0s[3] =  A_sx0[1];
        A_x0s[6] =  A_sx0[2];									
        A_x0s[1] =  A_sx0[3];								
        A_x0s[4] =  A_sx0[4];									
        A_x0s[7] =  A_sx0[5];								
        A_x0s[2] =  A_sx0[6]; 
        A_x0s[5] =  A_sx0[7]; 
        A_x0s[8] =  A_sx0[8];									
		
		//Конец грубой выставки
		//system ("pause");	

/************************************************************MAIN CYCLE****************************************************************/
	timeMinus = timePlus;
	for (i=0; i<3; i++)
		{
			W_s_prev[i] = W_s[i];
			MeasureS_prev[i] = MeasureS[i];
		}
	nCount = N_Count;
	while (nCount < 25000)
	{
		//считывание из файла
        fgets  (DataString, 1024, EntranceParameters); 
		sscanf (DataString, "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &nCount, &MeasureS[1], &MeasureS[2], &MeasureS[0], &W_s[1], &W_s[2], &W_s[0]); 
		timePlus = nCount*timeStep;

		/*for (i=0; i<3; i++)
			MeasureS[i] = MeasureS[i]*9.81;*/

		//temp[1] = MeasureS[1] + alpha_z * MeasureS[2] + alpha_y * MeasureS[0];//-0.02;
  //      temp[2] = MeasureS[2] - alpha_z * MeasureS[1] + alpha_x * MeasureS[0];//+0.037;
  //      temp[0] = MeasureS[0] - alpha_y * MeasureS[1] - alpha_x * MeasureS[2];
		//EquatingOfVectors(3, temp, MeasureS);

       /* temp[1] = W_s[1] + alpha_z * W_s[2] + alpha_y * W_s[0];
        temp[2] = W_s[2] - alpha_z * W_s[1] + alpha_x * W_s[0];
        temp[0] = W_s[0] - alpha_y * W_s[1] - alpha_x * W_s[2];
		EquatingOfVectors(3, temp, W_s);*/


		N_Count = N_Count +1.0;
		for (int i=0; i<3; i++)
		{
			W_s_Summ[i] = W_s_Summ[i] + W_s[i];
			W_s_avg[i] = W_s_Summ[i]/(N_Count);
		}			

		for (int i=0; i<3; i++)
		{
			MeasureS_Summ[i] = MeasureS_Summ[i] + MeasureS[i];
			MeasureS_Average[i] = MeasureS_Summ[i]/(N_Count);
		}		

		IntegrationOfOrientationMatrix  (A_sx0, W_s, W_s_prev, timePlus-timeMinus, Latitude, U);

#include "DefinitionOfMatrix.h"


		MatrixOnVector (3,3, A_xs, Temp2, Temp1);
		MatrixOnVector (3,3, A_x0s, MeasureS, MeasureX);
		MeasureX[2] = MeasureX[2] - Gravity;  

		if (iM == 12)
		{
			V[0] += MeasureX[0]*timeStep;
			V[1] += MeasureX[1]*timeStep;
			V[2] += MeasureX[2]*timeStep;
			for (i =0; i< 3 ; i++)
				MeasureX[i] = V[i];
		}


		if ((nCount< tt+2 || nCount > 39999) && o == 1) //(nCount < tt+2 || (((int)N_Count % 500) == 0 && nCount > tt+500))//(nCount< tt+2 || nCount > 39998)
		{
			/*MatrixOnVector (3,3, A_xs, Temp2, Temp1);
			MatrixOnVector (3,3, A_x0s, MeasureS_Average, MeasureX);
			MeasureX[2] = MeasureX[2] - Gravity;  */
			KalmanFilter_2 (iMq, iM, iMz, timePlus, KalmanMatrixA, KalmanMatrixH, ConditionVector_minus, ConditionVector_plus, CovarianceMatrixS_minus, CovarianceMatrixS_plus, 
							 CovarianceMatrixNoise, Gravity, U, W_s_avg, W_s_avg, MeasureS_Average, MeasureX, TransitionMatrixF, timePlus - timeMinus, StringOfMeasureH, r_noise, Latitude, A_x0s, ForHelp, MeasureS_Average, N_Count);
			N_Count=0;
			NullingOfArray(3, MeasureS_Summ);
			NullingOfArray(3, W_s_Summ);
			//fprintf(ForHelp, "%f %f %f %f %f %f %f\n", nCount, MeasureS_Average[0], MeasureS_Average[1], MeasureS_Average[2], W_s_avg[0], W_s_avg[1], W_s_avg[2]);
		}
		else if((nCount < tt+2 || (((int)N_Count % 500) == 0 && nCount > tt+500)) && o == 2)
		{
			MatrixOnVector (3,3, A_xs, Temp2, Temp1);
			MatrixOnVector (3,3, A_x0s, MeasureS_Average, MeasureX);
			MeasureX[2] = MeasureX[2] - Gravity;  
			KalmanFilter_2 (iMq, iM, iMz, timePlus, KalmanMatrixA, KalmanMatrixH, ConditionVector_minus, ConditionVector_plus, CovarianceMatrixS_minus, CovarianceMatrixS_plus, 
							 CovarianceMatrixNoise, Gravity, U, W_s_avg, W_s_avg, MeasureS_Average, MeasureX, TransitionMatrixF, timePlus - timeMinus, StringOfMeasureH, r_noise, Latitude, A_x0s, ForHelp, MeasureS_Average, N_Count);
			N_Count=0;
			NullingOfArray(3, MeasureS_Summ);
			NullingOfArray(3, W_s_Summ);
			//fprintf(ForHelp, "%f %f %f %f %f %f %f\n", nCount, MeasureS_Average[0], MeasureS_Average[1], MeasureS_Average[2], W_s_avg[0], W_s_avg[1], W_s_avg[2]);
		}
		else
		{
			/*MatrixOnVector (3,3, A_xs, Temp2, Temp1);
			MatrixOnVector (3,3, A_x0s, MeasureS, MeasureX);
			MeasureX[2] = MeasureX[2] - Gravity;  */
			KalmanFilter (iMq, iM, iMz, timePlus, KalmanMatrixA, KalmanMatrixH, ConditionVector_minus, ConditionVector_plus, CovarianceMatrixS_minus, CovarianceMatrixS_plus, 
							 CovarianceMatrixNoise, Gravity, U, W_s, W_s_prev, MeasureS, MeasureX, TransitionMatrixF, timePlus - timeMinus, StringOfMeasureH, r_noise, Latitude, A_x0s, ForHelp, MeasureS_prev, N_Count);
		}




		Heading = atan(A_sx0[3]/A_sx0[4]);
		Roll = -atan(A_sx0[2]/A_sx0[8]);
		Pitch = asin(A_sx0[5]);
		DeltaPitch   = -ConditionVector_plus[0]*cos(Heading) + ConditionVector_plus[1]*sin(Heading);
		DeltaRoll    =  (-ConditionVector_plus[0]*sin(Heading) - ConditionVector_plus[1]*cos(Heading))/cos(Pitch);
		DeltaHeading =  ConditionVector_plus[2] + (DeltaRoll)*sin(Pitch);
		HeadingCor = Heading - DeltaHeading;
		Roll = Roll - DeltaRoll;
		Pitch = Pitch - DeltaPitch;

		if (iM == 12)
			fprintf (ForHelp,"%f  %.12f  %.12f  %.12f  %.12f   %.12f  %.12f  %.12f   %.12f  %.12f  %.12f  %.12f  %.12f  %.12f \n", timePlus, 
								ConditionVector_plus[0]*60.0/Radian,   ConditionVector_plus[1]*60.0/Radian, ConditionVector_plus[2]*60.0/Radian, 
								ConditionVector_plus[3],			   ConditionVector_plus[4],			    ConditionVector_plus[5],
								ConditionVector_plus[6],			   ConditionVector_plus[7],			    ConditionVector_plus[8], Heading*180.0/Pi, HeadingCor*180.0/Pi, Roll*180.0/Pi, Pitch*180.0/Pi);
		else if (iM == 9)
			fprintf (FilterOut,"%f  %.12f  %.12f  %.12f  %.12f   %.12f  %.12f  %.12f   %.12f  %.12f  %.12f  %.12f  %.12f  %.12f \n", timePlus, 
								ConditionVector_plus[0]*60.0/Radian,   ConditionVector_plus[1]*60.0/Radian, ConditionVector_plus[2]*60.0/Radian, 
								ConditionVector_plus[3],			   ConditionVector_plus[4],			    ConditionVector_plus[5],
								ConditionVector_plus[6],			   ConditionVector_plus[7],			    ConditionVector_plus[8], Heading*180.0/Pi, HeadingCor*180.0/Pi, Roll*180.0/Pi, Pitch*180.0/Pi);
		

		printf ("Time = %.4f,  A(1)= %.4f,  A(2)= %.4f,  A(3)= %.4f\n", timePlus, ConditionVector_plus[0]*60.0/Radian, ConditionVector_plus[1]*60.0/Radian, ConditionVector_plus[2]*60.0/Radian);
	
		timeMinus = timePlus;
		for (i=0; i<3; i++)
		{
			W_s_prev[i] = W_s[i];
			MeasureS_prev[i] = MeasureS[i];
		}
	}

	fclose(EntranceParameters);
	fclose(ExitParameters);
	fclose(FilterOut);
	fclose(ForHelp);
	/*if (o==1)
		fprintf (deltaV_alignment, "%f\t %.12f\t %.12f\t %.12f\t%.12f\t %.12f\t %.12f\t%.12f\t %.12f\t%.12f \t%.12f \t%.12f \t%.12f \t%.12f %f  %f  %f \n", tt, 
								ConditionVector_plus[0]*60.0/Radian,   ConditionVector_plus[1]*60.0/Radian, ConditionVector_plus[2]*60.0/Radian, 
								ConditionVector_plus[3],			   ConditionVector_plus[4],			    ConditionVector_plus[5],
								ConditionVector_plus[6],			   ConditionVector_plus[7],			    ConditionVector_plus[8],
								ConditionVector_plus[9],			   ConditionVector_plus[10],			    ConditionVector_plus[11], Heading*180.0/Pi, HeadingCor*180.0/Pi, Roll*180.0/Pi, Pitch*180.0/Pi);

	if (o==2)
		fprintf (many_Runs_by_500, "%f\t %.12f\t %.12f\t %.12f\t%.12f\t %.12f\t %.12f\t%.12f\t %.12f\t%.12f \t%.12f \t%.12f \t%.12f \t%.12f\n", tt, 
								ConditionVector_plus[0]*60.0/Radian,   ConditionVector_plus[1]*60.0/Radian, ConditionVector_plus[2]*60.0/Radian, 
								ConditionVector_plus[3],			   ConditionVector_plus[4],			    ConditionVector_plus[5],
								ConditionVector_plus[6],			   ConditionVector_plus[7],			    ConditionVector_plus[8], Heading*180.0/Pi, HeadingCor*180.0/Pi, Roll*180.0/Pi, Pitch*180.0/Pi);
*/
	/*if (t == 3)
		return 0;*/
	/*}*/

	//}
}