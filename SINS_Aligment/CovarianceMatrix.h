           CovarianceMatrixS_minus[0]  = CovarianceMatrixS_plus[0]  = 5*Radian/60;
           CovarianceMatrixS_minus[iM*1+1] = CovarianceMatrixS_plus[iM*1+1] = 5*Radian/60;
           CovarianceMatrixS_minus[iM*2+2] = CovarianceMatrixS_plus[iM*2+2] = 5*Radian/60;
           
           CovarianceMatrixS_minus[iM*3+3]  = CovarianceMatrixS_plus[iM*3+3] = 0.001;
           CovarianceMatrixS_minus[iM*4+4] = CovarianceMatrixS_plus[iM*4+4] = 0.001;
           CovarianceMatrixS_minus[iM*5+5] = CovarianceMatrixS_plus[iM*5+5] = 0.001;
			
           CovarianceMatrixS_minus[iM*6+6] = CovarianceMatrixS_plus[iM*6+6] = 0.2*Radian/3600.0;
           CovarianceMatrixS_minus[iM*7+7] = CovarianceMatrixS_plus[iM*7+7] = 0.2*Radian/3600.0;
		   CovarianceMatrixS_minus[iM*8+8] = CovarianceMatrixS_plus[iM*8+8] = 0.2*Radian/3600.0;

		   if (iM == 12)
		   {
			   CovarianceMatrixS_minus[iM*9+9] = CovarianceMatrixS_plus[iM*9+9] = 0.001;
				CovarianceMatrixS_minus[iM*10+10] = CovarianceMatrixS_plus[iM*10+10] = 0.001;
				CovarianceMatrixS_minus[iM*11+11] = CovarianceMatrixS_plus[iM*11+11] = 0.001;
		   }

	FILE  *ExitParameters;
	ExitParameters = fopen("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//DataFile.txt", "w");
	
	FILE    *EntranceParameters;
	EntranceParameters = fopen("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//AzimutB_210530_Other_120814_Autolab_10-31-26_2.dat", "r");

	FILE    *FilterOut;
		     FilterOut = fopen("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output_Alignment.dat", "w");
	FILE    *ForHelp;
		     ForHelp = fopen("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Alignment_ForHelp.dat", "w");
	
	fprintf (FilterOut, "Time  Betta1  Betta2  Betta3  DeltaMeasure1  DeltaMeasure2  DeltaMeasure3  Drift1  Drift2  Drift3\n");
