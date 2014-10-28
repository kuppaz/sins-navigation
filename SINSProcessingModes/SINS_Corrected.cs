using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace SINSProcessingModes
{
    public class SINS_Corrected
    {
        public static StreamWriter Nav_FeedbackSolution;
        public static StreamWriter Nav_EstimateSolution;
        public static StreamWriter Nav_Errors;
        public static StreamWriter Nav_Autonomous;
        public static StreamWriter Nav_StateErrorsVector;
        public static StreamWriter Nav_Smoothed;

        public static StreamWriter ForHelpSmoothed;
        public static StreamWriter ForHelp;
        public static StreamWriter ForHelp_2 = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//ForHelp_2.txt");
        public static StreamWriter SlippageLog = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//SlippageLog.txt");
        public static StreamWriter Nav_vert_chan_test = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Nav_vert_chan_test.txt");
        public static StreamWriter Dif_GK = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Dif_GK.txt");
        public static StreamWriter KMLFileOut;
        public static StreamWriter KMLFileOutSmthd;


        public static StreamWriter Dif_GK_SM = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Dif_GK_SM.txt");
        public static StreamWriter STD_data = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_STD.txt");
        public static StreamWriter Kinematic_solution = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_Kinem.txt");
        public static StreamWriter Speed_Angles = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Speed_Angles.txt");
        public static StreamWriter DinamicOdometer = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//DinamicOdometer.txt");


        public static StreamWriter Smthing_Backward;
        public static StreamWriter Smthing_P;
        public static StreamWriter Smthing_X;
        public static StreamReader Back_Input_File_read, Back_Input_X, Back_Input_P;
        public static int NumberOfIterationForOneForSmoothing = 500000;


        //testes//testes

        public static void SINS_Corrected_Processing(int l, bool Do_Smoothing, StreamReader myFile, SINS_State SINSstate, SINS_State SINSstate2, Kalman_Vars KalmanVars, Proc_Help ProcHelp, SINS_State SINSstate_OdoMod, ParamsForModel OdoModel)
        {
            int t = 0;

            StreamWriter Imitator_Telemetric = new StreamWriter("D://SINS Solution//Imitator_Kompas_temp//Work//DataForImitator//Imitator_" + SINSstate.Global_file + ".dat");


            double lambda_last_odo_flg = SINSstate2.Longitude, phi_last_odo_flg = SINSstate2.Latitude;
            SINS_State SINSstate_Smooth = new SINS_State();

            SINS_State SINSstateDinamOdo = SINS_State.DeepCopy(SINSstate);

            //--- Смотрим, если выставлен режим модифицированных слабо связанных систем, то вмешиваемся в алгоритм БИНС. Если нет, то нет ---//
            SINSstateDinamOdo.flag_Autonomous_Solution = true;
            SINSstateDinamOdo.flag_autonomous_dinamic_mode = true;
            //--- ---//


            SINSstate.NumberOfFilesForSmoothing = Math.Floor(SINSstate.LastCountForRead / Convert.ToDouble(NumberOfIterationForOneForSmoothing)) + 1;

            if (!Do_Smoothing)
            {
                Nav_FeedbackSolution = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_SlnFB.txt");
                Nav_EstimateSolution = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_SlnEst.txt");
                Nav_Errors = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_Errs.txt");
                Nav_Autonomous = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_Auto.txt");
                Nav_StateErrorsVector = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_ErrVct.txt");
                Nav_Smoothed = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_smoothed_SlnFB.txt");
                ForHelp = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//ForHelp.txt");
                KMLFileOut = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//KMLFileOut_Forward.kml");
                KMLFileOutSmthd = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//KMLFileOut_Smoothed.kml");

                FillKMLOutputFile(KMLFileOut, "Start", "Forward");

                string str = "count  dr1 dr2 dV1 dV2 Alpha1_grad Alpha2_grad Beta3_grad Nu_1_grad Nu_2_grad/h Nu_3_grad/h dF_1 dF_2 dF_3";
                if (SINSstate.flag_iMx_r3_dV3) str = str + " dr3 dV3";
                if (SINSstate.flag_Odometr_SINS_case) str = str + " odo_dr1 odo_dr2";
                if (SINSstate.flag_Odometr_SINS_case && SINSstate.flag_Using_iMx_r_odo_3) str = str + " odo_dr3";
                if (SINSstate.flag_iMx_kappa_13_ds) str = str + " kappa1_grad kappa3_grad Scale";
                Nav_StateErrorsVector.WriteLine(str);
            }
            else
            {
                SINSstate.odotime_prev = SINSstate.Time+ SINSstate.timeStep;

                Back_Input_File_read = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_full.txt");
                Back_Input_X = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_full_X.txt");
                Back_Input_P = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_full_P.txt");
                ForHelp = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//ForHelp_back.txt");
                ForHelpSmoothed = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//ForHelp_Smoothed.txt");

                Nav_Smoothed = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_smoothed_SlnFB.txt");
                Nav_FeedbackSolution = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_back_SlnFB.txt");
                Nav_EstimateSolution = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_back_SlnEst.txt");
                Nav_Errors = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_back_Errs.txt");
                Nav_Autonomous = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_back_Auto.txt");
                Nav_StateErrorsVector = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//S_back_ErrVct.txt");
                KMLFileOut = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//KMLFileOut_Back.kml");
                KMLFileOutSmthd = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//KMLFileOut_Smoothed.kml");

                Nav_Smoothed.WriteLine("time  count LatRelStart  LongRelStart Altitude Latitude  Longitude LatSNS-Lat LngSNS-Lng AltSNS  SpeedSNS  V_x1  V_x2  V_x3  Yaw  Roll  Pitch ");

                FillKMLOutputFile(KMLFileOut, "Start", "Backward");
            }

            FillKMLOutputFile(KMLFileOutSmthd, "Start", "Smoothing");
            Nav_FeedbackSolution.WriteLine("time  count  OdoCnt  OdoV  LatRelStart  LongRelStart Altitude Latitude  Longitude LatSNS-Lat LngSNS-Lng AltSNS  SpeedSNS  V_x1  V_x2  V_x3  Yaw  Roll  Pitch Correct PositError PositErrStart");
            Nav_EstimateSolution.WriteLine("time  count  OdoCnt  OdoV  LatRelStart  LongRelStart Altitude Latitude  Longitude LatSNS-Lat LngSNS-Lng AltSNS  SpeedSNS V_x1  V_x2  V_x3  Correct  Yaw YawCor  Roll RollCor  Pitch PitchCor PositError V_abs");
            Nav_Errors.WriteLine("dLat  dLong  dV_x1  dV_x2  dV_x3  dHeading  dRoll  dPitch");
            DinamicOdometer.WriteLine("Time Count OdoTimeStepCount AbsOdoSpeed_x0 LatRelStart LongRelStart Altitude Altitude_Corr LatRelStartCor-ed LongRelStartCor-ed Latitude  Longitude LatSNS-Lat LngSNS-Lng AltSNS  SpeedSNS  V_x1  V_x2  V_x3 Yaw  Roll  Pitch");



            int temp_cnt_V_more = 0;
            int start_i = 0;
            start_i = l;

            if (Do_Smoothing) start_i = SINSstate.LastCountForRead - 1;


            //=========================================================================//
            //=========================================================================//
            for (int i = start_i; i <= SINSstate.LastCountForRead; i++)
            {
                if (Do_Smoothing) { i = i - 2; if (i < l + 1) break; }

                SINSstate.Do_Smoothing = Do_Smoothing;

                if (SINSstate.flag_UsingClasAlignment == false)
                    if (i < ProcHelp.AlgnCnt)
                    {
                        ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);
                        continue;
                    }

                if (!Do_Smoothing)
                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);
                if (Do_Smoothing)
                {
                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, Back_Input_File_read, SINSstate, SINSstate_OdoMod);
                    SINSstate.timeStep = -Math.Abs(SINSstate.timeStep);
                }


                ProcessingHelp.DefSNSData(ProcHelp, SINSstate);
                SINSstate2.Time = SINSstate.Time;



                //---------------------------------------------------------------------//
                SINSstateDinamOdo.Time = SINSstate.Time;
                SINSstateDinamOdo.timeStep = SINSstateDinamOdo.Freq = SINSstate.timeStep;
                SINSstateDinamOdo.OdometerData.odometer_left.isReady = SINSstate.OdometerData.odometer_left.isReady;
                SimpleOperations.CopyArray(SINSstateDinamOdo.F_z, SINSstate.F_z);
                SimpleOperations.CopyArray(SINSstateDinamOdo.W_z, SINSstate.W_z);

                SINSstate.OdoTimeStepCount++;
                if (SINSstate.Global_file == "Saratov_run_2014_07_23" || SINSstate.Global_file == "Saratov_run_2014_07_23_middle_interval_GPS")
                    SINSstate.OdoTimeStepCount = (SINSstate.Time - SINSstate.odotime_prev) / SINSstate.timeStep;
                SINSstateDinamOdo.OdoTimeStepCount = SINSstate.OdoTimeStepCount;

                //===Проверка на всякий===
                if (SINSstate.OdoTimeStepCount == 0.0 && SINSstate.OdometerData.odometer_left.isReady == 1)
                    SINSstate.OdoTimeStepCount = SINSstate.OdoTimeStepCount;

                //--- Формируем вектора измерений одометра для основной и одометрической копии ---//
                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    SINSstate.OdoSpeed_s = SimpleOperations.NullingOfArray(SINSstate.OdoSpeed_s);
                    SINSstate.OdoSpeed_x0 = SimpleOperations.NullingOfArray(SINSstate.OdoSpeed_x0);
                    SINSstate.OdometerVector = SimpleOperations.NullingOfArray(SINSstate.OdometerVector);
                    SINSstateDinamOdo.OdoSpeed_s = SimpleOperations.NullingOfArray(SINSstateDinamOdo.OdoSpeed_s);
                    SINSstateDinamOdo.OdoSpeed_x0 = SimpleOperations.NullingOfArray(SINSstateDinamOdo.OdoSpeed_x0);
                    SINSstateDinamOdo.OdometerVector = SimpleOperations.NullingOfArray(SINSstateDinamOdo.OdometerVector);

                    SINSstate.OdometerVector[1] = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;

                    if (SINSstate.OdometerVector[1] > 0.01)
                        SINSstate.OdometerVector[1] = SINSstate.OdometerVector[1];
                    SINSstate.OdoSpeed_s[1] = SINSstate.OdometerVector[1] / SINSstate.OdoTimeStepCount / SINSstate.timeStep;
                    SINSstateDinamOdo.OdometerVector[1] = SINSstate.OdometerVector[1];
                    SINSstateDinamOdo.OdoSpeed_s[1] = SINSstateDinamOdo.OdometerVector[1] / SINSstate.OdoTimeStepCount / SINSstate.timeStep;

                    //--- Если обратные связи, то сразу корректируем измерение одометра по честной оценке ---//
                    if (SINSstate.flag_FeedbackExist && SINSstate.flag_iMx_kappa_13_ds)
                    {
                        SimpleOperations.CopyArray(SINSstate.OdoSpeed_s, (Matrix.UnitMatrix(3) - Matrix.SkewSymmetricMatrix(SINSstate.ComulativeKappaEst)) / (1.0 + SINSstate.ComulativeKappaEst[1]) * SINSstate.OdoSpeed_s);
                        SimpleOperations.CopyArray(SINSstateDinamOdo.OdoSpeed_s, SINSstate.OdoSpeed_s);
                        SimpleOperations.CopyArray(SINSstate.OdometerVector, (Matrix.UnitMatrix(3) - Matrix.SkewSymmetricMatrix(SINSstate.ComulativeKappaEst)) / (1.0 + SINSstate.ComulativeKappaEst[1]) * SINSstate.OdometerVector);
                        SimpleOperations.CopyArray(SINSstateDinamOdo.OdometerVector, SINSstate.OdometerVector);
                    }
                }




                if (t == 0)
                {
                    if (!Do_Smoothing)
                    {
                        start_i = i;
                        SINSstate.Roll_prev = SINSstate.Roll;
                        SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
                        SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
                        SimpleOperations.CopyArray(SINSstateDinamOdo.F_z_prev, SINSstateDinamOdo.F_z);
                        SimpleOperations.CopyArray(SINSstateDinamOdo.W_z_prev, SINSstateDinamOdo.W_z);
                        SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                        SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                        SINSstateDinamOdo.OdometerLeftPrev = SINSstateDinamOdo.OdometerData.odometer_left.Value;
                        SINSstateDinamOdo.OdometerRightPrev = SINSstateDinamOdo.OdometerData.odometer_right.Value;
                    }
                    if (Do_Smoothing)
                        i = i + 2;
                    t = 1;
                }





                //---------------------------------------MAIN STEPS----------------------------------------------------
                SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate_OdoMod);
                SINSprocessing.StateIntegration_AT(SINSstateDinamOdo, KalmanVars, SINSstate2, SINSstate_OdoMod);

                if (SINSstate.flag_Odometr_SINS_case == true)
                {
                    //SimpleOperations.PrintMatrixToFile(KalmanVars.Matrix_A, SimpleData.iMx, SimpleData.iMx);

                    if (SINSstate.flag_FeedbackExist == true && SINSstate.flag_EstimateExist == false)
                        Odometr_SINS.Make_A_new(SINSstate, KalmanVars, SINSstate_OdoMod, SINSstateDinamOdo);
                    else if (SINSstate.flag_FeedbackExist == false && SINSstate.flag_EstimateExist == true)
                        Odometr_SINS.Make_A_new(SINSstate2, KalmanVars, SINSstate_OdoMod, SINSstateDinamOdo);
                    else //--- Это случай, когда автономное решение и по углам от него строится решение по одометру ---//
                        Odometr_SINS.Make_A_new(SINSstateDinamOdo, KalmanVars, SINSstate_OdoMod, SINSstateDinamOdo);

                    Odometr_SINS.MatrixNoise_ReDef(SINSstate, KalmanVars, SINSstate.flag_Alignment);
                }
                else
                {
                    if (SINSstate.flag_FeedbackExist == true)
                        SINSprocessing.Make_A(SINSstate, KalmanVars, SINSstate_OdoMod);
                    if (SINSstate.flag_FeedbackExist == false)
                        SINSprocessing.Make_A(SINSstate2, KalmanVars, SINSstate_OdoMod);

                    SINSprocessing.MatrixNoise_ReDef(SINSstate, KalmanVars, SINSstate.flag_Alignment);
                }
                KalmanProcs.Make_F(SINSstate.timeStep, KalmanVars);
                KalmanProcs.KalmanForecast(KalmanVars);
                //---------------------------------------MAIN STEPS----------------------------------------------------





                if (SINSstate.Use_Each_Odo_Measure == true)
                    SINSstate.flag_UsingOdoVelocity = true;

                //---------------Флаг коррекции по одометру---------------//
                if (SINSstate.flag_UseOnlyStops == false && SINSstate.flag_Odometr_SINS_case == false || SINSstate.flag_Odometr_SINS_case == true && (SINSstate.flag_UsingOdoVelocity || SINSstate.Use_Odo_Distance))
                {
                    if (SINSstate.Use_Each_Odo_Measure == true)
                        ModelsOfOdoCorrection.Model_Each_Odo_Measure(SINSstate, SINSstate_OdoMod, OdoModel, ForHelp);
                    else if (SINSstate.Use_Odo_Distance == true)
                        ModelsOfOdoCorrection.Model_With_Odo_Equations(SINSstate, SINSstate_OdoMod, KalmanVars, OdoModel, ForHelp);

                    //--- Считаем весовые матрицы ---//
                    if (SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        double[] d_2 = new double[3];

                        for (int u = 0; u < 3; u++) d_2[u] = SINSstate.A_x0s[u, 1];
                        SimpleOperations.CopyMatrix(SINSstate.Ds_ComulativeByOdoTrack, SINSstate.Ds_ComulativeByOdoTrack + SINSstate.OdometerVector[1] * SINSstate.A_x0s);
                        SimpleOperations.CopyMatrix(SINSstate.Ds2_ComulativeByOdoTrack, SINSstate.Ds2_ComulativeByOdoTrack + SINSstate.OdometerVector[1] * Matrix.SkewSymmetricMatrix(d_2));

                        for (int u = 0; u < 3; u++) d_2[u] = SINSstateDinamOdo.A_x0s[u, 1];
                        SimpleOperations.CopyMatrix(SINSstateDinamOdo.Ds_ComulativeByOdoTrack, SINSstateDinamOdo.Ds_ComulativeByOdoTrack + SINSstate.OdometerVector[1] * SINSstateDinamOdo.A_x0s);
                        SimpleOperations.CopyMatrix(SINSstateDinamOdo.Ds2_ComulativeByOdoTrack, SINSstateDinamOdo.Ds2_ComulativeByOdoTrack + SINSstate.OdometerVector[1] * Matrix.SkewSymmetricMatrix(d_2));
                    }
                    // --- -------------------------------------------- //

                    //--- Датчик проскальзывания как разность текущей скорректированной и скорости по одометру ---//
                    if (SINSstate.flag_using_slippage == true && SINSstate.flag_UsingOdoVelocity == true)
                        SINSprocessing.SlipageProcessing(SINSstate, SINSstate2, KalmanVars, SlippageLog, temp_cnt_V_more);
                }
                else
                {
                    SINSstate.flag_UsingCorrection = false;
                    SINSstate.OdoTimeStepCount++;
                }
                //-----------------------END----------------------------------
                //-----------------------------------------------------------------




                //---------------Формирование флага остановки------------
                if (SINSstate.FLG_Stop == 1 && SINSstate.flag_not_use_kns == false && SINSstate.flag_Odometr_SINS_case == false)
                {
                    SINSstate.flag_KNS = true;
                    SINSstate.flag_UsingCorrection = true;
                }
                else if (SINSstate.FLG_Stop == 1 && SINSstate.flag_not_use_kns == false && SINSstate.flag_Odometr_SINS_case == true && (SINSstate.flag_UsingOdoVelocity || SINSstate.Use_Odo_Distance))
                {
                    SINSstate.flag_KNS = true;
                    SINSstate.flag_UsingCorrection = true;
                }
                else
                    SINSstate.flag_KNS = false;




                //---------------------------------------------------------------------------------------------------------------------------------------
                //----------------------------ЭТАП КОРРЕКЦИИ start-----------------------------------------------------------------------------------------
                KalmanVars.cnt_measures = 0;
                for (int u = 0; u < SimpleData.iMx * SimpleData.iMz; u++) KalmanVars.Matrix_H[u] = 0.0;

                if (SINSstate.flag_using_Checkpotints == true)
                    CheckPointProcessing(SINSstate, SINSstateDinamOdo, SINSstate_OdoMod, KalmanVars);

                //-------------------------------------------------
                ProcHelp.corrected = 0;
                if (SINSstate.flag_UsingCorrection == true || (SINSstate.GPS_Data.gps_Altitude.isReady == 1 && SINSstate.flag_Using_SNS == true))
                {
                    for (int u = 1; u < SINSstate.Heading_Array.Length; u++) SINSstate.Heading_Array[u - 1] = SINSstate.Heading_Array[u];
                    SINSstate.Heading_Array[SINSstate.Heading_Array.Length - 1] = SINSstate.Heading;

                    //===КОРРЕКЦИЯ В СЛУЧАЕ БИНС+ ОДОМЕТР===//
                    if (SINSstate.flag_Odometr_SINS_case == false && SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        if (SINSstate.flag_UsingOdoPosition == true && SINSstate.flag_KNS == false)
                            CorrectionModel.Make_H_POSITION(KalmanVars, SINSstate, SINSstate_OdoMod, ProcHelp);

                        if (SINSstate.flag_UsingOdoVelocity == true && SINSstate.flag_KNS == false && SINSstate.flag_UsingOdoPosition == false && SINSstate.flag_Using_SNS == false)
                        {
                            if (SINSstate.flag_UsingScalarOdoMeasure == true && Math.Abs(SINSstate.Heading_Array[19] - SINSstate.Heading_Array[1]) > 3.0 * SimpleData.ToRadian)
                                CorrectionModel.Make_H_VELOCITY_Scalar(KalmanVars, SINSstate, SINSstate_OdoMod);
                            else
                                CorrectionModel.Make_H_VELOCITY(KalmanVars, SINSstate, SINSstate_OdoMod);
                        }
                    }
                    //===КОРРЕКЦИЯ В СЛУЧАЕ ОДОМЕТР + БИНС===//
                    else if (SINSstate.flag_Odometr_SINS_case == true && SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        if (SINSstate.flag_UsingOdoPosition == true && SINSstate.add_velocity_to_position == false && SINSstate.flag_KNS == false)
                        {
                            //---Если корректировать по измерениям, полученным в прямом проходе, то сглаженное решение будет стремиться именно к последнему---
                            Odometr_SINS.Make_H_POSITION(KalmanVars, SINSstate, SINSstateDinamOdo, ProcHelp);
                        }

                        if (SINSstate.add_velocity_to_position == true && SINSstate.flag_KNS == false)
                            Odometr_SINS.Make_H_VELOCITY(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstateDinamOdo);
                    }

                    //===SNS коррекция===//
                    if (SINSstate.flag_Using_SNS == true)
                        CorrectionModel.Make_H_GPS(KalmanVars, SINSstate, SINSstate_OdoMod);

                    //===ZUPT коррекция===//
                    if (SINSstate.flag_KNS == true)
                        CorrectionModel.Make_H_KNS(KalmanVars, SINSstate, SINSstate_OdoMod);

                    KalmanProcs.KalmanCorrection(KalmanVars);
                    ProcHelp.corrected = 1;
                }
                //----------------------------ЭТАП КОРРЕКЦИИ END---------------------------------
                //--------------------------------------------------------------------------------




                //====Вывод данных для телеметрического имитатора====
                if (SINSstate.flag_Imitator_Telemetric && SINSstate.Global_file != "Azimut-T_18-Oct-2013_11-05-11" && SINSstate.Global_file != "Saratov_run_2014_07_23" && SINSstate.Global_file != "Saratov_run_2014_07_23_middle_interval_GPS")
                {
                    double tmpRoundFreq50 = Math.Round(1.0 / SINSstate.Freq / 50.0, 0) * 50.0;
                    int tmpFreqOut10 = Convert.ToInt32(tmpRoundFreq50 / 10.0);

                    if ((i - l) % tmpFreqOut10 == 0)
                    {
                        if (SINSstate.flag_FeedbackExist)
                            Imitator_Telemetric.WriteLine(((i - l + tmpFreqOut10) / tmpRoundFreq50).ToString()
                                    + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Altitude
                                    + " " + (SINSstate.Heading * SimpleData.ToDegree) + " " + (SINSstate.Roll * SimpleData.ToDegree) + " " + (SINSstate.Pitch * SimpleData.ToDegree));
                        else if (SINSstate.flag_EstimateExist)
                            Imitator_Telemetric.WriteLine(((i - l + tmpFreqOut10) / tmpRoundFreq50).ToString()
                                    + " " + SINSstate2.Latitude + " " + SINSstate2.Longitude + " " + SINSstate2.Altitude
                                    + " " + (SINSstate2.Heading * SimpleData.ToDegree) + " " + (SINSstate2.Roll * SimpleData.ToDegree) + " " + (SINSstate2.Pitch * SimpleData.ToDegree));
                    }
                }





                if (SINSstate.flag_ControlPointCorrection)
                {
                    ForHelp.WriteLine(Math.Round(SINSstate.Time + SINSstate.Time_Alignment, 4) + " " + Math.Round(SINSstate.OdometerData.odometer_left.Value * 1000 + 3155, 0)
                        + " " + Math.Round(SINSstate.GPS_Data.gps_Latitude.Value * SimpleData.ToDegree, 8) + " " + Math.Round(SINSstate.GPS_Data.gps_Longitude.Value * SimpleData.ToDegree, 8) + " " + SINSstate.GPS_Data.gps_Altitude.Value
                        + " " + Math.Round(Math.Sqrt(Math.Pow(KalmanVars.ErrorConditionVector_p[0], 2) + Math.Pow(KalmanVars.ErrorConditionVector_p[1], 2)), 2)
                        + " " + SimpleOperations.CalculateDistanceBtwDots(SINSstate.GPS_Data.gps_Latitude_prev.Value, SINSstate.GPS_Data.gps_Longitude_prev.Value, SINSstate.GPS_Data.gps_Altitude_prev.Value,
                                        SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value) / (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerData.odometer_left_prev.Value));
                }

                //--- Расчет корректирующего вектора состояния ---
                SINSprocessing.CalcStateErrors(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate_OdoMod, SINSstateDinamOdo);
                if (SINSstate.flag_EstimateExist == true) SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2, SINSstate_OdoMod, SINSstateDinamOdo);
                if (SINSstate.flag_FeedbackExist == true) SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate, SINSstate_OdoMod, SINSstateDinamOdo);

                /*----------------------------------------END---------------------------------------------*/
                /*----------------------------------------------------------------------------------------*/






                //==============================================================================================================================================================
                //====================================================================Сглаживание================================================================================
                if (Do_Smoothing && SINSstate.flag_Smoothing)
                {
                    string[] BackInputX_LineArray = Back_Input_X.ReadLine().Split(' ');

                    for (int u = 1; u < SimpleData.iMxSmthd + 1; u++)
                        KalmanVars.ErrorVector_Straight[u - 1] = Convert.ToDouble(BackInputX_LineArray[u]);

                    double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                    double Time_Back = SINSstate.Count;

                    int u2 = 0;
                    string[] BackInputP_LineArray = Back_Input_P.ReadLine().Split(' ');

                    SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                    for (int u = 0; u < SimpleData.iMxSmthd; u++)
                    {
                        for (int u1 = u; u1 < SimpleData.iMxSmthd; u1++)
                        {
                            KalmanVars.CovarianceMatrix_SP_Straight[u * SimpleData.iMxSmthd + u1] = Convert.ToDouble(BackInputP_LineArray[u2]);
                            u2++;
                        }
                    }

                    if (SimpleData.iMxSmthd >= 2)
                    {
                        KalmanVars.ErrorVector_m[0] = SINSstate.Latitude;
                        KalmanVars.ErrorVector_m[1] = SINSstate.Longitude;
                    }
                    if (SimpleData.iMxSmthd >= 4)
                    {
                        KalmanVars.ErrorVector_m[2] = SINSstate.Vx_0[0];
                        KalmanVars.ErrorVector_m[3] = SINSstate.Vx_0[1];
                    }
                    if (SimpleData.iMxSmthd >= 7)
                    {
                        KalmanVars.ErrorVector_m[4] = SINSstate.Pitch;
                        KalmanVars.ErrorVector_m[5] = SINSstate.Roll;
                        KalmanVars.ErrorVector_m[6] = SINSstate.Heading;
                    }
                    if (SimpleData.iMxSmthd > 4)
                    {
                        if (SINSstate.Pitch - KalmanVars.ErrorVector_Straight[4] > Math.PI) KalmanVars.ErrorVector_Straight[4] += 2 * Math.PI;
                        if (SINSstate.Pitch - KalmanVars.ErrorVector_Straight[4] < -Math.PI) KalmanVars.ErrorVector_Straight[4] -= 2 * Math.PI;

                        if (SINSstate.Roll - KalmanVars.ErrorVector_Straight[5] > Math.PI) KalmanVars.ErrorVector_Straight[5] += 2 * Math.PI;
                        if (SINSstate.Roll - KalmanVars.ErrorVector_Straight[5] < -Math.PI) KalmanVars.ErrorVector_Straight[5] -= 2 * Math.PI;

                        if (SINSstate.Heading - KalmanVars.ErrorVector_Straight[6] > Math.PI) KalmanVars.ErrorVector_Straight[6] += 2 * Math.PI;
                        if (SINSstate.Heading - KalmanVars.ErrorVector_Straight[6] < -Math.PI) KalmanVars.ErrorVector_Straight[6] -= 2 * Math.PI;
                    }

                    Matrix MatrixS_ForNavDeltas = new Matrix(SimpleData.iMxSmthd, SimpleData.iMxSmthd);
                    MatrixS_ForNavDeltas = SimpleOperations.C_convultion_iMx(SINSstate)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                            * SimpleOperations.C_convultion_iMx(SINSstate).Transpose()
                                            ;
                    KalmanVars.CovarianceMatrix_SP_m = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas), SimpleData.iMxSmthd);

                    //==============================================================//
                    KalmanProcs.Smoothing(KalmanVars, SINSstate, SimpleData.iMxSmthd);
                    //==============================================================//

                    if (SimpleData.iMxSmthd >= 2)
                    {
                        SINSstate_Smooth.Latitude = KalmanVars.ErrorVector_Smoothed[0];
                        SINSstate_Smooth.Longitude = KalmanVars.ErrorVector_Smoothed[1];
                    }
                    if (SimpleData.iMxSmthd >= 4)
                    {
                        SINSstate_Smooth.Vx_0[0] = KalmanVars.ErrorVector_Smoothed[2];
                        SINSstate_Smooth.Vx_0[1]= KalmanVars.ErrorVector_Smoothed[3];
                    }
                    if (SimpleData.iMxSmthd >= 7)
                    {
                        SINSstate_Smooth.Pitch = KalmanVars.ErrorVector_m[4];
                        SINSstate_Smooth.Roll = KalmanVars.ErrorVector_m[5];
                        SINSstate_Smooth.Heading = KalmanVars.ErrorVector_m[6];
                    }

                    if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                    {
                        ForHelpSmoothed.WriteLine("Ошибка сглаживания в выколотой точке в момент времени " + SINSstate.Count + " = " +
                            Math.Sqrt(
                                Math.Pow((ProcHelp.LatSNS * SimpleData.ToRadian - KalmanVars.ErrorVector_m[0]) * SINSstate.R_n, 2)
                                + Math.Pow((ProcHelp.LongSNS * SimpleData.ToRadian - KalmanVars.ErrorVector_m[1]) * SINSstate.R_e * Math.Cos(KalmanVars.ErrorVector_m[0]), 2)
                                )
                            );
                    }



                    //===================Vertical Channel===================//
                    if (SINSstate.flag_iMx_r3_dV3)
                    {
                        int dimVertical = 0;
                        if (SimpleData.iMxSmthd == 4)
                            dimVertical = 2;
                        if (SimpleData.iMxSmthd == 2)
                            dimVertical = 1;

                        for (int u = 0; u < dimVertical; u++)
                            KalmanVars.ErrorVector_Straight[u] = Convert.ToDouble(BackInputX_LineArray[SimpleData.iMxSmthd + 1 + u]);

                        KalmanVars.ErrorVector_m[0] = SINSstate.Altitude;
                        if (dimVertical == 2)
                            KalmanVars.ErrorVector_m[1] = SINSstate.Vx_0[2];

                        SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                        for (int u = 0; u < dimVertical; u++)
                        {
                            for (int u1 = u; u1 < dimVertical; u1++)
                            {
                                KalmanVars.CovarianceMatrix_SP_Straight[u * dimVertical + u1] = Convert.ToDouble(BackInputP_LineArray[u2]);
                                u2++;
                            }
                        }

                        Matrix MatrixS_ForNavDeltas_r3 = new Matrix(dimVertical, dimVertical);
                        MatrixS_ForNavDeltas_r3 = SimpleOperations.C_convultion_iMx_r3(SINSstate)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                            * SimpleOperations.C_convultion_iMx_r3(SINSstate).Transpose()
                                            ;
                        KalmanVars.CovarianceMatrix_SP_m = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas_r3), dimVertical);

                        //==============================================================//
                        KalmanProcs.Smoothing(KalmanVars, SINSstate, dimVertical);
                        //==============================================================//

                        SINSstate_Smooth.Altitude = KalmanVars.ErrorVector_Smoothed[0];
                        if (SimpleData.iMxSmthd == 4)
                            SINSstate_Smooth.Vx_0[2] = KalmanVars.ErrorVector_Smoothed[1];
                    }
                }
                //==============================================================================================================================================================
                if (!Do_Smoothing && SINSstate.flag_Smoothing)
                {
                    if (i == start_i || Math.Floor(i / Convert.ToDouble(NumberOfIterationForOneForSmoothing)) + 1 != Math.Floor((i - 1) / Convert.ToDouble(NumberOfIterationForOneForSmoothing)) + 1)
                    {
                        int int_file_back = Convert.ToInt32(Math.Floor(i / Convert.ToDouble(NumberOfIterationForOneForSmoothing))) + 1;
                        string str_dir_file = "D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//";

                        if (Math.Floor(i / Convert.ToDouble(NumberOfIterationForOneForSmoothing)) + 1 != 1)
                        {
                            Smthing_Backward.Close();
                            Smthing_X.Close();
                            Smthing_P.Close();
                        }
                        Smthing_Backward = new StreamWriter(str_dir_file + "Backward_" + int_file_back.ToString() + ".txt");
                        Smthing_X = new StreamWriter(str_dir_file + "Backward_X_" + int_file_back.ToString() + ".txt");
                        Smthing_P = new StreamWriter(str_dir_file + "Backward_P_" + int_file_back.ToString() + ".txt");
                    }

                    string str_P = "";
                    //---Делаем свертку до S_X---
                    Matrix MatrixS_ForNavDeltas = new Matrix(SimpleData.iMxSmthd, SimpleData.iMxSmthd);
                    MatrixS_ForNavDeltas = SimpleOperations.C_convultion_iMx(SINSstate)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                            * SimpleOperations.C_convultion_iMx(SINSstate).Transpose()
                                            ;
                                                
                    KalmanVars.CovarianceMatrix_SP_Straight = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas), SimpleData.iMxSmthd);

                    for (int ii = 0; ii < SimpleData.iMxSmthd; ii++)
                        for (int ji = ii; ji < SimpleData.iMxSmthd; ji++)
                            str_P += KalmanVars.CovarianceMatrix_SP_Straight[ii * SimpleData.iMxSmthd + ji].ToString() + " ";

                    if (SINSstate.flag_iMx_r3_dV3)
                    {
                        Matrix MatrixS_ForNavDeltas_r3 = SimpleOperations.C_convultion_iMx_r3(SINSstate)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                            * SimpleOperations.C_convultion_iMx_r3(SINSstate).Transpose()
                                            ;
                        if (SimpleData.iMxSmthd == 4)
                        {
                            double[] ArrayS_ForNavDeltas_r3_SP = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas_r3), 2);
                            str_P += ArrayS_ForNavDeltas_r3_SP[0].ToString() + " " + ArrayS_ForNavDeltas_r3_SP[1].ToString() + " " + ArrayS_ForNavDeltas_r3_SP[3].ToString();
                        }
                        else if (SimpleData.iMxSmthd == 2)
                        {
                            double[] ArrayS_ForNavDeltas_r3_SP = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas_r3), 1);
                            str_P += ArrayS_ForNavDeltas_r3_SP[0].ToString();
                        }
                    }

                    Smthing_P.WriteLine(str_P);
                    //-----------------------

                    string str_X = "";
                    if (SimpleData.iMxSmthd == 2)
                    {
                        str_X = SINSstate.Count + " " + SINSstate.Latitude + " " + SINSstate.Longitude;
                        if (SINSstate.flag_iMx_r3_dV3)
                            str_X = str_X + " " + SINSstate.Altitude;
                    }
                    if (SimpleData.iMxSmthd == 4)
                    {
                        str_X = SINSstate.Count + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[1];
                        if (SINSstate.flag_iMx_r3_dV3)
                            str_X = str_X + " " + SINSstate.Altitude + " " + SINSstate.Vx_0[2];
                    }
                    if (SimpleData.iMxSmthd == 7)
                        str_X = SINSstate.Count + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[1] + " " + SINSstate.Pitch + " " + SINSstate.Roll + " " + SINSstate.Heading;
                    Smthing_X.WriteLine(str_X);
                    //-----------------------

                    string StringForBack = "";
                    StringForBack = ProcHelp.datastring + " " + SINSstate_OdoMod.Latitude.ToString() + " " + SINSstate_OdoMod.Longitude.ToString();
                    Smthing_Backward.WriteLine(StringForBack);
                }
                //=========================================================================Сглаживание END=======================================================================
                //==============================================================================================================================================================
                




                /*------------------------------------OUTPUT-------------------------------------------------*/
                if (i != (SINSstate.LastCountForRead - 1) && SINSstate.Global_file != "Saratov_run_2014_07_23")
                    ProcessingHelp.OutPutInfo(i, start_i, ProcHelp, OdoModel, SINSstate, SINSstate2, SINSstateDinamOdo, SINSstate_Smooth, KalmanVars, Nav_EstimateSolution, Nav_Autonomous,
                        Nav_FeedbackSolution, Nav_vert_chan_test, Nav_StateErrorsVector, Nav_Errors, STD_data, Speed_Angles, DinamicOdometer, Nav_Smoothed, KMLFileOut, KMLFileOutSmthd);
                else if (SINSstate.Global_file == "Saratov_run_2014_07_23")
                {
                    if (Math.Abs(SINSstate.Count - Math.Round(SINSstate.Count)) < 0.01// && SINSstate.Count - Math.Round(SINSstate.Count) > 0
                        && Math.Abs(SINSstate.CountPrev - SINSstate.Count) > 0.5
                        )
                    {
                        SINSstate.CountPrev = SINSstate.Count;
                        SINSstate.FreqOutput = 1;
                        ProcessingHelp.OutPutInfo(i, start_i, ProcHelp, OdoModel, SINSstate, SINSstate2, SINSstateDinamOdo, SINSstate_Smooth, KalmanVars, Nav_EstimateSolution, Nav_Autonomous,
                            Nav_FeedbackSolution, Nav_vert_chan_test, Nav_StateErrorsVector, Nav_Errors, STD_data, Speed_Angles, DinamicOdometer, Nav_Smoothed, KMLFileOut, KMLFileOutSmthd);
                    }
                }

                if (i > 10000 && i % 2000 == 0)
                    Console.WriteLine(SINSstate.Count.ToString()
                        + ",  FromSNS=" + Math.Round(ProcHelp.distance, 2) + " м" + ",  FromStart=" + Math.Round(ProcHelp.distance_from_start, 2) + " м"
                        + ",  Vx_1=" + Math.Round(SINSstate.Vx_0[0], 2) + ",  Vx_2=" + Math.Round(SINSstate.Vx_0[1], 3)
                        );

                if ((SINSstate.flag_UsingCorrection == true || (SINSstate.GPS_Data.gps_Altitude.isReady == 1 && SINSstate.flag_Using_SNS == true)) && SINSstate.flag_FeedbackExist == true)
                    SINSprocessing.NullingOfCorrectedErrors(SINSstate, KalmanVars);


                //--- Переопределение значений данных одометра ---
                SINSprocessing.Redifinition_OdoCounts(SINSstate, SINSstate2, SINSstate_OdoMod, OdoModel);
                //---------------------------------------END----------------------------------------------------

            }

            if (SINSstate.flag_Smoothing)
            {
                Smthing_Backward.Close();
                Smthing_P.Close();
                Smthing_X.Close();
                if(Do_Smoothing) ForHelpSmoothed.Close();
            }




            //===Формирование файлов для обратного прогона===
            if (!Do_Smoothing && SINSstate.flag_Smoothing)
            {
                StreamReader Smthing_Backward_R, Smthing_Backward_R_X, Smthing_Backward_R_P;
                StreamWriter Smthing_Backward_full = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_full.txt"),
                             Smthing_Backward_X = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_full_X.txt"),
                             Smthing_Backward_P = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_full_P.txt");

                for (int i = Convert.ToInt32(SINSstate.NumberOfFilesForSmoothing); i >= 1; i--)
                {
                    int j = 0;
                    string[] strTemp = new string[NumberOfIterationForOneForSmoothing],
                             strTemp_X = new string[NumberOfIterationForOneForSmoothing],
                             strTemp_P = new string[NumberOfIterationForOneForSmoothing];
                    Smthing_Backward_R = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_" + i + ".txt");
                    Smthing_Backward_R_X = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_X_" + i + ".txt");
                    Smthing_Backward_R_P = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//For Smoothing temp files//Backward_P_" + i + ".txt");

                    for (j = 0; j < NumberOfIterationForOneForSmoothing; j++)
                    {
                        if (Smthing_Backward_R.EndOfStream == true)
                            break;
                        strTemp[j] = Smthing_Backward_R.ReadLine();
                        strTemp_X[j] = Smthing_Backward_R_X.ReadLine();
                        strTemp_P[j] = Smthing_Backward_R_P.ReadLine();
                    }
                    for (int j1 = j - 1; j1 >= 0; j1--)
                    {
                        Smthing_Backward_full.WriteLine(strTemp[j1]);
                        Smthing_Backward_X.WriteLine(strTemp_X[j1]);
                        Smthing_Backward_P.WriteLine(strTemp_P[j1]);
                    }

                    Smthing_Backward_R.Close(); Smthing_Backward_R_X.Close(); Smthing_Backward_R_P.Close();
                }
                Smthing_Backward_full.Close(); Smthing_Backward_X.Close(); Smthing_Backward_P.Close();
            }





            if (SINSstate.Global_file == "Saratov_run_2014_07_23")
            {
                double SettedHeading = SimpleOperations.CalculateHeadingByTwoDots(49.80892188 * SimpleData.ToRadian, 45.3817334 * SimpleData.ToRadian, SINSstate.GPS_Data.gps_Altitude_prev.Value,
                                        49.80906066 * SimpleData.ToRadian, 45.38113053 * SimpleData.ToRadian, SINSstate.GPS_Data.gps_Altitude.Value);
                double difHeadings = SettedHeading - SINSstate.Heading;
                ForHelp.WriteLine("");
                ForHelp.WriteLine("В конечной выставке разница (нельзя сравнивать,т.к. конечный курс определяется по точкам после конечной выставки):");
                ForHelp.WriteLine("SettedHeadingEND = " + SettedHeading * SimpleData.ToDegree + " град; " + " difHeadings = " + difHeadings * SimpleData.ToDegree + " град.");
                ForHelp.WriteLine("");
                ForHelp.WriteLine("Оценки дрейфов в начале: nu_z1 = " + SINSstate.AlignAlgebraDrifts[0] + ", nu_z2 = " + SINSstate.AlignAlgebraDrifts[1] + ", nu_z3 = " + SINSstate.AlignAlgebraDrifts[2]);
            }

            FillKMLOutputFile(KMLFileOut, "End", "");
            FillKMLOutputFile(KMLFileOutSmthd, "End", "");

            ForHelp.Close(); Nav_FeedbackSolution.Close(); Nav_EstimateSolution.Close(); Nav_StateErrorsVector.Close(); Nav_Autonomous.Close();
            Dif_GK.Close(); Speed_Angles.Close(); Imitator_Telemetric.Close(); //InputForSmoothFile.Close();
            Nav_Smoothed.Close();
            KMLFileOut.Close(); KMLFileOutSmthd.Close();
        }







        //--- КОНТРОЛЬНЫЕ ТОЧКИ --- //

        public static void CheckPointProcessing(SINS_State SINSstate, SINS_State SINSstateDinamOdo, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars)
        {
            SINSstate.flag_ControlPointCorrection = false;


            //SQUARE 5.5 km
            //if (SINSstate.Global_file == "Imitator_Data")
            //{
            //    if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 300.62) < 0.001)
            //    {
            //        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 0.960075010802552, 0.646022214684887, 100.0);
            //        SINSstate.flag_UsingCorrection = true;
            //    }
            //}
            //CALIBR 200 METERS THEN RUN
            if (SINSstate.Global_file == "Imitator_Data")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 135.32) < 0.001)
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 0.959946460438717, 0.645798564231763, 100.0);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 0.959946460438717, 0.645798564231763, 100.0);
                    SINSstate.flag_UsingCorrection = true;
                }
                //if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 370.00) < 0.001)
                //{
                //    Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 0.9598413385365, 0.645991245362916, 100.0);
                //    SINSstate.flag_UsingCorrection = true;
                //}
            }
            //BY CIRCLE
            //if (SINSstate.Global_file == "Imitator_Data")
            //{
            //    if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 220.05) < 0.001)
            //    {
            //        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 0.959931087475909, 0.645771822753885, 100.0);
            //        SINSstate.flag_UsingCorrection = true;
            //    }
            //    if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 370.15) < 0.001)
            //    {
            //        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 0.959931087475909, 0.645771822753885, 100.0);
            //        SINSstate.flag_UsingCorrection = true;
            //    }
            //}
            if (SINSstate.Global_file == "Imitator_Telemetric")
            {
                //if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                if (
                    SINSstate.Count > 80000 && SINSstate.Count <= 80110 && SINSstate.GPS_Data.gps_Latitude.isReady == 1
                    )
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    SINSstate.flag_UsingCorrection = true;
                }
            }

            if (SINSstate.Global_file == "ktn004_15.03.2012")
            {
                if (
                    SINSstate.GPS_Data.gps_Latitude.isReady == 1 && SINSstate.GPS_CounterOfPoints % 10 == 0
                    //SINSstate.Count > 160152 && SINSstate.Count <= 160159 && SINSstate.GPS_Data.gps_Latitude.isReady == 1 //Стоянка
                    //SINSstate.Count > 78372 && SINSstate.Count <= 78412 && SINSstate.GPS_Data.gps_Latitude.isReady == 1 //Движение
                    //SINSstate.Count > 108362 && SINSstate.Count <= 108462 && SINSstate.GPS_Data.gps_Latitude.isReady == 1 //Движение
                    //SINSstate.Count > 134273 && SINSstate.Count <= 134274 && SINSstate.GPS_Data.gps_Latitude.isReady == 1 //Движение OK
                    //|| SINSstate.Count > 188383 && SINSstate.Count <= 188384 && SINSstate.GPS_Data.gps_Latitude.isReady == 1
                    )
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    SINSstate.flag_UsingCorrection = true;
                }
            }

            if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 875.97) < 0.01)
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 53.93522417 * SimpleData.ToRadian, 27.84293667 * SimpleData.ToRadian, 210.397);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 53.93522417 * SimpleData.ToRadian, 27.84293667 * SimpleData.ToRadian, 210.397);
                    SINSstate.flag_UsingCorrection = true;
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1576.38) < 0.01)
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 53.92735 * SimpleData.ToRadian, 27.84526944 * SimpleData.ToRadian, 210.397);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, 53.92735 * SimpleData.ToRadian, 27.84526944 * SimpleData.ToRadian, 210.397);
                    SINSstate.flag_UsingCorrection = true;
                }
            }



            //51882.017348 51282.781305 50314.277559 49810.665254 48625.735469 46969.706109 46099.38973  45275.137785 44455.564773 42015.698887 40798.997098 38318.319723 37522.01148 36624.152871
            //35772.02007  35084.502238 33450.816234 32439.517395 31374.969328 30586.404395 28841.333672 8066.56118   27237.678469 26396.367391 24978.914531 24344.341156 23488.823863
            // 21846.975871 20959.275109 19219.215453 18246.793246 17335.874867 16413.823246 14615.745918 13546.283195 12591.799629 9717.100547  8813.435602  6404.147453 
            //5654.336258  5265.48943 4881.444133  3604.662328  3181.956188  3171.288992 

            if (SINSstate.Global_file == "Saratov_run_2014_07_23")
            {
                SINSstate.flag_true_Marker = false;

                for (int i = 0; i < SINSstate.MarkersInputCount; i++)
                    if (Math.Abs(SINSstate.Count - SINSstate.MarkersInputData[i, 1]) < 0.01 && (SINSstate.MarkerNumberLastUsed != i || i == 0))
                    {
                        SINSstate.flag_true_Marker = true;
                        SINSstate.MarkerNumberLastUsed = i;
                        break;
                    }

                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1 && SINSstate.flag_true_Marker == false)
                {
                    SINSstate.flag_true_Marker = SINSstate.flag_true_Marker;
                    SINSstate.GPS_Data.gps_Latitude.isReady = 2;
                }
                if (SINSstate.GPS_Data.gps_Latitude.isReady != 1 && SINSstate.flag_true_Marker == true)
                    SINSstate.flag_true_Marker = SINSstate.flag_true_Marker;


                if (SINSstate.flag_true_Marker == true)
                //if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    SINSstate.GPS_Data.gps_Latitude.isReady = 1;
                    SINSstate.GPS_Data.gps_Latitude.Value = SINSstate.MarkersInputData[SINSstate.MarkerNumberLastUsed, 2] * SimpleData.ToRadian;
                    SINSstate.GPS_Data.gps_Longitude.Value = SINSstate.MarkersInputData[SINSstate.MarkerNumberLastUsed, 3] * SimpleData.ToRadian;
                    SINSstate.GPS_Data.gps_Altitude.Value = SINSstate.MarkersInputData[SINSstate.MarkerNumberLastUsed, 4];

                    if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 12591.7996) < 0.1)
                        SINSstate.Count = SINSstate.Count;

                    if (Math.Abs(SINSstate.GPS_Data.gps_Latitude.Value - 0.87256909644) > 0.00000001 && Math.Abs(SINSstate.GPS_Data.gps_Longitude.Value - 0.81807104) > 0.000001
                        //&& SINSstate.Count != 20104.108398
                        )
                    {
                        if (SINSstate.flag_Odometr_SINS_case == true)
                            Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                        else
                            CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                        SINSstate.flag_UsingCorrection = true;

                        SINSstate.flag_ControlPointCorrection = true;
                    }
                    else
                        SINSstate.Count = SINSstate.Count;
                }
            }
            
            if (SINSstate.Global_file == "Saratov_run_2014_07_23_middle_interval_GPS")
            {
                //SINSstate.counter_GPS_marks
                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    //13546.283195 - 1
                    //14615.745918 - 2
                    //15519.99802 - 3
                    //16413.823246 - 4
                    //17335.874867 - 5
                    //18246.793246 - 6
                    //19219.215453 - 7
                    if (SINSstate.Count != 15519.99802)
                    {
                        if (SINSstate.flag_Odometr_SINS_case == true)
                            Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                        else
                            CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstateDinamOdo, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                        SINSstate.flag_UsingCorrection = true;

                        SINSstate.flag_ControlPointCorrection = true;
                    }
                    else
                        SINSstate.Count = SINSstate.Count;
                }
            }
        }

        public static void FillKMLOutputFile(StreamWriter KMLFileOut, string Part, string Mode)
        {
            if (Part == "Start")
            {
                KMLFileOut.WriteLine("<?xml version='1.0' encoding='UTF-8'?>                                                 ");
                KMLFileOut.WriteLine("<kml xmlns='http://earth.google.com/kml/2.2'>                                          ");
                KMLFileOut.WriteLine("<Document>                                                                             ");
                KMLFileOut.WriteLine("<name>NavLab Nikitin Markers</name>                                                    ");
                KMLFileOut.WriteLine("<visibility>1</visibility>                                                             ");
                KMLFileOut.WriteLine("<open>1</open>                                                                         ");
                KMLFileOut.WriteLine("<Style id='MarkerIcon'>                                                                ");
                KMLFileOut.WriteLine("        <IconStyle>                                                                    ");
                KMLFileOut.WriteLine("        <scale>1</scale>                                                               ");
                KMLFileOut.WriteLine("            <Icon>                                                                     ");
                KMLFileOut.WriteLine("                <href>http://maps.google.com/mapfiles/kml/shapes/cross-hairs.png</href>");
                KMLFileOut.WriteLine("            </Icon>                                                                    ");
                KMLFileOut.WriteLine("        </IconStyle>                                                                   ");
                KMLFileOut.WriteLine("</Style>                                                                               ");
                KMLFileOut.WriteLine("<Style id='MarkerLine'>                                                                ");
                KMLFileOut.WriteLine("        <LineStyle>                                                                    ");
                if (Mode == "Smoothing")
                {
                    KMLFileOut.WriteLine("                <color>ffff5555</color>                                                ");
                    KMLFileOut.WriteLine("                <width>2</width>                                                       ");
                }
                if (Mode == "Forward")
                {
                    KMLFileOut.WriteLine("                <color>ff000000</color>                                                ");
                    KMLFileOut.WriteLine("                <width>2</width>                                                       ");
                }
                if (Mode == "Backward")
                {
                    KMLFileOut.WriteLine("                <color>ff0000ff</color>                                                ");
                    KMLFileOut.WriteLine("                <width>2</width>                                                       ");
                }
                KMLFileOut.WriteLine("        </LineStyle>                                                                   ");
                KMLFileOut.WriteLine("</Style>                                                                               ");
                KMLFileOut.WriteLine("<Folder>                                                                               ");
                KMLFileOut.WriteLine("        <name>Path</name>                                                              ");
                KMLFileOut.WriteLine("        <visibility>1</visibility>                                                     ");
                KMLFileOut.WriteLine("        <open>0</open>                                                                 ");
                KMLFileOut.WriteLine("        <Placemark>                                                                    ");
                KMLFileOut.WriteLine("            <name>Markers</name>                                                       ");
                KMLFileOut.WriteLine("                <visibility>1</visibility>                                             ");
                KMLFileOut.WriteLine("                <description>The markers scheme</description>                          ");
                KMLFileOut.WriteLine("                <styleUrl>#MarkerLine</styleUrl>                                       ");
                KMLFileOut.WriteLine("                <LineString>                                                           ");
                KMLFileOut.WriteLine("                    <extrude>0</extrude>                                               ");
                KMLFileOut.WriteLine("                    <tessellate>1</tessellate>                                         ");
                KMLFileOut.WriteLine("                    <altitudeMode>clampToGround</altitudeMode>                         ");
                KMLFileOut.WriteLine("                    <coordinates>                                                      ");
            }
            else if (Part == "End")
            {
                KMLFileOut.WriteLine("                   </coordinates>");
                KMLFileOut.WriteLine(" 	           </LineString>       ");
                KMLFileOut.WriteLine("		</Placemark>               ");
                KMLFileOut.WriteLine("</Folder>                        ");
                KMLFileOut.WriteLine("</Document>                      ");
                KMLFileOut.WriteLine("</kml>                           ");

            }
        }
    }
}
