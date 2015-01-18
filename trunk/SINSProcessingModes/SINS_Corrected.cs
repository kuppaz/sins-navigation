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
        public static StreamWriter Nav_FeedbackSolution, Nav_EstimateSolution, Nav_Errors, Nav_Autonomous, Nav_StateErrorsVector, ForHelp;
        public static StreamWriter Nav_Smoothed, ForHelpSmoothed;

        public static StreamWriter ForHelp_2 = new StreamWriter(SimpleData.PathOutputString + "Debaging//ForHelp_2.txt");
        public static StreamWriter SlippageLog = new StreamWriter(SimpleData.PathOutputString + "Debaging//SlippageLog.txt");
        public static StreamWriter KMLFileOut, KMLFileOutSmthd;

        public static StreamWriter STD_data = new StreamWriter(SimpleData.PathOutputString + "Debaging//S_STD.txt");
        public static StreamWriter Kinematic_solution = new StreamWriter(SimpleData.PathOutputString + "S_Kinem.txt");
        public static StreamWriter Speed_Angles = new StreamWriter(SimpleData.PathOutputString + "Debaging//Speed_Angles.txt");
        public static StreamWriter DinamicOdometer = new StreamWriter(SimpleData.PathOutputString + "DinamicOdometer.txt");

        public static StreamWriter Smthing_Backward, Smthing_P, Smthing_X;
        public static StreamReader Back_Input_File_read, Back_Input_X, Back_Input_P;

        public static int NumberOfIterationForOneForSmoothing = 500000;


        public static void SINS_Corrected_Processing(int l, bool NowSmoothing, StreamReader myFile, SINS_State SINSstate, SINS_State SINSstate2, Kalman_Vars KalmanVars, Proc_Help ProcHelp, SINS_State SINSstate_OdoMod)
        {
            int t = 0;

            SINS_State SINSstate_Smooth = new SINS_State();
            SINSstate.NumberOfFilesForSmoothing = Math.Floor(SINSstate.LastCountForRead / Convert.ToDouble(NumberOfIterationForOneForSmoothing)) + 1;


            string str_name_forvard_back = "";
            if (!NowSmoothing)
            {
                KMLFileOut = new StreamWriter(SimpleData.PathOutputString + "KMLFiles//KMLFileOut_Forward.kml");
                ProcessingHelp.FillKMLOutputFile(KMLFileOut, "Start", "Forward");
            }
            else
            {
                str_name_forvard_back = "_back";
                SINSstate.odotime_prev = SINSstate.Time + SINSstate.timeStep;

                Back_Input_File_read = new StreamReader(SimpleData.PathOutputString + "For Smoothing temp files//Backward_full.txt");
                Back_Input_X = new StreamReader(SimpleData.PathOutputString + "For Smoothing temp files//Backward_full_X.txt");
                Back_Input_P = new StreamReader(SimpleData.PathOutputString + "For Smoothing temp files//Backward_full_P.txt");
                ForHelpSmoothed = new StreamWriter(SimpleData.PathOutputString + "Debaging//ForHelp_Smoothed.txt");

                KMLFileOut = new StreamWriter(SimpleData.PathOutputString + "KMLFiles//KMLFileOut_Back.kml");
                ProcessingHelp.FillKMLOutputFile(KMLFileOut, "Start", "Backward");
            }

            Nav_FeedbackSolution = new StreamWriter(SimpleData.PathOutputString + "S" + str_name_forvard_back + "_SlnFeedBack.txt");
            Nav_EstimateSolution = new StreamWriter(SimpleData.PathOutputString + "S" + str_name_forvard_back + "_SlnEstimate.txt");
            Nav_Errors = new StreamWriter(SimpleData.PathOutputString + "S" + str_name_forvard_back + "_Errors.txt");
            Nav_Autonomous = new StreamWriter(SimpleData.PathOutputString + "S" + str_name_forvard_back + "_Autonomous.txt");
            Nav_StateErrorsVector = new StreamWriter(SimpleData.PathOutputString + "S" + str_name_forvard_back + "_ErrVect.txt");
            Nav_Smoothed = new StreamWriter(SimpleData.PathOutputString + "S_smoothed_SlnFeedBack.txt");
            ForHelp = new StreamWriter(SimpleData.PathOutputString + "Debaging//ForHelp" + str_name_forvard_back + ".txt");
            KMLFileOutSmthd = new StreamWriter(SimpleData.PathOutputString + "KMLFiles//KMLFileOut_Smoothed.kml");
            StreamWriter Imitator_Telemetric = new StreamWriter(SimpleData.PathTelemetricString + SINSstate.Global_file + ".dat");


            string str = "count  dr1 dr2 dV1 dV2 Alpha1_grad Alpha2_grad Beta3_grad Nu_1_grad Nu_2_grad/h Nu_3_grad/h dF_1 dF_2 dF_3";
            if (SINSstate.flag_iMx_r3_dV3)
                str = str + " dr3 dV3";
            if (SINSstate.flag_Odometr_SINS_case)
                str = str + " odo_dr1 odo_dr2";
            if (SINSstate.flag_Odometr_SINS_case && SINSstate.flag_Using_iMx_r_odo_3)
                str = str + " odo_dr3";
            if (SINSstate.flag_iMx_kappa_13_ds)
                str = str + " kappa1_grad kappa3_grad Scale";
            Nav_StateErrorsVector.WriteLine(str);

            ProcessingHelp.FillKMLOutputFile(KMLFileOutSmthd, "Start", "Smoothing");

            Nav_Errors.WriteLine("dLat  dLong  dV_x1  dV_x2  dV_x3  dHeading  dRoll  dPitch");
            Nav_Smoothed.WriteLine("time  count LatRelStart  LongRelStart Altitude Latitude  Longitude LatSNS-Lat LngSNS-Lng AltSNS  SpeedSNS  V_x1  V_x2  V_x3  Yaw  Roll  Pitch ");
            DinamicOdometer.WriteLine("Time Count OdoTimeStepCount AbsOdoSpeed_x0 LatRelStart LongRelStart Altitude Altitude_Corr LatRelStartCor-ed LongRelStartCor-ed Latitude  Longitude LatSNS-Lat LngSNS-Lng AltSNS  SpeedSNS  V_x1  V_x2  V_x3");
            Nav_FeedbackSolution.WriteLine("time  count  OdoCnt  OdoV  LatRelStart  LongRelStart Altitude Latitude  Longitude LatSNS-Lat LngSNS-Lng AltSNS  SpeedSNS  V_x1  V_x2  V_x3  Yaw  Roll  Pitch Correct PositError PositErrStart difHeadingSINStoODO difToTrueHeading");
            Nav_EstimateSolution.WriteLine("time  count  OdoCnt  OdoV  LatRelStart  LongRelStart Altitude Latitude  Longitude LatSNS-Lat LngSNS-Lng AltSNS  SpeedSNS V_x1  V_x2  V_x3  Correct  Yaw YawCor  Roll RollCor  Pitch PitchCor PositError V_abs");



            int temp_cnt_V_more = 0, start_i = l;
            SINSstate.NowSmoothing = NowSmoothing;
            if (SINSstate.NowSmoothing) start_i = SINSstate.LastCountForRead - 1;

            //=========================================================================//
            //=========================================================================//
            for (int i = start_i; i <= SINSstate.LastCountForRead; i++)
            {
                if (SINSstate.NowSmoothing) { i = i - 2; if (i < l + 1) break; }

                if (SINSstate.flag_UsingClasAlignment == false)
                    if (i < ProcHelp.AlgnCnt)
                    {
                        ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);
                        continue;
                    }

                if (!SINSstate.NowSmoothing)
                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);
                if (SINSstate.NowSmoothing)
                {
                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, Back_Input_File_read, SINSstate, SINSstate_OdoMod);
                    SINSstate.timeStep = -Math.Abs(SINSstate.timeStep);
                }

                if (t == 0)
                {
                    t = 1;
                    if (!SINSstate.NowSmoothing) start_i = i;
                    if (SINSstate.NowSmoothing) i = i + 2;
                }


                //---------------------------------------------------------------------//

                ProcessingHelp.DefSNSData(ProcHelp, SINSstate);

                //---------------- Формируем вектора измерений одометра ---------------//
                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    SINSstate.OdometerVector[1] = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;
                    SINSstate.OdoSpeed_s[1] = SINSstate.OdometerVector[1] / SINSstate.OdoTimeStepCount / SINSstate.timeStep;

                    //--- Если обратные связи, то сразу корректируем измерение одометра по честной оценке ---//
                    if (SINSstate.flag_FeedbackExist && SINSstate.flag_iMx_kappa_13_ds)
                    {
                        SimpleOperations.CopyArray(SINSstate.OdoSpeed_s,
                            (Matrix.UnitMatrix(3) - Matrix.SkewSymmetricMatrix(SINSstate.ComulativeKappaEst)) / (1.0 + SINSstate.ComulativeKappaEst[1]) * SINSstate.OdoSpeed_s);
                        SimpleOperations.CopyArray(SINSstate.OdometerVector,
                            (Matrix.UnitMatrix(3) - Matrix.SkewSymmetricMatrix(SINSstate.ComulativeKappaEst)) / (1.0 + SINSstate.ComulativeKappaEst[1]) * SINSstate.OdometerVector);
                    }
                }
                //---------------------------------------------------------------------//



                //-------------------------- MAIN STEPS ------------------------------//
                SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate_OdoMod);
                SINSprocessing.Make_A_bridge(SINSstate, SINSstate2, KalmanVars, SINSstate_OdoMod);             //--- Формируем матрицу А фильтра ---//
                KalmanProcs.Make_F(SINSstate.timeStep, KalmanVars);
                KalmanProcs.KalmanForecast(KalmanVars);



                //--------------- Формируем флаг коррекции по одометру---------------//
                SINSstate.flag_UsingCorrection = false;
                if (SINSstate.OdometerData.odometer_left.isReady == 1 && SINSstate.flag_UseOnlyStops == false)
                {
                    SINSstate.flag_UsingCorrection = true;

                    //--- Вычисляем весовые матрицы пройденного пути если SINS+odo по позиции ---//
                    if (SINSstate.flag_Odometr_SINS_case == false)
                    {
                        double[] d_2 = new double[3];
                        for (int u = 0; u < 3; u++) d_2[u] = SINSstate.A_x0s[u, 1];
                        SimpleOperations.CopyMatrix(SINSstate.Ds_ComulativeByOdoTrack, SINSstate.Ds_ComulativeByOdoTrack + SINSstate.OdometerVector[1] * SINSstate.A_x0s);
                        SimpleOperations.CopyMatrix(SINSstate.Ds2_ComulativeByOdoTrack, SINSstate.Ds2_ComulativeByOdoTrack + SINSstate.OdometerVector[1] * Matrix.SkewSymmetricMatrix(d_2));
                    }
                }

                //--- Датчик проскальзывания как разность текущей скорректированной и скорости по одометру -(Пока только для коррекции по скорости)--//
                if (SINSstate.flag_using_slippage == true && SINSstate.flag_UsingOdoVelocity == true)
                    SINSprocessing.SlipageProcessing(SINSstate, SINSstate2, KalmanVars, SlippageLog, temp_cnt_V_more);

                //---------------------------------------------------------------------//




                //---------------Формирование флага остановки------------//
                SINSstate.flag_ZUPT = false;
                if (SINSstate.FLG_Stop == 1 && SINSstate.flag_NotUse_ZUPT == false)
                {
                    SINSstate.flag_ZUPT = true;
                    SINSstate.flag_UsingCorrection = true;
                }




                //----------------------------ЭТАП КОРРЕКЦИИ start---------------------------------------------------------------------//
                //--- Счетчик cnt_measures заполняется по ходу, характеризует количество измерений, которые будут поданы на коррекцию ---//
                KalmanVars.cnt_measures = 0;
                SimpleOperations.NullingOfArray(KalmanVars.Matrix_H);

                if (SINSstate.flag_using_Checkpotints == true)
                    CheckPointProcessing(SINSstate, SINSstate_OdoMod, KalmanVars);

                if (SINSstate.flag_Using_SNS == true && SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                    SINSstate.flag_UsingCorrection = true;

                //---------------------------------------------------------------------//
                ProcHelp.corrected = 0;
                if (SINSstate.flag_UsingCorrection == true)
                {
                    for (int u = 1; u < SINSstate.Heading_Array.Length; u++)
                        SINSstate.Heading_Array[u - 1] = SINSstate.Heading_Array[u];
                    SINSstate.Heading_Array[SINSstate.Heading_Array.Length - 1] = SINSstate.Heading;

                    //===КОРРЕКЦИЯ В СЛУЧАЕ БИНС+ ОДОМЕТР===//
                    if (SINSstate.flag_Odometr_SINS_case == false && SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        if (SINSstate.flag_UsingOdoPosition == true && SINSstate.flag_ZUPT == false)
                            CorrectionModel.Make_H_POSITION(KalmanVars, SINSstate, SINSstate_OdoMod, ProcHelp);

                        if (SINSstate.flag_UsingOdoVelocity == true && SINSstate.flag_ZUPT == false && SINSstate.flag_UsingOdoPosition == false && SINSstate.flag_Using_SNS == false)
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
                        if (SINSstate.flag_UsingOdoPosition == true && SINSstate.flag_ZUPT == false)
                        {
                            //---Если корректировать по измерениям, полученным в прямом проходе, то сглаженное решение будет стремиться именно к последнему---
                            Odometr_SINS.Make_H_POSITION(KalmanVars, SINSstate, SINSstate_OdoMod, ProcHelp);
                        }

                        if (SINSstate.flag_UsingOdoVelocity == true && SINSstate.flag_ZUPT == false)
                            Odometr_SINS.Make_H_VELOCITY(KalmanVars, SINSstate, SINSstate_OdoMod);
                    }

                    //===SNS коррекция===//
                    if (SINSstate.flag_Using_SNS == true)
                        CorrectionModel.Make_H_GPS(KalmanVars, SINSstate, SINSstate_OdoMod);

                    //===ZUPT коррекция===//
                    if (SINSstate.flag_ZUPT == true)
                        CorrectionModel.Make_H_KNS(KalmanVars, SINSstate, SINSstate_OdoMod);

                    KalmanProcs.KalmanCorrection(KalmanVars);
                    ProcHelp.corrected = 1;
                }
                /*----------------------------------------------------------------------------------------*/


                //--- Вывод данных для телеметрического имитатора ---//
                ProcessingHelp.OutPutInfo_Telemetric(SINSstate, SINSstate2, Imitator_Telemetric, i, l);

                //--- Вывод в файл значений оцененной позиционной ошибки до коррекции (важно в обратных связях) ---//
                if (SINSstate.flag_ControlPointCorrection)
                {
                    ForHelp.WriteLine(Math.Round(SINSstate.Time + SINSstate.Time_Alignment, 4) + " " + Math.Round(SINSstate.OdometerData.odometer_left.Value * 1000 + 3155, 0)
                        + " " + Math.Round(SINSstate.GPS_Data.gps_Latitude.Value * SimpleData.ToDegree, 8) + " " + Math.Round(SINSstate.GPS_Data.gps_Longitude.Value * SimpleData.ToDegree, 8) + " " + SINSstate.GPS_Data.gps_Altitude.Value
                        + " " + Math.Round(Math.Sqrt(Math.Pow(KalmanVars.ErrorConditionVector_p[0], 2) + Math.Pow(KalmanVars.ErrorConditionVector_p[1], 2)), 2)
                        + " " + SimpleOperations.CalculateDistanceBtwDots(SINSstate.GPS_Data.gps_Latitude_prev.Value, SINSstate.GPS_Data.gps_Longitude_prev.Value, SINSstate.GPS_Data.gps_Altitude_prev.Value,
                                        SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value) / (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerData.odometer_left_prev.Value));
                }

                //--- Расчет корректирующего вектора состояния ---//
                SINSprocessing.CalcStateErrors(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate_OdoMod);
                if (SINSstate.flag_EstimateExist == true)
                    SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2, SINSstate_OdoMod);
                if (SINSstate.flag_FeedbackExist == true)
                    SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate, SINSstate_OdoMod);

                //----------------------------ЭТАП КОРРЕКЦИИ END---------------------------------//




                //=================================================Сглаживание============================================================
                if (SINSstate.flag_Smoothing && !SINSstate.NowSmoothing)
                {
                    //--- В X сглаживаются координаты, поэтому будет это X^+ или X^- зависит от места вызова функции в коде ---//
                    if (i == start_i || Math.Floor(i / Convert.ToDouble(NumberOfIterationForOneForSmoothing)) + 1 != Math.Floor((i - 1) / Convert.ToDouble(NumberOfIterationForOneForSmoothing)) + 1)
                    {
                        int int_file_back = Convert.ToInt32(Math.Floor(i / Convert.ToDouble(NumberOfIterationForOneForSmoothing))) + 1;
                        string str_dir_file = SimpleData.PathOutputString + "For Smoothing temp files//";

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
                    SINSprocessing.FuncSmoothing_Forward(SINSstate, SINSstate_Smooth, SINSstate_OdoMod, KalmanVars, ProcHelp, Smthing_X, Smthing_P, Smthing_Backward);
                }
                if (SINSstate.flag_Smoothing && SINSstate.NowSmoothing)
                    SINSprocessing.FuncSmoothing_BackwardAndSmooth(SINSstate, SINSstate_Smooth, KalmanVars, ProcHelp, Back_Input_X, Back_Input_P, ForHelpSmoothed);
                
                //===============================================Сглаживание END===========================================================




                /*------------------------------------OUTPUT-------------------------------------------------*/
                //--- OUTPUT в файлы ---//
                if (i != (SINSstate.LastCountForRead - 1) && SINSstate.Global_file != "Saratov_run_2014_07_23")
                    ProcessingHelp.OutPutInfo(i, start_i, ProcHelp, SINSstate, SINSstate2, SINSstate_OdoMod, SINSstate_Smooth, KalmanVars, Nav_EstimateSolution, Nav_Autonomous, 
                        Nav_FeedbackSolution, Nav_StateErrorsVector, Nav_Errors, STD_data, Speed_Angles, DinamicOdometer, Nav_Smoothed, KMLFileOut, KMLFileOutSmthd);
                else if (SINSstate.Global_file == "Saratov_run_2014_07_23")
                {
                    //--- Раз в секунду вывод ---//
                    if (Math.Abs(SINSstate.Count - Math.Round(SINSstate.Count)) < 0.01 && Math.Abs(SINSstate.CountPrev - SINSstate.Count) > 0.5)
                    {
                        SINSstate.FreqOutput = 1;
                        SINSstate.CountPrev = SINSstate.Count;
                        ProcessingHelp.OutPutInfo(i, start_i, ProcHelp, SINSstate, SINSstate2, SINSstate_OdoMod, SINSstate_Smooth, KalmanVars, Nav_EstimateSolution, Nav_Autonomous,
                            Nav_FeedbackSolution, Nav_StateErrorsVector, Nav_Errors, STD_data, Speed_Angles, DinamicOdometer, Nav_Smoothed, KMLFileOut, KMLFileOutSmthd);
                    }
                }

                //--- OUTPUT в консоль ---//
                if (i > 10000 && i % 2000 == 0)
                    Console.WriteLine(SINSstate.Count.ToString()
                        + ",  FromSNS=" + Math.Round(ProcHelp.distance, 2) + " м" + ",  FromStart=" + Math.Round(ProcHelp.distance_from_start, 2) + " м"
                        + ",  Vx_1=" + Math.Round(SINSstate.Vx_0[0], 2) + ",  Vx_2=" + Math.Round(SINSstate.Vx_0[1], 3)
                        );


                //--- Списывание ошибок в случае обратных связей ---//
                if (SINSstate.flag_UsingCorrection == true && SINSstate.flag_FeedbackExist == true)
                    SINSprocessing.NullingOfCorrectedErrors(SINSstate, KalmanVars);

                //--- Переопределение значений данных одометра ---
                SINSprocessing.Redifinition_OdoCounts(SINSstate, SINSstate2, SINSstate_OdoMod);
            }
            /*----------------------------------------END---------------------------------------------*/
            /*----------------------------------------------------------------------------------------*/


            if (SINSstate.flag_Smoothing)
            {
                if (SINSstate.NowSmoothing) ForHelpSmoothed.Close();
                Smthing_Backward.Close(); Smthing_P.Close(); Smthing_X.Close();

                //===Формирование файлов для обратного прогона===
                if (!SINSstate.NowSmoothing)
                    SINSprocessing.FuncSmoothing_TransposeFileForBackward(SINSstate, NumberOfIterationForOneForSmoothing);
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

            ProcessingHelp.FillKMLOutputFile(KMLFileOut, "End", "");
            ProcessingHelp.FillKMLOutputFile(KMLFileOutSmthd, "End", "");

            ForHelp.Close();
            Nav_FeedbackSolution.Close();
            Nav_EstimateSolution.Close();
            Nav_StateErrorsVector.Close();
            Nav_Autonomous.Close();
            Speed_Angles.Close();
            Imitator_Telemetric.Close();
            Nav_Smoothed.Close();
            KMLFileOut.Close();
            KMLFileOutSmthd.Close();
        }







        //--- КОНТРОЛЬНЫЕ ТОЧКИ --- //
        public static void CheckPointProcessing(SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars)
        {
            SINSstate.flag_ControlPointCorrection = false;


            if (SINSstate.Global_file == "Azimut_15.08.2012" || SINSstate.Global_file == "Azimut_24.08.2012" || SINSstate.Global_file == "Azimut_29.08.2012")
            {
                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    SINSstate.flag_UsingCorrection = true;
                }
            }

            //COMMON IMITATOR EACH isReady 5.5 km
            if (SINSstate.Global_file == "Imitator_Data")
            {
                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    SINSstate.flag_UsingCorrection = true;
                }
            }
            //SQUARE 5.5 km
            //if (SINSstate.Global_file == "Imitator_Data")
            //{
            //    if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 300.62) < 0.001)
            //    {
            //        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 0.960075010802552, 0.646022214684887, 100.0);
            //        SINSstate.flag_UsingCorrection = true;
            //    }
            //}
            //CALIBR 200 METERS THEN RUN
            //if (SINSstate.Global_file == "Imitator_Data")
            //{
            //    if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 135.32) < 0.001)
            //    {
            //        if (SINSstate.flag_Odometr_SINS_case == true)
            //            Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 0.959946460438717, 0.645798564231763, 100.0);
            //        else
            //            CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 0.959946460438717, 0.645798564231763, 100.0);
            //        SINSstate.flag_UsingCorrection = true;
            //    }
            //    //if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 370.00) < 0.001)
            //    //{
            //    //    Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 0.9598413385365, 0.645991245362916, 100.0);
            //    //    SINSstate.flag_UsingCorrection = true;
            //    //}
            //}
            //BY CIRCLE
            //if (SINSstate.Global_file == "Imitator_Data")
            //{
            //    if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 220.05) < 0.001)
            //    {
            //        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 0.959931087475909, 0.645771822753885, 100.0);
            //        SINSstate.flag_UsingCorrection = true;
            //    }
            //    if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 370.15) < 0.001)
            //    {
            //        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 0.959931087475909, 0.645771822753885, 100.0);
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
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    SINSstate.flag_UsingCorrection = true;
                }
            }

            if (SINSstate.Global_file == "ktn004_15.03.2012")
            {
                if (
                    SINSstate.GPS_Data.gps_Latitude.isReady == 1 && SINSstate.GPS_CounterOfPoints % 5 == 0
                    //SINSstate.Count > 160152 && SINSstate.Count <= 160159 && SINSstate.GPS_Data.gps_Latitude.isReady == 1 //Стоянка
                    //SINSstate.Count > 78372 && SINSstate.Count <= 78412 && SINSstate.GPS_Data.gps_Latitude.isReady == 1 //Движение
                    //SINSstate.Count > 108362 && SINSstate.Count <= 108462 && SINSstate.GPS_Data.gps_Latitude.isReady == 1 //Движение
                    //SINSstate.Count > 134273 && SINSstate.Count <= 134274 && SINSstate.GPS_Data.gps_Latitude.isReady == 1 //Движение OK
                    //|| SINSstate.Count > 188383 && SINSstate.Count <= 188384 && SINSstate.GPS_Data.gps_Latitude.isReady == 1
                    )
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                    SINSstate.flag_UsingCorrection = true;
                }
            }

            if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 875.97) < 0.01)
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 53.93522417 * SimpleData.ToRadian, 27.84293667 * SimpleData.ToRadian, 210.397);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 53.93522417 * SimpleData.ToRadian, 27.84293667 * SimpleData.ToRadian, 210.397);
                    SINSstate.flag_UsingCorrection = true;
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1576.38) < 0.01)
                {
                    if (SINSstate.flag_Odometr_SINS_case == true)
                        Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 53.92735 * SimpleData.ToRadian, 27.84526944 * SimpleData.ToRadian, 210.397);
                    else
                        CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 53.92735 * SimpleData.ToRadian, 27.84526944 * SimpleData.ToRadian, 210.397);
                    SINSstate.flag_UsingCorrection = true;
                }
            }



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
                    SINSstate.GPS_Data.gps_Latitude.isReady = 2;


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
                            Odometr_SINS.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                        else
                            CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value);
                        SINSstate.flag_UsingCorrection = true;

                        SINSstate.flag_ControlPointCorrection = true;
                    }
                    else
                        SINSstate.Count = SINSstate.Count;

                }
            }

        }

    }
}
