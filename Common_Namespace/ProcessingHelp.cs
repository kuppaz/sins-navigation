using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using Common_Namespace;

namespace Common_Namespace
{
    public partial class ProcessingHelp
    {

        /*-------------------------------Вспомогательные функции---------------------------------------------------------*/

        public static void DefSNSData(Proc_Help ProcHelp, SINS_State SINSstate)
        {
            if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
            {
                ProcHelp.LatSNS = SINSstate.GPS_Data.gps_Latitude.Value * 180 / Math.PI;
                ProcHelp.LongSNS = SINSstate.GPS_Data.gps_Longitude.Value * 180 / Math.PI;
                ProcHelp.AltSNS = SINSstate.GPS_Data.gps_Altitude.Value;
                ProcHelp.SpeedSNS = Math.Sqrt(SINSstate.GPS_Data.gps_Ve.Value * SINSstate.GPS_Data.gps_Ve.Value + SINSstate.GPS_Data.gps_Vn.Value * SINSstate.GPS_Data.gps_Vn.Value);
                ProcHelp.Ve_SNS = SINSstate.GPS_Data.gps_Ve.Value;
                ProcHelp.Vn_SNS = SINSstate.GPS_Data.gps_Vn.Value;
            }
        }

        public static void ReadSINSStateFromString(Proc_Help ProcHelp, StreamReader myFile, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            string[] dataArray;

            ProcHelp.datastring = myFile.ReadLine();

            if (SINSstate.Global_file == "Azimut-T_18-Oct-2013_11-05-11" || SINSstate.Global_file == "topo")
            {
                if (SINSstate.firstLineRead == false)
                {
                    ProcHelp.datastring = myFile.ReadLine();
                    dataArray = ProcHelp.datastring.Split(' ');
                    SINSstate.Count++;
                    SINSstate.Time_prev = Convert.ToDouble(dataArray[1]);
                    SINSstate.OdometerLeftPrev = Convert.ToDouble(dataArray[23]) / 1000.0;
                    SINSstate.firstLineRead = true;
                    ProcHelp.datastring = myFile.ReadLine();
                }

                dataArray = ProcHelp.datastring.Split(' ');

                SINSstate.Count++;
                SINSstate.Time = Convert.ToDouble(dataArray[1]) - SINSstate.Time_Alignment;
                SINSstate.timeStep = SINSstate.Freq = SINSstate.Time - SINSstate.Time_prev;

                if (SINSstate.timeStep < 0.0)
                    SINSstate.timeStep = SINSstate.Freq = SINSstate.Time + SINSstate.Time_Alignment - SINSstate.Time_prev;

                //if (SINSstate.timeStep < 0.00001 && SINSstate.timeStep >= 0.0 || SINSstate.F_z[1] == Convert.ToDouble(dataArray[15]) && SINSstate.F_z[0] == Convert.ToDouble(dataArray[17]))
                //{
                //    ProcHelp.datastring = myFile.ReadLine();
                //    dataArray = ProcHelp.datastring.Split(' ');
                //    SINSstate.Count++;
                //    SINSstate.Time = Convert.ToDouble(dataArray[1]) - SINSstate.Time_Alignment;
                //    SINSstate.timeStep = SINSstate.Freq = SINSstate.Time - SINSstate.Time_prev;

                //    if (SINSstate.timeStep < 0.0)
                //        SINSstate.timeStep = SINSstate.Freq = SINSstate.Time + SINSstate.Time_Alignment - SINSstate.Time_prev;
                //}


                SINSstate.F_z[1] = Convert.ToDouble(dataArray[15]); SINSstate.W_z[1] = Convert.ToDouble(dataArray[12]);
                SINSstate.F_z[2] = Convert.ToDouble(dataArray[16]); SINSstate.W_z[2] = Convert.ToDouble(dataArray[13]);
                SINSstate.F_z[0] = Convert.ToDouble(dataArray[17]); SINSstate.W_z[0] = Convert.ToDouble(dataArray[14]);


                if ((Convert.ToInt32(dataArray[2]) == 1 && SINSstate.timeStep > 0.5) || SINSstate.Count <= ProcHelp.AlgnCnt)
                    SINSstate.FLG_Stop = 1;
                else
                    SINSstate.FLG_Stop = 0;

                SINSstate.OdometerData.odometer_left.Value = Convert.ToDouble(dataArray[23]) / 1000.0;

                if (SINSstate.Count % 5 == 0)
                {
                    SINSstate.odotime_cur = SINSstate.Time;
                    SINSstate.OdometerData.odometer_left.isReady = 1;

                    SINSstate.OdoTimeStepCount = (SINSstate.odotime_cur - SINSstate.odotime_prev) / SINSstate.timeStep - 1.0;
                }
                else
                    SINSstate.OdometerData.odometer_left.isReady = 2;

                SINSstate.Time_prev = SINSstate.Time;
            }
            else
            {
                if (ProcHelp.datastring.Contains("Count") || ProcHelp.datastring.Contains("Latitude"))
                    ProcHelp.datastring = myFile.ReadLine();

                dataArray = ProcHelp.datastring.Split(' ');
                int t = 0;

                for (int y = 0; y < dataArray.Length; y++)
                {
                    if (dataArray[y] != "")
                        t++;
                }
                string[] dataArray2 = new string[t];
                t = 0;

                for (int y = 0; y < dataArray.Length; y++)
                {
                    if (dataArray[y] != "")
                    {
                        dataArray2[t] = dataArray[y];
                        t++;
                    }
                }


                //SINSstate.Time = Convert.ToDouble(dataArray2[0]);
                SINSstate.Count = Convert.ToDouble(dataArray2[0]);

                if (ProcHelp.initCount == false) { ProcHelp.initCount = true; SINSstate.initCount = SINSstate.Count - 1; }

                SINSstate.Time = (SINSstate.Count - SINSstate.initCount) * Math.Abs(SINSstate.timeStep);

                SINSstate.F_z[1] = Convert.ToDouble(dataArray2[1]); SINSstate.W_z[1] = Convert.ToDouble(dataArray2[4]);
                SINSstate.F_z[2] = Convert.ToDouble(dataArray2[2]); SINSstate.W_z[2] = Convert.ToDouble(dataArray2[5]);
                SINSstate.F_z[0] = Convert.ToDouble(dataArray2[3]); SINSstate.W_z[0] = Convert.ToDouble(dataArray2[6]);

                //Юстировочные углы
                if (SINSstate.Global_file == "ktn004_15.03.2012" || SINSstate.Global_file == "ktn004_21.03.2012" || SINSstate.Global_file == "Saratov_run_2014_07_23" || SINSstate.Global_file == "Saratov_run_2014_07_23_middle_interval_GPS")
                {
                    double[] fz = new double[3], Wz = new double[3];

                    fz[1] = SINSstate.F_z[1] + SINSstate.alpha_z * SINSstate.F_z[2] + SINSstate.alpha_y * SINSstate.F_z[0];//-0.02;
                    fz[2] = SINSstate.F_z[2] - SINSstate.alpha_z * SINSstate.F_z[1] + SINSstate.alpha_x * SINSstate.F_z[0];//+0.037;
                    fz[0] = SINSstate.F_z[0] - SINSstate.alpha_y * SINSstate.F_z[1] - SINSstate.alpha_x * SINSstate.F_z[2];

                    Wz[1] = SINSstate.W_z[1] + SINSstate.alpha_z * SINSstate.W_z[2] + SINSstate.alpha_y * SINSstate.W_z[0];
                    Wz[2] = SINSstate.W_z[2] - SINSstate.alpha_z * SINSstate.W_z[1] + SINSstate.alpha_x * SINSstate.W_z[0];
                    Wz[0] = SINSstate.W_z[0] - SINSstate.alpha_y * SINSstate.W_z[1] - SINSstate.alpha_x * SINSstate.W_z[2];

                    SimpleOperations.CopyArray(SINSstate.F_z, fz);
                    SimpleOperations.CopyArray(SINSstate.W_z, Wz);
                }

                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    SINSstate.GPS_Data.gps_Latitude_prev.Value=SINSstate.GPS_Data.gps_Latitude.Value;
                    SINSstate.GPS_Data.gps_Longitude_prev.Value = SINSstate.GPS_Data.gps_Longitude.Value;
                    SINSstate.GPS_Data.gps_Altitude_prev.Value = SINSstate.GPS_Data.gps_Altitude.Value;

                    SINSstate.OdometerData.odometer_left_prev.Value = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerData.odometer_right_prev.Value = SINSstate.OdometerData.odometer_right.Value;
                }
                SINSstate.GPS_Data.gps_Latitude.Value = Convert.ToDouble(dataArray2[7]);
                SINSstate.GPS_Data.gps_Latitude.isReady = Convert.ToInt32(dataArray2[8]);
                SINSstate.GPS_Data.gps_Longitude.Value = Convert.ToDouble(dataArray2[9]);
                SINSstate.GPS_Data.gps_Longitude.isReady = Convert.ToInt32(dataArray2[10]);
                SINSstate.GPS_Data.gps_Altitude.Value = Convert.ToDouble(dataArray2[11]);
                SINSstate.GPS_Data.gps_Altitude.isReady = Convert.ToInt32(dataArray2[12]);

                SINSstate.GPS_Data.gps_Vn.Value = Convert.ToDouble(dataArray2[13]);
                SINSstate.GPS_Data.gps_Vn.isReady = Convert.ToInt32(dataArray2[14]);
                SINSstate.GPS_Data.gps_Ve.Value = Convert.ToDouble(dataArray2[15]);
                SINSstate.GPS_Data.gps_Ve.isReady = Convert.ToInt32(dataArray2[16]);

                SINSstate.FLG_Stop = Convert.ToInt32(dataArray2[17]);

                SINSstate.OdometerData.odometer_left.Value = Convert.ToDouble(dataArray2[18]);
                SINSstate.OdometerData.odometer_left.isReady = Convert.ToInt32(dataArray2[19]);
                SINSstate.OdometerData.odometer_right.Value = Convert.ToDouble(dataArray2[20]);
                SINSstate.OdometerData.odometer_right.isReady = Convert.ToInt32(dataArray2[21]);

                //if (SINSstate.Global_file == "AZIMUT_T_2013_10_18_12_55")
                //{
                //    SINSstate.OdometerData.odometer_left.Value -= 50735.7232;
                //    SINSstate.OdometerData.odometer_right.Value -= 50735.7232;
                //}

                if (SINSstate.Global_file == "Saratov_run_2014_07_23" || SINSstate.Global_file == "Saratov_run_2014_07_23_middle_interval_GPS")
                {
                    SINSstate.timeStep = SINSstate.Freq = Convert.ToDouble(dataArray2[22]);
                    SINSstate.Time = Convert.ToDouble(dataArray2[0]) - SINSstate.Time_Alignment;

                    //SINSstate.OdometerData.odometer_left.Value = SINSstate.OdometerData.odometer_left.Value * 1.0015;

                }

                if (dataArray2.Length > 22 && SINSstate.Do_Smoothing == true)
                {
                    SINSstate_OdoMod.Latitude = Convert.ToDouble(dataArray2[22]);
                    SINSstate_OdoMod.Longitude = Convert.ToDouble(dataArray2[23]);

                    if (SINSstate.Global_file == "Saratov_run_2014_07_23" || SINSstate.Global_file == "Saratov_run_2014_07_23_middle_interval_GPS")
                    {
                        SINSstate_OdoMod.Latitude = Convert.ToDouble(dataArray2[23]);
                        SINSstate_OdoMod.Longitude = Convert.ToDouble(dataArray2[24]);
                    }
                }

            }

        }



        public static void OutPutInfo_Chechpoints(SINS_State SINSstate, SINS_State SINSstate2, StreamWriter Dif_GK)
        {
            double[] Min_Distance_to_Points = new double[6];

            if (true)
            {
                for (int ij = 0; ij < SINSstate.NumberOfControlPoints; ij++)
                {
                    if (SINSstate.Count == SINSstate.ControlPointCount[ij])
                    {
                        for (int ii = 0; ii < 3; ii++)
                        {
                            if (SINSstate.flag_FeedbackExist)
                            {
                                Dif_GK.WriteLine("Широта " + ii.ToString() + ": " + SINSstate.GK_Latitude[ii].ToString() + " разница со стартом: " + ((SINSstate.GK_Latitude[ii] - SINSstate.Latitude) * SINSstate.R_n).ToString());
                                Dif_GK.WriteLine("Долгота " + ii.ToString() + ": " + SINSstate.GK_Longitude[ii].ToString() + " разница со стартом: " + ((SINSstate.GK_Longitude[ii] - SINSstate.Longitude) * SINSstate.R_e).ToString());
                            }
                            else
                            {
                                Dif_GK.WriteLine("Широта " + ii.ToString() + ": " + SINSstate.GK_Latitude[ii].ToString() + " разница со стартом: " + ((SINSstate.GK_Latitude[ii] - SINSstate2.Latitude) * SINSstate.R_n).ToString());
                                Dif_GK.WriteLine("Долгота " + ii.ToString() + ": " + SINSstate.GK_Longitude[ii].ToString() + " разница со стартом: " + ((SINSstate.GK_Longitude[ii] - SINSstate2.Longitude) * SINSstate.R_e).ToString());
                            }

                            Dif_GK.WriteLine(" ");
                        }
                        Dif_GK.WriteLine(" ");
                    }
                }
            }
            else
            {
                if (SINSstate.flag_FeedbackExist)
                {
                    Min_Distance_to_Points[0] = (SINSstate.GK_Latitude[0] - SINSstate.Latitude) * SINSstate.R_n; Min_Distance_to_Points[1] = (SINSstate.GK_Longitude[0] - SINSstate.Longitude) * SINSstate.R_e;
                    Min_Distance_to_Points[2] = (SINSstate.GK_Latitude[1] - SINSstate.Latitude) * SINSstate.R_n; Min_Distance_to_Points[3] = (SINSstate.GK_Longitude[1] - SINSstate.Longitude) * SINSstate.R_e;
                    Min_Distance_to_Points[4] = (SINSstate.GK_Latitude[2] - SINSstate.Latitude) * SINSstate.R_n; Min_Distance_to_Points[5] = (SINSstate.GK_Longitude[2] - SINSstate.Longitude) * SINSstate.R_e;
                    Dif_GK.WriteLine((SINSstate.Count * Math.Abs(SINSstate.timeStep)).ToString() + " " + Min_Distance_to_Points[0].ToString() + " " + Min_Distance_to_Points[1].ToString() + " " + Min_Distance_to_Points[2].ToString() + " " + Min_Distance_to_Points[3].ToString() + " " + Min_Distance_to_Points[4].ToString() + " " + Min_Distance_to_Points[5].ToString());
                }
                else
                {
                    Min_Distance_to_Points[0] = (SINSstate.GK_Latitude[0] - SINSstate2.Latitude) * SINSstate.R_n; Min_Distance_to_Points[1] = (SINSstate.GK_Longitude[0] - SINSstate2.Longitude) * SINSstate.R_e;
                    Min_Distance_to_Points[2] = (SINSstate.GK_Latitude[1] - SINSstate2.Latitude) * SINSstate.R_n; Min_Distance_to_Points[3] = (SINSstate.GK_Longitude[1] - SINSstate2.Longitude) * SINSstate.R_e;
                    Min_Distance_to_Points[4] = (SINSstate.GK_Latitude[2] - SINSstate2.Latitude) * SINSstate.R_n; Min_Distance_to_Points[5] = (SINSstate.GK_Longitude[2] - SINSstate2.Longitude) * SINSstate.R_e;
                    Dif_GK.WriteLine((SINSstate.Count * Math.Abs(SINSstate.timeStep)).ToString() + " " + Min_Distance_to_Points[0].ToString() + " " + Min_Distance_to_Points[1].ToString() + " " + Min_Distance_to_Points[2].ToString() + " " + Min_Distance_to_Points[3].ToString() + " " + Min_Distance_to_Points[4].ToString() + " " + Min_Distance_to_Points[5].ToString());
                }
            }
        }




        public static void OutPutInfo(int i, int start_i, Proc_Help ProcHelp, ParamsForModel OdoModel, SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstateDinamOdo, SINS_State SINSstate_Smooth, Kalman_Vars KalmanVars, StreamWriter Nav_EstimateSolution, StreamWriter Nav_Autonomous,
                StreamWriter Nav_FeedbackSolution, StreamWriter Nav_vert_chan_test, StreamWriter Nav_StateErrorsVector, StreamWriter Nav_Errors, StreamWriter STD_data, StreamWriter Speed_Angles, StreamWriter DinamicOdometer, StreamWriter Nav_Smoothed)
        {
            double Lat = 0.0, Long = 0.0;
            double[] Vx_0 = new double[3];
            if (SINSstate.flag_FeedbackExist || SINSstate.flag_Autonomous_Solution) 
            { 
                Lat = SINSstate.Latitude; 
                Long = SINSstate.Longitude;
                SimpleOperations.CopyArray(Vx_0, SINSstate.Vx_0);
            }
            else 
            { 
                Lat = SINSstate2.Latitude; 
                Long = SINSstate2.Longitude; 
                SimpleOperations.CopyArray(Vx_0, SINSstate2.Vx_0); 
            }

            ProcHelp.distance = Math.Sqrt(Math.Pow((Lat - ProcHelp.LatSNS * SimpleData.ToRadian) * SimpleOperations.RadiusN(Lat, SINSstate.Altitude), 2) +
                                 Math.Pow((Long - ProcHelp.LongSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(Lat, SINSstate.Altitude) * Math.Cos(Lat), 2));
            ProcHelp.distance_from_start = Math.Sqrt(Math.Pow((Lat - SINSstate.Latitude_Start) * SimpleOperations.RadiusN(Lat, SINSstate.Altitude), 2) +
                                 Math.Pow((Long - SINSstate.Longitude_Start) * SimpleOperations.RadiusE(Lat, SINSstate.Altitude) * Math.Cos(Lat), 2));


            if (i % SINSstate.FreqOutput == 0 && SINSstate.Do_Smoothing == false)
                Speed_Angles.WriteLine(SINSstate.Time + " " + SINSstate.CourseHeading + " " + SINSstate.Heading + " " + SINSstate.CoursePitch 
                    + " " + SINSstate.beta_c + " " + SINSstate.alpha_c + " " + SINSstate.gamma_c
                    + " " + SINSstate.OdoSpeed_x0[0] + " " + SINSstate.OdoSpeed_x0[1] + " " + Vx_0[0] + " " + Vx_0[1]
                    + " " + KalmanVars.Matrix_H[4] + " " + KalmanVars.Matrix_H[5] + " " + KalmanVars.Matrix_H[6]);

            //--- Вывод всяких СКО ---
            if (i % SINSstate.FreqOutput == 0 || i == start_i) 
            {
                string str = (SINSstate.Time + SINSstate.Time_Alignment).ToString(), str_hat = "";

                if (i == start_i)
                {
                    str_hat += "time sigm_dr1 sigm_dr2 ";
                    if (SINSstate.flag_iMx_r3_dV3) 
                        str_hat += "std_dr3 ";
                    if (SINSstate.flag_Odometr_SINS_case)
                        str_hat += "std_odo_dr1 std_odo_dr2 ";
                    if (SINSstate.flag_Odometr_SINS_case == true && SINSstate.flag_Using_iMx_r_odo_3 == true)
                        str_hat += "std_odo_dr3 ";
                    if (SINSstate.flag_iMx_kappa_13_ds)
                        str_hat += "stdKappa1 stdKappa3 stdScale ";

                    str_hat += "std_dV1 std_dV2 stdAlpha1 stdAlpha2 stdBeta3 stdNu1 stdNu2 stdNu3 std_df1 std_df2 std_df3";
                    STD_data.WriteLine(str_hat);
                }

                if (SINSstate.flag_iMx_r3_dV3)
                    str = str + " " + KalmanProcs.Sigmf_Disp(0, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(1, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(SINSstate.iMx_r3_dV3 + 0, KalmanVars);
                else str = str + " " + KalmanProcs.Sigmf_Disp(0, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(1, KalmanVars);

                if (SINSstate.flag_Odometr_SINS_case == true)
                    str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.iMx_r12_odo, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(SINSstate.iMx_r12_odo + 1, KalmanVars);
                if (SINSstate.flag_Odometr_SINS_case == true && SINSstate.flag_Using_iMx_r_odo_3 == true)
                    str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.iMx_r12_odo + 2, KalmanVars);

                str = str + " " + KalmanProcs.Sigmf_Disp(2, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(3, KalmanVars);
                str = str + " " + KalmanProcs.Sigmf_Disp(4, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(5, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(6, KalmanVars);
                str = str + " " + KalmanProcs.Sigmf_Disp(7, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(8, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(9, KalmanVars);
                str = str + " " + KalmanProcs.Sigmf_Disp(10, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(11, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(12, KalmanVars);

                if (SINSstate.flag_iMx_kappa_13_ds)
                    str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.iMx_odo_model + 0, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(SINSstate.iMx_odo_model + 1, KalmanVars);// +" " + KalmanProcs.Sigmf_Disp(SINSstate.iMx_odo_model + 2, KalmanVars);

                STD_data.WriteLine(str);
            }


            /*----------------------------------OUTPUT DinamicOdometer------------------------------------------------------*/
            if (i % SINSstate.FreqOutput == 0)
            {
                ProcHelp.datastring = (SINSstateDinamOdo.Time + SINSstateDinamOdo.Time_Alignment) + " " + SINSstateDinamOdo.Count
                                 + " " + SINSstateDinamOdo.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstateDinamOdo.OdoSpeed_s)
                                 + " " + Math.Round(((SINSstateDinamOdo.Latitude - SINSstateDinamOdo.Latitude_Start) * SINSstateDinamOdo.R_n), 2)
                                 + " " + Math.Round(((SINSstateDinamOdo.Longitude - SINSstateDinamOdo.Longitude_Start) * SINSstateDinamOdo.R_e * Math.Cos(SINSstateDinamOdo.Latitude)), 2) 
                                 + " " + SINSstateDinamOdo.Altitude + " " + SINSstateDinamOdo.Altitude_Corr + " "
                                 + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstateDinamOdo.Latitude_Corr) * SINSstateDinamOdo.R_n), 2)
                                 + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstateDinamOdo.Longitude_Corr) * SINSstateDinamOdo.R_e * Math.Cos(SINSstateDinamOdo.Latitude)), 2)
                                 + " " + ((SINSstateDinamOdo.Latitude)) + " " + ((SINSstateDinamOdo.Longitude))
                                 + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstateDinamOdo.Latitude) * SINSstateDinamOdo.R_n), 2)
                                 + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstateDinamOdo.Longitude) * SINSstateDinamOdo.R_e), 2)
                                 + " " + Math.Round(ProcHelp.AltSNS, 2) + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                                 + " " + Math.Round(SINSstateDinamOdo.Vx_0[0], 3) + " " + Math.Round(SINSstateDinamOdo.Vx_0[1], 3) + " " + Math.Round(SINSstateDinamOdo.Vx_0[2], 3)
                                 + " " + Math.Round((SINSstateDinamOdo.Heading * SimpleData.ToDegree), 4)
                                 + " " + Math.Round((SINSstateDinamOdo.Roll * SimpleData.ToDegree), 4)
                                 + " " + Math.Round((SINSstateDinamOdo.Pitch * SimpleData.ToDegree), 4)
                                 + " " + Math.Round(ProcHelp.distance, 3) + " " + Math.Round(ProcHelp.distance_from_start, 3) + " " + Math.Round(SINSstateDinamOdo.V_norm, 3)
                                 + " " + OdoModel.V_increment_odo + " " + (OdoModel.V_increment_SINS + SINSstateDinamOdo.dV_q)
                                 + " " + Math.Round(SINSstateDinamOdo.Vx_0[0], 3) + " " + Math.Round(SINSstateDinamOdo.Vx_0[1], 3)
                                 + " " + SINSstateDinamOdo.OdometerVector[1] + " " + SINSstateDinamOdo.OdoSpeed_x0[1]
                                 ;
                DinamicOdometer.WriteLine(ProcHelp.datastring);

            }



            /*----------------------------------OUTPUT ESTIMATE------------------------------------------------------*/
            if (SINSstate.flag_FeedbackExist == false && SINSstate.flag_Autonomous_Solution == false && i % SINSstate.FreqOutput == 0)
            {
                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.Count
                                 + " " + SINSstate.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstate.OdoSpeed_s)
                                 + " " + Math.Round(((SINSstate2.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n), 2)
                                 + " " + Math.Round(((SINSstate2.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate2.Latitude)), 2) + " " + SINSstate2.Altitude
                                 + " " + ((SINSstate2.Latitude)) + " " + ((SINSstate2.Longitude))
                                 + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate2.Latitude) * SINSstate.R_n), 2)
                                 + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate2.Longitude) * SINSstate.R_e * Math.Cos(SINSstate2.Latitude)), 2)
                                 + " " + Math.Round(ProcHelp.AltSNS, 2) + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                                 + " " + Math.Round(SINSstate2.Vx_0[0], 3) + " " + Math.Round(SINSstate2.Vx_0[1], 3) + " " + Math.Round(SINSstate2.Vx_0[2], 3)
                                 + " " + ProcHelp.corrected
                                 + " " + Math.Round((SINSstate.Heading * SimpleData.ToDegree), 4) + " " + Math.Round((SINSstate2.Heading * SimpleData.ToDegree), 4)
                                 + " " + Math.Round((SINSstate.Roll * SimpleData.ToDegree), 4) + " " + Math.Round((SINSstate2.Roll * SimpleData.ToDegree), 4)
                                 + " " + Math.Round((SINSstate.Pitch * SimpleData.ToDegree), 4) + " " + Math.Round((SINSstate2.Pitch * SimpleData.ToDegree), 4) 
                                 + " " + Math.Round(ProcHelp.distance, 3) + " " + Math.Round(ProcHelp.distance_from_start, 3) + " " + Math.Round(SINSstate.V_norm, 3)
                                 + " " + OdoModel.V_increment_odo + " " + (OdoModel.V_increment_SINS + SINSstate.dV_q) 
                                 + " " + Math.Round(SINSstate.Vx_0[0], 3) + " " + Math.Round(SINSstate.Vx_0[1], 3)
                                 + " " + SINSstate.OdometerVector[1] + " " + SINSstate.OdoSpeed_x0[1]
                                 ;
                Nav_EstimateSolution.WriteLine(ProcHelp.datastring);

            }
            else
            {
                /*----------------------------------OUTPUT AUTONOMOUS------------------------------------------------------*/
                if (SINSstate.flag_Autonomous_Solution == true && i % SINSstate.FreqOutput == 0)
                {
                    ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.OdoTimeStepCount + " " + SINSstate.OdometerVector[1]
                         + " " + Math.Round(((SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n), 3)
                         + " " + Math.Round(((SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 3) + " " + SINSstate.Altitude
                         + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate.Latitude) * SINSstate.R_n), 3)
                         + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 3)
                         + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate.Latitude_Start) * SINSstate.R_n), 3)
                         + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 3)
                         + " " + ((ProcHelp.LatSNS * SimpleData.ToRadian)) + " " + ((ProcHelp.LongSNS * SimpleData.ToRadian ) )
                         + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                         + " " + Math.Round(SINSstate.Vx_0[0],3) + " " + Math.Round(SINSstate.Vx_0[1],3) + " " + Math.Round(SINSstate.Vx_0[2], 3)
                         + " " + Math.Round((SINSstate.Heading * SimpleData.ToDegree), 3) 
                         + " " + Math.Round((SINSstate.Roll * SimpleData.ToDegree), 3) + " " + Math.Round((SINSstate.Pitch * SimpleData.ToDegree), 3)
                         + " " + Math.Round(ProcHelp.distance, 3) + " " + Math.Round(ProcHelp.distance_from_start, 3) 
                         + " " + SINSstate.Azimth + " " + SINSstate.OdometerData.odometer_left.isReady
                         + " " + SINSstate.OdometerData.odometer_left.Value
                         + " " + Math.Round(SimpleOperations.AbsoluteVectorValue(SINSstate.Vx_0), 3)
                        ;
                    Nav_Autonomous.WriteLine(ProcHelp.datastring);
                }
                /*----------------------------------OUTPUT FEEDBACK------------------------------------------------------*/
                else if (i % SINSstate.FreqOutput == 0)
                {
                    ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.Count
                                        + " " + SINSstate.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstate.OdoSpeed_s)
                                        + " " + Math.Round(((SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n), 2)
                                        + " " + Math.Round(((SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 2) + " " + SINSstate.Altitude
                                        + " " + ((SINSstate.Latitude)) + " " + ((SINSstate.Longitude))
                                        + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate.Latitude) * SINSstate.R_n), 2)
                                        + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 2)
                                        + " " + Math.Round(ProcHelp.AltSNS, 2) + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                                        + " " + Math.Round(SINSstate.Vx_0[0], 3) + " " + Math.Round(SINSstate.Vx_0[1], 3) + " " + Math.Round(SINSstate.Vx_0[2], 3)
                                        + " " + Math.Round((SINSstate.Heading * SimpleData.ToDegree), 8) 
                                        + " " + Math.Round((SINSstate.Roll * SimpleData.ToDegree), 8) + " " + Math.Round((SINSstate.Pitch * SimpleData.ToDegree), 8)
                                        + " " + ProcHelp.corrected + " " + SINSstate.OdometerData.odometer_left.isReady
                                        + " " + ProcHelp.distance + " " + ProcHelp.distance_from_start + " " + SINSstate.FLG_Stop
                                        + " " + SINSstate.OdoSpeed_x0[0] + " " + SINSstate.OdoSpeed_x0[1]
                                        + " " + SimpleOperations.AbsoluteVectorValue(SINSstate.Vx_0)
                        // + " " + OdoModel.V_increment_odo + " " + (OdoModel.V_increment_SINS + SINSstate.dV_q) + " " + OdoModel.Can
                                        ;
                    Nav_FeedbackSolution.WriteLine(ProcHelp.datastring);
                }
            }


            if (SINSstate.Do_Smoothing == true && i % SINSstate.FreqOutput == 0)
            {
                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.Count
                                        //+ " " + SINSstate.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstate.OdoSpeed_s)
                                        + " " + Math.Round(((SINSstate_Smooth.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n), 2)
                                        + " " + Math.Round(((SINSstate_Smooth.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate_Smooth.Latitude)), 2) + " " + SINSstate_Smooth.Altitude
                                        + " " + ((SINSstate_Smooth.Latitude)) + " " + ((SINSstate_Smooth.Longitude))
                                        + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate_Smooth.Latitude) * SINSstate.R_n), 2)
                                        + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate_Smooth.Longitude) * SINSstate.R_e * Math.Cos(SINSstate_Smooth.Latitude)), 2)
                                        + " " + Math.Round(ProcHelp.AltSNS, 2) + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                                        + " " + Math.Round(SINSstate_Smooth.Vx_0[0], 3) + " " + Math.Round(SINSstate_Smooth.Vx_0[1], 3) + " " + Math.Round(SINSstate_Smooth.Vx_0[2], 3)
                                        + " " + Math.Round((SINSstate_Smooth.Heading * SimpleData.ToDegree), 8)
                                        + " " + Math.Round((SINSstate_Smooth.Roll * SimpleData.ToDegree), 8) + " " + Math.Round((SINSstate_Smooth.Pitch * SimpleData.ToDegree), 8)
                                        //+ " " + ProcHelp.corrected + " " + SINSstate.OdometerData.odometer_left.isReady
                                        //+ " " + ProcHelp.distance + " " + ProcHelp.distance_from_start + " " + SINSstate.FLG_Stop
                                        //+ " " + SINSstate.OdoSpeed_x0[0] + " " + SINSstate.OdoSpeed_x0[1]
                                        //+ " " + SimpleOperations.AbsoluteVectorValue(SINSstate.Vx_0)
                                        ;
                Nav_Smoothed.WriteLine(ProcHelp.datastring);
            }

            if (i % SINSstate.FreqOutput == 0)
            {
                /*----------------------------------OUTPUT ERRORS------------------------------------------------------*/
                if (!SINSstate.flag_Smoothing)
                {
                    ProcHelp.datastring = (SINSstate.DeltaLatitude * SimpleData.ToDegree) + " " + (SINSstate.DeltaLongitude * SimpleData.ToDegree) + " " + SINSstate.DeltaV_1 + " " + SINSstate.DeltaV_2 + " "
                                    + SINSstate.DeltaV_3 + " " + SINSstate.DeltaHeading + " " + SINSstate.DeltaRoll + " " + SINSstate.DeltaPitch;
                }
                Nav_Errors.WriteLine(ProcHelp.datastring);



                /*----------------------------------OUTPUT StateErrors------------------------------------------------------*/

                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + KalmanVars.ErrorConditionVector_p[0] + " " + KalmanVars.ErrorConditionVector_p[1] + " " + KalmanVars.ErrorConditionVector_p[2]
                                 + " " + KalmanVars.ErrorConditionVector_p[3] + " " + (KalmanVars.ErrorConditionVector_p[4] * SimpleData.ToDegree) + " " + (KalmanVars.ErrorConditionVector_p[5] * SimpleData.ToDegree)
                                  + " " + (KalmanVars.ErrorConditionVector_p[6] * SimpleData.ToDegree) + " " + (KalmanVars.ErrorConditionVector_p[7] * SimpleData.ToDegree * 3600.0) + " " + (KalmanVars.ErrorConditionVector_p[8] * SimpleData.ToDegree * 3600.0)
                                   + " " + (KalmanVars.ErrorConditionVector_p[9] * SimpleData.ToDegree * 3600.0) + " " + KalmanVars.ErrorConditionVector_p[10] + " " + KalmanVars.ErrorConditionVector_p[11]
                                    + " " + KalmanVars.ErrorConditionVector_p[12];
                if (SINSstate.flag_iMx_r3_dV3)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.iMx_r3_dV3] + " " + KalmanVars.ErrorConditionVector_p[SINSstate.iMx_r3_dV3 + 1];

                if (SINSstate.flag_Odometr_SINS_case)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.iMx_r12_odo] + " " + KalmanVars.ErrorConditionVector_p[SINSstate.iMx_r12_odo + 1];
                if (SINSstate.flag_Odometr_SINS_case && SINSstate.flag_Using_iMx_r_odo_3)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.iMx_r12_odo + 2];
                
                if (SINSstate.flag_iMx_kappa_13_ds)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.iMx_odo_model] * SimpleData.ToDegree
                                                              + " " + KalmanVars.ErrorConditionVector_p[SINSstate.iMx_odo_model + 1] * SimpleData.ToDegree
                                                              + " " + KalmanVars.ErrorConditionVector_p[SINSstate.iMx_odo_model + 2];

                //if (SINSstate.flag_DoFeedBackKappa && SINSstate.flag_Odometr_SINS_case)
                if (SINSstate.flag_FeedbackExist)
                {
                    if (SINSstate.flag_Odometr_SINS_case)
                        ProcHelp.datastring = ProcHelp.datastring + " " + SINSstate.ComulativeKappaEst[0] * SimpleData.ToDegree + " " + SINSstate.ComulativeKappaEst[2] * SimpleData.ToDegree + " " + SINSstate.ComulativeKappaEst[1];
                    if (!SINSstate.flag_Odometr_SINS_case && SINSstate.flag_iMx_kappa_13_ds)
                        ProcHelp.datastring = ProcHelp.datastring + " " + SINSstate.ComulativeKappaEst[0] * SimpleData.ToDegree + " " + SINSstate.ComulativeKappaEst[2] * SimpleData.ToDegree + " " + SINSstate.ComulativeKappaEst[1];
                }
                if (SINSstate.flag_DoFeedBackDeltaFW)
                {
                    ProcHelp.datastring = ProcHelp.datastring + " " + SINSstate.ComulativeInstrumental_Fz[0] + " " + SINSstate.ComulativeInstrumental_Fz[1] + " " + SINSstate.ComulativeInstrumental_Fz[2];
                    ProcHelp.datastring = ProcHelp.datastring + " " + SINSstate.ComulativeInstrumental_Wz[0] * SimpleData.ToDegree * 3600.0 + " " + SINSstate.ComulativeInstrumental_Wz[1] * SimpleData.ToDegree * 3600.0 + " " + SINSstate.ComulativeInstrumental_Wz[2] * SimpleData.ToDegree * 3600.0;
                }

                //ProcHelp.datastring += " " + KalmanVars.kappa1_est + " " + KalmanVars.kappa3_est;

                Nav_StateErrorsVector.WriteLine(ProcHelp.datastring);
            }

        }


    }
}
