﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace Common_Namespace
{
    public static class ImitatorFirstProcessing
    {
        public static void mainProcessing(SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, Proc_Help ProcHelp, StreamReader myFile, string GlobalPrefixTelemetric, ParamToStart ParamStart)
        {
            double Params_OdoKappa1 = 0.0, Params_OdoKappa3 = 0.0, Params_OdoIncrement = 0.0, Params_OdoScaleErr = 0.0, Params_OdoFrequency = 0.0;
            double OdometerData_odometer_left_Value = 0.0;
            double Params_df_s = 0.0, Params_dnu_s = 0.0;
            double[] Params_df_0 = new double[3], Params_dnu_0 = new double[3];

            Random rnd_1 = new Random(), rnd_2 = new Random(), rnd_3 = new Random(), rnd_4 = new Random(), rnd_5 = new Random(), rnd_6 = new Random();

            StreamWriter outFile = new StreamWriter(GlobalPrefixTelemetric + "_Errors.dat");
            StreamWriter Autonomous = new StreamWriter(SimpleData.PathOutputString + "S_AutoClear.txt");

            Autonomous.WriteLine("Time OdoCnt OdoV Latitude Longitude Altitude LatSNS-Lat LngSNS-Lng LatSNS LongSNS LatSNSrad LongSNSrad SpeedSNS V_x1  V_x2  V_x3 Yaw  Roll  Pitch PosError PosError_Start Azimth");

            //----------------------------------------------------------------------------------------//
            if (SINSstate.flag_AccuracyClass_NoErr)
            {
                Params_OdoKappa1 = 0 * SimpleData.ToRadian;
                Params_OdoKappa3 = 0 * SimpleData.ToRadian;
                Params_OdoIncrement = 0.0; // в сантиметрах
                Params_OdoScaleErr = 1.0;
                Params_OdoFrequency = 5;

                for (int j = 0; j < 3; j++)
                {
                    Params_df_0[j] = 0.0; //далее умножается G
                    Params_dnu_0[j] = 0.0; //град/час
                }
                Params_df_s = 0.0; //(rnd_1.NextDouble() - 0.5) / Params_df_s //100.0 - норма
                Params_dnu_s = 0.0; //(rnd_5.NextDouble() - 0.5) / Params_dnu_s //10000.0 - норма
            }
            else
            {
                Params_OdoKappa1 = ParamStart.Modeling_Params_OdoKappa1;
                Params_OdoKappa3 = ParamStart.Modeling_Params_OdoKappa3;
                Params_OdoIncrement = ParamStart.Modeling_Params_OdoIncrement; // в сантиметрах
                Params_OdoScaleErr = ParamStart.Modeling_Params_OdoScaleErr;
                Params_OdoFrequency = ParamStart.Modeling_Params_OdoFrequency;
                Params_df_s = ParamStart.Modeling_Params_df_s; //(rnd_1.NextDouble() - 0.5) / Params_df_s //100.0 - норма
                Params_dnu_s = ParamStart.Modeling_Params_dnu_s; //(rnd_5.NextDouble() - 0.5) / Params_dnu_s //10000.0 - норма

                if (SINSstate.flag_AccuracyClass_0_0grph)
                    for (int j = 0; j < 3; j++)
                    {
                        Params_df_0[j] = 0.0; //далее умножается G
                        Params_dnu_0[j] = 0.0; //град/час
                    }
                if (SINSstate.flag_AccuracyClass_0_02grph)
                    for (int j = 0; j < 3; j++)
                    {
                        Params_df_0[j] = 1E-5; //далее умножается G
                        Params_dnu_0[j] = 0.02; //град/час
                    }
                if (SINSstate.flag_AccuracyClass_0_2_grph)
                    for (int j = 0; j < 3; j++)
                    {
                        Params_df_0[j] = 1E-4; //далее умножается G
                        Params_dnu_0[j] = 0.2; //град/час
                    }
                if (SINSstate.flag_AccuracyClass_2_0_grph)
                    for (int j = 0; j < 3; j++)
                    {
                        Params_df_0[j] = 1E-3; //далее умножается G
                        Params_dnu_0[j] = 2.0; //град/час
                    }
            }
            //----------------------------------------------------------------------------------------//




            bool addNoisSample = true;
            int noisSampleCountDUS = 0, noisSampleCountAccs = 0;
            double[] avgSampleAccs = new double[3], avgSampleDUC = new double[3];
            double[] noisSampleDUS_1, noisSampleDUS_2, noisSampleDUS_3, noisSampleAccs_1, noisSampleAccs_2, noisSampleAccs_3;
            double[] sampleCurrSumDUC = new double[3], sampleCurrAvgDUC = new double[3], sampleCurrSumAccs = new double[3], sampleCurrAvgAccs = new double[3];

            string pathToSampleDUC = "D://SINS Solution//MovingImitator_Azimut//Imitator_data//20141207_AA_sensors.txt";
            //string pathToSampleDUC = "D://SINS Solution//MovingImitator_Azimut//Imitator_data//20141212_AA_accselsNoise.dat";
            StreamReader NoisImputSampleDUS = new StreamReader(pathToSampleDUC);
            if (addNoisSample)
            {
                for (noisSampleCountDUS = 0; ; noisSampleCountDUS++)
                {
                    if (NoisImputSampleDUS.EndOfStream == true || noisSampleCountDUS > 2000000)
                        break;
                    NoisImputSampleDUS.ReadLine();
                }
                NoisImputSampleDUS.Close();
                NoisImputSampleDUS = new StreamReader(pathToSampleDUC);
            }

            noisSampleDUS_1 = new double[noisSampleCountDUS];
            noisSampleDUS_2 = new double[noisSampleCountDUS];
            noisSampleDUS_3 = new double[noisSampleCountDUS];
            if (addNoisSample)
            {
                if (pathToSampleDUC == "D://SINS Solution//MovingImitator_Azimut//Imitator_data//20141207_AA_sensors.dat")
                {
                    for (int i = 0; i < noisSampleCountDUS; i++)
                    {
                        string str = NoisImputSampleDUS.ReadLine();
                        string[] strArray = str.Split(' ');
                        int t1 = 0;
                        for (int y = 0; y < strArray.Length; y++)
                        {
                            if (strArray[y] != "")
                                t1++;
                        }
                        string[] strArray2 = new string[t1];
                        t1 = 0;

                        for (int y = 0; y < strArray.Length; y++)
                        {
                            if (strArray[y] != "")
                            {
                                strArray2[t1] = strArray[y];
                                t1++;
                            }
                        }
                        noisSampleDUS_1[i] = Convert.ToDouble(strArray2[1]) * Math.PI / 180.0;
                        noisSampleDUS_2[i] = Convert.ToDouble(strArray2[1]) * Math.PI / 180.0;
                        noisSampleDUS_3[i] = Convert.ToDouble(strArray2[1]) * Math.PI / 180.0;
                    }
                }
                else if (pathToSampleDUC == "D://SINS Solution//MovingImitator_Azimut//Imitator_data//20141212_AA_accselsNoise.dat")
                {
                    for (int i = 0; i < noisSampleCountDUS; i++)
                    {
                        string str = NoisImputSampleDUS.ReadLine();
                        string[] strArray = str.Split(' ');
                        noisSampleDUS_1[i] = Convert.ToDouble(strArray[6]);
                        noisSampleDUS_2[i] = Convert.ToDouble(strArray[4]);
                        noisSampleDUS_3[i] = Convert.ToDouble(strArray[5]);
                    }
                }
            }
            avgSampleDUC[0] = noisSampleDUS_1.Sum() / noisSampleCountDUS;
            avgSampleDUC[1] = noisSampleDUS_2.Sum() / noisSampleCountDUS;
            avgSampleDUC[2] = noisSampleDUS_3.Sum() / noisSampleCountDUS;
            for (int i = 0; i < noisSampleCountDUS; i++)
            {
                noisSampleDUS_1[i] -= avgSampleDUC[0];
                noisSampleDUS_2[i] -= avgSampleDUC[1];
                noisSampleDUS_3[i] -= avgSampleDUC[2];
            }
            NoisImputSampleDUS.Close();


            /////////-----///////
            string pathToSampleACCS = "D://SINS Solution//MovingImitator_Azimut//Imitator_data//20141212_AA_accselsNoise.dat";
            StreamReader NoisImputSampleACCS = new StreamReader(pathToSampleACCS);
            if (addNoisSample)
            {
                for (noisSampleCountAccs = 0; ; noisSampleCountAccs++)
                {
                    if (NoisImputSampleACCS.EndOfStream == true || noisSampleCountAccs > 2000000)
                        break;
                    NoisImputSampleACCS.ReadLine();
                }
                NoisImputSampleACCS.Close();
                NoisImputSampleACCS = new StreamReader(pathToSampleACCS);
            }

            noisSampleAccs_1 = new double[noisSampleCountAccs];
            noisSampleAccs_2 = new double[noisSampleCountAccs];
            noisSampleAccs_3 = new double[noisSampleCountAccs];
            if (addNoisSample)
            {
                for (int i = 0; i < noisSampleCountAccs; i++)
                {
                    string str = NoisImputSampleACCS.ReadLine();
                    string[] strArray = str.Split(' ');
                    noisSampleAccs_1[i] = Convert.ToDouble(strArray[3]);
                    noisSampleAccs_2[i] = Convert.ToDouble(strArray[1]);
                    noisSampleAccs_3[i] = Convert.ToDouble(strArray[2]);
                }
            }
            avgSampleAccs[0] = noisSampleAccs_1.Sum() / noisSampleCountAccs;
            avgSampleAccs[1] = noisSampleAccs_2.Sum() / noisSampleCountAccs;
            avgSampleAccs[2] = noisSampleAccs_3.Sum() / noisSampleCountAccs;
            for (int i = 0; i < noisSampleCountAccs; i++)
            {
                noisSampleAccs_1[i] -= avgSampleAccs[0];
                noisSampleAccs_2[i] -= avgSampleAccs[1];
                noisSampleAccs_3[i] -= avgSampleAccs[2];
            }

            if (addNoisSample)
            {
                Params_df_0[0] = 5E-4;
                Params_df_0[1] = -5E-4;
                Params_df_0[2] = 3E-4;
                Params_dnu_0[0] = 0.005;
                Params_dnu_0[1] = -0.003;
                Params_dnu_0[2] = 0.004;
            }








            double[] Params_df_0_x0 = new double[3], Params_dnu_0_x0 = new double[3];
            SimpleOperations.CopyArray(Params_dnu_0_x0, SINSstate.A_x0s * Params_dnu_0);
            SimpleOperations.CopyArray(Params_df_0_x0, SINSstate.A_x0s * Params_df_0);

            outFile.WriteLine("Latitude= " + SINSstate.Latitude + " Longitude= " + SINSstate.Longitude + " Height= " + SINSstate.Altitude + " SINS_Freq= " + 1.0 / SINSstate.timeStep + " df_0(E)= "
                + Params_df_0_x0[0] + " df_0(N)= " + Params_df_0_x0[1] + " df_s= " + Params_df_s + " nu_0= " + Params_dnu_0_x0[0] + " nu_s= " + Params_dnu_s + " OdoKappa1= " + Params_OdoKappa1 + " OdoKappa3= " + Math.Abs(Params_OdoKappa3)
                + " OdoScale= " + Params_OdoScaleErr + " OdoIncrement= " + Params_OdoIncrement + " OdoFreq= " + Params_OdoFrequency + " Heading= " + (SINSstate.Heading - Params_OdoKappa3).ToString()
                + " Roll= " + SINSstate.Roll + " Pitch= " + (SINSstate.Pitch + Params_OdoKappa1).ToString());


            int t = 0;
            for (int i = 1; i < SINSstate.LastCountForRead; i++)
            {
                ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);
                ProcessingHelp.DefSNSData(ProcHelp, SINSstate);

                if (t == 0) { SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z); SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z); t = 1; }

                SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate, SINSstate_OdoMod);


                /*----------------------------------OUTPUT AUTONOMOUS------------------------------------------------------*/
                if (i % SINSstate.FreqOutput == 0)
                {
                    ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.OdoTimeStepCount + " " + SINSstate.OdometerVector[1]
                         + " " + Math.Round(((SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n), 3)
                         + " " + Math.Round(((SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 3) + " " + SINSstate.Altitude
                         + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate.Latitude) * SINSstate.R_n), 3)
                         + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 3)
                         + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate.Latitude_Start) * SINSstate.R_n), 3)
                         + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 3)
                         + " " + ((ProcHelp.LatSNS * SimpleData.ToRadian)) + " " + ((ProcHelp.LongSNS * SimpleData.ToRadian))
                         + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                         + " " + Math.Round(SINSstate.Vx_0[0], 3) + " " + Math.Round(SINSstate.Vx_0[1], 3) + " " + Math.Round(SINSstate.Vx_0[2], 3)
                         + " " + Math.Round((SINSstate.Heading * SimpleData.ToDegree), 3)
                         + " " + Math.Round((SINSstate.Roll * SimpleData.ToDegree), 3) + " " + Math.Round((SINSstate.Pitch * SimpleData.ToDegree), 3)
                         + " " + Math.Round(ProcHelp.distance, 3) + " " + Math.Round(ProcHelp.distance_from_start, 3)
                         + " " + SINSstate.Azimth + " " + SINSstate.OdometerData.odometer_left.isReady
                         + " " + SINSstate.OdometerData.odometer_left.Value
                         + " " + Math.Round(SimpleOperations.AbsoluteVectorValue(SINSstate.Vx_0), 3)
                        ;
                    Autonomous.WriteLine(ProcHelp.datastring);
                }



                /*------------------------------------FORMING ERRORS-------------------------------------------------*/

                //---расчет с учетом инкремента---//
                double odometer_left_ValueTrue = SINSstate.OdometerData.odometer_left.Value;
                SINSstate.OdometerData.odometer_left.Value = SINSstate.OdometerData.odometer_left.Value * Params_OdoScaleErr;
                if (Params_OdoIncrement > 0.001)
                {
                    double tmp1 = Math.Floor(SINSstate.OdometerData.odometer_left.Value);
                    double tmp2 = Math.Floor(SINSstate.OdometerData.odometer_left.Value * 100) - Math.Floor(SINSstate.OdometerData.odometer_left.Value) * 100;
                    double tmp3 = Math.Floor((tmp2) / Params_OdoIncrement);
                    OdometerData_odometer_left_Value = tmp1 + tmp3 * Params_OdoIncrement / 100.0;
                }
                else
                    OdometerData_odometer_left_Value = SINSstate.OdometerData.odometer_left.Value;

                if (i % Params_OdoFrequency == 0)
                    SINSstate.OdometerData.odometer_left.isReady = 1;
                else
                    SINSstate.OdometerData.odometer_left.isReady = 2;

                //---Поворот приборных осей относительно динамической---//
                double[] kappa = new double[3];
                kappa[0] = Params_OdoKappa1;
                kappa[2] = Params_OdoKappa3;
                SimpleOperations.CopyArray(SINSstate.F_z, SimpleOperations.A_odoZ(kappa[0], kappa[2]) * SINSstate.F_z);
                SimpleOperations.CopyArray(SINSstate.W_z, SimpleOperations.A_odoZ(kappa[0], kappa[2]) * SINSstate.W_z);


                if (Math.Abs(Params_df_s) > 0.1 && Math.Abs(Params_dnu_s) > 0.1)
                {
                    SINSstate.F_z[0] += (rnd_1.NextDouble() - 0.5) / Params_df_s;
                    SINSstate.F_z[1] += (rnd_2.NextDouble() - 0.5) / Params_df_s;
                    SINSstate.F_z[2] += (rnd_3.NextDouble() - 0.5) / Params_df_s;
                    SINSstate.W_z[0] -= (rnd_4.NextDouble() - 0.5) / Params_dnu_s;
                    SINSstate.W_z[1] -= (rnd_5.NextDouble() - 0.5) / Params_dnu_s;
                    SINSstate.W_z[2] -= (rnd_6.NextDouble() - 0.5) / Params_dnu_s;
                }

                SINSstate.F_z[0] += Params_df_0[0] * 9.81;
                SINSstate.F_z[1] += Params_df_0[1] * 9.81;
                SINSstate.F_z[2] += Params_df_0[2] * 9.81;

                SINSstate.W_z[0] -= Params_dnu_0[0] * SimpleData.ToRadian / 3600.0;
                SINSstate.W_z[1] -= Params_dnu_0[1] * SimpleData.ToRadian / 3600.0;
                SINSstate.W_z[2] -= Params_dnu_0[2] * SimpleData.ToRadian / 3600.0;




                SINSstate.GPS_Data.gps_Latitude.isReady = 2;
                SINSstate.GPS_Data.gps_Longitude.isReady = 2;
                SINSstate.GPS_Data.gps_Altitude.isReady = 2;

                SimpleOperations.CopyArray(SINSstate.Vz, SINSstate.A_sx0 * SINSstate.Vx_0);
                if (odometer_left_ValueTrue % ParamStart.Imitator_GPS_IsReadyDistance < SINSstate.Vz[1] * SINSstate.timeStep + 0.01 && odometer_left_ValueTrue > 1.0)
                {
                    SINSstate.GPS_Data.gps_Latitude.isReady = 1;
                    SINSstate.GPS_Data.gps_Longitude.isReady = 1;
                    SINSstate.GPS_Data.gps_Altitude.isReady = 1;
                }




                //===ПОПЫТКА ДОБАВИТЬ ШУМЫ ПО СЭМПЛУ С РЕАЛЬНЫХ ДАТЧИКОВ===
                if (addNoisSample)
                {
                    SINSstate.W_z[0] -= noisSampleDUS_1[Convert.ToInt32(SINSstate.Count) % noisSampleCountDUS];
                    SINSstate.W_z[1] -= noisSampleDUS_2[Convert.ToInt32(SINSstate.Count) % noisSampleCountDUS];
                    SINSstate.W_z[2] -= noisSampleDUS_3[Convert.ToInt32(SINSstate.Count) % noisSampleCountDUS];

                    SINSstate.F_z[0] += noisSampleAccs_1[Convert.ToInt32(SINSstate.Count) % noisSampleCountAccs];
                    SINSstate.F_z[1] += noisSampleAccs_2[Convert.ToInt32(SINSstate.Count) % noisSampleCountAccs];
                    SINSstate.F_z[2] += noisSampleAccs_3[Convert.ToInt32(SINSstate.Count) % noisSampleCountAccs];
                }
                //===ПОПЫТКА ДОБАВИТЬ ШУМЫ ПО СЭМПЛУ С РЕАЛЬНЫХ ДАТЧИКОВ===





                /*------------------------------------OUTPUT-------------------------------------------------*/

                outFile.WriteLine(SINSstate.Count + " " + SINSstate.F_z[1] + " " + SINSstate.F_z[2] + " " + SINSstate.F_z[0] + " " + SINSstate.W_z[1] + " " + SINSstate.W_z[2] + " " + SINSstate.W_z[0]
                     + " " + SINSstate.Latitude + " " + SINSstate.GPS_Data.gps_Latitude.isReady.ToString() + " " + SINSstate.Longitude.ToString()
                     + " " + SINSstate.GPS_Data.gps_Longitude.isReady.ToString() + " " + SINSstate.Altitude.ToString() + " " + SINSstate.GPS_Data.gps_Altitude.isReady.ToString()
                     + " " + SINSstate.Vx_0[1].ToString() + " " + SINSstate.GPS_Data.gps_Vn.isReady.ToString() + " " + SINSstate.Vx_0[0].ToString() + " " + SINSstate.GPS_Data.gps_Ve.isReady.ToString()
                     + " " + SINSstate.FLG_Stop.ToString() 
                     + " " + OdometerData_odometer_left_Value.ToString() + " " + SINSstate.OdometerData.odometer_left.isReady.ToString()
                     + " " + SINSstate.OdometerData.odometer_left.Value.ToString() + " " + SINSstate.OdometerData.odometer_right.isReady.ToString()
                     + " " + (SINSstate.Heading - Params_OdoKappa3) + " " + SINSstate.Roll + " " + (SINSstate.Pitch + Params_OdoKappa1)); 


            }

            outFile.Close();
            Autonomous.Close();
        }
    }
}
