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
            //if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
            if (SINSstate.GPS_Data.gps_Latitude.Value > 0.1)
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

            if (myFile.EndOfStream) return;

            if (SINSstate.Global_file == "topo")
            {
                if (SINSstate.firstLineRead == false)
                {
                    ProcHelp.datastring = myFile.ReadLine();
                    dataArray = ProcHelp.datastring.Split(' ');
                    SINSstate.Count++;
                    SINSstate.Time_prev = Convert.ToDouble(dataArray[1]);
                    SINSstate.OdometerLeftPrev = Convert.ToDouble(dataArray[23]) / 1000.0;
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


                if ((Convert.ToInt32(dataArray[2]) == 1 && SINSstate.timeStep > 0.5) || SINSstate.Count <= ProcHelp.AlignmentCounts)
                    SINSstate.FLG_Stop = 1;
                else
                    SINSstate.FLG_Stop = 0;

                SINSstate.OdometerData.odometer_left.Value = Convert.ToDouble(dataArray[23]) / 1000.0;

                if (SINSstate.Count % SINSstate.OdoLimitMeasuresNum == 0)
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
                    if (dataArray[y] != "")
                        t++;
                string[] dataArray2 = new string[t];
                t = 0;

                for (int y = 0; y < dataArray.Length; y++)
                    if (dataArray[y] != "")
                    {
                        dataArray2[t] = dataArray[y];
                        t++;
                    }

                //SINSstate.Time = Convert.ToDouble(dataArray2[0]);
                SINSstate.Count = Convert.ToDouble(dataArray2[0]);

                if (ProcHelp.initCount == false) { ProcHelp.initCount = true; SINSstate.initCount = SINSstate.Count - 1; }

                SINSstate.Time = (SINSstate.Count - SINSstate.initCount) * Math.Abs(SINSstate.timeStep);

                SINSstate.F_z[1] = Convert.ToDouble(dataArray2[1]);
                SINSstate.F_z[2] = Convert.ToDouble(dataArray2[2]);
                SINSstate.F_z[0] = Convert.ToDouble(dataArray2[3]);

                SINSstate.W_z[1] = Convert.ToDouble(dataArray2[4]);
                SINSstate.W_z[2] = Convert.ToDouble(dataArray2[5]);
                SINSstate.W_z[0] = Convert.ToDouble(dataArray2[6]);


                // --- Введение ЛОЖНЫХ углов установки БИНС на корпусе объекта, если определено --- //
                if (Math.Abs(SINSstate.WRONG_alpha_kappa_1) > 0.0 || Math.Abs(SINSstate.WRONG_alpha_kappa_3) > 0.0)
                {
                    double[] fz = new double[3], Wz = new double[3];

                    fz[0] = SINSstate.F_z[0] - SINSstate.WRONG_alpha_kappa_3 * SINSstate.F_z[1];
                    fz[1] = SINSstate.F_z[1] + SINSstate.WRONG_alpha_kappa_3 * SINSstate.F_z[0] - SINSstate.WRONG_alpha_kappa_1 * SINSstate.F_z[2];
                    fz[2] = SINSstate.F_z[2] + SINSstate.WRONG_alpha_kappa_1 * SINSstate.F_z[1];

                    Wz[0] = SINSstate.W_z[0] - SINSstate.WRONG_alpha_kappa_3 * SINSstate.W_z[1];
                    Wz[1] = SINSstate.W_z[1] + SINSstate.WRONG_alpha_kappa_3 * SINSstate.W_z[0] - SINSstate.WRONG_alpha_kappa_1 * SINSstate.W_z[2];
                    Wz[2] = SINSstate.W_z[2] + SINSstate.WRONG_alpha_kappa_1 * SINSstate.W_z[1];

                    SimpleOperations.CopyArray(SINSstate.F_z, fz);
                    SimpleOperations.CopyArray(SINSstate.W_z, Wz);
                }


                // --- Поворот показаний датчиков на заднные углы несоосности БИНС и динамических осей объекта --- //
                {
                    double[] fz = new double[3], Wz = new double[3];

                    fz[0] = SINSstate.F_z[0] - SINSstate.alpha_kappa_3 * SINSstate.F_z[1];
                    fz[1] = SINSstate.F_z[1] + SINSstate.alpha_kappa_3 * SINSstate.F_z[0] - SINSstate.alpha_kappa_1 * SINSstate.F_z[2];
                    fz[2] = SINSstate.F_z[2] + SINSstate.alpha_kappa_1 * SINSstate.F_z[1];

                    Wz[0] = SINSstate.W_z[0] - SINSstate.alpha_kappa_3 * SINSstate.W_z[1];
                    Wz[1] = SINSstate.W_z[1] + SINSstate.alpha_kappa_3 * SINSstate.W_z[0] - SINSstate.alpha_kappa_1 * SINSstate.W_z[2];
                    Wz[2] = SINSstate.W_z[2] + SINSstate.alpha_kappa_1 * SINSstate.W_z[1];

                    SimpleOperations.CopyArray(SINSstate.F_z, fz);
                    SimpleOperations.CopyArray(SINSstate.W_z, Wz);
                }



                //---Запоминаем координаты предыдущей контрольной точки
                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    SINSstate.GPS_Data.gps_Latitude_prev.Value = SINSstate.GPS_Data.gps_Latitude.Value;
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

                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                    SINSstate.GPS_CounterOfPoints++;

                SINSstate.GPS_Data.gps_Vn.Value = Convert.ToDouble(dataArray2[13]);
                SINSstate.GPS_Data.gps_Vn.isReady = Convert.ToInt32(dataArray2[14]);
                SINSstate.GPS_Data.gps_Ve.Value = Convert.ToDouble(dataArray2[15]);
                SINSstate.GPS_Data.gps_Ve.isReady = Convert.ToInt32(dataArray2[16]);

                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    SINSstate.GPS_Data.gps_Vn.Value_prev = SINSstate.GPS_Data.gps_Vn.Value;
                    SINSstate.GPS_Data.gps_Ve.Value_prev = SINSstate.GPS_Data.gps_Ve.Value;
                }

                SINSstate.FLG_Stop = Convert.ToInt32(dataArray2[17]);

                SINSstate.OdometerData.odometer_left.Value = Convert.ToDouble(dataArray2[18]);
                SINSstate.OdometerData.odometer_left.isReady = Convert.ToInt32(dataArray2[19]);
                SINSstate.OdometerData.odometer_right.Value = Convert.ToDouble(dataArray2[20]);
                SINSstate.OdometerData.odometer_right.isReady = Convert.ToInt32(dataArray2[21]);


                // --- Введение ЛОЖНОЙ ошибки масштаба одометра, если определено --- //
                if (Math.Abs(SINSstate.WRONG_scaleError) > 0.0)
                    SINSstate.OdometerData.odometer_left.Value /= (1.0 + SINSstate.WRONG_scaleError);


                // --- Делаем поправку на введенное значение ошибки масштаба одометра
                SINSstate.OdometerData.odometer_left.Value /= (1.0 + SINSstate.alpha_scaleError);
                SINSstate.OdometerData.odometer_right.Value /= (1.0 + SINSstate.alpha_scaleError);

                SINSstate.OdometerData.odometer_left_ABS.Value += Math.Abs(SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev);


                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    SINSstate.OdometerData.odometer_left.Value_prev = SINSstate.OdometerData.odometer_left.Value;

                    SINSstate.OdoLimitMeasuresNum_Count++;
                    if (SINSstate.OdoLimitMeasuresNum_Count < SINSstate.OdoLimitMeasuresNum)
                    {
                        SINSstate.OdometerData.odometer_left.isReady = 2;
                        SINSstate.OdometerData.odometer_right.isReady = 2;
                    }
                    else
                        SINSstate.OdoLimitMeasuresNum_Count = 0;
                }

                if (SINSstate.Global_file == "Saratov_run_2014_07_23")
                {
                    SINSstate.timeStep = SINSstate.Freq = Convert.ToDouble(dataArray2[22]);
                    SINSstate.Time = Convert.ToDouble(dataArray2[0]) - SINSstate.Time_Alignment;

                    //SINSstate.OdometerData.odometer_left.Value = SINSstate.OdometerData.odometer_left.Value * 1.0015;
                }

                if (SINSstate.Global_file.ToLower().Contains("imitator") && dataArray2.Length >= 22)
                    SINSstate.HeadingImitator = Convert.ToDouble(dataArray2[22]);


                //--- Поправки для пересчета в Кроссовского ---//
                if (SINSstate.Global_file == "ktn004_15.03.2012")
                {
                    double amendmentLatitude = 2.8596974074074074074074074074074e-6
                        , amendmentLongitude = - 3.1372635679012345679012345679012e-5;

                    SINSstate.GPS_Data.gps_Latitude.Value += amendmentLatitude;
                    SINSstate.GPS_Data.gps_Longitude.Value += amendmentLongitude;
                }
                //--- Поправки для пересчета в Кроссовского ---//
                if (SINSstate.Global_file == "Azimut_15.08.2012")
                {
                    double amendmentLatitude = 3.7787777777777777777777777777778e-4 * SimpleData.ToRadian
                        , amendmentLongitude = -0.00151686666666666666666666666667 * SimpleData.ToRadian;

                    SINSstate.GPS_Data.gps_Latitude.Value += amendmentLatitude;
                    SINSstate.GPS_Data.gps_Longitude.Value += amendmentLongitude;
                }






                // --- Сохраняем оригинальные считанные значение, если проставлен флаг вывода в GRTV ---//
                if (SINSstate.flag_GRTV_output)
                {
                    for (int y = 0; y < 3; y++)
                    {
                        SINSstate.F_z_orig[y] = SINSstate.F_z[y] / 9.81;
                        SINSstate.W_z_orig[y] = SINSstate.W_z[y];
                    }

                    SINSstate.OdometerData.odometer_left.Value_orig = Convert.ToDouble(dataArray2[18]);
                    SINSstate.OdometerData.odometer_left.isReady_orig = Convert.ToInt32(dataArray2[19]);
                    if (SINSstate.OdometerData.odometer_left.isReady_orig != 1)
                    {
                        SINSstate.OdometerData.odometer_left.isReady_orig = 0;
                        SINSstate.OdometerData.odometer_left.Value_orig = SINSstate.OdometerData.odometer_left.Value_prev;
                    }

                    SINSstate.GPS_Data.gps_Vn.Value_orig = Convert.ToDouble(dataArray2[13]);
                    SINSstate.GPS_Data.gps_Vn.isReady_orig = Convert.ToInt32(dataArray2[14]);
                    if (SINSstate.GPS_Data.gps_Vn.isReady_orig != 1)
                    {
                        SINSstate.GPS_Data.gps_Vn.isReady_orig = 0;
                        SINSstate.GPS_Data.gps_Vn.Value_orig = SINSstate.GPS_Data.gps_Vn.Value_prev;
                    }
                    SINSstate.GPS_Data.gps_Ve.Value_orig = Convert.ToDouble(dataArray2[15]);
                    SINSstate.GPS_Data.gps_Ve.isReady_orig = Convert.ToInt32(dataArray2[16]);
                    if (SINSstate.GPS_Data.gps_Ve.isReady_orig != 1)
                    {
                        SINSstate.GPS_Data.gps_Ve.isReady_orig = 0;
                        SINSstate.GPS_Data.gps_Ve.Value_orig = SINSstate.GPS_Data.gps_Ve.Value_prev;
                    }



                    SINSstate.GPS_Data.gps_Latitude.Value_orig = Convert.ToDouble(dataArray2[7]);
                    SINSstate.GPS_Data.gps_Longitude.Value_orig = Convert.ToDouble(dataArray2[9]);
                    SINSstate.GPS_Data.gps_Altitude.Value_orig = Convert.ToDouble(dataArray2[11]);

                    if (SINSstate.GPS_Data.gps_Latitude.isReady != 1)
                    {
                        SINSstate.GPS_Data.gps_Latitude.Value_orig = SINSstate.GPS_Data.gps_Latitude_prev.Value;
                        SINSstate.GPS_Data.gps_Longitude.Value_orig = SINSstate.GPS_Data.gps_Longitude_prev.Value;
                        SINSstate.GPS_Data.gps_Altitude.Value_orig = SINSstate.GPS_Data.gps_Altitude_prev.Value;
                    }

                    SINSstate.GPS_Data.gps_Latitude.isReady_orig = Convert.ToInt32(dataArray2[8]);
                    SINSstate.GPS_Data.gps_Longitude.isReady_orig = Convert.ToInt32(dataArray2[10]);
                    SINSstate.GPS_Data.gps_Altitude.isReady_orig = Convert.ToInt32(dataArray2[12]);

                    if (SINSstate.GPS_Data.gps_Latitude.isReady_orig != 1)
                        SINSstate.GPS_Data.gps_Latitude.isReady_orig = 0;
                    if (SINSstate.GPS_Data.gps_Longitude.isReady_orig != 1)
                        SINSstate.GPS_Data.gps_Longitude.isReady_orig = 0;
                    if (SINSstate.GPS_Data.gps_Altitude.isReady_orig != 1)
                        SINSstate.GPS_Data.gps_Altitude.isReady_orig = 0;
                }

            }


            if (SINSstate.firstLineRead == false)
            {
                SINSstate.Roll_prev = SINSstate.Roll;
                SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
                SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
                SINSstate.OdometerStartValue = SINSstate.OdometerData.odometer_left.Value;
                SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                SINSstate.firstLineRead = true;
            }
        }



        public static void OutPutInfo_Chechpoints(SINS_State SINSstate, SINS_State SINSstate2, StreamWriter Dif_GK)
        {
            double[] Min_Distance_to_Points = new double[6];

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

        public static void OutPutInfo_Telemetric(SINS_State SINSstate, SINS_State SINSstate2, System.IO.StreamWriter Imitator_Telemetric, int i, int l)
        {
            //====Вывод данных для телеметрического имитатора====
            if (SINSstate.flag_Imitator_Telemetric && SINSstate.Global_file != "Saratov_run_2014_07_23")
            {
                double tmpRoundFreq50 = Math.Round(1.0 / SINSstate.Freq / 50.0, 0) * 50.0;
                int tmpFreqOut10 = Convert.ToInt32(tmpRoundFreq50 / 10.0);

                if ((i - l) % tmpFreqOut10 == 0)
                {
                    if (SINSstate.flag_FeedbackExist)
                        Imitator_Telemetric.WriteLine(((i - l + tmpFreqOut10) / tmpRoundFreq50).ToString()
                                + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Height
                                + " " + (SINSstate.Heading * SimpleData.ToDegree) + " " + (SINSstate.Roll * SimpleData.ToDegree) + " " + (SINSstate.Pitch * SimpleData.ToDegree));
                    else if (SINSstate.flag_EstimateExist)
                        Imitator_Telemetric.WriteLine(((i - l + tmpFreqOut10) / tmpRoundFreq50).ToString()
                                + " " + SINSstate2.Latitude + " " + SINSstate2.Longitude + " " + SINSstate2.Height
                                + " " + (SINSstate2.Heading * SimpleData.ToDegree) + " " + (SINSstate2.Roll * SimpleData.ToDegree) + " " + (SINSstate2.Pitch * SimpleData.ToDegree));
                }
            }
        }




        public static void OutPutInfo(int i, int start_i, Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod, SINS_State SINSstate_Smooth, Kalman_Vars KalmanVars, StreamWriter Nav_EstimateSolution, StreamWriter Nav_Autonomous,
                StreamWriter Nav_FeedbackSolution, StreamWriter Nav_StateErrorsVector, StreamWriter Nav_Errors, StreamWriter STD_data, StreamWriter Speed_Angles, StreamWriter DinamicOdometer, StreamWriter Nav_Smoothed, StreamWriter KMLFileOut, StreamWriter KMLFileOutSmoothed,
                StreamWriter GRTV_output,
                StreamWriter Cicle_Debag_Solution,
                StreamWriter Check_Measurement
            )
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


            ProcHelp.distance = Math.Sqrt(Math.Pow((Lat - ProcHelp.LatSNS * SimpleData.ToRadian) * SimpleOperations.RadiusN(Lat, SINSstate.Height), 2) +
                                 Math.Pow((Long - ProcHelp.LongSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(Lat, SINSstate.Height) * Math.Cos(Lat), 2));
            ProcHelp.distance_from_start = Math.Sqrt(Math.Pow((Lat - SINSstate.Latitude_Start) * SimpleOperations.RadiusN(Lat, SINSstate.Height), 2) +
                                 Math.Pow((Long - SINSstate.Longitude_Start) * SimpleOperations.RadiusE(Lat, SINSstate.Height) * Math.Cos(Lat), 2));


            //if (SINSstate.Time + SINSstate.Time_Alignment < 2955.53) return;


            // --- Вывод в файл данных, необходимых для формирования GRTV бинарного фалйа --- //
            if (SINSstate.flag_GRTV_output == true && SINSstate.NowSmoothing == false)
            {
                if (i == start_i)
                {
                    StreamWriter GRTV_init_output = new StreamWriter(SimpleData.PathOutputString + "S_GRTV_init_output.txt");

                    GRTV_init_output.WriteLine(
                        "fSamplingInterval " + SINSstate.timeStep + "\n"
                        + "nInstallationModel 0\n"                               // 0 – продольная ось прибора совпадает с продольной осью объекта
                        + "flLatitudeID " + SINSstate.Latitude_Start + " 1\n"
                        + "flLongitudeID " + SINSstate.Longitude_Start + " 1\n"
                        + "flHeightID " + SINSstate.Height_Start + " 1\n"
                        + "flTrueHeadID " + SINSstate.Heading + " 1\n"     //начальная долгота, заданная с пульта [рад]
                        + "flAzimuthMisalignment 0.0 0\n"                          // Угол азимутального рассогласования
                        + "flElevation 0.0 0"                                    //начальный угол возвышения ИНС [рад]
                        );

                    GRTV_init_output.Close();
                }


                int modeGRTV = 16;
                if (i < ProcHelp.AlignmentCounts) modeGRTV = 8;

                GRTV_output.WriteLine(
                    SINSstate.Count
                    + " " + modeGRTV + " "  
                    + " " + SINSstate.F_z_orig[1] + " " + SINSstate.F_z_orig[2] + " " + SINSstate.F_z_orig[0]
                    + " " + SINSstate.W_z_orig[1] + " " + SINSstate.W_z_orig[2] + " " + SINSstate.W_z_orig[0]

                    + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Height
                    + " " + SINSstate.Vx_0[1] + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[2]

                    + " " + SINSstate.Heading + " " + SINSstate.Pitch + " " + SINSstate.Roll
                    + " " + SINSstate.Latitude + " 1 " + SINSstate.Longitude + " 1 " + SINSstate.Height + " 1"
                    + " " + SINSstate.Vx_0[1] + " 1 " + SINSstate.Vx_0[0] + " 1 " + SINSstate.Vx_0[2] + " 1"

                    + " " + SINSstate.OdometerData.odometer_left.Value_orig + " " + SINSstate.OdometerData.odometer_left.isReady_orig

                    //метка времени - отмечает момент времени формирования пакета СНС-данных

                    //+ " " + SINSstate.GPS_Data.gps_Latitude.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Latitude.Value_orig + " " + SINSstate.GPS_Data.gps_Latitude.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Longitude.Value_orig + " " + SINSstate.GPS_Data.gps_Longitude.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Altitude.Value_orig + " " + SINSstate.GPS_Data.gps_Altitude.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Vn.Value_orig + " " + SINSstate.GPS_Data.gps_Vn.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Ve.Value_orig + " " + SINSstate.GPS_Data.gps_Vn.isReady_orig

                    + " 0" + " 0" + " 0" + " 0" + " 0"+ " 0" + " 0" + " 0" + " 0" + " 0" + " 0"


                    + " " + " 0 0" //Скорость GPS вертикальная
                    );
            }


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
                    if (SINSstate.flag_Odometr_SINS_case == true && SINSstate.flag_iMx_r3_dV3 == true)
                        str_hat += "std_odo_dr3 ";

                    str_hat += "std_dV1 std_dV2 ";
                    if (SINSstate.flag_iMx_r3_dV3)
                        str_hat += "std_dV3 ";

                    str_hat += "stdAlpha1(grad) stdAlpha2(grad) stdBeta3(grad) stdNu1 stdNu2 stdNu3 std_df1 std_df2 std_df3 ";

                    if (SINSstate.flag_iMx_kappa_13_ds)
                        str_hat += "stdKappa1(grad) stdKappa3(grad) stdScale ";

                    STD_data.WriteLine(str_hat);
                }

                if (SINSstate.flag_iMx_r3_dV3)
                    str = str + " " + KalmanProcs.Sigmf_Disp(0, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(1, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_dr3, KalmanVars);
                else 
                    str = str + " " + KalmanProcs.Sigmf_Disp(0, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(1, KalmanVars);

                if (SINSstate.flag_Odometr_SINS_case == true)
                    str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_r_odo_12, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_r_odo_12 + 1, KalmanVars);
                if (SINSstate.flag_Odometr_SINS_case == true && SINSstate.flag_iMx_r3_dV3 == true)
                    str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_r_odo_3, KalmanVars);

                str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_dV_12, KalmanVars) + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_dV_12 + 1, KalmanVars);
                if (SINSstate.flag_iMx_r3_dV3)
                    str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_dV3, KalmanVars);

                str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_alphaBeta, KalmanVars) * SimpleData.ToDegree
                    + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_alphaBeta + 1, KalmanVars) * SimpleData.ToDegree
                    + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_alphaBeta + 2, KalmanVars) * SimpleData.ToDegree;

                str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_Nu0, KalmanVars)
                    + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_Nu0 + 1, KalmanVars)
                    + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_Nu0 + 2, KalmanVars);

                str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_f0_12, KalmanVars)
                    + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_f0_12 + 1, KalmanVars);

                if (SINSstate.value_iMx_f0_3 > 0)
                    str += " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_f0_3, KalmanVars);
                else
                    str += " 0";

                if (SINSstate.flag_iMx_kappa_13_ds)
                {
                    if (SINSstate.value_iMx_kappa_1 > 0)
                        str += " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_kappa_1 + 0, KalmanVars) * SimpleData.ToDegree;
                    else
                        str += " 0";

                    str = str + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_kappa_3_ds + 0, KalmanVars) * SimpleData.ToDegree
                        + " " + KalmanProcs.Sigmf_Disp(SINSstate.value_iMx_kappa_3_ds + 1, KalmanVars);
                }

                STD_data.WriteLine(str);
            }


            /*----------------------------------OUTPUT DinamicOdometer------------------------------------------------------*/
            if (i % SINSstate.FreqOutput == 0)
            {
                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.Count
                                 + " " + SINSstate.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_s)
                                 + " " + Math.Round(((SINSstate_OdoMod.Latitude - SINSstate.Latitude_Start) * SINSstate_OdoMod.R_n), 2)
                                 + " " + Math.Round(((SINSstate_OdoMod.Longitude - SINSstate.Longitude_Start) * SINSstate_OdoMod.R_e * Math.Cos(SINSstate_OdoMod.Latitude)), 2)
                                 + " " + SINSstate_OdoMod.Height + " " + SINSstate_OdoMod.Altitude_Corr + " "
                                 + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate_OdoMod.Latitude_Corr) * SINSstate_OdoMod.R_n), 2)
                                 + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate_OdoMod.Longitude_Corr) * SINSstate_OdoMod.R_e * Math.Cos(SINSstate_OdoMod.Latitude)), 2)
                                 + " " + ((SINSstate_OdoMod.Latitude) * SimpleData.ToDegree) + " " + ((SINSstate_OdoMod.Longitude) * SimpleData.ToDegree)
                                 + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate_OdoMod.Latitude) * SINSstate_OdoMod.R_n), 2)
                                 + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate_OdoMod.Longitude) * SINSstate_OdoMod.R_e), 2)
                                 + " " + Math.Round(ProcHelp.AltSNS, 2) + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                                 + " " + Math.Round(SINSstate_OdoMod.Vx_0[0], 3) + " " + Math.Round(SINSstate_OdoMod.Vx_0[1], 3) + " " + Math.Round(SINSstate_OdoMod.Vx_0[2], 3)
                                 + " " + SINSstate_OdoMod.OdometerVector[1] + " " + SINSstate_OdoMod.OdoSpeed_x0[1]
                                 ;
                DinamicOdometer.WriteLine(ProcHelp.datastring);
            }


            /*----------------------------------OUTPUT ESTIMATE------------------------------------------------------*/
            if (SINSstate.flag_FeedbackExist == false && SINSstate.flag_Autonomous_Solution == false && i % SINSstate.FreqOutput == 0)
            {
                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.Count
                                 + " " + SINSstate.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstate.OdoSpeed_s)
                                 + " " + Math.Round(((SINSstate2.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n), 2)
                                 + " " + Math.Round(((SINSstate2.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate2.Latitude)), 2) + " " + SINSstate2.Height
                                 + " " + ((SINSstate2.Latitude) * SimpleData.ToDegree) + " " + ((SINSstate2.Longitude) * SimpleData.ToDegree)
                                 + " " + Math.Round(SINSstate2.Vx_0[0], 3) + " " + Math.Round(SINSstate2.Vx_0[1], 3) + " " + Math.Round(SINSstate2.Vx_0[2], 3)
                                 //+ " " + ProcHelp.corrected
                                 //+ " " + Math.Round((SINSstate.Heading * SimpleData.ToDegree), 4) 
                                 + " " + Math.Round((SINSstate2.Heading * SimpleData.ToDegree), 4)
                                 //+ " " + Math.Round((SINSstate.Roll * SimpleData.ToDegree), 4) 
                                 + " " + Math.Round((SINSstate2.Roll * SimpleData.ToDegree), 4)
                                 //+ " " + Math.Round((SINSstate.Pitch * SimpleData.ToDegree), 4) 
                                 + " " + Math.Round((SINSstate2.Pitch * SimpleData.ToDegree), 4)
                                 + " " + Math.Round(ProcHelp.distance, 3) 
                                 + " " + Math.Round(ProcHelp.distance_from_start, 3) + " " + Math.Round(SINSstate.V_norm, 3)
                                 + " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate2.Latitude) * SINSstate.R_n), 2)
                                 + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate2.Longitude) * SINSstate.R_e * Math.Cos(SINSstate2.Latitude)), 2)
                                 + " " + Math.Round(ProcHelp.AltSNS, 2) + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                                 + " " + SINSstate.OdometerVector[1] + " " + SINSstate.OdoSpeed_x0[1]
                                 + " " + Math.Round((SINSstate2.Heading - SINSstate.HeadingImitator) * SimpleData.ToDegree_sec, 8)
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
                         + " " + Math.Round(((SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 3) + " " + SINSstate.Height
                         + " " + SINSstate.Latitude * SimpleData.ToDegree
                         + " " + SINSstate.Longitude * SimpleData.ToDegree
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
                    Nav_Autonomous.WriteLine(ProcHelp.datastring);
                }
                /*----------------------------------OUTPUT FEEDBACK------------------------------------------------------*/
                else if (i % SINSstate.FreqOutput == 0)
                {
                    int iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_kappa_1 = SINSstate.value_iMx_kappa_1, iMx_alphaBeta = SINSstate.value_iMx_alphaBeta;

                    ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.Count;
                    if (SINSstate.global_paramsCycleScanning == "")
                        ProcHelp.datastring += " " + SINSstate.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstate.OdoSpeed_s);

                    ProcHelp.datastring += " " + Math.Round(((SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n), 2)
                                        + " " + Math.Round(((SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 2) + " " + SINSstate.Height
                                        + " " + ((SINSstate.Latitude) * SimpleData.ToDegree) + " " + ((SINSstate.Longitude) * SimpleData.ToDegree)
                                        + " " + Math.Round(SINSstate.Vx_0[0], 3) + " " + Math.Round(SINSstate.Vx_0[1], 3) + " " + Math.Round(SINSstate.Vx_0[2], 5)
                                        + " " + Math.Round((SINSstate.Heading * SimpleData.ToDegree), 8)
                                        + " " + Math.Round((SINSstate.Roll * SimpleData.ToDegree), 8) + " " + Math.Round((SINSstate.Pitch * SimpleData.ToDegree), 8)
                                        + " " + ProcHelp.distance
                                        + " " + (SINSstate.Height - ProcHelp.AltSNS)
                                        + " " + ProcHelp.distance_from_start
                                        ;
                    if (SINSstate.global_paramsCycleScanning == "")
                    {
                        ProcHelp.datastring += " " + Math.Round(((ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate.Latitude) * SINSstate.R_n), 2)
                                        + " " + Math.Round(((ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 2)
                                        + " " + Math.Round(ProcHelp.AltSNS, 2) + " " + Math.Round(ProcHelp.SpeedSNS, 3)
                                        + " " + Math.Round((SINSstate.Heading - SINSstate.HeadingImitator) * SimpleData.ToDegree_sec, 8);
                    }
                    else
                    {
                        ProcHelp.datastring += " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_alphaBeta + 2)] * SimpleData.ToDegree;

                        if (SINSstate.Vertical_kappa1 > 0)
                            ProcHelp.datastring += " " + SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_kappa1] * SimpleData.ToDegree;
                        else
                            ProcHelp.datastring += " " + SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_1 + 0] * SimpleData.ToDegree;

                        ProcHelp.datastring += " " + SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_3_ds + 0] * SimpleData.ToDegree
                                               + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_3_ds + 1];
                    }
                    
                    Nav_FeedbackSolution.WriteLine(ProcHelp.datastring);

                    ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment)
                                        + " " + SINSstate.Height
                                        + " " + ProcHelp.distance ;

                    Cicle_Debag_Solution.WriteLine(ProcHelp.datastring);


                    // --- Сохраняем значения для передачи в цикл по подбору стартовых настроек ---//
                    if (SINSstate.Vertical_kappa1 > 0)
                        SINSstate.global_kappa1_grad[SINSstate.global_indx] = SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_kappa1] * SimpleData.ToDegree;
                    else
                        SINSstate.global_kappa1_grad[SINSstate.global_indx] = SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_1 + 0] * SimpleData.ToDegree;

                    SINSstate.global_kappa3_grad[SINSstate.global_indx] = SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_3_ds + 0] * SimpleData.ToDegree;
                    SINSstate.global_scale[SINSstate.global_indx] = SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_3_ds + 1];
                    SINSstate.global_HorizontalError[SINSstate.global_indx] = ProcHelp.distance;
                    SINSstate.global_HorizontalErrorFromStart[SINSstate.global_indx] = ProcHelp.distance_from_start;
                    SINSstate.global_VerticalError[SINSstate.global_indx] = SINSstate.Height - ProcHelp.AltSNS;
                    SINSstate.global_V_Up[SINSstate.global_indx] = SINSstate.Vx_0[2];
                    SINSstate.global_indx++;
                }
            }


            if (SINSstate.NowSmoothing == true
                && i % SINSstate.FreqOutput == 0
                )
            {
                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.Count
                    //+ " " + SINSstate.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstate.OdoSpeed_s)
                        + " " + (SINSstate_Smooth.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n
                        + " " + (SINSstate_Smooth.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate_Smooth.Latitude) + " " + SINSstate_Smooth.Height
                        + " " + SINSstate_Smooth.Latitude * SimpleData.ToDegree
                        + " " + SINSstate_Smooth.Longitude * SimpleData.ToDegree
                        + " " + (ProcHelp.LatSNS * SimpleData.ToRadian - SINSstate_Smooth.Latitude) * SINSstate.R_n
                        + " " + (ProcHelp.LongSNS * SimpleData.ToRadian - SINSstate_Smooth.Longitude) * SINSstate.R_e * Math.Cos(SINSstate_Smooth.Latitude)
                        + " " + ProcHelp.AltSNS + " " + ProcHelp.SpeedSNS
                        + " " + SINSstate_Smooth.Vx_0[0] + " " + SINSstate_Smooth.Vx_0[1]  + " " + SINSstate_Smooth.Vx_0[2]
                        + " " + SINSstate_Smooth.Heading * SimpleData.ToDegree
                        + " " + SINSstate_Smooth.Roll * SimpleData.ToDegree
                        + " " + SINSstate_Smooth.Pitch * SimpleData.ToDegree
                        //----- инструментальные ошибки -----//
                        + " " + SINSstate_Smooth.Cumulative_KalmanErrorVector[Math.Max(SINSstate.value_iMx_kappa_1, SINSstate.Vertical_kappa1)] * SimpleData.ToDegree
                        + " " + SINSstate_Smooth.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_3_ds + 0] * SimpleData.ToDegree
                        + " " + SINSstate_Smooth.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_3_ds + 1]
                        + " " + SINSstate_Smooth.Cumulative_KalmanErrorVector[Math.Max(SINSstate.value_iMx_f0_3, SINSstate.Vertical_f0_3)]
                        ;

                if (SINSstate.Global_file == "Saratov_run_2014_07_23")
                    ProcHelp.datastring = ProcHelp.datastring + " " + Math.Round(SINSstate.Count);

                Nav_Smoothed.WriteLine(ProcHelp.datastring);
            }


            
            if (i % 15 == 0)
            {
                //------------------------------------------------------------//
                bool flag_SK42 = false;
                double[] PhiLambdaH_WGS84 = new double[3];

                if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд завод)"
                        || (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд куликовка)")
                        || (SINSstate.Global_file == "GRTV_Ekat_151029_1_zaezd")
                        || (SINSstate.Global_file == "GRTV_Ekat_151029_2_zaezd")
                        || (SINSstate.Global_file == "GRTV_ktn004_marsh16_afterbdnwin_20032012")
                        || (SINSstate.Global_file == "GRTV_ktn004_marsh16_repeat_21032012")
                    )
                    flag_SK42 = true;

                if (SINSstate.flag_FeedbackExist)
                {
                    PhiLambdaH_WGS84[0] = SINSstate.Latitude; PhiLambdaH_WGS84[1] = SINSstate.Longitude; PhiLambdaH_WGS84[2] = SINSstate.Height;
                    if (flag_SK42)
                        PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(SINSstate.Latitude, SINSstate.Longitude, SINSstate.Height, 1);

                    KMLFileOut.WriteLine(PhiLambdaH_WGS84[1] * SimpleData.ToDegree + "," + PhiLambdaH_WGS84[0] * SimpleData.ToDegree
                        //+ "," + SINSstate.Altitude);
                        + "," + (SINSstate.Time + SINSstate.Time_Alignment));
                }
                else
                    KMLFileOut.WriteLine(SINSstate2.Longitude * SimpleData.ToDegree + "," + SINSstate2.Latitude * SimpleData.ToDegree
                        //+ "," + SINSstate2.Altitude);
                        + "," + (SINSstate.Time + SINSstate.Time_Alignment));

                if (SINSstate.NowSmoothing)
                {
                    PhiLambdaH_WGS84[0] = SINSstate_Smooth.Latitude; PhiLambdaH_WGS84[1] = SINSstate_Smooth.Longitude; PhiLambdaH_WGS84[2] = SINSstate_Smooth.Height;
                    if (flag_SK42)
                        PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(SINSstate_Smooth.Latitude, SINSstate_Smooth.Longitude, SINSstate_Smooth.Height, 1);

                    KMLFileOutSmoothed.WriteLine(PhiLambdaH_WGS84[1] * SimpleData.ToDegree + "," + PhiLambdaH_WGS84[0] * SimpleData.ToDegree
                        //+ "," + SINSstate_Smooth.Altitude);
                        + "," + (SINSstate.Time + SINSstate.Time_Alignment));
                }
            }



            if (i % SINSstate.FreqOutput == 0)
            {
                /*----------------------------------OUTPUT ERRORS------------------------------------------------------*/
                if (!SINSstate.flag_Smoothing)
                {
                    if (SINSstate.flag_FeedbackExist == false)
                        ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment)
                            + " " + SINSstate.DeltaLatitude * SimpleOperations.RadiusN(Lat, SINSstate.Height)
                            + " " + SINSstate.DeltaLongitude * SimpleOperations.RadiusE(Lat, SINSstate.Height) * Math.Cos(Lat)
                            + " " + SINSstate.DeltaAltitude
                            + " " + SINSstate.DeltaV_1 
                            + " " + SINSstate.DeltaV_2 
                            + " " + SINSstate.DeltaV_3
                            + " " + SINSstate.DeltaHeading * SimpleData.ToDegree
                            + " " + SINSstate.DeltaRoll * SimpleData.ToDegree
                            + " " + SINSstate.DeltaPitch * SimpleData.ToDegree
                            ;
                    else
                        ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment)
                            + " " + SINSstate.Cumulative_StateErrorVector[0] * SimpleOperations.RadiusN(Lat, SINSstate.Height)
                            + " " + SINSstate.Cumulative_StateErrorVector[1] * SimpleOperations.RadiusE(Lat, SINSstate.Height) * Math.Cos(Lat)
                            + " " + SINSstate.Cumulative_StateErrorVector[2]
                            + " " + SINSstate.Cumulative_StateErrorVector[3] 
                            + " " + SINSstate.Cumulative_StateErrorVector[4] 
                            + " " + SINSstate.Cumulative_StateErrorVector[5]
                            + " " + SINSstate.Cumulative_StateErrorVector[6] * SimpleData.ToDegree 
                            + " " + SINSstate.Cumulative_StateErrorVector[7] * SimpleData.ToDegree 
                            + " " + SINSstate.Cumulative_StateErrorVector[8] * SimpleData.ToDegree
                            ;
                    Nav_Errors.WriteLine(ProcHelp.datastring);
                }





                /*----------------------------------OUTPUT Check_Measurement------------------------------------------------------*/
                if (SINSstate.flag_FeedbackExist == false)
                {
                    Check_Measurement.WriteLine(SINSstate.Count
                        + " " + KalmanVars.pdResidual[0] + " " + KalmanVars.pdResidual[1] + " " + KalmanVars.pdResidual[2]
                        + " " + KalmanVars.pdResidual[3] + " " + KalmanVars.pdResidual[4] + " " + KalmanVars.pdResidual[5]
                        + " " + KalmanVars.pdSigmaApriori[0] + " " + KalmanVars.pdSigmaApriori[1] + " " + KalmanVars.pdSigmaApriori[2]
                        + " " + KalmanVars.pdSigmaApriori[3] + " " + KalmanVars.pdSigmaApriori[4] + " " + KalmanVars.pdSigmaApriori[5]);
                }
                else if (SINSstate.NowSmoothing == false)
                {
                    Check_Measurement.WriteLine(SINSstate.Count
                        //+ " " + KalmanVars.Measure[0] + " " + KalmanVars.Measure[1] + " " + KalmanVars.Measure[2] + " " + KalmanVars.Measure[3]
                        //+ " " + KalmanVars.Vertical_Measure[0] + " " + KalmanVars.Vertical_Measure[1]
                        + " " + Math.Abs(KalmanVars.Measure[0]) + " " + Math.Abs(KalmanVars.Measure[1]) + " " + Math.Abs(KalmanVars.Measure[2])
                        + " " + Math.Abs(KalmanVars.Measure[3]) + " " + Math.Abs(KalmanVars.Vertical_Measure[0]) + " " + Math.Abs(KalmanVars.Vertical_Measure[1])
                        + " " + Math.Abs(SINSstate.OdoAcceleration_s)
                        );
                }
                



                /*----------------------------------OUTPUT StateErrors------------------------------------------------------*/

                int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_kappa_1 = SINSstate.value_iMx_kappa_1,
                iMx_r12_odo = SINSstate.value_iMx_r_odo_12, value_iMx_dr3 = SINSstate.value_iMx_dr3, value_iMx_dV3 = SINSstate.value_iMx_dV3;

                int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                    iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                    iMx_Nu0 = SINSstate.value_iMx_Nu0,
                    f0_12 = SINSstate.value_iMx_f0_12,
                    f0_3 = SINSstate.value_iMx_f0_3,
                    iMx_r_odo_3 = SINSstate.value_iMx_r_odo_3
                    ;

                double[] Vertical_ErrorConditionVector = new double[KalmanVars.Vertical_ErrorConditionVector_p.Length];

                if (SINSstate.flag_FeedbackExist == false)
                    SimpleOperations.CopyArray(Vertical_ErrorConditionVector, KalmanVars.Vertical_ErrorConditionVector_p);
                if (SINSstate.flag_FeedbackExist == true)
                    SimpleOperations.CopyArray(Vertical_ErrorConditionVector, SINSstate.Vertical_Cumulative_KalmanErrorVector);

                if (SINSstate.flag_FeedbackExist == false)
                {
                    ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment)
                        + " " + KalmanVars.ErrorConditionVector_p[0]
                        + " " + KalmanVars.ErrorConditionVector_p[1]
                        + " " + KalmanVars.ErrorConditionVector_p[(iMx_dV_12 + 0)]
                        + " " + KalmanVars.ErrorConditionVector_p[(iMx_dV_12 + 1)]
                        + " " + KalmanVars.ErrorConditionVector_p[(iMx_alphaBeta + 0)] * SimpleData.ToDegree
                        + " " + KalmanVars.ErrorConditionVector_p[(iMx_alphaBeta + 1)] * SimpleData.ToDegree
                        + " " + KalmanVars.ErrorConditionVector_p[(iMx_alphaBeta + 2)] * SimpleData.ToDegree
                        + " " + KalmanVars.ErrorConditionVector_p[(iMx_Nu0 + 0)] * SimpleData.ToDegree * 3600.0
                        + " " + KalmanVars.ErrorConditionVector_p[(iMx_Nu0 + 1)] * SimpleData.ToDegree * 3600.0
                        + " " + KalmanVars.ErrorConditionVector_p[(iMx_Nu0 + 2)] * SimpleData.ToDegree * 3600.0
                        + " " + KalmanVars.ErrorConditionVector_p[(f0_12 + 0)]
                        + " " + KalmanVars.ErrorConditionVector_p[(f0_12 + 1)];

                    if (f0_3 > 0)
                        ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[(f0_3 + 0)];
                    else
                        ProcHelp.datastring = ProcHelp.datastring + " 0";
                }
                else
                {
                    ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment)
                        + " " + SINSstate.Cumulative_KalmanErrorVector[0]
                        + " " + SINSstate.Cumulative_KalmanErrorVector[1]
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_dV_12 + 0)]
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_dV_12 + 1)]
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_alphaBeta + 0)] * SimpleData.ToDegree
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_alphaBeta + 1)] * SimpleData.ToDegree
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_alphaBeta + 2)] * SimpleData.ToDegree
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_Nu0 + 0)] * SimpleData.ToDegree * 3600.0
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_Nu0 + 1)] * SimpleData.ToDegree * 3600.0
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_Nu0 + 2)] * SimpleData.ToDegree * 3600.0
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(f0_12 + 0)]
                        + " " + SINSstate.Cumulative_KalmanErrorVector[(f0_12 + 1)];

                    if (f0_3 > 0)
                        ProcHelp.datastring = ProcHelp.datastring + " " + SINSstate.Cumulative_KalmanErrorVector[(f0_3 + 0)];
                    else
                        ProcHelp.datastring = ProcHelp.datastring + " 0";
                }

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    if (SINSstate.flag_FeedbackExist == false)
                        ProcHelp.datastring = ProcHelp.datastring
                            + " " + KalmanVars.ErrorConditionVector_p[value_iMx_dr3]
                            + " " + KalmanVars.ErrorConditionVector_p[value_iMx_dV3];
                    else
                        ProcHelp.datastring = ProcHelp.datastring
                            + " " + SINSstate.Cumulative_KalmanErrorVector[value_iMx_dr3]
                            + " " + SINSstate.Cumulative_KalmanErrorVector[value_iMx_dV3];
                }
                if (SINSstate.flag_Odometr_SINS_case)
                {
                    if (SINSstate.flag_FeedbackExist == false)
                        ProcHelp.datastring = ProcHelp.datastring 
                            + " " + KalmanVars.ErrorConditionVector_p[iMx_r12_odo] 
                            + " " + KalmanVars.ErrorConditionVector_p[iMx_r12_odo + 1];
                    else
                        ProcHelp.datastring = ProcHelp.datastring 
                            + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_r12_odo] 
                            + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_r12_odo + 1];
                }
                if (SINSstate.flag_Odometr_SINS_case && SINSstate.flag_iMx_r3_dV3)
                {
                    if (SINSstate.flag_FeedbackExist == false)
                        ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[iMx_r_odo_3];
                    else
                        ProcHelp.datastring = ProcHelp.datastring + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_r_odo_3];
                }

                if (SINSstate.flag_iMx_kappa_13_ds)
                {
                    if (SINSstate.flag_FeedbackExist == false)
                    {
                        if (iMx_kappa_1 > 0)
                            ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[iMx_kappa_1] * SimpleData.ToDegree;
                        else
                            ProcHelp.datastring = ProcHelp.datastring + " 0";

                        ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[iMx_kappa_3_ds + 0] * SimpleData.ToDegree
                                                              + " " + KalmanVars.ErrorConditionVector_p[iMx_kappa_3_ds + 1];
                    }
                    else
                    {
                        if (iMx_kappa_1 > 0)
                            ProcHelp.datastring = ProcHelp.datastring + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_1] * SimpleData.ToDegree;
                        else
                            ProcHelp.datastring = ProcHelp.datastring + " 0";

                        ProcHelp.datastring = ProcHelp.datastring + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_3_ds + 0] * SimpleData.ToDegree
                                                              + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_3_ds + 1];
                    }
                }

                // ----------------------------------------------------------//
                // ----------------------------------------------------------//
                if (SINSstate.flag_SeparateHorizVSVertical == true)
                {
                    ProcHelp.datastring +=
                        " - " + Vertical_ErrorConditionVector[0]
                        + " " + Vertical_ErrorConditionVector[1]
                        + " " + Vertical_ErrorConditionVector[SINSstate.Vertical_rOdo3]
                        ;

                    if (SINSstate.Vertical_f0_12 > 0)
                        ProcHelp.datastring += " " + Vertical_ErrorConditionVector[SINSstate.Vertical_f0_12 + 0] + " " + Vertical_ErrorConditionVector[SINSstate.Vertical_f0_12 + 1];

                    if (SINSstate.Vertical_f0_3 > 0)
                        ProcHelp.datastring += " " + Vertical_ErrorConditionVector[SINSstate.Vertical_f0_3 + 0];

                    if (SINSstate.Vertical_kappa1 > 0)
                        ProcHelp.datastring += " " + Vertical_ErrorConditionVector[SINSstate.Vertical_kappa1] * SimpleData.ToDegree;

                    if (SINSstate.Vertical_kappa3Scale > 0)
                        ProcHelp.datastring += " " + Vertical_ErrorConditionVector[SINSstate.Vertical_kappa3Scale + 0] * SimpleData.ToDegree
                            + " " + Vertical_ErrorConditionVector[SINSstate.Vertical_kappa3Scale + 1];
                }

                Nav_StateErrorsVector.WriteLine(ProcHelp.datastring);

            }

        }



        public static void FillKMLOutputFile(SINS_State SINSstate, StreamWriter KMLFileOut, string Part, string Mode)
        {
            if (Part == "Start")
            {
                KMLFileOut.WriteLine("<?xml version='1.0' encoding='UTF-8'?>                                                 ");
                KMLFileOut.WriteLine("<kml xmlns='http://earth.google.com/kml/2.2'>                                          ");
                KMLFileOut.WriteLine("<Document>                                                                             ");
                KMLFileOut.WriteLine("<name>" + SINSstate.Global_file + SINSstate.global_paramsCycleScanning + "</name>");
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
                KMLFileOut.WriteLine("              </LineString>       ");
                KMLFileOut.WriteLine("          </Placemark>               ");
                KMLFileOut.WriteLine("</Folder>                        ");
                KMLFileOut.WriteLine("</Document>                      ");
                KMLFileOut.WriteLine("</kml>                           ");

            }
        }


    }
}
