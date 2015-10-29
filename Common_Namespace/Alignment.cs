using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;

namespace Common_Namespace
{
    public class Alignment : SimpleOperations
    {
        public static int RougthAlignment(Proc_Help ProcHelp, SINS_State SINSstate, StreamReader myFile, Kalman_Vars KalmanVars, SINS_State SINSstate_OdoMod, StreamWriter GRTV_output)
        {
            int k = 0, i = 0;
            double[] f_avg = new double[3]; double[] w_avg = new double[3]; double[] w_avg_x = new double[3]; double[] U_s = new double[3];
            Matrix A_xs = new Matrix(3, 3);

            StreamWriter FinalAlignmentParams = new StreamWriter(SimpleData.PathOutputString + "Alignment//AlignmentFinal_Params.txt");
            FinalAlignmentParams.WriteLine(SINSstate.Count + " Latitude= " + SINSstate.Latitude + " Longitude= " + SINSstate.Longitude + " Heading= " + SINSstate.Heading + " Roll= " + SINSstate.Roll + " Pitch= " + SINSstate.Pitch);
            StreamWriter FinalAlignment = new StreamWriter(SimpleData.PathOutputString + "Alignment//AlignmentFinal_Data.txt");

            StreamWriter Alignment_avg_rougth = new StreamWriter(SimpleData.PathOutputString + "Alignment//Alignment_avg_rougth.txt");
            StreamWriter Alignment_avg_rougthMovingAVG = new StreamWriter(SimpleData.PathOutputString + "Alignment//Alignment_avg_rougth_MovingAVG.txt");

            double[] array_sigma_f_1 = new double[200000];
            double[] array_sigma_f_2 = new double[200000];
            double[] array_sigma_f_3 = new double[200000];
            double[] array_sigma_w_1 = new double[200000];
            double[] array_sigma_w_2 = new double[200000];
            double[] array_sigma_w_3 = new double[200000];
            double[] sigma_f = new double[3];
            double[] sigma_w = new double[3];

            Alignment_avg_rougth.WriteLine("count f_1 f_2 f_3 w_1 w_2 w_3 heading roll pitch Latitude");
            Alignment_avg_rougthMovingAVG.WriteLine("count MA_f_1 MA_f_2 MA_f_3 MA_w_1 MA_w_2 MA_w_3");


            while (true)
            {
                i++;
                if (i < 1) { myFile.ReadLine(); continue; }
                if (SINSstate.FLG_Stop == 0 && false)
                {
                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);
                }
                else
                {
                    i--;
                    break;
                }
            }

            int t = i;

            double AvgMean = 0, Otklon = 0;
            double Latitude = 0.0, Pitch = 0.0, Roll = 0.0, Heading = 0.0;
            int max_cnt_jj = 100;
            double[] BufAvg = new double[max_cnt_jj];

            int MovingWindow = 500;
            double[] MovingAverageAccGyro = new double[6];

            for (i = t; ; i++)
            {
                ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);

                if (/*SINSstate.FLG_Stop == 0 || */(ProcHelp.AlgnCnt != 0 && i == ProcHelp.AlgnCnt))
                    break;

                //if (i == 1000)
                //    break;


                array_sigma_f_1[k] = SINSstate.F_z[0];
                array_sigma_f_2[k] = SINSstate.F_z[1];
                array_sigma_f_3[k] = SINSstate.F_z[2];
                array_sigma_w_1[k] = SINSstate.W_z[0];
                array_sigma_w_2[k] = SINSstate.W_z[1];
                array_sigma_w_3[k] = SINSstate.W_z[2];

                f_avg[0] += SINSstate.F_z[0]; w_avg[0] += SINSstate.W_z[0];
                f_avg[1] += SINSstate.F_z[1]; w_avg[1] += SINSstate.W_z[1];
                f_avg[2] += SINSstate.F_z[2]; w_avg[2] += SINSstate.W_z[2];
                k++;

                for (int u = 1; u <= Math.Min(k, MovingWindow); u++)
                {
                    MovingAverageAccGyro[0] += array_sigma_f_1[k - u];
                    MovingAverageAccGyro[1] += array_sigma_f_2[k - u];
                    MovingAverageAccGyro[2] += array_sigma_f_3[k - u];
                    MovingAverageAccGyro[3] += array_sigma_w_1[k - u];
                    MovingAverageAccGyro[4] += array_sigma_w_2[k - u];
                    MovingAverageAccGyro[5] += array_sigma_w_3[k - u];
                }
                for (int u = 0; u < 6; u++)
                    MovingAverageAccGyro[u] = MovingAverageAccGyro[u] / MovingWindow;



                //---Считаем максимальное отклонение от текущего среднего по max_cnt_jj съемам данных----
                for (int ij = 1; ij < max_cnt_jj; ij++)
                    BufAvg[ij - 1] = BufAvg[ij];
                BufAvg[max_cnt_jj - 1] = f_avg[2] / k;

                AvgMean = 0;
                for (int ij = 0; ij < max_cnt_jj; ij++)
                    AvgMean += BufAvg[ij];

                AvgMean = AvgMean / max_cnt_jj;
                Otklon = Math.Abs(AvgMean - BufAvg[0]);

                for (int ij = 0; ij < max_cnt_jj; ij++)
                {
                    if (Otklon < Math.Abs(AvgMean - BufAvg[ij]))
                        Otklon = Math.Abs(AvgMean - BufAvg[ij]);
                }
                //-------



                Pitch = Math.Atan2(f_avg[1], Math.Sqrt(f_avg[0] * f_avg[0] + f_avg[2] * f_avg[2]));
                Roll = -Math.Atan2(f_avg[0], f_avg[2]);
                A_xs = SimpleOperations.A_xs(Heading, Roll, Pitch);
                w_avg_x = Matrix.Multiply(A_xs, w_avg);

                Heading = -Math.Atan2(w_avg_x[0], w_avg_x[1]);
                Latitude = Math.Atan2(w_avg_x[2], Math.Sqrt(w_avg_x[1] * w_avg_x[1] + w_avg_x[0] * w_avg_x[0]));

                SINSstate.A_sx0 = SimpleOperations.A_sx0(Heading, Roll, Pitch);
                U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);

                //FinalAlignment.WriteLine(k + " " + SINSstate.Count + " " + Heading + " " + Roll + " " + Pitch + " " + Latitude
                //    + " " + SINSstate.F_z[0] + " " + SINSstate.F_z[1] + " " + SINSstate.F_z[2]
                //    + " " + SINSstate.W_z[0] + " " + SINSstate.W_z[1] + " " + SINSstate.W_z[2]
                //    + " " + U_s[0] + " " + U_s[1] + " " + U_s[2]);

                if (Math.Abs(w_avg[0] / k - U_s[0]) < 0.000005) { }
                else
                {
                    Heading = Heading - Math.PI;
                    SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                    U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);
                }

                for (int j = 0; j < 3; j++)
                    SINSstate.AlignAlgebraDrifts[j] = w_avg[j] / k - U_s[j];

                if (k > MovingWindow && k % 5 == 0)
                {
                    Alignment_avg_rougth.WriteLine(SINSstate.Count.ToString() + " " + (f_avg[0] / k).ToString() + " " + (f_avg[1] / k).ToString() + " " + (f_avg[2] / k).ToString() + " " + (w_avg[0] / k).ToString() + " " + (w_avg[1] / k).ToString() + " " + (w_avg[2] / k).ToString()
                        + " " + Heading.ToString() + " " + Roll.ToString() + " " + Pitch.ToString() + " " + Latitude.ToString() + " " + Otklon.ToString()
                        + " " + (w_avg_x[0] / k).ToString() + " " + (w_avg_x[1] / k).ToString() + " " + (w_avg_x[2] / k).ToString()
                        );

                    Alignment_avg_rougthMovingAVG.WriteLine(SINSstate.Time.ToString() + " " + MovingAverageAccGyro[0] + " " + MovingAverageAccGyro[1] + " " + MovingAverageAccGyro[2] + " " + MovingAverageAccGyro[3] + " " + MovingAverageAccGyro[4]
                        + " " + MovingAverageAccGyro[5]);
                }


                GRTV_output.WriteLine(
                    SINSstate.Count
                    + " " + "4" + " "
                    + " " + SINSstate.F_z_orig[1] + " " + SINSstate.F_z_orig[2] + " " + SINSstate.F_z_orig[0]
                    + " " + SINSstate.W_z_orig[1] + " " + SINSstate.W_z_orig[2] + " " + SINSstate.W_z_orig[0]

                    + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Altitude
                    + " " + SINSstate.Vx_0[1] + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[2]

                    + " " + SINSstate.Heading + " " + SINSstate.Pitch + " " + SINSstate.Roll
                    + " " + SINSstate.Latitude + " 1 " + SINSstate.Longitude + " 1 " + SINSstate.Altitude + " 1"
                    + " " + SINSstate.Vx_0[1] + " 1 " + SINSstate.Vx_0[0] + " 1 " + SINSstate.Vx_0[2] + " 1"

                    + " " + SINSstate.OdometerData.odometer_left.Value_orig + " " + SINSstate.OdometerData.odometer_left.isReady_orig

                    //метка времени - отмечает момент времени формирования пакета СНС-данных
                    + " " + SINSstate.GPS_Data.gps_Latitude.isReady_orig
                    + " " + SINSstate.GPS_Data.gps_Latitude.Value_orig + " " + SINSstate.GPS_Data.gps_Latitude.isReady_orig
                    + " " + SINSstate.GPS_Data.gps_Longitude.Value_orig + " " + SINSstate.GPS_Data.gps_Longitude.isReady_orig
                    + " " + SINSstate.GPS_Data.gps_Altitude.Value_orig + " " + SINSstate.GPS_Data.gps_Altitude.isReady_orig
                    + " " + SINSstate.GPS_Data.gps_Vn.Value_orig + " " + SINSstate.GPS_Data.gps_Vn.isReady_orig
                    + " " + SINSstate.GPS_Data.gps_Ve.Value_orig + " " + SINSstate.GPS_Data.gps_Vn.isReady_orig
                    + " " + " 0 0" //Скорость GPS вертикальная
                    );
            }


            f_avg[0] = f_avg[0] / k; w_avg[0] = w_avg[0] / k;
            f_avg[1] = f_avg[1] / k; w_avg[1] = w_avg[1] / k;
            f_avg[2] = f_avg[2] / k; w_avg[2] = w_avg[2] / k;

            for (int j = 0; j < k; j++)
            {
                sigma_f[0] += Math.Pow((array_sigma_f_1[j] - f_avg[0]), 2);
                sigma_f[1] += Math.Pow((array_sigma_f_2[j] - f_avg[1]), 2);
                sigma_f[2] += Math.Pow((array_sigma_f_3[j] - f_avg[2]), 2);
                sigma_w[0] += Math.Pow((array_sigma_w_1[j] - w_avg[0]), 2);
                sigma_w[1] += Math.Pow((array_sigma_w_2[j] - w_avg[1]), 2);
                sigma_w[2] += Math.Pow((array_sigma_w_3[j] - w_avg[2]), 2);
            }

            sigma_f[0] = Math.Sqrt(sigma_f[0] / k);
            sigma_f[1] = Math.Sqrt(sigma_f[1] / k);
            sigma_f[2] = Math.Sqrt(sigma_f[2] / k);
            sigma_w[0] = Math.Sqrt(sigma_w[0] / k);
            sigma_w[1] = Math.Sqrt(sigma_w[1] / k);
            sigma_w[2] = Math.Sqrt(sigma_w[2] / k);

            //шумы ньютонометров и дусов
            for (int j = 0; j < 3; j++)
            {
                if (SimpleOperations.AbsoluteVectorValue(sigma_f) > 1E-5)
                    KalmanVars.Noise_Vel[j] = sigma_f[j] / 1.0;
                if (SimpleOperations.AbsoluteVectorValue(sigma_w) > 1E-9)
                    KalmanVars.Noise_Angl[j] = sigma_w[j] / 1.0;
            }

            SINSstate.Pitch = Math.Atan2(f_avg[1], Math.Sqrt(f_avg[0] * f_avg[0] + f_avg[2] * f_avg[2]));
            SINSstate.Roll = -Math.Atan2(f_avg[0], f_avg[2]);

            A_xs = SimpleOperations.A_xs(SINSstate);
            w_avg_x = Matrix.Multiply(A_xs, w_avg);





            if (true)
            {
                SINSstate.Heading = -Math.Atan2(w_avg_x[0], w_avg_x[1]);
                Latitude = Math.Atan2(w_avg_x[2], Math.Sqrt(w_avg_x[1] * w_avg_x[1] + w_avg_x[0] * w_avg_x[0]));
            }
            else
            {
                double[] l1 = new double[3], l2 = new double[3], l3 = new double[3];
                l3[0] = A_xs[2, 0]; l3[1] = A_xs[2, 1]; l3[2] = A_xs[2, 2];
                for (int ij = 0; ij < 3; ij++)
                {
                    l2[ij] = (w_avg[ij] - SimpleData.U * l3[ij] * Math.Sin(SINSstate.Latitude)) / (SimpleData.U * Math.Cos(SINSstate.Latitude));
                }

                double l2_mod = Math.Sqrt(l2[0] * l2[0] + l2[1] * l2[1] + l2[2] * l2[2]);
                l2[0] = l2[0] / l2_mod;
                l2[1] = l2[1] / l2_mod;
                l2[2] = l2[2] / l2_mod;

                l1[0] = -l2[2] * l3[1] + l2[1] * l3[2];
                l1[1] = l2[2] * l3[0] - l2[0] * l3[2];
                l1[2] = -l2[1] * l3[0] + l2[0] * l3[1];

                SINSstate.Heading = -Math.Atan2(w_avg_x[0], w_avg_x[1]);
                SINSstate.Heading = Math.Atan2(l1[1], l1[2]);
            }





            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);

            //if (Math.Abs(w_avg[0] - U_s[0]) < 0.000005) { }
            //else
            //{
            //    SINSstate.Heading = SINSstate.Heading - Math.PI;
            //    SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            //    U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);
            //}


            for (int j = 0; j < 3; j++)
                SINSstate.AlignAlgebraDrifts[j] = w_avg[j] - U_s[j];

            SINSstate.Time_Alignment = SINSstate.Time;




            if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2")
            {
                SINSstate.Heading = -3.0504734;
            }
            if (SINSstate.Global_file == "ktn004_15.03.2012")
            {
                SINSstate.Heading = 15.28 * SimpleData.ToRadian;
            }

            if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд завод)")
            {
                SINSstate.Heading = 153.4796 * SimpleData.ToRadian;
            }
            if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд куликовка)")
            {
                SINSstate.Heading = -78.61045 * SimpleData.ToRadian;
                //SINSstate.Heading -= 0.0663 * SimpleData.ToRadian;
            }
            if (SINSstate.Global_file == "GRTVout_GCEF_format_030715выезд")
            {
                //SINSstate.Heading = -141.45 * SimpleData.ToRadian; //-134.0102

                //SINSstate.Heading = -117.2561 * SimpleData.ToRadian;
                //SINSstate.Roll = 2.9201 * SimpleData.ToRadian;
                //SINSstate.Pitch = -0.84457 * SimpleData.ToRadian;

                for (int j = 0; j < 3; j++)
                {
                    KalmanVars.Noise_Vel[j] = KalmanVars.Noise_Vel[j] / Math.Sqrt(Math.Abs(SINSstate.Freq));
                    KalmanVars.Noise_Angl[j] = KalmanVars.Noise_Angl[j] / Math.Sqrt(Math.Abs(SINSstate.Freq));
                }
            }





            if (SINSstate.Global_file == "Saratov_run_2014_07_23")
            {
                double lat_dif_true = (49.99452656 * SimpleData.ToRadian - SINSstate.Latitude_Start) * SimpleOperations.RadiusN(49.99452656 * SimpleData.ToRadian, SINSstate.Altitude_Start);
                double long_dif_true = (46.87201806 * SimpleData.ToRadian - SINSstate.Longitude_Start) * SimpleOperations.RadiusE(49.99452656 * SimpleData.ToRadian, SINSstate.Altitude_Start) * Math.Cos(49.99452656 * SimpleData.ToRadian);
                double SettedHeading = Math.Atan2(long_dif_true, lat_dif_true);

                FinalAlignmentParams.WriteLine("Heading_ALign = " + SINSstate.Heading);
                FinalAlignmentParams.WriteLine("AlignAlgebraDrifts1=" + SINSstate.AlignAlgebraDrifts[0] + " AlignAlgebraDrifts2=" + SINSstate.AlignAlgebraDrifts[1] + " AlignAlgebraDrifts3=" + SINSstate.AlignAlgebraDrifts[2]);
                FinalAlignmentParams.WriteLine("");

                if (SINSstate.Time > 10000.0)
                {
                    SettedHeading = SimpleOperations.CalculateHeadingByTwoDots(49.80892188 * SimpleData.ToRadian, 45.3817334 * SimpleData.ToRadian, SINSstate.GPS_Data.gps_Altitude_prev.Value,
                                        49.80906066 * SimpleData.ToRadian, 45.38113053 * SimpleData.ToRadian, SINSstate.GPS_Data.gps_Altitude.Value);

                    SINSstate.Heading = SettedHeading;
                    SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                    SimpleOperations.CopyArray(U_s, SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude));
                }
                else
                {
                    double difHeadings = SettedHeading - SINSstate.Heading;

                    SINSstate.Heading = SettedHeading;
                    SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                    SimpleOperations.CopyArray(U_s, SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude));
                }

                for (int j = 0; j < 3; j++)
                    SINSstate.AlignAlgebraDrifts[j] = w_avg[j] - U_s[j];

                FinalAlignmentParams.WriteLine("Heading_GPS = " + SINSstate.Heading);
                FinalAlignmentParams.WriteLine("AlignAlgebraDrifts1=" + SINSstate.AlignAlgebraDrifts[0] + " AlignAlgebraDrifts2=" + SINSstate.AlignAlgebraDrifts[1] + " AlignAlgebraDrifts3=" + SINSstate.AlignAlgebraDrifts[2]);
                FinalAlignmentParams.WriteLine("Noise_Vel_1=" + KalmanVars.Noise_Vel[0] + " Noise_Vel_2=" + KalmanVars.Noise_Vel[1] + " Noise_Vel_3=" + KalmanVars.Noise_Vel[2]);
                FinalAlignmentParams.WriteLine("Noise_Angl_1=" + KalmanVars.Noise_Angl[0] + " Noise_Angl_2=" + KalmanVars.Noise_Angl[1] + " Noise_Angl_3=" + KalmanVars.Noise_Angl[2]);

                for (int j = 0; j < 3; j++)
                {
                    KalmanVars.Noise_Vel[j] = KalmanVars.Noise_Vel[j] * 5.0;
                    KalmanVars.Noise_Angl[j] = KalmanVars.Noise_Angl[j] * 5.0;
                    //KalmanVars.Noise_Vel[j] = KalmanVars.Noise_Vel.Max();
                    //KalmanVars.Noise_Angl[j] = KalmanVars.Noise_Angl.Max();
                }
            }

            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //



            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time - SINSstate.Time_Alignment, SINSstate.Longitude_Start);
            //Далее произойдет обнуление SINSstate.Time
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);


            Alignment_avg_rougth.Close();
            Alignment_avg_rougthMovingAVG.Close();
            FinalAlignmentParams.Close(); FinalAlignment.Close();
            return i;
        }




        public static void OutPutInfo_Nav_Alignment(Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate2, StreamReader myFile, Kalman_Vars KalmanVars,
                        StreamWriter Alignment_Errors, StreamWriter Alignment_SINSstate, StreamWriter Alignment_Corrected_State, StreamWriter Alignment_StateErrorsVector)
        {
            if (SINSstate.Count % SINSstate.FreqOutput == 0)
            {
                ProcHelp.datastring = KalmanVars.ErrorConditionVector_p[0].ToString() + " " + KalmanVars.ErrorConditionVector_p[1].ToString()
                                     + " " + KalmanVars.ErrorConditionVector_p[2].ToString() + " " + KalmanVars.ErrorConditionVector_p[3].ToString()
                                      + " " + (KalmanVars.ErrorConditionVector_p[4] * 180.0 / 3.141592).ToString() + " " + (KalmanVars.ErrorConditionVector_p[5] * 180.0 / 3.141592).ToString() + " " + (KalmanVars.ErrorConditionVector_p[6] * 180.0 / 3.141592).ToString()
                                       + " " + KalmanVars.ErrorConditionVector_p[7].ToString() + " " + KalmanVars.ErrorConditionVector_p[8].ToString() + " " + KalmanVars.ErrorConditionVector_p[9].ToString()
                                        + " " + KalmanVars.ErrorConditionVector_p[10].ToString() + " " + KalmanVars.ErrorConditionVector_p[11].ToString() + " " + KalmanVars.ErrorConditionVector_p[12].ToString();
                Alignment_Errors.WriteLine(ProcHelp.datastring);

                ProcHelp.datastring = (SINSstate.Count * SINSstate.timeStep).ToString() + " " + SINSstate.Count.ToString() + " " +
                                (SINSstate.Latitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + " " + SINSstate.Altitude.ToString() + " "
                                + ProcHelp.LatSNS.ToString() + " " + ProcHelp.LongSNS.ToString() + " " + SINSstate.Vx_0[0].ToString() + " " + SINSstate.Vx_0[1].ToString() + " " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " "
                                  + (SINSstate.Roll * SimpleData.ToDegree).ToString() + " " + (SINSstate.Pitch * SimpleData.ToDegree).ToString();
                Alignment_SINSstate.WriteLine(ProcHelp.datastring);

                ProcHelp.datastring = (SINSstate.Count * SINSstate.timeStep).ToString() + " " + SINSstate.Count.ToString() + " " + (SINSstate2.Latitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Latitude * SimpleData.ToDegree).ToString()
                                + " " + (SINSstate2.Longitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + " " + SINSstate2.Altitude.ToString() + " "
                                + SINSstate2.Vx_0[0].ToString() + " " + SINSstate2.Vx_0[1].ToString() + " " + SINSstate2.Vx_0[2].ToString() + " " + 
                                (SINSstate.Heading * SimpleData.ToDegree).ToString() + " " + (SINSstate2.Heading * SimpleData.ToDegree).ToString() + " "
                                + (SINSstate.Roll * SimpleData.ToDegree).ToString() + " " + (SINSstate2.Roll * SimpleData.ToDegree).ToString() + " "
                                + (SINSstate.Pitch * SimpleData.ToDegree).ToString() + " " + (SINSstate2.Pitch * SimpleData.ToDegree).ToString();
                Alignment_Corrected_State.WriteLine(ProcHelp.datastring);

                ProcHelp.datastring = (SINSstate.DeltaLatitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.DeltaLongitude * SimpleData.ToDegree).ToString() + " " + SINSstate.DeltaV_1.ToString() + " " + SINSstate.DeltaV_2.ToString() + " "
                                + SINSstate.DeltaV_3.ToString() + " " + SINSstate.DeltaHeading.ToString() + " " + SINSstate.DeltaRoll.ToString() + " " + SINSstate.DeltaPitch.ToString();
                Alignment_StateErrorsVector.WriteLine(ProcHelp.datastring);
            }
        }




        public static void OutPutInfo_Class_Alignment(Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate2, StreamReader myFile, Kalman_Align KalmanAlign,
                        StreamWriter Alignment_Errors, StreamWriter Alignment_SINSstate, StreamWriter Alignment_Corrected_State, StreamWriter Alignment_StateErrorsVector)
        {
            if (SINSstate.Count % SINSstate.FreqOutput == 0)
            {
                ProcHelp.datastring = (SINSstate.Count * SINSstate.timeStep).ToString() + " " + (KalmanAlign.ErrorConditionVector_p[0] * 180.0 / 3.141592).ToString() + " " + (KalmanAlign.ErrorConditionVector_p[1] * 180.0 / 3.141592).ToString()
                                     + " " + (KalmanAlign.ErrorConditionVector_p[2] * 180.0 / 3.141592).ToString()
                                     + " " + KalmanAlign.ErrorConditionVector_p[3].ToString() + " " + (KalmanAlign.ErrorConditionVector_p[4]).ToString() + " " + (KalmanAlign.ErrorConditionVector_p[5]).ToString()
                                     + " " + (KalmanAlign.ErrorConditionVector_p[6]).ToString() + " " + KalmanAlign.ErrorConditionVector_p[7].ToString() + " " + KalmanAlign.ErrorConditionVector_p[8].ToString();
                Alignment_StateErrorsVector.WriteLine(ProcHelp.datastring);

                ProcHelp.datastring = (SINSstate.Count * SINSstate.timeStep).ToString() + " " + SINSstate.Count.ToString() + " " +
                                (SINSstate.Latitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + " " + SINSstate.Altitude.ToString() + " "
                                + SINSstate.Vx_0[0].ToString() + " " + SINSstate.Vx_0[1].ToString() + " "
                                + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " " + " " + ((SINSstate.Heading - SINSstate.DeltaHeading) * SimpleData.ToDegree).ToString() + " "
                                + (SINSstate.Roll * SimpleData.ToDegree).ToString() + " " + ((SINSstate.Roll - SINSstate.DeltaRoll) * SimpleData.ToDegree).ToString() + " "
                                + (SINSstate.Pitch * SimpleData.ToDegree).ToString() + " " + ((SINSstate.Pitch - SINSstate.DeltaPitch) * SimpleData.ToDegree).ToString();
                Alignment_SINSstate.WriteLine(ProcHelp.datastring);
            }
        }

    }
}
