using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using Common_Namespace;
using SINS_motion_processing;


namespace SINS_motion_processing
{
    public partial class SINS_Alignment
    {
        static StreamWriter Alignment_Errors = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Alignment_Errors.txt");
        static StreamWriter Alignment_SINSstate = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Alignment_SINSstate.txt");
        static StreamWriter Alignment_Corrected_State = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Alignment_Corrected_State.txt");
        static StreamWriter Alignment_StateErrorsVector = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Alignment_StateErrorsVector.txt");
        static StreamWriter Alignment_avg_rougth = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Alignment_avg_rougth.txt");

        public static int RougthAlignment(Proc_Help ProcHelp, SINS_State SINSstate, StreamReader myFile, Kalman_Vars KalmanVars)
        {
            int k = 0, i = 0;
            double[] f_avg = new double[3]; double[] w_avg = new double[3]; double[] w_avg_x = new double[3]; double[] U_s = new double[3];
            Matrix A_xs = new Matrix(3, 3);

            double[] array_sigma_f_1 = new double[60000];
            double[] array_sigma_f_2 = new double[60000];
            double[] array_sigma_f_3 = new double[60000];
            double[] array_sigma_w_1 = new double[60000];
            double[] array_sigma_w_2 = new double[60000];
            double[] array_sigma_w_3 = new double[60000];
            double[] sigma_f = new double[3];
            double[] sigma_w = new double[3];

            Alignment_Errors.WriteLine("dR1  dR2  dV1  dV2  Alpha1 Alpha2 Beta3  Nu1  Nu2  Nu3  dF1  dF2  dF3");
            Alignment_Corrected_State.WriteLine("Time  Count  LatCrtd Lat  LongCrtd    Long  AltitudeCrtd V1 V2 V3 HeadingCor Heading  Roll  Pitch");
            Alignment_avg_rougth.WriteLine("f_1 f_2 f_3 w_1 w_2 w_3 heading roll pitch Latitude");

            if (SINSstate.Global_file != "Saratov_01.11.2012")
            {
                while (true)
                {
                    i++;
                    if (i < 10) { myFile.ReadLine(); continue; }
                    if (SINSstate.FLG_Stop == 0)
                    {
                        ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate);
                    }
                    else
                    {
                        i--;
                        break;
                    }
                }
            }
            else
            {
                while (true)
                {
                    i++;
                    if (i < 20) { myFile.ReadLine(); continue; }

                    break;
                }
            }
            int t = i;

            for (i = t; ; i++)
            {
                ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate);

                array_sigma_f_1[k] = SINSstate.F_z[0];
                array_sigma_f_2[k] = SINSstate.F_z[1];
                array_sigma_f_3[k] = SINSstate.F_z[2];
                array_sigma_w_1[k] = SINSstate.W_z[0];
                array_sigma_w_2[k] = SINSstate.W_z[1];
                array_sigma_w_3[k] = SINSstate.W_z[2];

                if (SINSstate.Global_file != "Saratov_01.11.2012")
                {
                    if (SINSstate.FLG_Stop == 0 || (ProcHelp.AlgnCnt != 0 && i == ProcHelp.AlgnCnt))
                        break;
                }
                else
                {
                    if (ProcHelp.AlgnCnt != 0 && i == ProcHelp.AlgnCnt)
                        break;
                }

                if ((i > 7000) || SINSstate.Global_file != "Saratov_01.11.2012")
                {

                    f_avg[0] += SINSstate.F_z[0]; w_avg[0] += SINSstate.W_z[0];
                    f_avg[1] += SINSstate.F_z[1]; w_avg[1] += SINSstate.W_z[1];
                    f_avg[2] += SINSstate.F_z[2]; w_avg[2] += SINSstate.W_z[2];
                    k++;

                    SINSstate.Pitch = Math.Atan(f_avg[1] / Math.Sqrt(f_avg[0] * f_avg[0] + f_avg[2] * f_avg[2]));
                    SINSstate.Roll = -Math.Atan(f_avg[0] / f_avg[2]);
                    A_xs = SimpleOperations.A_xs(SINSstate);
                    w_avg_x = Matrix.Multiply(A_xs, w_avg);

                    SINSstate.Heading = -Math.Atan(w_avg_x[0] / w_avg_x[1]);
                    //SINSstate.Latitude = Math.Atan(w_avg_x[2] / Math.Sqrt(w_avg_x[1] * w_avg_x[1] + w_avg_x[0] * w_avg_x[0]));

                    if (k > 200)
                        Alignment_avg_rougth.WriteLine((f_avg[0] / k).ToString() + " " + (f_avg[1] / k).ToString() + " " + (f_avg[2] / k).ToString() + " " + (w_avg[0] / k).ToString() + " " + (w_avg[1] / k).ToString() + " " + (w_avg[2] / k).ToString()
                            + " " + SINSstate.Heading.ToString() + " " + SINSstate.Roll.ToString() + " " + SINSstate.Pitch.ToString() + " " + SINSstate.Latitude.ToString());
                }
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

            KalmanVars.Noise_Vel = sigma_f[0] * 9.78049;
            KalmanVars.Noise_Angl = sigma_w[0];
            for (int j = 1; j < 3; j++)
            {
                if (KalmanVars.Noise_Vel < sigma_f[j] * 9.78049)
                    KalmanVars.Noise_Vel = sigma_f[j] * 9.78049;
                if (KalmanVars.Noise_Angl < sigma_w[j])
                    KalmanVars.Noise_Angl = sigma_w[j];
            }

            SINSstate.Pitch = Math.Atan2(f_avg[1], Math.Sqrt(f_avg[0] * f_avg[0] + f_avg[2] * f_avg[2]));
            SINSstate.Roll = -Math.Atan2(f_avg[0], f_avg[2]);
            A_xs = SimpleOperations.A_xs(SINSstate);
            w_avg_x = Matrix.Multiply(A_xs, w_avg);

            SINSstate.Heading = -Math.Atan2(w_avg_x[0], w_avg_x[1]);
            double Latitude = Math.Atan2(w_avg_x[2], Math.Sqrt(w_avg_x[1] * w_avg_x[1] + w_avg_x[0] * w_avg_x[0]));

            //double[] W_my = new double[3];
            //W_my[0] = -Math.Cos(SINSstate.Latitude) * Math.Sin(-1.93);
            //W_my[1] = Math.Cos(SINSstate.Latitude) * Math.Cos(-1.93);
            //W_my[2] = Math.Sin(SINSstate.Latitude);
            //W_my[0] = W_my[0] * SimpleData.U;
            //W_my[1] = W_my[1] * SimpleData.U;
            //W_my[2] = W_my[2] * SimpleData.U;

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);
            if (Math.Abs(w_avg[0] - U_s[0]) < 0.00001) { }
            else
            {
                SINSstate.Heading = SINSstate.Heading - Math.PI;
                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);
            }

            SINSstate.Time_Alignment = SINSstate.Time;

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time - SINSstate.Time_Alignment, SINSstate.Longitude_Start);
            //Далее произойдет обнуление SINSstate.Time
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);


            return i;
        }


        public static int SINSAlignment(Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate2, StreamReader myFile, Kalman_Vars KalmanVars)
        {
            int i = 0, t = 0;

            if (SINSstate.UsingClasAlignment == true || SINSstate.UsingNavAlignment == true)
            {
                i = RougthAlignment(ProcHelp, SINSstate, myFile, KalmanVars);
                SINSstate.Alignment = true;
            }

            if (SINSstate.UsingNavAlignment == true)
            {
                SimpleData.iMx = 13;
                SimpleData.iMq = 13;
                SimpleData.iMz = 7;

                Alignment.InitOfCovarianceMatrixes(KalmanVars);     //---Инициализация ковариационных матриц матриц вектора ошибок---//
                /*----------------------------------------------------------------------------------------*/

                while (i < 30000)
                {
                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate);
                    if (t == 0) { SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z); SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z); t = 1; }

                    SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate2);

                    Alignment.Make_A_easy(SINSstate, KalmanVars);
                    Alignment.Make_H(KalmanVars, SINSstate);
                    KalmanProcs.Make_F(SINSstate.timeStep, KalmanVars);
                    SINSprocessing.MatrixNoise_ReDef(SINSstate, KalmanVars, SINSstate.Alignment);   //изменить все эти функции

                    KalmanProcs.KalmanCorrection(KalmanVars);

                    Alignment.Make_StateErrors(KalmanVars.ErrorConditionVector_p, SINSstate);
                    Alignment.Do_SINSstate_Correction(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2);
                    KalmanProcs.KalmanForecast(KalmanVars);

                    OutPutInfo_Alignment(ProcHelp, SINSstate, SINSstate2, myFile, KalmanVars);

                    SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
                    SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z);

                    if (i > 10000 && i % 500 == 0)
                        Console.WriteLine(SINSstate.Count.ToString() + ",  " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + ",  " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + ",  " +
                                (SINSstate2.Heading * SimpleData.ToDegree).ToString() + ",  " + KalmanVars.ErrorConditionVector_p[0].ToString() + ",  " + KalmanVars.ErrorConditionVector_p[1].ToString());
                    i++;
                }
                Alignment_Errors.Close(); Alignment_Corrected_State.Close(); Alignment_SINSstate.Close(); Alignment_StateErrorsVector.Close();
                /*----------------------------------------------------------------------------------------*/

            }
            else if (SINSstate.UsingClasAlignment == true)
            {

            }


            SINSstate.Alignment = false;
            return i;
        }

        private static void OutPutInfo_Alignment(Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate2, StreamReader myFile, Kalman_Vars KalmanVars)
        {
            ProcHelp.datastring = KalmanVars.ErrorConditionVector_p[0].ToString() + " " + KalmanVars.ErrorConditionVector_p[1].ToString()
                                 + " " + KalmanVars.ErrorConditionVector_p[2].ToString() + " " + KalmanVars.ErrorConditionVector_p[3].ToString()
                                  + " " + (KalmanVars.ErrorConditionVector_p[4] * 180.0 / 3.141592).ToString() + " " + (KalmanVars.ErrorConditionVector_p[5] * 180.0 / 3.141592).ToString() + " " + (KalmanVars.ErrorConditionVector_p[6] * 180.0 / 3.141592).ToString()
                                   + " " + KalmanVars.ErrorConditionVector_p[7].ToString() + " " + KalmanVars.ErrorConditionVector_p[8].ToString() + " " + KalmanVars.ErrorConditionVector_p[9].ToString()
                                    + " " + KalmanVars.ErrorConditionVector_p[10].ToString() + " " + KalmanVars.ErrorConditionVector_p[11].ToString() + " " + KalmanVars.ErrorConditionVector_p[12].ToString();
            Alignment_Errors.WriteLine(ProcHelp.datastring);

            ProcHelp.datastring = (SINSstate.Count * SINSstate.timeStep).ToString() + " " + SINSstate.Count.ToString() + " " +
                            (SINSstate.Latitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + " " + SINSstate.Altitude.ToString() + " "
                            + ProcHelp.LatSNS.ToString() + " " + ProcHelp.LongSNS.ToString() + " " + SINSstate.Vx_0[0].ToString() + " " + SINSstate.Vx_0[1].ToString() + " " + SINSstate.GPS_Data.gps_Latitude.isReady.ToString() + " " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " "
                              + (SINSstate.Roll * SimpleData.ToDegree).ToString() + " " + (SINSstate.Pitch * SimpleData.ToDegree).ToString();
            Alignment_SINSstate.WriteLine(ProcHelp.datastring);

            ProcHelp.datastring = (SINSstate.Count * SINSstate.timeStep).ToString() + " " + SINSstate.Count.ToString() + " " + (SINSstate2.Latitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Latitude * SimpleData.ToDegree).ToString()
                            + " " + (SINSstate2.Longitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + " " + SINSstate2.Altitude.ToString() + " "
                            + SINSstate2.Vx_0[0].ToString() + " " + SINSstate2.Vx_0[1].ToString() + " " + SINSstate2.Vx_0[2].ToString() + " " + (SINSstate2.Heading * SimpleData.ToDegree).ToString() + " " +
                            (SINSstate.Heading * SimpleData.ToDegree).ToString() + " "
                            + (SINSstate2.Roll * SimpleData.ToDegree).ToString() + " " + (SINSstate2.Pitch * SimpleData.ToDegree).ToString();
            Alignment_Corrected_State.WriteLine(ProcHelp.datastring);

            ProcHelp.datastring = (SINSstate.DeltaLatitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.DeltaLongitude * SimpleData.ToDegree).ToString() + " " + SINSstate.DeltaV_1.ToString() + " " + SINSstate.DeltaV_2.ToString() + " "
                            + SINSstate.DeltaV_3.ToString() + " " + SINSstate.DeltaHeading.ToString() + " " + SINSstate.DeltaRoll.ToString() + " " + SINSstate.DeltaPitch.ToString();
            Alignment_StateErrorsVector.WriteLine(ProcHelp.datastring);
        }
    }
}
