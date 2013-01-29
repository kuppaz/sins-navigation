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
    public partial class Odometr_SINS
    {
        public static void Odometr_SINSProcessing(int StartCount, Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod, ParamsForModel OdoModel, StreamReader myFile, Kalman_Vars KalmanVars, 
                                            StreamWriter ForHelp, StreamWriter Nav_EstimateSolution, StreamWriter Nav_Autonomous, StreamWriter Nav_FeedbackSolution, StreamWriter Nav_vert_chan_test, StreamWriter Nav_StateErrorsVector, StreamWriter Nav_Errors)
        {
            int t = 0;
            SINSstate.UsingCorrection = false;
            SINSstate.feedbackExist = true;

            for (int i = StartCount; i < SINSstate.LastCountForRead; i++)
            {
                if (SINSstate.UsingClasAlignment == false) { if (i < ProcHelp.AlgnCnt) { myFile.ReadLine(); continue; } }

                ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate);
                ProcessingHelp.DefSNSData(ProcHelp, SINSstate);

                if (t == 0) 
                { 
                    SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z); 
                    SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z); 
                    t = 1; 
                }

                double[] dS_x = new double[3];

                if (SINSstate.OdometerData.odometer_left.isReady != 1)
                {
                    SINSstate.OdoTimeStepCount++;
                    SINSstate.UsingCorrection = false;
                }
                else if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    SINSstate.OdoTimeStepCount++;

                    SINSstate.OdometerVector[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev);
                    SINSstate.OdoSpeed[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev) / SINSstate.OdoTimeStepCount / SINSstate.timeStep;

                    SimpleOperations.CopyArray(dS_x, SINSstate.A_x0s * SINSstate.OdometerVector);

                    SINSstate2.Latitude = SINSstate.Latitude = SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude + dS_x[1] / SINSstate.R_n;
                    SINSstate2.Longitude = SINSstate.Longitude = SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude + dS_x[0] / SINSstate.R_e / Math.Cos(SINSstate_OdoMod.Latitude);
                    SINSstate2.Altitude = SINSstate.Altitude = SINSstate_OdoMod.Altitude = SINSstate_OdoMod.Altitude + dS_x[2];

                    SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_x0n_prev, SINSstate_OdoMod.A_x0n);
                    SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Longitude);
                }



                if (SINSstate.FLG_Stop == 1)
                {
                    //SINSstate.KNS_flg = true;
                    //SINSstate.UsingCorrection = true;
                }
                else
                {
                    SINSstate.KNS_flg = false;
                }


                //---------------------------------------MAIN STEPS----------------------------------------------------
                StateIntegration_AT_odo(SINSstate, KalmanVars, SINSstate2, SINSstate_OdoMod);
                SINSprocessing.Make_A(SINSstate, KalmanVars);

                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                    KalmanProcs.KalmanForecast(KalmanVars);



                if (SINSstate.UsingCorrection == true || (SINSstate.GPS_Data.gps_Altitude.isReady == 1 && SINSstate.usingSNS == true))
                {
                    SINSprocessing.MAKE_H_AND_CORRECTION(KalmanVars, SINSstate, SINSstate_OdoMod);

                    //Может надо будет переместить обратно в условие
                    SINSprocessing.CalcStateErrors(KalmanVars.ErrorConditionVector_p, SINSstate);
                    if (SINSstate.feedbackExist == false)
                        SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2);
                    else
                        SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate);

                    ProcHelp.corrected = 1;
                }
                else
                {
                    ProcHelp.corrected = 0;
                }

                ForHelp.WriteLine(KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0].ToString() + " " + KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1]);

                /*------------------------------------OUTPUT-------------------------------------------------*/
                ProcessingHelp.OutPutInfo(i, ProcHelp, OdoModel, SINSstate, SINSstate2, KalmanVars, Nav_EstimateSolution, Nav_Autonomous, Nav_FeedbackSolution, Nav_vert_chan_test, Nav_StateErrorsVector, Nav_Errors);

                if (i > 10000 && i % 2000 == 0)
                    Console.WriteLine(SINSstate.Count.ToString() + ",  " + (SINSstate.Latitude * SimpleData.ToDegree - ProcHelp.LatSNS).ToString() + ",  " + ProcHelp.distance_from_start.ToString() + ",  " + SINSstate.F_x[2].ToString().ToString());

                if ((SINSstate.UsingCorrection == true || (SINSstate.GPS_Data.gps_Altitude.isReady == 1 && SINSstate.usingSNS == true)) && SINSstate.feedbackExist == true)
                {
                    SINSprocessing.NullingOfCorrectedErrors(KalmanVars);
                }


                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    SINSstate.OdometerLeftPrev_2 = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerRightPrev_2 = SINSstate.OdometerData.odometer_right.Value;
                    SINSstate.OdoSpeedPrev_2 = OdoModel.V_odo;
                    SINSstate.OdoTimeStepCount_2 = 0;

                    if (SINSstate.UsingCorrection == true || SINSstate.Odometr_SINS_case == true)
                    {
                        SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                        SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                        SINSstate.OdoSpeedPrev = OdoModel.V_odo;
                        SINSstate.OdoTimeStepCount = 0;

                        SINSstate.Latitude_prev = SINSstate.Latitude; SINSstate2.Latitude_prev = SINSstate2.Latitude;
                        SINSstate.Longitude_prev = SINSstate.Longitude; SINSstate2.Longitude_prev = SINSstate2.Longitude;
                        SINSstate.Altitude_prev = SINSstate.Altitude; SINSstate2.Altitude_prev = SINSstate2.Altitude;
                    }
                }

            }
        }









        public static void StateIntegration_AT_odo(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate2, SINS_State SINSstate_OdoMod)
        {
            double[] fz = new double[3], Wz = new double[3], Vx = new double[3], u = new double[3], tempV = new double[3], Wz_avg = new double[3];
            double[] Vx_0 = new double[3];

            Matrix AT_z_xi = new Matrix(3, 3); Matrix B_x_eta = new Matrix(3, 3);
            Matrix dAT = new Matrix(3, 3); Matrix D_x_z = new Matrix(3, 3);
            Matrix W_x_xi = new Matrix(3, 3); Matrix C_eta_xi = new Matrix(3, 3);

            Matrix Hat1 = new Matrix(3, 3);
            Matrix Hat2 = new Matrix(3, 3);
            Matrix E = Matrix.UnitMatrix(3);
            Matrix dMatrix = new Matrix(3, 3);

            double W_z_abs, Omega_x_abs, dlt, dlt2, Altitude, Altitude_prev, dh, dVx, dVy, dVh;
            double kren, tang, gkurs, Azimth;

            if (SINSstate.Odometr_SINS_case)
                SimpleOperations.CopyMatrix(SINSstate.A_x0n, SINSstate_OdoMod.A_x0n_prev);

            SimpleOperations.CopyMatrix(AT_z_xi, SINSstate.AT);
            SimpleOperations.CopyMatrix(B_x_eta, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            //C_eta_xi = Matrix.DoA_eta_xi(SINSstate.Time);
            Altitude = SINSstate.Altitude;
            Altitude_prev = SINSstate.Altitude_prev;
            Azimth = SINSstate.Azimth;

            fz[1] = SINSstate.F_z[1]; Wz[1] = SINSstate.W_z[1];
            fz[2] = SINSstate.F_z[2]; Wz[2] = SINSstate.W_z[2];
            fz[0] = SINSstate.F_z[0]; Wz[0] = SINSstate.W_z[0];
            SimpleOperations.CopyArray(SINSstate.F_z, fz);
            SimpleOperations.CopyArray(SINSstate.W_z, Wz);
            SimpleOperations.CopyArray(Vx_0, SINSstate.Vx_0);

            Vx[0] = Vx_0[0] * Math.Cos(Azimth) + Vx_0[1] * Math.Sin(Azimth);
            Vx[1] = -Vx_0[0] * Math.Sin(Azimth) + Vx_0[1] * Math.Cos(Azimth);
            Vx[2] = Vx_0[2];

            SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.u_x = SimpleOperations.U_x0(SINSstate.Latitude);

            u[0] = SimpleData.U * Math.Cos(SINSstate.Latitude) * Math.Sin(Azimth);
            u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude) * Math.Cos(Azimth);
            u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);


            //-------------ИНТЕГРИРОВАНИЕ МАТРИЦЫ AT_Z_XI И ПЕРВОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z---------

            if (SINSstate.UsingAvegering == true)
            {
                for (int i = 0; i < 3; i++)
                {
                    fz[i] = (fz[i] + SINSstate.F_z_prev[i]) / 2.0;
                    Wz[i] = (Wz[i] + SINSstate.W_z_prev[i]) / 2.0;
                }
            }

            W_z_abs = Math.Sqrt(Wz[0] * Wz[0] + Wz[1] * Wz[1] + Wz[2] * Wz[2]);
            dlt = Math.Sin(W_z_abs * SINSstate.timeStep) / W_z_abs;
            dlt2 = (1.0 - Math.Cos(W_z_abs * SINSstate.timeStep)) / (W_z_abs * W_z_abs);

            Hat1 = Matrix.SkewSymmetricMatrix(Wz);
            Hat2 = Matrix.SkewSymmetricMatrixSquare(Wz);

            SimpleOperations.CopyMatrix(dMatrix, (E + Hat1 * dlt + Hat2 * dlt2));
            SimpleOperations.CopyMatrix(AT_z_xi, (dMatrix * AT_z_xi));

            //Нормировка
            for (int i = 0; i < 3; i++)
            {
                tempV[i] = Math.Sqrt(AT_z_xi[i, 0] * AT_z_xi[i, 0] + AT_z_xi[i, 1] * AT_z_xi[i, 1] + AT_z_xi[i, 2] * AT_z_xi[i, 2]);
                for (int j = 0; j < 3; j++)
                    AT_z_xi[i, j] = AT_z_xi[i, j] / tempV[i];
            }

            SimpleOperations.CopyMatrix(SINSstate.AT, AT_z_xi);

            SimpleOperations.CopyMatrix(W_x_xi, B_x_eta * SINSstate.A_nxi);
            //CopyMatrix(W_x_xi, B_x_eta * C_eta_xi);
            SimpleOperations.CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());
            //--------------------------------------------------------------------------------------




            //---------------------------------ИНТЕГРИРОВАНИЕ СКОРОСТЕЙ----------------------------
            SimpleOperations.CopyArray(SINSstate.F_x, D_x_z * fz);

            SINSstate.g = 9.78049 * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014;
            if (true)
                SINSstate.g -= 2 * 0.000001538 * Altitude;

            //SINSstate.Omega_x[0] = -Vx[1] / R_n;
            //SINSstate.Omega_x[1] = Vx[0] / R_e;

            SINSstate.Omega_x[0] = -Vx_0[1] / SINSstate.R_n * Math.Cos(Azimth) + Vx_0[0] / SINSstate.R_e * Math.Sin(Azimth);
            SINSstate.Omega_x[1] = Vx_0[1] / SINSstate.R_n * Math.Sin(Azimth) + Vx_0[0] / SINSstate.R_e * Math.Cos(Azimth);

            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * Vx_0[0] / SINSstate.R_e;
            //Vx[2] = 0.0;


            if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                SimpleOperations.CopyMatrix(B_x_eta, SINSstate_OdoMod.A_x0n);

                Matrix B = new Matrix(3, 3);
                double gamma = 0.0;
                double[] gamma_ = new double[3];

                SimpleOperations.CopyMatrix(B, SINSstate_OdoMod.A_x0n * SINSstate_OdoMod.A_x0n_prev.Transpose() - Matrix.UnitMatrix(3));

                if (Math.Abs(B[0, 0] + B[1, 1] - B[2, 2]) > Math.Abs(B[0, 0] - B[1, 1] + B[2, 2]))
                {
                    if (Math.Abs(B[0, 0] + B[1, 1] - B[2, 2]) > Math.Abs(-B[0, 0] + B[1, 1] + B[2, 2]))
                        gamma = Math.Asin(0.5 * Math.Abs(B[0, 1] - B[1, 0]) * Math.Sqrt((B[0, 0] + B[1, 1] + B[2, 2]) / (B[0, 0] + B[1, 1] - B[2, 2])));
                    else
                        gamma = Math.Asin(0.5 * Math.Abs(B[1, 2] - B[2, 1]) * Math.Sqrt((B[0, 0] + B[1, 1] + B[2, 2]) / (-B[0, 0] + B[1, 1] + B[2, 2])));
                }
                else if (Math.Abs(B[0, 0] - B[1, 1] + B[2, 2]) > Math.Abs(-B[0, 0] + B[1, 1] + B[2, 2]))
                    gamma = Math.Asin(0.5 * Math.Abs(B[2, 0] - B[0, 2]) * Math.Sqrt((B[0, 0] + B[1, 1] + B[2, 2]) / (B[0, 0] - B[1, 1] + B[2, 2])));
                else
                    gamma = Math.Asin(0.5 * Math.Abs(B[1, 2] - B[2, 1]) * Math.Sqrt((B[0, 0] + B[1, 1] + B[2, 2]) / (-B[0, 0] + B[1, 1] + B[2, 2])));

                if (gamma != 0 && gamma.ToString() != "NaN")
                {
                    gamma_[0] = gamma * (B[1, 2] - B[2, 1]) / 2.0 / Math.Sin(gamma);
                    gamma_[1] = gamma * (B[2, 0] - B[0, 2]) / 2.0 / Math.Sin(gamma);
                    gamma_[2] = gamma * (B[0, 1] - B[1, 0]) / 2.0 / Math.Sin(gamma);
                }

                SINSstate.Omega_x[0] = gamma_[0] / SINSstate.OdoTimeStepCount / SINSstate.timeStep;
                SINSstate.Omega_x[1] = gamma_[1] / SINSstate.OdoTimeStepCount / SINSstate.timeStep;
                SINSstate.Omega_x[2] = gamma_[2] / SINSstate.OdoTimeStepCount / SINSstate.timeStep;

                Vx[0] = gamma_[1] / SINSstate.OdoTimeStepCount / SINSstate.timeStep * SINSstate.R_e;
                Vx[1] = -gamma_[0] / SINSstate.OdoTimeStepCount / SINSstate.timeStep * SINSstate.R_n;
            }


            SimpleOperations.CopyMatrix(W_x_xi, B_x_eta * SINSstate.A_nxi);
            //CopyMatrix(W_x_xi, B_x_eta * C_eta_xi);
            SimpleOperations.CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());



            //----------------Вычисление углов и переприсвоение матриц---------------------------
            SimpleOperations.CopyMatrix(SINSstate.A_sx0, D_x_z.Transpose());
            SimpleOperations.CopyMatrix(SINSstate.A_x0s, D_x_z);
            SimpleOperations.CopyMatrix(SINSstate.A_x0n, B_x_eta);
            SimpleOperations.CopyMatrix(SINSstate.A_nx0, B_x_eta.Transpose());

            //---ОПРЕДЕЛЕНИЕ ГЕОГРАФИЧЕСКИХ КООРДИНАТ---
            SINSstate.Longitude = Math.Atan2(B_x_eta[2, 1], B_x_eta[2, 0]);
            SINSstate.Latitude = Math.Atan2(B_x_eta[2, 2], Math.Sqrt(B_x_eta[0, 2] * B_x_eta[0, 2] + B_x_eta[1, 2] * B_x_eta[1, 2]));
            Azimth = Math.Atan2(B_x_eta[0, 2], B_x_eta[1, 2]);
            SINSstate.Altitude_prev = SINSstate.Altitude;
            SINSstate.Altitude = Altitude;

            // определение углов курса,крена,тангажа
            kren = -Math.Atan2(SINSstate.A_sx0[0, 2], SINSstate.A_sx0[2, 2]);
            tang = Math.Atan2(SINSstate.A_sx0[1, 2], Math.Sqrt(SINSstate.A_sx0[0, 2] * SINSstate.A_sx0[0, 2] + SINSstate.A_sx0[2, 2] * SINSstate.A_sx0[2, 2]));
            //if (SINSstate.Global_file != "Saratov_01.11.2012")
            gkurs = Math.Atan2(SINSstate.A_sx0[1, 0], SINSstate.A_sx0[1, 1]);
            //else
            //    gkurs = -Math.Atan2(SINSstate.A_sx0[1, 0], SINSstate.A_sx0[1, 1]);

            Vx_0[0] = Vx[0] * Math.Cos(Azimth) - Vx[1] * Math.Sin(Azimth);
            Vx_0[1] = Vx[0] * Math.Sin(Azimth) + Vx[1] * Math.Cos(Azimth);
            Vx_0[2] = Vx[2];

            SINSstate.Heading = gkurs - Azimth;
            SINSstate.Roll = kren;
            SINSstate.Pitch = tang;
            SINSstate.Azimth = Azimth;

            SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.u_x = SimpleOperations.U_x0(SINSstate.Latitude);

            SimpleOperations.CopyArray(SINSstate.W_x, SINSstate.A_x0s * SINSstate.W_z);

            SimpleOperations.CopyArray(SINSstate.Vx_0_prev, SINSstate.Vx_0);
            SimpleOperations.CopyArray(SINSstate.Vx_0, Vx_0);
            SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
            SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
            //--------------------------------------------------------------------------------------
        }
    }
}
