using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace SINSAlignment
{
    class Alignment_Navigation
    {
        public static void InitOfCovarianceMatrixes(Kalman_Vars KalmanVars)
        {
            for (int i = 0; i < SimpleData.iMx * SimpleData.iMx; i++)
                KalmanVars.CovarianceMatrixS_m[i] = KalmanVars.CovarianceMatrixS_p[i] = 0.0;

            KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0] = KalmanVars.CovarianceMatrixS_p[0 * SimpleData.iMx + 0] = 0.1;    // позиционные ошибки
            KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1] = KalmanVars.CovarianceMatrixS_p[1 * SimpleData.iMx + 1] = 0.1;

            KalmanVars.CovarianceMatrixS_m[2 * SimpleData.iMx + 2] = KalmanVars.CovarianceMatrixS_p[2 * SimpleData.iMx + 2] = 0.01;   // 0.01 м/с
            KalmanVars.CovarianceMatrixS_m[3 * SimpleData.iMx + 3] = KalmanVars.CovarianceMatrixS_p[3 * SimpleData.iMx + 3] = 0.01;

            KalmanVars.CovarianceMatrixS_m[4 * SimpleData.iMx + 4] = KalmanVars.CovarianceMatrixS_p[4 * SimpleData.iMx + 4] = 5.0 * SimpleData.ToRadian_min;  // 5 угл. минут
            KalmanVars.CovarianceMatrixS_m[5 * SimpleData.iMx + 5] = KalmanVars.CovarianceMatrixS_p[5 * SimpleData.iMx + 5] = 5.0 * SimpleData.ToRadian_min;
            KalmanVars.CovarianceMatrixS_m[6 * SimpleData.iMx + 6] = KalmanVars.CovarianceMatrixS_p[6 * SimpleData.iMx + 6] = 5.0 * SimpleData.ToRadian_min;

            KalmanVars.CovarianceMatrixS_m[7 * SimpleData.iMx + 7] = KalmanVars.CovarianceMatrixS_p[7 * SimpleData.iMx + 7] = 0.001;    // м/с^2
            KalmanVars.CovarianceMatrixS_m[8 * SimpleData.iMx + 8] = KalmanVars.CovarianceMatrixS_p[8 * SimpleData.iMx + 8] = 0.001;
            KalmanVars.CovarianceMatrixS_m[9 * SimpleData.iMx + 9] = KalmanVars.CovarianceMatrixS_p[9 * SimpleData.iMx + 9] = 0.001;

            KalmanVars.CovarianceMatrixS_m[10 * SimpleData.iMx + 10] = KalmanVars.CovarianceMatrixS_p[10 * SimpleData.iMx + 10] = 0.2 * SimpleData.ToRadian / 3600.0; // 0.02 град/час
            KalmanVars.CovarianceMatrixS_m[11 * SimpleData.iMx + 11] = KalmanVars.CovarianceMatrixS_p[11 * SimpleData.iMx + 11] = 0.2 * SimpleData.ToRadian / 3600.0;
            KalmanVars.CovarianceMatrixS_m[12 * SimpleData.iMx + 12] = KalmanVars.CovarianceMatrixS_p[12 * SimpleData.iMx + 12] = 0.2 * SimpleData.ToRadian / 3600.0;
        }


        public static StreamWriter Alignment_Scalyar = new StreamWriter(SimpleData.PathOutputString + "Alignment_Scalyar.dat");
        StreamWriter Alignment_Measures = new StreamWriter(SimpleData.PathOutputString + "Alignment_Measures.dat");

        public static void Make_H(Kalman_Vars KalmanVars, SINS_State SINSstate)
        {
            double[] Wz = new double[3]; double[] Fz = new double[3];
            int i = 0;

            

            for (i = 0; i < 3; i++)
            {
                //Wz[i] = (SINSstate.W_z[i] + SINSstate.W_z_prev[i]) / 2.0;
                //Fz[i] = (SINSstate.F_z[i] + SINSstate.F_z_prev[i]) / 2.0;
                Wz[i] = SINSstate.W_z[i];
                Fz[i] = SINSstate.F_z[i];
            }

            //Динамические скоростные измерения
            KalmanVars.Matrix_H[0 * SimpleData.iMx + 2] = 1.0;
            KalmanVars.Matrix_H[1 * SimpleData.iMx + 3] = 1.0;

            KalmanVars.Measure[0] = SINSstate.Vx_0[0];
            KalmanVars.Measure[1] = SINSstate.Vx_0[1];

            KalmanVars.Noize_Z[0] = 0.01;
            KalmanVars.Noize_Z[1] = 0.01;

            KalmanVars.cnt_measures = 2;


            //Позиционные измерения, широта, долгота
            if (true)
            {
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 0] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * SimpleData.iMx + 1] = 1.0;

                KalmanVars.Measure[KalmanVars.cnt_measures] = (SINSstate.Longitude - SINSstate.Longitude_Start) * Math.Cos(SINSstate.Latitude_Start) * SimpleOperations.RadiusE(SINSstate.Latitude_Start, SINSstate.AltSNS);
                KalmanVars.Measure[KalmanVars.cnt_measures + 1] = (SINSstate.Latitude - SINSstate.Latitude_Start) * SimpleOperations.RadiusN(SINSstate.Latitude_Start, SINSstate.AltSNS);

                KalmanVars.Noize_Z[KalmanVars.cnt_measures] = 0.75;
                KalmanVars.Noize_Z[KalmanVars.cnt_measures + 1] = 0.75;

                KalmanVars.cnt_measures = KalmanVars.cnt_measures + 2;
            }

            
            //Скалярное измерение по модулю угловой скорости
            if (false)
            {
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 7] = 2 * Wz[0];
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 8] = 2 * Wz[1];
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 9] = 2 * Wz[2];

                KalmanVars.Measure[KalmanVars.cnt_measures] = Math.Pow(SimpleOperations.AbsoluteVectorValue(Wz), 2) - SimpleData.U * SimpleData.U;
                KalmanVars.Noize_Z[KalmanVars.cnt_measures] = 0.00001;//0.01 * SimpleData.ToRadian / 3600.0;

                KalmanVars.cnt_measures++;
            }

            //i++;
            ////Скалярное измерение по модулю силы тяжести
            //KalmanVars.Matrix_H[i * SimpleData.iMx + 7] = 2 * Fz[0];
            //KalmanVars.Matrix_H[i * SimpleData.iMx + 8] = 2 * Fz[1];
            //KalmanVars.Matrix_H[i * SimpleData.iMx + 9] = 2 * Fz[2];
            //KalmanVars.Measure[i] = Math.Pow(SimpleOperations.AbsoluteVectorValue(Fz), 2) - SINSstate.g * SINSstate.g;
            //KalmanVars.Noize_Z[i] = 0.05;
            //KalmanVars.cnt_measures = KalmanVars.cnt_measures + 1;

            //Скалярное измерение по модулvю силы тяжести и угловой скорости
            if (false)
            {
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 7] = Fz[0];
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 8] = Fz[1];
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 9] = Fz[2];
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 10] = Wz[0];
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 11] = Wz[1];
                KalmanVars.Matrix_H[KalmanVars.cnt_measures * SimpleData.iMx + 12] = Wz[2];

                KalmanVars.Measure[KalmanVars.cnt_measures] = SimpleOperations.SkalyarProduct(Fz, Wz) - SimpleData.U * SINSstate.g * Math.Sin(SINSstate.Latitude);
                KalmanVars.Noize_Z[KalmanVars.cnt_measures] = 0.001;

                KalmanVars.cnt_measures++;
            }


            Alignment_Scalyar.WriteLine(SINSstate.Count.ToString() + " " + KalmanVars.Measure[0].ToString() + " " + KalmanVars.Measure[1].ToString() + " " + KalmanVars.Measure[2].ToString()
                                       + " " + KalmanVars.Measure[3].ToString() + " " + KalmanVars.Measure[4].ToString() + " " + KalmanVars.Measure[5].ToString() + " " + KalmanVars.Measure[6].ToString());
            //Alignment_Measures.WriteLine(SINSstate.Count.ToString() + " " + Wz[0] + " " + SINSstate.W_z[0].ToString() + " " + Wz[1] + " " + Wz[2] + " " + Fz[0] + " " + Fz[1] + " " + Fz[2]);


            /*KalmanVars.Measure[6] = SINSstate.F_x[0];
            KalmanVars.Measure[7] = SINSstate.F_x[1];
            KalmanVars.Matrix_H[6 * SimpleData.iMx + 7] = -SINSstate.g;
            KalmanVars.Matrix_H[6 * SimpleData.iMx + 12] = 1.0;
            KalmanVars.Matrix_H[7 * SimpleData.iMx + 6] = SINSstate.g;
            KalmanVars.Matrix_H[7 * SimpleData.iMx + 13] = 1.0;*/
        }

        public static void Make_A(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            KalmanVars.Matrix_A[1] = SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[2] = -SINSstate.Omega_x[1];
            KalmanVars.Matrix_A[3] = 1.0;
            KalmanVars.Matrix_A[SimpleData.iMx] = -SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[SimpleData.iMx + 2] = SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[SimpleData.iMx + 4] = 1.0;
            KalmanVars.Matrix_A[2 * SimpleData.iMx] = SINSstate.Omega_x[1];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 1] = -SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 5] = 1.0;

            KalmanVars.Matrix_A[3 * SimpleData.iMx + 1] = -(SINSstate.u_x[1] * SINSstate.Vx_0[1] + SINSstate.u_x[2] * SINSstate.Vx_0[2]) / SimpleData.A;
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 4] = 2 * SINSstate.u_x[2] + SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 5] = -2 * SINSstate.u_x[1] - SINSstate.Omega_x[1];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 6] = -SINSstate.u_x[1] * SINSstate.Vx_0[1] - SINSstate.u_x[2] * SINSstate.Vx_0[2];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 7] = -SINSstate.g;
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 10] = SINSstate.Vx_0[2];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 11] = -SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 12] = 1.0;

            KalmanVars.Matrix_A[4 * SimpleData.iMx + 0] = SINSstate.u_x[2] * SINSstate.Vx_0[2] / SimpleData.A;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 1] = SINSstate.u_x[1] * SINSstate.Vx_0[0] / SimpleData.A;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 3] = -2 * SINSstate.u_x[2] - SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 5] = SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 6] = SINSstate.u_x[1] * SINSstate.Vx_0[0] + SINSstate.g;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 7] = -SINSstate.u_x[2] * SINSstate.Vx_0[2];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 8] = SINSstate.u_x[1] * SINSstate.Vx_0[2];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 9] = -SINSstate.Vx_0[2];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 11] = SINSstate.Vx_0[0];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 13] = 1.0;

            KalmanVars.Matrix_A[5 * SimpleData.iMx + 0] = -SINSstate.u_x[2] * SINSstate.Vx_0[1] / SimpleData.A;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 1] = SINSstate.u_x[2] * SINSstate.Vx_0[0] / SimpleData.A;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 2] = 2 * SINSstate.g / SimpleData.A;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 3] = 2 * SINSstate.u_x[1] + SINSstate.Omega_x[1];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 4] = -SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 6] = SINSstate.u_x[2] * SINSstate.Vx_0[0];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 7] = SINSstate.u_x[2] * SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 8] = -SINSstate.u_x[1] * SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 9] = SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 10] = -SINSstate.Vx_0[0];
            //KalmanVars.Matrix_A[5 * SimpleData.iMx + 9] = -SINSstate.V_x[2];
            //KalmanVars.Matrix_A[5 * SimpleData.iMx + 11] = SINSstate.V_x[0];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 14] = 1.0;

            KalmanVars.Matrix_A[6 * SimpleData.iMx + 0] = -SINSstate.u_x[2] / SimpleData.A;
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 2] = -SINSstate.Omega_x[0] / SimpleData.A;
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 4] = -1.0 / SimpleData.A;
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 7] = SINSstate.u_x[2] + SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 8] = -SINSstate.u_x[1];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 9] = -1.0;

            KalmanVars.Matrix_A[7 * SimpleData.iMx + 1] = -SINSstate.u_x[2] / SimpleData.A;
            KalmanVars.Matrix_A[7 * SimpleData.iMx + 2] = -SINSstate.Omega_x[1] / SimpleData.A;
            KalmanVars.Matrix_A[7 * SimpleData.iMx + 3] = 1.0 / SimpleData.A;
            KalmanVars.Matrix_A[7 * SimpleData.iMx + 6] = -(SINSstate.u_x[2] + SINSstate.Omega_x[2]);
            KalmanVars.Matrix_A[7 * SimpleData.iMx + 10] = -1.0;

            KalmanVars.Matrix_A[8 * SimpleData.iMx + 0] = SINSstate.Omega_x[0] / SimpleData.A;
            KalmanVars.Matrix_A[8 * SimpleData.iMx + 1] = (SINSstate.u_x[1] + SINSstate.Omega_x[1]) / SimpleData.A;
            KalmanVars.Matrix_A[8 * SimpleData.iMx + 6] = SINSstate.u_x[1] + SINSstate.Omega_x[1];
            KalmanVars.Matrix_A[8 * SimpleData.iMx + 7] = -SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[8 * SimpleData.iMx + 11] = -1.0;
        }





        public static void Make_A_easy(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            KalmanVars.Matrix_A[2] = 1.0;
            KalmanVars.Matrix_A[SimpleData.iMx + 3] = 1.0;

            KalmanVars.Matrix_A[2 * SimpleData.iMx + 3] = 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 5] = -SINSstate.g;
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 10] = SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 11] = SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 12] = SINSstate.A_x0s[0, 2];

            KalmanVars.Matrix_A[3 * SimpleData.iMx + 2] = -2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 4] = SINSstate.g;
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 10] = SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 11] = SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 12] = SINSstate.A_x0s[1, 2];

            KalmanVars.Matrix_A[4 * SimpleData.iMx + 0] = -SINSstate.u_x[2] / SimpleData.A;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 3] = -1.0 / SimpleData.A;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 5] = SINSstate.u_x[2];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 6] = -SINSstate.u_x[1];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 7] = -SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 8] = -SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 9] = -SINSstate.A_x0s[0, 2];

            KalmanVars.Matrix_A[5 * SimpleData.iMx + 1] = -SINSstate.u_x[2] / SimpleData.A;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 2] = 1.0 / SimpleData.A;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 4] = -SINSstate.u_x[2];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 7] = -SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 8] = -SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 9] = -SINSstate.A_x0s[1, 2];

            KalmanVars.Matrix_A[6 * SimpleData.iMx + 1] = SINSstate.u_x[1] / SimpleData.A;
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 4] = SINSstate.u_x[1];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 7] = -SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 8] = -SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 9] = -SINSstate.A_x0s[2, 2];
        }



        public static void CalcStateErrors(double[] ErrorVector, SINS_State SINSstate)
        {
            SINSstate.DeltaLatitude = ErrorVector[1] / SINSstate.R_n;
            SINSstate.DeltaLongitude = ErrorVector[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

            SINSstate.DeltaV_1 = ErrorVector[2] + SINSstate.Vx_0[1] * ErrorVector[6] + SINSstate.Vx_0[1] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
            SINSstate.DeltaV_2 = ErrorVector[3] - SINSstate.Vx_0[0] * ErrorVector[6] - SINSstate.Vx_0[0] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);

            SINSstate.DeltaRoll = -(ErrorVector[4] * Math.Sin(SINSstate.Heading) + ErrorVector[5] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
            SINSstate.DeltaPitch = -ErrorVector[4] * Math.Cos(SINSstate.Heading) + ErrorVector[5] * Math.Sin(SINSstate.Heading);
            SINSstate.DeltaHeading = ErrorVector[6] + SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude) + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);
        }

        public static void StateCorrection(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate2)
        {
            //SINSstate2.Altitude = SINSstate.Altitude - SINSstate.DeltaAltitude;

            SINSstate2.Latitude = SINSstate.Latitude - SINSstate.DeltaLatitude;
            SINSstate2.Longitude = SINSstate.Longitude - SINSstate.DeltaLongitude;

            SINSstate2.Vx_0[0] = SINSstate.Vx_0[0] - SINSstate.DeltaV_1;
            SINSstate2.Vx_0[1] = SINSstate.Vx_0[1] - SINSstate.DeltaV_2;

            SINSstate2.Roll = SINSstate.Roll - SINSstate.DeltaRoll;
            SINSstate2.Pitch = SINSstate.Pitch - SINSstate.DeltaPitch;
            SINSstate2.Heading = SINSstate.Heading - SINSstate.DeltaHeading;

            //корректированная матрица ориентации
            SINSstate2.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate2.A_x0s = SINSstate2.A_sx0.Transpose();
            SINSstate2.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate2.A_nx0 = SINSstate2.A_x0n.Transpose();
        }


        public static void MatrixNoise_ReDef(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            int iMx = SimpleData.iMx, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.value_iMx_dr3;

            //ПЕРЕДЕЛАТЬ

            double[] Noise_Vel_in_Mx = new double[3], Noise_Angl_in_Mx = new double[3];

            for (int i = 0; i < iMx * iMq; i++)
                KalmanVars.CovarianceMatrixNoise[i] = 0.0;

            //CopyArray(Noise_Vel_in_Mx, SINSstate.A_x0s * KalmanVars.Noise_Vel);
            //CopyArray(Noise_Angl_in_Mx, SINSstate.A_x0s * KalmanVars.Noise_Angl);
            for (int j = 0; j < 3; j++)
            {
                Noise_Vel_in_Mx[j] = Math.Sqrt(SINSstate.A_x0s[j, 0] * SINSstate.A_x0s[j, 0] * KalmanVars.Noise_Vel[0] * KalmanVars.Noise_Vel[0] +
                                               SINSstate.A_x0s[j, 1] * SINSstate.A_x0s[j, 1] * KalmanVars.Noise_Vel[1] * KalmanVars.Noise_Vel[1] +
                                               SINSstate.A_x0s[j, 2] * SINSstate.A_x0s[j, 2] * KalmanVars.Noise_Vel[2] * KalmanVars.Noise_Vel[2]);
                Noise_Angl_in_Mx[j] = Math.Sqrt(SINSstate.A_x0s[j, 0] * SINSstate.A_x0s[j, 0] * KalmanVars.Noise_Angl[0] * KalmanVars.Noise_Angl[0] +
                                               SINSstate.A_x0s[j, 1] * SINSstate.A_x0s[j, 1] * KalmanVars.Noise_Angl[1] * KalmanVars.Noise_Angl[1] +
                                               SINSstate.A_x0s[j, 2] * SINSstate.A_x0s[j, 2] * KalmanVars.Noise_Angl[2] * KalmanVars.Noise_Angl[2]);
            }
            double sqrt_freq = Math.Sqrt(SINSstate.Freq);


            KalmanVars.CovarianceMatrixNoise[2 * iMq + 0] = Math.Abs(KalmanVars.Noise_Vel[0] * SINSstate.A_x0s[0, 0] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[2 * iMq + 1] = Math.Abs(KalmanVars.Noise_Vel[1] * SINSstate.A_x0s[0, 1] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[3 * iMq + 0] = Math.Abs(KalmanVars.Noise_Vel[0] * SINSstate.A_x0s[1, 0] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[3 * iMq + 1] = Math.Abs(KalmanVars.Noise_Vel[1] * SINSstate.A_x0s[1, 1] * sqrt_freq);

            KalmanVars.CovarianceMatrixNoise[4 * iMq + 2] = Math.Abs(KalmanVars.Noise_Angl[0] * SINSstate.A_x0s[0, 0] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[4 * iMq + 3] = Math.Abs(KalmanVars.Noise_Angl[1] * SINSstate.A_x0s[0, 1] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[4 * iMq + 4] = Math.Abs(KalmanVars.Noise_Angl[2] * SINSstate.A_x0s[0, 2] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[5 * iMq + 2] = Math.Abs(KalmanVars.Noise_Angl[0] * SINSstate.A_x0s[1, 0] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[5 * iMq + 3] = Math.Abs(KalmanVars.Noise_Angl[1] * SINSstate.A_x0s[1, 1] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[5 * iMq + 4] = Math.Abs(KalmanVars.Noise_Angl[2] * SINSstate.A_x0s[1, 2] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[6 * iMq + 2] = Math.Abs(KalmanVars.Noise_Angl[0] * SINSstate.A_x0s[2, 0] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[6 * iMq + 3] = Math.Abs(KalmanVars.Noise_Angl[1] * SINSstate.A_x0s[2, 1] * sqrt_freq);
            KalmanVars.CovarianceMatrixNoise[6 * iMq + 4] = Math.Abs(KalmanVars.Noise_Angl[2] * SINSstate.A_x0s[2, 2] * sqrt_freq);


            //KalmanVars.CovarianceMatrixNoise[0 * iMq + 0] = KalmanVars.CovarianceMatrixNoise[1 * iMq + 1] = KalmanVars.Noise_Pos * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[2 * iMq + 2] = Noise_Vel_in_Mx[0] * 9.78049 * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[3 * iMq + 3] = Noise_Vel_in_Mx[1] * 9.78049 * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[4 * iMq + 4] = Noise_Angl_in_Mx[0] * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[5 * iMq + 5] = Noise_Angl_in_Mx[1] * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[6 * iMq + 6] = Noise_Angl_in_Mx[2] * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[7 * iMq + 7] = KalmanVars.CovarianceMatrixNoise[8 * iMq + 8] = KalmanVars.CovarianceMatrixNoise[9 * iMq + 9] = KalmanVars.Noise_Drift * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[10 * iMq + 10] = KalmanVars.CovarianceMatrixNoise[11 * iMq + 11] = KalmanVars.CovarianceMatrixNoise[12 * iMq + 12] = KalmanVars.Noise_Accel * sqrt_freq;

            //if (SINSstate.flag_iMx_r3_dV3)
            //{
            //    KalmanVars.CovarianceMatrixNoise[iMx_r3_dV3 * iMq + iMx_r3_dV3] = KalmanVars.Noise_Pos * sqrt_freq;
            //    KalmanVars.CovarianceMatrixNoise[(iMx_r3_dV3 + 1) * iMq + (iMx_r3_dV3 + 1)] = Noise_Vel_in_Mx[2] * 9.78049 * sqrt_freq;
            //}
        }


    }
}
