using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;

namespace Common_Namespace
{
    public class Alignment : SimpleOperations
    {
        static StreamWriter Alignment_Scalyar = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//Motion Imitator//MovingImitator//SINS motion processing_new data//Alignment_Scalyar.dat");
        static StreamWriter Alignment_Measures = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//Motion Imitator//MovingImitator//SINS motion processing_new data//Alignment_Measures.dat");

        /*----------------------Для выставки-------------------------------------*/

        public static void Make_H(Kalman_Vars KalmanVars, SINS_State SINSstate)
        {
            double[] Wz = new double[3]; double[] Fz = new double[3];
            int i = 0;

            for (i = 0; i < 3; i++)
            {
                Wz[i] = (SINSstate.W_z[i] + SINSstate.W_z_prev[i]) / 2.0;
                Fz[i] = (SINSstate.F_z[i] + SINSstate.F_z_prev[i]) / 2.0;
            }

            //Позиционные измерения, широта, долгота
            KalmanVars.Measure[0] = (SINSstate.Longitude - SINSstate.LongSNS) * Math.Cos(SINSstate.Latitude_Start) * RadiusE(SINSstate.Latitude_Start, SINSstate.AltSNS);
            KalmanVars.Measure[1] = (SINSstate.Latitude - SINSstate.Latitude_Start) * RadiusN(SINSstate.Latitude_Start, SINSstate.AltSNS);
            for (i = 0; i < 2; i++)
            {
                KalmanVars.Matrix_H[i * SimpleData.iMx + i] = 1.0;
                KalmanVars.Noize_Z[i] = 0.75;
            }
            KalmanVars.cnt_measures = 2;

            //Динамические скоростные измерения
            for (i = 2; i < 4; i++)
            {
                KalmanVars.Measure[i] = SINSstate.Vx_0[i-2];
                KalmanVars.Matrix_H[i * SimpleData.iMx + i] = 1.0;
                KalmanVars.Noize_Z[i] = 0.003;
            }
            KalmanVars.cnt_measures = KalmanVars.cnt_measures + 2;
            i--;

            i++;
            //Скалярное измерение по модулю угловой скорости
            KalmanVars.Matrix_H[i * SimpleData.iMx + 10] = 2 * Wz[0];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 11] = 2 * Wz[1];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 12] = 2 * Wz[2];
            KalmanVars.Measure[i] = Math.Pow(AbsoluteVectorValue(Wz), 2) - SimpleData.U * SimpleData.U;
            KalmanVars.Noize_Z[i] = 0.03 * SimpleData.ToRadian / 3600.0;
            KalmanVars.cnt_measures = KalmanVars.cnt_measures + 1;

            i++;
            //Скалярное измерение по модулю силы тяжести
            KalmanVars.Matrix_H[i * SimpleData.iMx + 7] = 2 * Fz[0];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 8] = 2 * Fz[1];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 9] = 2 * Fz[2];
            KalmanVars.Measure[i] = Math.Pow(AbsoluteVectorValue(Fz), 2) - SINSstate.g * SINSstate.g;
            KalmanVars.Noize_Z[i] = 0.05;
            KalmanVars.cnt_measures = KalmanVars.cnt_measures + 1;

            i++;
            //Скалярное измерение по модулvю силы тяжести и угловой скорости
            KalmanVars.Matrix_H[i * SimpleData.iMx + 7] = Wz[0];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 8] = Wz[1];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 9] = Wz[2];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 10] = Fz[0];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 11] = Fz[1];
            KalmanVars.Matrix_H[i * SimpleData.iMx + 12] = Fz[2];
            KalmanVars.Measure[i] = SkalyarProduct(Fz, Wz) - SimpleData.U * SINSstate.g * Math.Sin(SINSstate.Latitude);
            KalmanVars.Noize_Z[i] = 0.005;
            KalmanVars.cnt_measures = KalmanVars.cnt_measures + 1;


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
            KalmanVars.Matrix_A[7 * SimpleData.iMx + 8] = SINSstate.u_x[0];
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
            KalmanVars.Matrix_A[1] = SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[6] = SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[SimpleData.iMx + 3] = 1.0;
            KalmanVars.Matrix_A[SimpleData.iMx + 0] = -SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[SimpleData.iMx + 6] = -SINSstate.Vx_0[0];

            KalmanVars.Matrix_A[2 * SimpleData.iMx + 3] = (SINSstate.Omega_x[2] + 2 * SINSstate.u_x[2]);
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 5] = -SINSstate.g;
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 7] = SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 8] = SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 9] = SINSstate.A_x0s[0, 2];

            KalmanVars.Matrix_A[3 * SimpleData.iMx + 2] = -2 * SINSstate.u_x[2] - SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 4] = SINSstate.g;
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 7] = SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 8] = SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 9] = SINSstate.A_x0s[1, 2];


            KalmanVars.Matrix_A[4 * SimpleData.iMx + 0] = -SINSstate.u_x[2] / SimpleData.A;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 3] = -1.0 / SimpleData.A;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 5] = SINSstate.u_x[2] + SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 6] = -SINSstate.u_x[1] - SINSstate.Omega_x[1];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 10] = SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 11] = SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 12] = SINSstate.A_x0s[0, 2];

            KalmanVars.Matrix_A[5 * SimpleData.iMx + 1] = -SINSstate.u_x[2] / SimpleData.A;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 2] = 1.0 / SimpleData.A;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 4] = -SINSstate.u_x[2] - SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 6] = SINSstate.u_x[0] + SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 10] = SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 11] = SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 12] = SINSstate.A_x0s[1, 2];

            KalmanVars.Matrix_A[6 * SimpleData.iMx + 0] = (SINSstate.u_x[0] + SINSstate.Omega_x[0]) / SimpleData.A;
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 1] = (SINSstate.u_x[1] + SINSstate.Omega_x[1]) / SimpleData.A;
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 4] = SINSstate.u_x[1] + SINSstate.Omega_x[1];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 5] = -SINSstate.u_x[0] - SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 10] = SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 11] = SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 12] = SINSstate.A_x0s[2, 2];
        }

        public static void Make_StateErrors(double[] ErrorVector, SINS_State SINSstate)
        {
            SINSstate.DeltaLatitude = ErrorVector[1] / SINSstate.R_n;
            SINSstate.DeltaLongitude = ErrorVector[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

            //SINSstate.DeltaV_1 = -(-ErrorVector[3] - SINSstate.V_x[1] * ErrorVector[8] - SINSstate.V_x[2] * (ErrorVector[0] / SimpleData.A_S_axis - ErrorVector[7]));
            //SINSstate.DeltaV_2 = -(-ErrorVector[4] + SINSstate.V_x[1] * ErrorVector[8] - SINSstate.V_x[2] * (ErrorVector[1] / SimpleData.A_S_axis + ErrorVector[6]));

            SINSstate.DeltaV_1 = ErrorVector[2] ;
            SINSstate.DeltaV_2 = ErrorVector[3] ;

            SINSstate.DeltaRoll = -(ErrorVector[4] * Math.Sin(SINSstate.Heading) + ErrorVector[5] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
            SINSstate.DeltaPitch = -ErrorVector[4] * Math.Cos(SINSstate.Heading) + ErrorVector[5] * Math.Sin(SINSstate.Heading);
            SINSstate.DeltaHeading = ErrorVector[6] + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);
        }

        public static void Do_SINSstate_Correction(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate2)
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
            SINSstate2.A_sx0 = A_sx0(SINSstate);
            SINSstate2.A_x0s = SINSstate2.A_sx0.Transpose();
            SINSstate2.A_x0n = A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate2.A_nx0 = SINSstate2.A_x0n.Transpose();
        }

        public static void StateForecast(SINS_State SINSstate)
        {
            double[] temp1 = new double[3];
            Matrix tempM = new Matrix(3, 3);

            SINSstate.g_0 = NormalGravity_g0(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.u_x = U_x0(SINSstate.Latitude);
            SINSstate.g_x[0] = 0.0;
            SINSstate.g_x[1] = 0.0;
            SINSstate.g_x[2] = -GilmertGravityForce(SINSstate.Latitude, SINSstate.Altitude);
            //SINSstate.g_x[2] = -9.78049 * SimpleData.a_SemimajorAxis * SimpleData.a_SemimajorAxis * (1.0 + 0.005317099 * Math.Sin(SINSstate.Latitude) * Math.Sin(SINSstate.Latitude)) / (SimpleData.a_SemimajorAxis + SINSstate.Altitude) / (SimpleData.a_SemimajorAxis + SINSstate.Altitude);

            SINSstate.Omega_x[0] = -SINSstate.Vx_0[1] / SINSstate.R_n;
            SINSstate.Omega_x[1] = SINSstate.Vx_0[0] / SINSstate.R_e;
            SINSstate.Omega_x[2] = SINSstate.Vx_0[0] * Math.Tan(SINSstate.Latitude) / SINSstate.R_e;

            SINSstate.F_x = SINSstate.A_x0s * SINSstate.F_z;
            SINSstate.Altitude = SINSstate.Altitude + SINSstate.Vx_0[2] * SINSstate.timeStep;
            //SINSstate.Altitude = SINSstate.Altitude + (SINSstate.A_x0s* SINSstate.OdometerVector)[2] * SINSstate.timeStep;

            //---Интегрирование скорости---//
            temp1 = (Matrix.SkewSymmetricMatrix(SINSstate.Omega_x) + 2 * Matrix.SkewSymmetricMatrix(SINSstate.u_x)) * SINSstate.Vx_0;
            temp1 = temp1 + Matrix.ConvertToMatrix(SINSstate.F_x);
            temp1 = SINSstate.Vx_0 + Matrix.ConvertToMatrix(temp1 + Matrix.ConvertToMatrix(SINSstate.g_x)) * SINSstate.timeStep;
            SINSstate.Vx_0 = temp1;

            //---Интегрирвание Матрицы ориентации A_sx0 и A_x0n---//
            tempM = Matrix.SkewSymmetricMatrix(SINSstate.u_x) + Matrix.SkewSymmetricMatrix(SINSstate.Omega_x);
            tempM = SINSstate.A_sx0 + (Matrix.SkewSymmetricMatrix(SINSstate.W_z) * SINSstate.A_sx0 - SINSstate.A_sx0 * (tempM)) * SINSstate.timeStep;
            SINSstate.A_sx0 = tempM;
            SINSstate.A_x0s = tempM.Transpose();
            tempM = SINSstate.A_x0n + Matrix.SkewSymmetricMatrix(SINSstate.Omega_x) * SINSstate.A_x0n * SINSstate.timeStep;
            SINSstate.A_x0n = tempM;
            SINSstate.A_nx0 = tempM.Transpose();

            //---Пересчет всех углов для X^- ---//
            SINSstate.Roll = -Math.Atan(SINSstate.A_sx0[0, 2] / SINSstate.A_sx0[2, 2]);
            SINSstate.Pitch = Math.Atan(SINSstate.A_sx0[1, 2] / Math.Sqrt(SINSstate.A_sx0[0, 2] * SINSstate.A_sx0[0, 2] + SINSstate.A_sx0[2, 2] * SINSstate.A_sx0[2, 2]));
            SINSstate.Heading = Math.Atan(SINSstate.A_sx0[1, 0] / SINSstate.A_sx0[1, 1]);

            SINSstate.Latitude = Math.Atan(SINSstate.A_x0n[2, 2] / SINSstate.A_x0n[1, 2]);
            SINSstate.Longitude = Math.Atan(SINSstate.A_x0n[1, 1] / SINSstate.A_x0n[1, 0]);

            //---Прогноз для векторов: U_x^-, g_x^-, \Omega_x^- ---//
            SINSstate.u_x = U_x0(SINSstate.Latitude);
            SINSstate.g_x[0] = 0.0;
            SINSstate.g_x[1] = 0.0;
            SINSstate.g_x[2] = -GilmertGravityForce(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.Omega_x[0] = -SINSstate.Vx_0[1] / (SINSstate.R_n + SINSstate.Altitude);
            SINSstate.Omega_x[1] = SINSstate.Vx_0[0] / (SINSstate.R_e + SINSstate.Altitude);
            SINSstate.Omega_x[2] = SINSstate.Vx_0[0] * Math.Tan(SINSstate.Latitude) / (SINSstate.R_e + SINSstate.Altitude);
        }

        public static void InitOfCovarianceMatrixes(Kalman_Vars KalmanVars)
        {
            for (int i = 0; i < SimpleData.iMx * SimpleData.iMx; i++)
                KalmanVars.CovarianceMatrixS_m[i] = KalmanVars.CovarianceMatrixS_p[i] = 0.0;

            KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0] =  KalmanVars.CovarianceMatrixS_p[0 * SimpleData.iMx + 0] = 0.1;    // позиционные ошибки
            KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1] =  KalmanVars.CovarianceMatrixS_p[1 * SimpleData.iMx + 1] = 0.1;

            KalmanVars.CovarianceMatrixS_m[2 * SimpleData.iMx + 2] =  KalmanVars.CovarianceMatrixS_p[2 * SimpleData.iMx + 2] = 0.001;   // 0.01 м/с
            KalmanVars.CovarianceMatrixS_m[3 * SimpleData.iMx + 3] =  KalmanVars.CovarianceMatrixS_p[3 * SimpleData.iMx + 3] = 0.001;

            KalmanVars.CovarianceMatrixS_m[4 * SimpleData.iMx + 4] =  KalmanVars.CovarianceMatrixS_p[4 * SimpleData.iMx + 4] = 5.0 * SimpleData.ToRadian_min;  // 5 угл. минут
            KalmanVars.CovarianceMatrixS_m[5 * SimpleData.iMx + 5] =  KalmanVars.CovarianceMatrixS_p[5 * SimpleData.iMx + 5] = 5.0 * SimpleData.ToRadian_min;
            KalmanVars.CovarianceMatrixS_m[6 * SimpleData.iMx + 6] =  KalmanVars.CovarianceMatrixS_p[6 * SimpleData.iMx + 6] = 5.0 * SimpleData.ToRadian_min;

            KalmanVars.CovarianceMatrixS_m[10 * SimpleData.iMx + 10] = KalmanVars.CovarianceMatrixS_p[10 * SimpleData.iMx + 10] = 0.02 * SimpleData.ToRadian / 3600.0; // 0.02 град/час
            KalmanVars.CovarianceMatrixS_m[11 * SimpleData.iMx + 11] = KalmanVars.CovarianceMatrixS_p[11 * SimpleData.iMx + 11] = 0.02 * SimpleData.ToRadian / 3600.0;
            KalmanVars.CovarianceMatrixS_m[12 * SimpleData.iMx + 12] = KalmanVars.CovarianceMatrixS_p[12 * SimpleData.iMx + 12] = 0.02 * SimpleData.ToRadian / 3600.0;

            KalmanVars.CovarianceMatrixS_m[7 * SimpleData.iMx + 7] = KalmanVars.CovarianceMatrixS_p[7 * SimpleData.iMx + 7] = 0.001;    // м/с^2
            KalmanVars.CovarianceMatrixS_m[8 * SimpleData.iMx + 8] = KalmanVars.CovarianceMatrixS_p[8 * SimpleData.iMx + 8] = 0.001;
            KalmanVars.CovarianceMatrixS_m[9 * SimpleData.iMx + 9] = KalmanVars.CovarianceMatrixS_p[9 * SimpleData.iMx + 9] = 0.001;
        }
    }
}
