using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace Common_Namespace
{
    public class SINSprocessing : SimpleOperations
    {
        public static double dVh_global = 0.0;
        public static int Can = 0; 

        public static void Redifinition_OdoCounts(SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod)
        {
            if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                if (SINSstate.flag_UsingCorrection == true || SINSstate.flag_UseOnlyStops == true)
                // || SINSstate.flag_slipping == true - в большей степени SlipageProcessing это обработка выбросов одометра, а не проскальзывания. Поэтому не обнуляем OdoTimeStepCount 
                {
                    SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                    SINSstate.OdoTimeStepCount = 0;

                    SINSstate.odotime_prev = SINSstate.Time;

                    SINSstate.Latitude_prev = SINSstate.Latitude; SINSstate2.Latitude_prev = SINSstate2.Latitude;
                    SINSstate.Longitude_prev = SINSstate.Longitude; SINSstate2.Longitude_prev = SINSstate2.Longitude;
                    SINSstate.Altitude_prev = SINSstate.Altitude; SINSstate2.Altitude_prev = SINSstate2.Altitude;
                }
            }
        }






        public static void ApplyCompensatedErrorsToSolution(SINS_State SINSstate)
        {
            if (SINSstate.Heading > Math.PI) SINSstate.Heading = SINSstate.Heading - 2.0 * Math.PI;
            if (SINSstate.Heading < -Math.PI) SINSstate.Heading = SINSstate.Heading + 2.0 * Math.PI;

            //корректированная матрица ориентации
            SINSstate.A_sx0 = A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.u_x = U_x0(SINSstate.Latitude);

            SINSstate.Omega_x[0] = -(SINSstate.Vx_0[1] + SINSstate.Vx_0_prev[1]) / 2.0 / SINSstate.R_n;
            SINSstate.Omega_x[1] = (SINSstate.Vx_0[0] + SINSstate.Vx_0_prev[0]) / 2.0 / SINSstate.R_e;
            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * SINSstate.Omega_x[1];

            SINSstate.g = 9.78049 * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014;
            SINSstate.g -= 2 * 0.000001538 * SINSstate.Altitude;
        }



        public static void CalcStateErrors(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            SINSstate.DeltaLatitude = ErrorVector[1] / SINSstate.R_n;
            SINSstate.DeltaLongitude = ErrorVector[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

            SINSstate.DeltaV_1 = ErrorVector[2] + SINSstate.Vx_0[1] * ErrorVector[6] + SINSstate.Vx_0[1] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
            SINSstate.DeltaV_2 = ErrorVector[3] - SINSstate.Vx_0[0] * ErrorVector[6] - SINSstate.Vx_0[0] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
            //--- В случае обратных связей не должно быть списывания углов бетта
            if (!SINSstate.flag_FeedbackExist)
            {
                SINSstate.DeltaV_1 += SINSstate.Vx_0[1] * ErrorVector[6] + SINSstate.Vx_0[1] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
                SINSstate.DeltaV_2 += - SINSstate.Vx_0[0] * ErrorVector[6] - SINSstate.Vx_0[0] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
            }

            if (SINSstate.flag_iMx_r3_dV3)
            {
                SINSstate.DeltaAltitude = ErrorVector[SINSstate.iMx_r3_dV3];
                SINSstate.DeltaV_3 = ErrorVector[SINSstate.iMx_r3_dV3 + 1];

                //--- В случае обратных связей не должно быть списывания углов бетта
                if (!SINSstate.flag_FeedbackExist)
                {
                    SINSstate.DeltaV_1 += SINSstate.Vx_0[2] * (ErrorVector[0] / SINSstate.R_e - ErrorVector[5]);
                    SINSstate.DeltaV_2 += SINSstate.Vx_0[2] * (ErrorVector[1] / SINSstate.R_n - ErrorVector[4]);
                    SINSstate.DeltaV_3 += SINSstate.Vx_0[0] * (ErrorVector[5] - ErrorVector[0] / SINSstate.R_e) - SINSstate.Vx_0[1] * (ErrorVector[4] + ErrorVector[1] / SINSstate.R_n);
                }
            }

            SINSstate.DeltaRoll = -(ErrorVector[4] * Math.Sin(SINSstate.Heading) + ErrorVector[5] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
            SINSstate.DeltaPitch = -ErrorVector[4] * Math.Cos(SINSstate.Heading) + ErrorVector[5] * Math.Sin(SINSstate.Heading);
            SINSstate.DeltaHeading = ErrorVector[6] + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);
            //--- В случае обратных связей не должно быть повторного списывания меридиальной составляющей
            if (!SINSstate.flag_FeedbackExist)
                SINSstate.DeltaHeading = SINSstate.DeltaHeading + SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);


            //--- Случай Одометр+БИНС. Обратная связть ---//
            if (SINSstate.flag_Odometr_SINS_case == true)
            {
                SINSstate_OdoMod.DeltaLatitude = ErrorVector[SINSstate.iMx_r12_odo + 1] / SINSstate_OdoMod.R_n;
                SINSstate_OdoMod.DeltaLongitude = ErrorVector[SINSstate.iMx_r12_odo + 0] / SINSstate_OdoMod.R_e / Math.Cos(SINSstate_OdoMod.Latitude);

                if (SINSstate.flag_Using_iMx_r_odo_3)
                    SINSstate_OdoMod.DeltaAltitude = ErrorVector[SINSstate.iMx_r12_odo + 2];
            }
        }



        public static void StateCorrection(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod)
        {
            SINSstate2.Latitude = SINSstate.Latitude - SINSstate.DeltaLatitude;
            SINSstate2.Longitude = SINSstate.Longitude - SINSstate.DeltaLongitude;


            SINSstate2.Vx_0[0] = SINSstate.Vx_0[0] - SINSstate.DeltaV_1;
            SINSstate2.Vx_0[1] = SINSstate.Vx_0[1] - SINSstate.DeltaV_2;

            if (SINSstate.flag_iMx_r3_dV3)
            {
                SINSstate2.Vx_0[2] = SINSstate.Vx_0[2] - SINSstate.DeltaV_3;
                SINSstate2.Altitude = SINSstate.Altitude - SINSstate.DeltaAltitude;
            }

            SINSstate2.Roll = SINSstate.Roll - SINSstate.DeltaRoll;
            SINSstate2.Pitch = SINSstate.Pitch - SINSstate.DeltaPitch;
            SINSstate2.Heading = SINSstate.Heading - SINSstate.DeltaHeading;

            SINSprocessing.ApplyCompensatedErrorsToSolution(SINSstate2);



            if (SINSstate.flag_FeedbackExist)
            {
                //--- Ведем расчет оценки ошибок модели одометра в случае обратных связей ---//
                if (SINSstate.flag_iMx_kappa_13_ds)
                {
                    SINSstate2.Cumulative_KappaEst[0] += ErrorVector[SINSstate.iMx_odo_model + 0];
                    SINSstate2.Cumulative_KappaEst[2] += ErrorVector[SINSstate.iMx_odo_model + 1];
                    SINSstate2.Cumulative_KappaEst[1] += ErrorVector[SINSstate.iMx_odo_model + 2];
                }

                //--- Кумулируем ошибки вектора ошибок "x" для вывода в файлы ---//
                for (int i = 0; i < SimpleData.iMx; i++)
                    SINSstate2.Cumulative_KalmanErrorVector[i] += ErrorVector[i];

                //--- Кумулируем ошибки в большом для вывода в файлы ---//
                SINSstate2.Cumulative_StateErrorVector[0] += SINSstate.DeltaLatitude;
                SINSstate2.Cumulative_StateErrorVector[1] += SINSstate.DeltaLongitude;
                SINSstate2.Cumulative_StateErrorVector[2] += SINSstate.DeltaAltitude;
                SINSstate2.Cumulative_StateErrorVector[3] += SINSstate.DeltaV_1;
                SINSstate2.Cumulative_StateErrorVector[4] += SINSstate.DeltaV_2;
                SINSstate2.Cumulative_StateErrorVector[5] += SINSstate.DeltaV_3;
                SINSstate2.Cumulative_StateErrorVector[6] += SINSstate.DeltaHeading;
                SINSstate2.Cumulative_StateErrorVector[7] += SINSstate.DeltaRoll;
                SINSstate2.Cumulative_StateErrorVector[8] += SINSstate.DeltaPitch;
            }

                

            //---Случай Одометр+БИНС. Обратная связть.---
            if (SINSstate.flag_Odometr_SINS_case == true)
            {
                SINSstate_OdoMod.Latitude_Corr = SINSstate_OdoMod.Latitude - SINSstate_OdoMod.DeltaLatitude;
                SINSstate_OdoMod.Longitude_Corr = SINSstate_OdoMod.Longitude - SINSstate_OdoMod.DeltaLongitude;
                if (SINSstate.flag_Using_iMx_r_odo_3)
                    SINSstate_OdoMod.Altitude_Corr = SINSstate_OdoMod.Altitude - SINSstate_OdoMod.DeltaAltitude;


                if (SINSstate.flag_FeedbackExist == true)
                {
                    SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude - SINSstate_OdoMod.DeltaLatitude;
                    SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude - SINSstate_OdoMod.DeltaLongitude;

                    if (SINSstate.flag_Using_iMx_r_odo_3)
                        SINSstate_OdoMod.Altitude = SINSstate_OdoMod.Altitude - SINSstate_OdoMod.DeltaAltitude;

                    SINSstate_OdoMod.A_x0n = A_x0n(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Longitude);
                    SINSstate_OdoMod.A_nx0 = SINSstate_OdoMod.A_x0n.Transpose();
                }
            }
            else if (SINSstate.flag_FeedbackExist == true)
            {
                SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude - SINSstate_OdoMod.DeltaLatitude;
                SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude - SINSstate_OdoMod.DeltaLongitude;
            }
        }



        public static void NullingOfCorrectedErrors(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {

            for (int i = 0; i < SimpleData.iMx; i++)
            {
                KalmanVars.ErrorConditionVector_p[i] = 0.0;
                KalmanVars.ErrorConditionVector_m[i] = 0.0;
            }
        }







        public static void InitOfCovarianceMatrixes(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            double[] pdP = new double[SimpleData.iMx * SimpleData.iMx];

            for (int i = 0; i < SimpleData.iMx * SimpleData.iMx; i++)
                KalmanVars.CovarianceMatrixS_m[i] = KalmanVars.CovarianceMatrixS_p[i] = 0.0;


            KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0] = KalmanVars.CovarianceMatrixS_p[0 * SimpleData.iMx + 0] = SINSstate.stdR;    // позиционные ошибки
            KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1] = KalmanVars.CovarianceMatrixS_p[1 * SimpleData.iMx + 1] = SINSstate.stdR;

            KalmanVars.CovarianceMatrixS_m[2 * SimpleData.iMx + 2] = KalmanVars.CovarianceMatrixS_p[2 * SimpleData.iMx + 2] = SINSstate.stdV;   // 0.01 м/с
            KalmanVars.CovarianceMatrixS_m[3 * SimpleData.iMx + 3] = KalmanVars.CovarianceMatrixS_p[3 * SimpleData.iMx + 3] = SINSstate.stdV;

            KalmanVars.CovarianceMatrixS_m[4 * SimpleData.iMx + 4] = KalmanVars.CovarianceMatrixS_p[4 * SimpleData.iMx + 4] = Math.Max(SINSstate.stdAlpha1, 1E-3);  // 5 угл. минут
            KalmanVars.CovarianceMatrixS_m[5 * SimpleData.iMx + 5] = KalmanVars.CovarianceMatrixS_p[5 * SimpleData.iMx + 5] = Math.Max(SINSstate.stdAlpha2, 1E-3);
            KalmanVars.CovarianceMatrixS_m[6 * SimpleData.iMx + 6] = KalmanVars.CovarianceMatrixS_p[6 * SimpleData.iMx + 6] = Math.Max(SINSstate.stdBeta3, 1E-3);

            KalmanVars.CovarianceMatrixS_m[7 * SimpleData.iMx + 7] = KalmanVars.CovarianceMatrixS_p[7 * SimpleData.iMx + 7] = Math.Max(SINSstate.stdNu * SimpleData.ToRadian / 3600.0, 1E-6); //0.2 * SimpleData.ToRadian / 3600.0; // 0.2 град/час
            KalmanVars.CovarianceMatrixS_m[8 * SimpleData.iMx + 8] = KalmanVars.CovarianceMatrixS_p[8 * SimpleData.iMx + 8] = Math.Max(SINSstate.stdNu * SimpleData.ToRadian / 3600.0, 1E-6);//0.2 * SimpleData.ToRadian / 3600.0;
            KalmanVars.CovarianceMatrixS_m[9 * SimpleData.iMx + 9] = KalmanVars.CovarianceMatrixS_p[9 * SimpleData.iMx + 9] = Math.Max(SINSstate.stdNu * SimpleData.ToRadian / 3600.0, 1E-6);//0.2 * SimpleData.ToRadian / 3600.0;

            KalmanVars.CovarianceMatrixS_m[10 * SimpleData.iMx + 10] = KalmanVars.CovarianceMatrixS_p[10 * SimpleData.iMx + 10] = Math.Max(SINSstate.stdF[0], 1E-4);    // м/с^2
            KalmanVars.CovarianceMatrixS_m[11 * SimpleData.iMx + 11] = KalmanVars.CovarianceMatrixS_p[11 * SimpleData.iMx + 11] = Math.Max(SINSstate.stdF[1], 1E-4);
            KalmanVars.CovarianceMatrixS_m[12 * SimpleData.iMx + 12] = KalmanVars.CovarianceMatrixS_p[12 * SimpleData.iMx + 12] = Math.Max(SINSstate.stdF.Max(), 1E-4);

            if (SINSstate.flag_iMx_r3_dV3 == true)
            {
                KalmanVars.CovarianceMatrixS_m[SINSstate.iMx_r3_dV3 * SimpleData.iMx + SINSstate.iMx_r3_dV3] = KalmanVars.CovarianceMatrixS_p[SINSstate.iMx_r3_dV3 * SimpleData.iMx + SINSstate.iMx_r3_dV3] = SINSstate.stdR;
                KalmanVars.CovarianceMatrixS_m[(SINSstate.iMx_r3_dV3 + 1) * SimpleData.iMx + (SINSstate.iMx_r3_dV3 + 1)] = KalmanVars.CovarianceMatrixS_p[(SINSstate.iMx_r3_dV3 + 1) * SimpleData.iMx + (SINSstate.iMx_r3_dV3 + 1)] = SINSstate.stdV;
            }

            if (SINSstate.flag_iMx_kappa_13_ds == true)
            {
                KalmanVars.CovarianceMatrixS_m[SINSstate.iMx_odo_model * SimpleData.iMx + SINSstate.iMx_odo_model]
                    = KalmanVars.CovarianceMatrixS_p[SINSstate.iMx_odo_model * SimpleData.iMx + SINSstate.iMx_odo_model] = SINSstate.stdKappa1 * SimpleData.ToRadian_min;
                KalmanVars.CovarianceMatrixS_m[(SINSstate.iMx_odo_model + 1) * SimpleData.iMx + (SINSstate.iMx_odo_model + 1)]
                    = KalmanVars.CovarianceMatrixS_p[(SINSstate.iMx_odo_model + 1) * SimpleData.iMx + (SINSstate.iMx_odo_model + 1)] = SINSstate.stdKappa3 * SimpleData.ToRadian_min;
                KalmanVars.CovarianceMatrixS_m[(SINSstate.iMx_odo_model + 2) * SimpleData.iMx + (SINSstate.iMx_odo_model + 2)]
                    = KalmanVars.CovarianceMatrixS_p[(SINSstate.iMx_odo_model + 2) * SimpleData.iMx + (SINSstate.iMx_odo_model + 2)] = SINSstate.stdScale;
            }
        }






        public static int MatrixNoise_ReDef(SINS_State SINSstate, Kalman_Vars KalmanVars, bool AlignmentFLG)
        {
            int iMx = SimpleData.iMx, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model, iMx_r12_odo = SINSstate.iMx_r12_odo;

            int tmpCounter = 0;
            double sqrt_freq = Math.Sqrt(Math.Abs(SINSstate.Freq));
            double[] Noise_Vel_in_Mx = new double[3], Noise_Angl_in_Mx = new double[3];

            for (int i = 0; i < iMx * iMq; i++)
                KalmanVars.CovarianceMatrixNoise[i] = 0.0;

            for (int j = 0; j < 3; j++)
            {
                // выбираем, где для каких осей инструментальных шумов формируется матрица шумов //
                Noise_Vel_in_Mx[j] = Math.Sqrt(Math.Pow(SINSstate.A_x0s[j, 0] * KalmanVars.Noise_Vel[0], 2) +
                                               Math.Pow(SINSstate.A_x0s[j, 1] * KalmanVars.Noise_Vel[1], 2) +
                                               Math.Pow(SINSstate.A_x0s[j, 2] * KalmanVars.Noise_Vel[2], 2));
                Noise_Angl_in_Mx[j] = Math.Sqrt(Math.Pow(SINSstate.A_x0s[j, 0] * KalmanVars.Noise_Angl[0], 2) +
                                               Math.Pow(SINSstate.A_x0s[j, 1] * KalmanVars.Noise_Angl[1], 2) +
                                               Math.Pow(SINSstate.A_x0s[j, 2] * KalmanVars.Noise_Angl[2], 2));

            }

            if (SINSstate.flag_iMqDeltaR)
            {
                KalmanVars.CovarianceMatrixNoise[0 * iMq + tmpCounter + 0] = KalmanVars.Noise_Pos * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[1 * iMq + tmpCounter + 1] = KalmanVars.Noise_Pos * sqrt_freq;
                tmpCounter = tmpCounter + 2;
            }

            // так как в векторе состояния дрейфы в проекции на приборные оси, надо задавать соответственно матрицу шумов //
            KalmanVars.CovarianceMatrixNoise[2 * iMq + tmpCounter + 0] = Noise_Vel_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[2 * iMq + tmpCounter + 2] = SINSstate.Vx_0[1] * Noise_Angl_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[3 * iMq + tmpCounter + 1] = Noise_Vel_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[3 * iMq + tmpCounter + 3] = SINSstate.Vx_0[0] * Noise_Angl_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[4 * iMq + tmpCounter + 2] = Noise_Angl_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[5 * iMq + tmpCounter + 3] = Noise_Angl_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[6 * iMq + tmpCounter + 4] = Noise_Angl_in_Mx[2] * sqrt_freq;
            tmpCounter = tmpCounter + 5;

            if (SINSstate.flag_iMqDeltaNu)
            {
                KalmanVars.CovarianceMatrixNoise[7 * iMq + tmpCounter + 0] = KalmanVars.Noise_Drift * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[8 * iMq + tmpCounter + 1] = KalmanVars.Noise_Drift * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[9 * iMq + tmpCounter + 2] = KalmanVars.Noise_Drift * sqrt_freq;
                tmpCounter = tmpCounter + 3;
            }
            if (SINSstate.flag_iMqDeltaF)
            {
                KalmanVars.CovarianceMatrixNoise[10 * iMq + tmpCounter + 0] = KalmanVars.Noise_Accel * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[11 * iMq + tmpCounter + 1] = KalmanVars.Noise_Accel * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[12 * iMq + tmpCounter + 2] = KalmanVars.Noise_Accel * sqrt_freq;
                tmpCounter = tmpCounter + 3;
            }

            if (SINSstate.flag_iMx_r3_dV3)
            {
                if (SINSstate.flag_iMqDeltaR)
                {
                    KalmanVars.CovarianceMatrixNoise[(iMx_r3_dV3 + 0) * iMq + tmpCounter + 0] = KalmanVars.Noise_Pos * sqrt_freq;
                    tmpCounter = tmpCounter + 1;
                }
                KalmanVars.CovarianceMatrixNoise[(iMx_r3_dV3 + 1) * iMq + tmpCounter + 0] = Noise_Vel_in_Mx[2] * sqrt_freq;
                tmpCounter = tmpCounter + 1;
            }

            if (SINSstate.flag_iMx_kappa_13_ds)
            {
                if (SINSstate.flag_iMqVarkappa13)
                {
                    KalmanVars.CovarianceMatrixNoise[(iMx_odo_model + 0) * iMq + tmpCounter + 0] = KalmanVars.Noise_OdoKappa * sqrt_freq;
                    KalmanVars.CovarianceMatrixNoise[(iMx_odo_model + 1) * iMq + tmpCounter + 1] = KalmanVars.Noise_OdoKappa * sqrt_freq;
                    tmpCounter = tmpCounter + 2;
                }
                if (SINSstate.flag_iMqKappa)
                {
                    KalmanVars.CovarianceMatrixNoise[(iMx_odo_model + 2) * iMq + tmpCounter + 0] = KalmanVars.Noise_OdoScale * sqrt_freq;
                    tmpCounter = tmpCounter + 1;
                }
            }

            return tmpCounter;
            //PrintMatrixToFile(KalmanVars.CovarianceMatrixNoise, iMx, iMq);
        }




        public static void Make_A_bridge(SINS_State SINSstate, SINS_State SINSstate2, Kalman_Vars KalmanVars, SINS_State SINSstate_OdoMod)
        {
            if (SINSstate.flag_Odometr_SINS_case == true)
            {
                if (SINSstate.flag_FeedbackExist == true)
                    Odometr_SINS.Make_A(SINSstate, KalmanVars, SINSstate_OdoMod);
                else if (SINSstate.flag_FeedbackExist == false)
                    Odometr_SINS.Make_A(SINSstate2, KalmanVars, SINSstate_OdoMod);

                Odometr_SINS.MatrixNoise_ReDef(SINSstate, KalmanVars, SINSstate.flag_Alignment);
            }
            else
            {
                if (SINSstate.flag_FeedbackExist == true)
                    SINSprocessing.Make_A(SINSstate, KalmanVars, SINSstate_OdoMod);
                else if (SINSstate.flag_FeedbackExist == false)
                    SINSprocessing.Make_A(SINSstate2, KalmanVars, SINSstate_OdoMod);

                SINSprocessing.MatrixNoise_ReDef(SINSstate, KalmanVars, SINSstate.flag_Alignment);
            }
        }


        public static void Make_A(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;

            for (int i = 0; i < iMx * iMx; i++)
                KalmanVars.Matrix_A[i] = 0;

            SINSstate.W_x[0] = SINSstate.Omega_x[0];
            SINSstate.W_x[1] = SINSstate.Omega_x[1] + SimpleData.U * Math.Cos(SINSstate.Latitude);
            SINSstate.W_x[2] = SINSstate.Omega_x[2] + SimpleData.U * Math.Sin(SINSstate.Latitude);

            KalmanVars.Matrix_A[0 * iMx + 1] = SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[0 * iMx + 2] = 1.0;
            KalmanVars.Matrix_A[0 * iMx + 6] = SINSstate.Vx_0[1];

            KalmanVars.Matrix_A[1 * iMx + 0] = -SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[1 * iMx + 3] = 1.0;
            KalmanVars.Matrix_A[1 * iMx + 6] = -SINSstate.Vx_0[0];

            KalmanVars.Matrix_A[2 * iMx + 0] = SINSstate.u_x[0] * SINSstate.Vx_0[1] / SINSstate.R_e;
            KalmanVars.Matrix_A[2 * iMx + 1] = SINSstate.u_x[1] * SINSstate.Vx_0[1] / SINSstate.R_n;
            KalmanVars.Matrix_A[2 * iMx + 3] = SINSstate.Omega_x[2] + 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[2 * iMx + 4] = SINSstate.u_x[1] * SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[2 * iMx + 5] = -SINSstate.g - SINSstate.u_x[0] * SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[2 * iMx + 7] = -SINSstate.Vx_0[1] * SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[2 * iMx + 8] = -SINSstate.Vx_0[1] * SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[2 * iMx + 9] = -SINSstate.Vx_0[1] * SINSstate.A_x0s[2, 2];
            KalmanVars.Matrix_A[2 * iMx + 10] = SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[2 * iMx + 11] = SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[2 * iMx + 12] = SINSstate.A_x0s[0, 2];


            KalmanVars.Matrix_A[3 * iMx + 0] = -SINSstate.u_x[0] * SINSstate.Vx_0[0] / SINSstate.R_e;
            KalmanVars.Matrix_A[3 * iMx + 1] = -SINSstate.u_x[1] * SINSstate.Vx_0[0] / SINSstate.R_n;
            KalmanVars.Matrix_A[3 * iMx + 2] = -SINSstate.Omega_x[2] - 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[3 * iMx + 4] = -SINSstate.u_x[1] * SINSstate.Vx_0[0] + SINSstate.g;
            KalmanVars.Matrix_A[3 * iMx + 5] = SINSstate.u_x[0] * SINSstate.Vx_0[0];
            KalmanVars.Matrix_A[3 * iMx + 7] = SINSstate.Vx_0[0] * SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[3 * iMx + 8] = SINSstate.Vx_0[0] * SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[3 * iMx + 9] = SINSstate.Vx_0[0] * SINSstate.A_x0s[2, 2];
            KalmanVars.Matrix_A[3 * iMx + 10] = SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[3 * iMx + 11] = SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[3 * iMx + 12] = SINSstate.A_x0s[1, 2];


            KalmanVars.Matrix_A[4 * iMx + 0] = -SINSstate.u_x[2] / SINSstate.R_e;
            KalmanVars.Matrix_A[4 * iMx + 3] = -1.0 / SINSstate.R_n;
            KalmanVars.Matrix_A[4 * iMx + 5] = SINSstate.u_x[2];
            KalmanVars.Matrix_A[4 * iMx + 6] = -SINSstate.u_x[1];
            KalmanVars.Matrix_A[4 * iMx + 7] = -SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[4 * iMx + 8] = -SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[4 * iMx + 9] = -SINSstate.A_x0s[0, 2];

            KalmanVars.Matrix_A[5 * iMx + 1] = -SINSstate.u_x[2] / SINSstate.R_n;
            KalmanVars.Matrix_A[5 * iMx + 2] = 1.0 / SINSstate.R_e;
            KalmanVars.Matrix_A[5 * iMx + 4] = -SINSstate.u_x[2];
            KalmanVars.Matrix_A[5 * iMx + 6] = SINSstate.u_x[0];
            KalmanVars.Matrix_A[5 * iMx + 7] = -SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[5 * iMx + 8] = -SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[5 * iMx + 9] = -SINSstate.A_x0s[1, 2];

            KalmanVars.Matrix_A[6 * iMx + 0] = (SINSstate.Omega_x[0] + SINSstate.u_x[0]) / SINSstate.R_e;
            KalmanVars.Matrix_A[6 * iMx + 1] = (SINSstate.Omega_x[1] + SINSstate.u_x[1]) / SINSstate.R_n;
            KalmanVars.Matrix_A[6 * iMx + 4] = SINSstate.Omega_x[1] + SINSstate.u_x[1];
            KalmanVars.Matrix_A[6 * iMx + 5] = -SINSstate.Omega_x[0] - SINSstate.u_x[0];
            KalmanVars.Matrix_A[6 * iMx + 7] = -SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[6 * iMx + 8] = -SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[6 * iMx + 9] = -SINSstate.A_x0s[2, 2];

            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_A[0 * iMx + 0] += SINSstate.Vx_0[2] / SINSstate.R_e;
                KalmanVars.Matrix_A[0 * iMx + 5] += -SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[0 * iMx + iMx_r3_dV3] = -SINSstate.Omega_x[1];

                KalmanVars.Matrix_A[1 * iMx + 1] += SINSstate.Vx_0[2] / SINSstate.R_n;
                KalmanVars.Matrix_A[1 * iMx + 4] += SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[1 * iMx + iMx_r3_dV3] = SINSstate.Omega_x[0];

                KalmanVars.Matrix_A[iMx_r3_dV3 * iMx + 0] = SINSstate.Omega_x[1] - SINSstate.Vx_0[0] / SINSstate.R_e;
                KalmanVars.Matrix_A[iMx_r3_dV3 * iMx + 1] = -SINSstate.Omega_x[0] - SINSstate.Vx_0[1] / SINSstate.R_n;
                KalmanVars.Matrix_A[iMx_r3_dV3 * iMx + 4] = -SINSstate.Vx_0[1];
                KalmanVars.Matrix_A[iMx_r3_dV3 * iMx + 5] = SINSstate.Vx_0[0];
                KalmanVars.Matrix_A[iMx_r3_dV3 * iMx + iMx_r3_dV3 + 1] = 1.0;


                KalmanVars.Matrix_A[2 * iMx + 1] += SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_n;
                KalmanVars.Matrix_A[2 * iMx + 4] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[2 * iMx + 6] += -SINSstate.u_x[0] * SINSstate.Vx_0[2];

                KalmanVars.Matrix_A[2 * iMx + 7] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 0];
                KalmanVars.Matrix_A[2 * iMx + 8] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 1];
                KalmanVars.Matrix_A[2 * iMx + 9] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 2];

                KalmanVars.Matrix_A[2 * iMx + iMx_r3_dV3 + 1] = -SINSstate.Omega_x[1] - 2 * SINSstate.u_x[1];



                KalmanVars.Matrix_A[3 * iMx + 0] += -SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_e;
                KalmanVars.Matrix_A[3 * iMx + 5] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[3 * iMx + 6] += -SINSstate.u_x[1] * SINSstate.Vx_0[2];

                KalmanVars.Matrix_A[3 * iMx + 7] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 0];
                KalmanVars.Matrix_A[3 * iMx + 8] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 1];
                KalmanVars.Matrix_A[3 * iMx + 9] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 2];

                KalmanVars.Matrix_A[3 * iMx + iMx_r3_dV3 + 1] = SINSstate.Omega_x[0] + 2 * SINSstate.u_x[0];



                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 0] = SINSstate.u_x[2] * SINSstate.Vx_0[1] / SINSstate.R_e;
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 1] = -SINSstate.u_x[2] * SINSstate.Vx_0[0] / SINSstate.R_n;
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 2] = SINSstate.Omega_x[1] + 2 * SINSstate.u_x[1];
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 3] = -SINSstate.Omega_x[0] - 2 * SINSstate.u_x[0];
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 4] = -SINSstate.u_x[2] * SINSstate.Vx_0[0];
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 5] = -SINSstate.u_x[2] * SINSstate.Vx_0[1];
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 6] = SINSstate.u_x[0] * SINSstate.Vx_0[0] + SINSstate.u_x[1] * SINSstate.Vx_0[1];

                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 7] = -SINSstate.Vx_0[0] * SINSstate.A_x0s[1, 0] + SINSstate.Vx_0[1] * SINSstate.A_x0s[0, 0];
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 8] = -SINSstate.Vx_0[0] * SINSstate.A_x0s[1, 1] + SINSstate.Vx_0[1] * SINSstate.A_x0s[0, 1];
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 9] = -SINSstate.Vx_0[0] * SINSstate.A_x0s[1, 2] + SINSstate.Vx_0[1] * SINSstate.A_x0s[0, 2];

                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 10] = SINSstate.A_x0s[2, 0];
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 11] = SINSstate.A_x0s[2, 1];
                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + 12] = SINSstate.A_x0s[2, 2];

                KalmanVars.Matrix_A[(iMx_r3_dV3 + 1) * iMx + iMx_r3_dV3] = 2 * 0.000001538;
            }

        }







        public static void StateIntegration_AT(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate2, SINS_State SINSstate_OdoMod)
        {
            double[] fz = new double[3], Wz = new double[3], u = new double[3], tempV = new double[3], Wz_avg = new double[3];
            double[] Vx_0 = new double[3], Vx_0_prev = new double[3];

            Matrix AT_z_xi = new Matrix(3, 3); Matrix B_x_eta = new Matrix(3, 3);
            Matrix dAT = new Matrix(3, 3); Matrix D_x_z = new Matrix(3, 3);
            Matrix W_x_xi = new Matrix(3, 3); Matrix C_eta_xi = new Matrix(3, 3);

            Matrix Hat1 = new Matrix(3, 3);
            Matrix Hat2 = new Matrix(3, 3);
            Matrix E = Matrix.UnitMatrix(3);
            Matrix dMatrix = new Matrix(3, 3);

            double W_z_abs, Omega_x_abs, dlt, dlt2, Altitude, Altitude_prev, dh, dVx, dVy, dVh, Azimth;

            CopyMatrix(AT_z_xi, SINSstate.AT);
            CopyMatrix(B_x_eta, SINSstate.A_x0n);

            SINSstate2.Time = SINSstate.Time;
            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            Altitude = SINSstate.Altitude;
            Altitude_prev = SINSstate.Altitude_prev;

            fz[1] = SINSstate.F_z[1];
            fz[2] = SINSstate.F_z[2];
            fz[0] = SINSstate.F_z[0];
            Wz[1] = SINSstate.W_z[1];
            Wz[2] = SINSstate.W_z[2];
            Wz[0] = SINSstate.W_z[0];

            if (SINSstate.flag_FeedbackExist)
            {
                for (int i = 0; i < 3; i++)
                {
                    fz[i] -= SINSstate.Cumulative_KalmanErrorVector[10 + i];
                    Wz[i] -= SINSstate.Cumulative_KalmanErrorVector[7 + i];
                }
            }


            CopyArray(SINSstate.F_z, fz);
            CopyArray(SINSstate.W_z, Wz);
            CopyArray(Vx_0, SINSstate.Vx_0);
            CopyArray(Vx_0_prev, SINSstate.Vx_0_prev);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);

            SINSstate.u_x = U_x0(SINSstate.Latitude);

            u[0] = 0.0;
            u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude);
            u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);

            if (SINSstate.flag_UseAlgebraDrift)
                for (int i = 0; i < 3; i++)
                    Wz[i] = Wz[i] - SINSstate.AlignAlgebraDrifts[i];



            //-------------ИНТЕГРИРОВАНИЕ МАТРИЦЫ AT_Z_XI И ПЕРВОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z---------
            if (SINSstate.flag_UsingAvegering == true)
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

            CopyMatrix(dMatrix, (E + Hat1 * dlt + Hat2 * dlt2));
            CopyMatrix(AT_z_xi, (dMatrix * AT_z_xi));

            //Нормировка
            for (int i = 0; i < 3; i++)
            {
                tempV[i] = Math.Sqrt(AT_z_xi[i, 0] * AT_z_xi[i, 0] + AT_z_xi[i, 1] * AT_z_xi[i, 1] + AT_z_xi[i, 2] * AT_z_xi[i, 2]);
                for (int j = 0; j < 3; j++)
                    AT_z_xi[i, j] = AT_z_xi[i, j] / tempV[i];
            }

            CopyMatrix(SINSstate.AT, AT_z_xi);

            CopyMatrix(W_x_xi, B_x_eta * SINSstate.A_nxi);
            CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());
            //--------------------------------------------------------------------------------------



            //---------------------------------ИНТЕГРИРОВАНИЕ СКОРОСТЕЙ----------------------------
            CopyArray(SINSstate.F_x, D_x_z * fz);
            SINSstate.g = SimpleData.Gravity_Normal * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014 - 2 * 0.000001538 * Altitude;

            dVx = SINSstate.F_x[0] + Vx_0[1] * (2.0 * u[2] + SINSstate.Omega_x[2]) - Vx_0[2] * (2.0 * u[1] + SINSstate.Omega_x[1]);
            dVy = SINSstate.F_x[1] - Vx_0[0] * (2.0 * u[2] + SINSstate.Omega_x[2]) + Vx_0[2] * (2.0 * u[0] + SINSstate.Omega_x[0]);

            Vx_0[0] += dVx * SINSstate.timeStep;
            Vx_0[1] += dVy * SINSstate.timeStep;

            //--------------------------------------------------------------------------------------




            //--- Интегрируем вертикальную скорость ---//
            if (SINSstate.flag_iMx_r3_dV3 && (SINSstate.flag_UsingAltitudeCorrection || SINSstate.flag_Using_SNS))
            {
                dVh = SINSstate.F_x[2] - SINSstate.g + (Vx_0[0] + Vx_0_prev[0]) / 2.0 * (2 * u[1] + SINSstate.Omega_x[1]) - (Vx_0[1] + Vx_0_prev[1]) / 2.0 * (2 * u[0] + SINSstate.Omega_x[0]);
                Vx_0[2] += dVh * SINSstate.timeStep;

                dh = (Vx_0[2] + Vx_0_prev[2]) / 2.0;
                Altitude += dh * SINSstate.timeStep;
            }
            //--- Если выставлен соответствующий флаг - переопределяем Vup вертикальной скоростью, вычисленной по одомеру ---//
            if (SINSstate.flag_iMx_r3_dV3 && SINSstate.flag_VupOdo_till_VupSINS)
            {
                Vx_0[2] = SINSstate_OdoMod.Vx_0[2];
            }


            //---------ИНТЕГРИРОВАНИЕ МАТРИЦЫ B_X_ETA И ВТОРОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z--------------

            SINSstate.Omega_x[0] = -(Vx_0[1] + Vx_0_prev[1]) / 2.0 / SINSstate.R_n;
            SINSstate.Omega_x[1] = (Vx_0[0] + Vx_0_prev[0]) / 2.0 / SINSstate.R_e;
            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * SINSstate.Omega_x[1];

            Omega_x_abs = Math.Sqrt(SINSstate.Omega_x[0] * SINSstate.Omega_x[0] + SINSstate.Omega_x[1] * SINSstate.Omega_x[1] + SINSstate.Omega_x[2] * SINSstate.Omega_x[2]);
            if (Omega_x_abs != 0)
            {
                dlt = Math.Sin(Omega_x_abs * SINSstate.timeStep) / Omega_x_abs;
                dlt2 = (1.0 - Math.Cos(Omega_x_abs * SINSstate.timeStep)) / (Omega_x_abs * Omega_x_abs);
            }
            else
            {
                dlt = 1.0;
                dlt2 = 0.0;
            }

            Hat1 = Matrix.SkewSymmetricMatrix(SINSstate.Omega_x);
            Hat2 = Matrix.SkewSymmetricMatrixSquare(SINSstate.Omega_x);

            CopyMatrix(dMatrix, E + Hat1 * dlt + Hat2 * dlt2);
            CopyMatrix(B_x_eta, dMatrix * B_x_eta);

            //Нормировка
            for (int i = 0; i < 3; i++)
            {
                tempV[i] = Math.Sqrt(B_x_eta[i, 0] * B_x_eta[i, 0] + B_x_eta[i, 1] * B_x_eta[i, 1] + B_x_eta[i, 2] * B_x_eta[i, 2]);
                for (int j = 0; j < 3; j++)
                    B_x_eta[i, j] = B_x_eta[i, j] / tempV[i];
            }


            CopyMatrix(W_x_xi, B_x_eta * SINSstate.A_nxi);
            CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());

            //----------------Вычисление углов и переприсвоение матриц---------------------------
            CopyMatrix(SINSstate.A_sx0, D_x_z.Transpose());
            CopyMatrix(SINSstate.A_x0s, D_x_z);
            CopyMatrix(SINSstate.A_x0n, B_x_eta);
            CopyMatrix(SINSstate.A_nx0, B_x_eta.Transpose());



            //---ОПРЕДЕЛЕНИЕ ГЕОГРАФИЧЕСКИХ КООРДИНАТ---
            SINSstate.Longitude = Math.Atan2(SINSstate.A_x0n[2, 1], SINSstate.A_x0n[2, 0]);
            SINSstate.Latitude = Math.Atan2(SINSstate.A_x0n[2, 2], Math.Sqrt(SINSstate.A_x0n[0, 2] * SINSstate.A_x0n[0, 2] + SINSstate.A_x0n[1, 2] * SINSstate.A_x0n[1, 2]));
            Azimth = Math.Atan2(SINSstate.A_x0n[0, 2], SINSstate.A_x0n[1, 2]);

            SINSstate.Altitude_prev = SINSstate.Altitude;
            SINSstate.Altitude = Altitude;


            //SINSstate.Heading = gkurs - Azimth;
            SINSstate.Heading = Math.Atan2(SINSstate.A_sx0[1, 0], SINSstate.A_sx0[1, 1]);
            SINSstate.Roll = -Math.Atan2(SINSstate.A_sx0[0, 2], SINSstate.A_sx0[2, 2]);
            SINSstate.Pitch = Math.Atan2(SINSstate.A_sx0[1, 2], Math.Sqrt(SINSstate.A_sx0[0, 2] * SINSstate.A_sx0[0, 2] + SINSstate.A_sx0[2, 2] * SINSstate.A_sx0[2, 2]));
            //SINSstate.Azimth = Azimth;

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.u_x = U_x0(SINSstate.Latitude);

            CopyArray(SINSstate.Vx_0_prev, SINSstate.Vx_0);
            CopyArray(SINSstate.Vx_0, Vx_0);

            CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
            CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
            CopyArray(SINSstate.W_x, SINSstate.A_x0s * Wz);





            //--------------------------------------------------------------------------------------
            //--------------------------------------------------------------------------------------

            if (SINSstate.flag_OdoSINSWeakConnect == false && SINSstate.flag_OdoSINSWeakConnect_MODIF == false)
                SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_x0s, SINSstate.A_x0s);

            //--------- ДЛЯ SINSstate_OdoModel ---------//

            SimpleOperations.CopyArray(SINSstate_OdoMod.Vx_0_prev, SINSstate_OdoMod.Vx_0);
            SimpleOperations.CopyArray(SINSstate.OdoSpeed_x0, SINSstate.A_x0s * SINSstate.OdoSpeed_s);
            SimpleOperations.CopyArray(SINSstate_OdoMod.OdoSpeed_s, SINSstate.OdoSpeed_s);

            //--- Если flag_Odometr_SINS_case = true (режим Одометр+БИНС) и модифицированый вариант, то считаем для одометрического счисления свою матрицу ориентации ---//
            if (SINSstate.flag_OdoSINSWeakConnect_MODIF)
            {
                SimpleOperations.CopyMatrix(W_x_xi, SINSstate_OdoMod.A_x0n * SINSstate.A_nxi);
                SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_x0s, W_x_xi * SINSstate.AT.Transpose());

                SimpleOperations.CopyArray(SINSstate_OdoMod.OdoSpeed_x0, SINSstate_OdoMod.A_x0s * SINSstate.OdoSpeed_s);
            }
            //--- если слабосвязанный вариант, то используем БИНСовую матрицу ориентации ---//
            else if (SINSstate.flag_OdoSINSWeakConnect)
                SimpleOperations.CopyArray(SINSstate_OdoMod.OdoSpeed_x0, SINSstate.A_x0s * SINSstate.OdoSpeed_s);
            else
                SimpleOperations.CopyArray(SINSstate_OdoMod.OdoSpeed_x0, SINSstate.OdoSpeed_x0);


            //---------ВЫЧИСЛЕНИЕ МАТРИЦЫ B_X_ETA И ВТОРОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z для одометрического счисления--------------
            {
                SimpleOperations.CopyArray(SINSstate_OdoMod.Vx_0, SINSstate_OdoMod.OdoSpeed_x0);

                SINSstate_OdoMod.Omega_x[0] = -(SINSstate_OdoMod.Vx_0[1] + SINSstate_OdoMod.Vx_0_prev[1]) / 2.0 / SINSstate_OdoMod.R_n;
                SINSstate_OdoMod.Omega_x[1] = (SINSstate_OdoMod.Vx_0[0] + SINSstate_OdoMod.Vx_0_prev[0]) / 2.0 / SINSstate_OdoMod.R_e;
                SINSstate_OdoMod.Omega_x[2] = Math.Tan(SINSstate_OdoMod.Latitude) * SINSstate_OdoMod.Omega_x[1];

                //--- Если используется Слабосвязанный вариант Одометр+БИНС, то в качестве матрица ориетации используем БИНСовую ---//
                if (SINSstate.flag_OdoSINSWeakConnect)
                    SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_x0s, SINSstate.A_x0s);

                //--- Производим одометрическое счисление координат ---//
                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    double[] dS_x = new double[3];
                    SimpleOperations.CopyArray(dS_x, SINSstate_OdoMod.A_x0s * SINSstate.OdometerVector);

                    SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude + dS_x[1] / SimpleOperations.RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);
                    SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude + dS_x[0] / SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude) / Math.Cos(SINSstate_OdoMod.Latitude);
                    SINSstate_OdoMod.Altitude = SINSstate_OdoMod.Altitude + dS_x[2];
                }

                //----------------Вычисление углов и переприсвоение матриц---------------------------
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Longitude);
                SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_nx0, SINSstate_OdoMod.A_x0n.Transpose());

                if (SINSstate.flag_OdoSINSWeakConnect_MODIF)
                {
                    SimpleOperations.CopyMatrix(W_x_xi, SINSstate_OdoMod.A_x0n * SINSstate.A_nxi);
                    SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_x0s, W_x_xi * SINSstate.AT.Transpose());
                }

                SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_sx0, SINSstate_OdoMod.A_x0s.Transpose());
            }

            SINSstate_OdoMod.R_e = RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);
            SINSstate_OdoMod.R_n = RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);
            //--------------------------------------------------------------------------------------
        }








        //------------------ Функции для СГЛАЖИВАНИЯ --------------------//
        public static void FuncSmoothing_BackwardAndSmooth(SINS_State SINSstate, SINS_State SINSstate_Smooth, Kalman_Vars KalmanVars, Proc_Help ProcHelp, StreamReader Back_Input_X, StreamReader Back_Input_P, StreamWriter ForHelpSmoothed)
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
                SINSstate_Smooth.Vx_0[1] = KalmanVars.ErrorVector_Smoothed[3];
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



        public static void FuncSmoothing_Forward(SINS_State SINSstate, SINS_State SINSstate_Smooth, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, Proc_Help ProcHelp, StreamWriter Smthing_X, StreamWriter Smthing_P, StreamWriter Smthing_Backward)
        {
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



        public static void FuncSmoothing_TransposeFileForBackward(SINS_State SINSstate, int NumberOfIterationForOneForSmoothing)
        {
            StreamReader Smthing_Backward_R, Smthing_Backward_R_X, Smthing_Backward_R_P;
            StreamWriter Smthing_Backward_full = new StreamWriter(SimpleData.PathOutputString + "For Smoothing temp files//Backward_full.txt"),
                         Smthing_Backward_X = new StreamWriter(SimpleData.PathOutputString + "For Smoothing temp files//Backward_full_X.txt"),
                         Smthing_Backward_P = new StreamWriter(SimpleData.PathOutputString + "For Smoothing temp files//Backward_full_P.txt");

            for (int i = Convert.ToInt32(SINSstate.NumberOfFilesForSmoothing); i >= 1; i--)
            {
                int j = 0;
                string[] strTemp = new string[NumberOfIterationForOneForSmoothing],
                         strTemp_X = new string[NumberOfIterationForOneForSmoothing],
                         strTemp_P = new string[NumberOfIterationForOneForSmoothing];
                Smthing_Backward_R = new StreamReader(SimpleData.PathOutputString + "For Smoothing temp files//Backward_" + i + ".txt");
                Smthing_Backward_R_X = new StreamReader(SimpleData.PathOutputString + "For Smoothing temp files//Backward_X_" + i + ".txt");
                Smthing_Backward_R_P = new StreamReader(SimpleData.PathOutputString + "For Smoothing temp files//Backward_P_" + i + ".txt");

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
    }
}
