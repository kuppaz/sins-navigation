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
                    if (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev == 0)
                        SINSstate.OdometerZUPT_counter++;
                    else
                        SINSstate.OdometerZUPT_counter = 0;

                    SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                    SINSstate.OdoSpeed_s_prev[1] = SINSstate.OdoSpeed_s[1];
                    SINSstate.OdoTimeStepCount = 0;

                    SINSstate.odotime_prev = SINSstate.Time;

                    SINSstate_OdoMod.Latitude_prev = SINSstate.Latitude;
                    SINSstate_OdoMod.Longitude_prev = SINSstate.Longitude;
                    SINSstate_OdoMod.Height_prev = SINSstate.Height;

                    SINSstate_OdoMod.Heading_prev = SINSstate_OdoMod.Heading;
                    SINSstate_OdoMod.Roll_prev = SINSstate_OdoMod.Roll;
                    SINSstate_OdoMod.Pitch_prev = SINSstate_OdoMod.Pitch;

                    /* --- Запоминаем предыдущие значения показания одометра --- */
                    for (int i = 0; i < SINSstate.OdometerLeft_ArrayOfPrev.Length - 1; i++)
                    {
                        SINSstate.OdometerLeft_ArrayOfPrev[SINSstate.OdometerLeft_ArrayOfPrev.Length - 1 - i]
                            = SINSstate.OdometerLeft_ArrayOfPrev[SINSstate.OdometerLeft_ArrayOfPrev.Length - 1 - i - 1];
                        SINSstate.OdometerLeft_ArrayOfPrevTime[SINSstate.OdometerLeft_ArrayOfPrev.Length - 1 - i]
                            = SINSstate.OdometerLeft_ArrayOfPrevTime[SINSstate.OdometerLeft_ArrayOfPrev.Length - 1 - i - 1];
                    }
                    SINSstate.OdometerLeft_ArrayOfPrev[0] = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerLeft_ArrayOfPrevTime[0] = SINSstate.Time + SINSstate.Time_Alignment;
                }
            }

            SINSstate.Latitude_prev = SINSstate.Latitude; SINSstate2.Latitude_prev = SINSstate2.Latitude;
            SINSstate.Longitude_prev = SINSstate.Longitude; SINSstate2.Longitude_prev = SINSstate2.Longitude;
            SINSstate.Height_prev = SINSstate.Height; SINSstate2.Height_prev = SINSstate2.Height;

            SINSstate.Heading_prev = SINSstate.Heading; SINSstate2.Heading_prev = SINSstate2.Heading;
            SINSstate.Roll_prev = SINSstate.Roll; SINSstate2.Roll_prev = SINSstate2.Roll;
            SINSstate.Pitch_prev = SINSstate.Pitch; SINSstate2.Pitch_prev = SINSstate2.Pitch;
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

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);
            SINSstate.u_x = U_x0(SINSstate.Latitude);

            SINSstate.Omega_x[0] = -(SINSstate.Vx_0[1] + SINSstate.Vx_0_prev[1]) / 2.0 / SINSstate.R_n;
            SINSstate.Omega_x[1] = (SINSstate.Vx_0[0] + SINSstate.Vx_0_prev[0]) / 2.0 / SINSstate.R_e;
            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * SINSstate.Omega_x[1];

            SINSstate.g = 9.78049 * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014;
            SINSstate.g -= 2 * 0.000001538 * SINSstate.Height;
        }



        public static void CalcStateErrors(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_kappa_1 = SINSstate.value_iMx_kappa_1,
                iMx_r12_odo = SINSstate.value_iMx_r_odo_12, value_iMx_dr3 = SINSstate.value_iMx_dr3, value_iMx_dV3 = SINSstate.value_iMx_dV3;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3,
                iMx_r_odo_3 = SINSstate.value_iMx_r_odo_3
                ;

            SINSstate.DeltaLatitude = ErrorVector[1] / SINSstate.R_n;
            SINSstate.DeltaLongitude = ErrorVector[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

            SINSstate.DeltaV_1 = ErrorVector[iMx_dV_12 + 0]
                //+ SINSstate.Vx_0[1] * ErrorVector[iMx_alphaBeta + 2] + SINSstate.Vx_0[1] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude)
                ;
            SINSstate.DeltaV_2 = ErrorVector[iMx_dV_12 + 1]
                //- SINSstate.Vx_0[0] * ErrorVector[iMx_alphaBeta + 2] - SINSstate.Vx_0[0] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude)
                ;


            //--- В случае обратных связей не должно быть списывания углов бетта
            if (!SINSstate.flag_FeedbackExist)
            {
                SINSstate.DeltaV_1 += SINSstate.Vx_0[1] * ErrorVector[iMx_alphaBeta + 2] + SINSstate.Vx_0[1] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
                SINSstate.DeltaV_2 += -SINSstate.Vx_0[0] * ErrorVector[iMx_alphaBeta + 2] - SINSstate.Vx_0[0] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
            }

            if (SINSstate.flag_iMx_r3_dV3)
            {
                SINSstate.DeltaAltitude = ErrorVector[value_iMx_dr3];
                SINSstate.DeltaV_3 = ErrorVector[value_iMx_dV3];

                //--- В случае обратных связей не должно быть списывания углов бетта
                if (!SINSstate.flag_FeedbackExist)
                {
                    SINSstate.DeltaV_1 += SINSstate.Vx_0[2] * (ErrorVector[0] / SINSstate.R_e - ErrorVector[iMx_alphaBeta + 1]);
                    SINSstate.DeltaV_2 += SINSstate.Vx_0[2] * (ErrorVector[1] / SINSstate.R_n - ErrorVector[iMx_alphaBeta + 0]);
                    SINSstate.DeltaV_3 += SINSstate.Vx_0[0] * (ErrorVector[iMx_alphaBeta + 1] - ErrorVector[0] / SINSstate.R_e) - SINSstate.Vx_0[1] * (ErrorVector[iMx_alphaBeta + 0] + ErrorVector[1] / SINSstate.R_n);
                }
            }

            SINSstate.DeltaRoll = -(ErrorVector[iMx_alphaBeta + 0] * Math.Sin(SINSstate.Heading) + ErrorVector[iMx_alphaBeta + 1] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
            SINSstate.DeltaPitch = -ErrorVector[iMx_alphaBeta + 0] * Math.Cos(SINSstate.Heading) + ErrorVector[iMx_alphaBeta + 1] * Math.Sin(SINSstate.Heading);
            SINSstate.DeltaHeading = ErrorVector[iMx_alphaBeta + 2] + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);
            //--- В случае обратных связей не должно быть повторного списывания меридиальной составляющей
            if (!SINSstate.flag_FeedbackExist)
                SINSstate.DeltaHeading = SINSstate.DeltaHeading + SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);


            //--- Случай Одометр+БИНС. Обратная связть ---//
            if (SINSstate.flag_Odometr_SINS_case == true)
            {
                SINSstate_OdoMod.DeltaLatitude = ErrorVector[iMx_r12_odo + 1] / SINSstate_OdoMod.R_n;
                SINSstate_OdoMod.DeltaLongitude = ErrorVector[iMx_r12_odo + 0] / SINSstate_OdoMod.R_e / Math.Cos(SINSstate_OdoMod.Latitude);

                if (SINSstate.flag_iMx_r3_dV3)
                    SINSstate_OdoMod.DeltaAltitude = ErrorVector[iMx_r_odo_3];
            }



            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                SINSstate.DeltaAltitude = KalmanVars.Vertical_ErrorConditionVector_p[0];
                SINSstate.DeltaV_3 = KalmanVars.Vertical_ErrorConditionVector_p[1];

                SINSstate_OdoMod.DeltaAltitude = KalmanVars.Vertical_ErrorConditionVector_p[SINSstate.Vertical_rOdo3];
            }
        }



        public static void StateCorrection(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars)
        {
            SINSstate2.Latitude = SINSstate.Latitude - SINSstate.DeltaLatitude;
            SINSstate2.Longitude = SINSstate.Longitude - SINSstate.DeltaLongitude;


            SINSstate2.Vx_0[0] = SINSstate.Vx_0[0] - SINSstate.DeltaV_1;
            SINSstate2.Vx_0[1] = SINSstate.Vx_0[1] - SINSstate.DeltaV_2;

            if (SINSstate.flag_iMx_r3_dV3)
            {
                SINSstate2.Vx_0[2] = SINSstate.Vx_0[2] - SINSstate.DeltaV_3;
                SINSstate2.Height = SINSstate.Height - SINSstate.DeltaAltitude;
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
                    if (SINSstate.value_iMx_kappa_1 > 0)
                        SINSstate2.Cumulative_KappaEst[0] += ErrorVector[SINSstate.value_iMx_kappa_1 + 0];
                    SINSstate2.Cumulative_KappaEst[2] += ErrorVector[SINSstate.value_iMx_kappa_3_ds + 0];// +KalmanVars.Vertical_ErrorConditionVector_p[SINSstate.Vertical_kappa3Scale + 0];
                    SINSstate2.Cumulative_KappaEst[1] += ErrorVector[SINSstate.value_iMx_kappa_3_ds + 1];// +KalmanVars.Vertical_ErrorConditionVector_p[SINSstate.Vertical_kappa3Scale + 1];
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
                if (SINSstate.flag_iMx_r3_dV3)
                    SINSstate_OdoMod.Altitude_Corr = SINSstate_OdoMod.Height - SINSstate_OdoMod.DeltaAltitude;


                if (SINSstate.flag_FeedbackExist == true)
                {
                    SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude - SINSstate_OdoMod.DeltaLatitude;
                    SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude - SINSstate_OdoMod.DeltaLongitude;

                    if (SINSstate.flag_iMx_r3_dV3)
                        SINSstate_OdoMod.Height = SINSstate_OdoMod.Height - SINSstate_OdoMod.DeltaAltitude;

                    SINSstate_OdoMod.A_x0n = A_x0n(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Longitude);
                    SINSstate_OdoMod.A_nx0 = SINSstate_OdoMod.A_x0n.Transpose();
                }
            }
            else if (SINSstate.flag_FeedbackExist == true)
            {
                SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude - SINSstate_OdoMod.DeltaLatitude;
                SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude - SINSstate_OdoMod.DeltaLongitude;
            }



            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                SINSstate2.Vx_0[2] = SINSstate.Vx_0[2] - SINSstate.DeltaV_3;
                SINSstate2.Height = SINSstate.Height - SINSstate.DeltaAltitude;

                SINSstate_OdoMod.Height = SINSstate_OdoMod.Height - SINSstate_OdoMod.DeltaAltitude;

                for (int i = 0; i < SimpleData.iMx_Vertical; i++)
                    SINSstate2.Vertical_Cumulative_KalmanErrorVector[i] += KalmanVars.Vertical_ErrorConditionVector_p[i];
            }
        }



        public static void NullingOfCorrectedErrors(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {

            for (int i = 0; i < SimpleData.iMx; i++)
            {
                KalmanVars.ErrorConditionVector_p[i] = 0.0;
                KalmanVars.ErrorConditionVector_m[i] = 0.0;
            }


            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                for (int i = 0; i < SimpleData.iMx_Vertical; i++)
                {
                    KalmanVars.Vertical_ErrorConditionVector_p[i] = 0.0;
                    KalmanVars.Vertical_ErrorConditionVector_m[i] = 0.0;
                }
            }
        }







        public static void InitOfCovarianceMatrixes(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_kappa_1 = SINSstate.value_iMx_kappa_1,
                iMx_r12_odo = SINSstate.value_iMx_r_odo_12, value_iMx_dr3 = SINSstate.value_iMx_dr3, value_iMx_dV3 = SINSstate.value_iMx_dV3;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3,
                iMx_r_odo_3 = SINSstate.value_iMx_r_odo_3
                ;

            SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrixS_m);
            SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrixS_p);


            // --- нач. ковариации для ошибки координат --- //
            KalmanVars.CovarianceMatrixS_m[0 * iMx + 0] = KalmanVars.CovarianceMatrixS_p[0 * iMx + 0] = SINSstate.stdR;    // позиционные ошибки
            KalmanVars.CovarianceMatrixS_m[1 * iMx + 1] = KalmanVars.CovarianceMatrixS_p[1 * iMx + 1] = SINSstate.stdR;

            // --- нач. ковариации для ошибки скорости --- //
            KalmanVars.CovarianceMatrixS_m[(iMx_dV_12 + 0) * iMx + (iMx_dV_12 + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_dV_12 + 0) * iMx + (iMx_dV_12 + 0)] = SINSstate.stdV;   // 0.01 м/с
            KalmanVars.CovarianceMatrixS_m[(iMx_dV_12 + 1) * iMx + (iMx_dV_12 + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_dV_12 + 1) * iMx + (iMx_dV_12 + 1)] = SINSstate.stdV;

            // --- нач. ковариации для ошибок углов ориентации --- //
            KalmanVars.CovarianceMatrixS_m[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 0)] 
                = Math.Sign(SINSstate.stdAlpha1) * Math.Max(Math.Abs(SINSstate.stdAlpha1), 1E-6);  // 5 угл. минут
            KalmanVars.CovarianceMatrixS_m[(iMx_alphaBeta + 1) * iMx + (iMx_alphaBeta + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 1) * iMx + (iMx_alphaBeta + 1)] 
                = Math.Sign(SINSstate.stdAlpha2) * Math.Max(Math.Abs(SINSstate.stdAlpha2), 1E-6);
            KalmanVars.CovarianceMatrixS_m[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 2)] = KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 2)] 
                = Math.Sign(SINSstate.stdBeta3) * Math.Max(Math.Abs(SINSstate.stdBeta3), 1E-6);

            // --- нач. ковариации для дрейфов ДУС --- //
            KalmanVars.CovarianceMatrixS_m[(iMx_Nu0 + 0) * iMx + (iMx_Nu0 + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 0) * iMx + (iMx_Nu0 + 0)] 
                = Math.Sign(SINSstate.stdNu_Oz[0]) * Math.Max(Math.Abs(SINSstate.stdNu_Oz[0]) * SimpleData.ToRadian / 3600.0, 1E-10); //0.2 * ToRadian / 3600.0; // 0.2 град/час
            KalmanVars.CovarianceMatrixS_m[(iMx_Nu0 + 1) * iMx + (iMx_Nu0 + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 1) * iMx + (iMx_Nu0 + 1)] 
                = Math.Sign(SINSstate.stdNu_Oz[1]) * Math.Max(Math.Abs(SINSstate.stdNu_Oz[1]) * SimpleData.ToRadian / 3600.0, 1E-10);//0.2 * ToRadian / 3600.0;
            KalmanVars.CovarianceMatrixS_m[(iMx_Nu0 + 2) * iMx + (iMx_Nu0 + 2)] = KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 2) * iMx + (iMx_Nu0 + 2)] 
                = Math.Sign(SINSstate.stdNu_Oz[2]) * Math.Max(Math.Abs(SINSstate.stdNu_Oz[2]) * SimpleData.ToRadian / 3600.0, 1E-10);//0.2 * ToRadian / 3600.0;

            // --- нач. ковариации для горизонтальных ньютонометров --- //
            KalmanVars.CovarianceMatrixS_m[(f0_12 + 0) * iMx + (f0_12 + 0)] = KalmanVars.CovarianceMatrixS_p[(f0_12 + 0) * iMx + (f0_12 + 0)] 
                = Math.Sign(SINSstate.stdF_Oz[0]) * Math.Max(Math.Abs(SINSstate.stdF_Oz[0]), 1E-6);    // м/с^2
            KalmanVars.CovarianceMatrixS_m[(f0_12 + 1) * iMx + (f0_12 + 1)] = KalmanVars.CovarianceMatrixS_p[(f0_12 + 1) * iMx + (f0_12 + 1)] 
                = Math.Sign(SINSstate.stdF_Oz[1]) * Math.Max(Math.Abs(SINSstate.stdF_Oz[1]), 1E-6);

            // --- нач. ковариации для вертикального ньютонометра, если он включен в вектор ошибок --- //
            if (SINSstate.value_iMx_f0_3 > 0)
                KalmanVars.CovarianceMatrixS_m[(f0_3 + 0) * iMx + (f0_3 + 0)] = KalmanVars.CovarianceMatrixS_p[(f0_3 + 0) * iMx + (f0_3 + 0)] 
                    = Math.Sign(SINSstate.stdF_Oz[2]) * Math.Max(Math.Abs(SINSstate.stdF_Oz[2]), 1E-6);


            // --- нач. ковариации для ошибок вертикального канала --- //
            if (SINSstate.flag_iMx_r3_dV3 == true)
            {
                KalmanVars.CovarianceMatrixS_m[value_iMx_dr3 * iMx + value_iMx_dr3] = KalmanVars.CovarianceMatrixS_p[value_iMx_dr3 * iMx + value_iMx_dr3] = SINSstate.stdR;
                KalmanVars.CovarianceMatrixS_m[(value_iMx_dV3 + 0) * iMx + (value_iMx_dV3 + 0)] = KalmanVars.CovarianceMatrixS_p[(value_iMx_dV3 + 0) * iMx + (value_iMx_dV3 + 0)] = SINSstate.stdV;
            }

            // --- нач. ковариации для ошибок масштаба и ошибок углов установки БИНС на корпусе --- //
            if (SINSstate.flag_iMx_kappa_13_ds == true)
            {
                if (SINSstate.value_iMx_kappa_1 > 0)
                    KalmanVars.CovarianceMatrixS_m[(iMx_kappa_1 + 0) * iMx + (iMx_kappa_1 + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_kappa_1 + 0) * iMx + (iMx_kappa_1 + 0)] = SINSstate.stdKappa1 * SimpleData.ToRadian_min;

                KalmanVars.CovarianceMatrixS_m[(iMx_kappa_3_ds + 0) * iMx + (iMx_kappa_3_ds + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_kappa_3_ds + 0) * iMx + (iMx_kappa_3_ds + 0)] = SINSstate.stdKappa3 * SimpleData.ToRadian_min;
                KalmanVars.CovarianceMatrixS_m[(iMx_kappa_3_ds + 1) * iMx + (iMx_kappa_3_ds + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_kappa_3_ds + 1) * iMx + (iMx_kappa_3_ds + 1)] = SINSstate.stdScale;
            }




            // ---------------------ВЕРТИКАЛЬНЫЙ КАНАЛ ОТДЕЛЬНО-----------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                int iMxV = SimpleData.iMx_Vertical,
                    vert_f0_12 = SINSstate.Vertical_f0_12,
                    vert_f0_3 = SINSstate.Vertical_f0_3,
                    vert_kappa1 = SINSstate.Vertical_kappa1,
                    vert_kappa3d = SINSstate.Vertical_kappa3Scale
                    ;

                SimpleOperations.NullingOfArray(KalmanVars.Vertical_CovarianceMatrixS_m);
                SimpleOperations.NullingOfArray(KalmanVars.Vertical_CovarianceMatrixS_p);

                // --- нач. ковариации ошибок высоты и верт. скорости --- //
                KalmanVars.Vertical_CovarianceMatrixS_m[0 * iMxV + 0] = KalmanVars.Vertical_CovarianceMatrixS_p[0 * iMxV + 0] = SINSstate.stdR;
                KalmanVars.Vertical_CovarianceMatrixS_m[1 * iMxV + 1] = KalmanVars.Vertical_CovarianceMatrixS_p[1 * iMxV + 1] = SINSstate.stdV;

                // --- нач. ковариации ошибки верт. ньютонометра --- //
                KalmanVars.Vertical_CovarianceMatrixS_m[vert_f0_3 * iMxV + vert_f0_3] = KalmanVars.Vertical_CovarianceMatrixS_p[vert_f0_3 * iMxV + vert_f0_3]
                    = Math.Sign(SINSstate.stdF_Oz[2]) * Math.Max(Math.Abs(SINSstate.stdF_Oz[2]), 1E-6);

                // --- нач. ковариации ошибок горизонтальны ньютонометров, если они включены --- //
                if (SINSstate.Vertical_f0_12 > 0)
                {
                    KalmanVars.Vertical_CovarianceMatrixS_m[(vert_f0_12 + 0) * iMxV + (vert_f0_12 + 0)] = KalmanVars.Vertical_CovarianceMatrixS_p[(vert_f0_12 + 0) * iMxV + (vert_f0_12 + 0)] 
                        = Math.Sign(SINSstate.stdF_Oz[0]) * Math.Max(Math.Abs(SINSstate.stdF_Oz[0]), 1E-6);    // м/с^2
                    KalmanVars.Vertical_CovarianceMatrixS_m[(vert_f0_12 + 1) * iMxV + (vert_f0_12 + 1)] = KalmanVars.Vertical_CovarianceMatrixS_p[(vert_f0_12 + 1) * iMxV + (vert_f0_12 + 1)] 
                        = Math.Sign(SINSstate.stdF_Oz[1]) * Math.Max(Math.Abs(SINSstate.stdF_Oz[1]), 1E-6);
                }

                // --- нач. ковариации ошибок мастаба одометра и углов установки БИНС на корпусе --- //
                if (SINSstate.Vertical_kappa1 > 0)
                {
                    KalmanVars.Vertical_CovarianceMatrixS_m[vert_kappa1 * SimpleData.iMx_Vertical + vert_kappa1]
                        = KalmanVars.Vertical_CovarianceMatrixS_p[vert_kappa1 * SimpleData.iMx_Vertical + vert_kappa1] = SINSstate.stdKappa1 * SimpleData.ToRadian_min;

                    if (SINSstate.Vertical_kappa3Scale > 0)
                    {
                        KalmanVars.Vertical_CovarianceMatrixS_m[(vert_kappa3d + 0) * SimpleData.iMx_Vertical + (vert_kappa3d + 0)]
                            = KalmanVars.Vertical_CovarianceMatrixS_p[(vert_kappa3d + 0) * SimpleData.iMx_Vertical + (vert_kappa3d + 0)] = SINSstate.stdKappa3 * SimpleData.ToRadian_min;
                        KalmanVars.Vertical_CovarianceMatrixS_m[(vert_kappa3d + 1) * SimpleData.iMx_Vertical + (vert_kappa3d + 1)]
                            = KalmanVars.Vertical_CovarianceMatrixS_p[(vert_kappa3d + 1) * SimpleData.iMx_Vertical + (vert_kappa3d + 1)] = SINSstate.stdScale;
                    }
                }
            }
        }






        public static void MatrixNoise_ReDef(SINS_State SINSstate, Kalman_Vars KalmanVars, bool AlignmentFLG)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_kappa_1 = SINSstate.value_iMx_kappa_1,
                iMx_r12_odo = SINSstate.value_iMx_r_odo_12, value_iMx_dr3 = SINSstate.value_iMx_dr3, value_iMx_dV3 = SINSstate.value_iMx_dV3;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3,
                iMx_r_odo_3 = SINSstate.value_iMx_r_odo_3
                ;

            double sqrt_freq_vert = Math.Sqrt(Math.Abs(SINSstate.Freq)); ;
            double sqrt_freq = Math.Sqrt(Math.Abs(SINSstate.Freq));

            //sqrt_freq = 1.0;
            //sqrt_freq_vert = sqrt_freq;


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
                KalmanVars.CovarianceMatrixNoise[0 * iMq + 0] = KalmanVars.Noise_Pos * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[1 * iMq + 1] = KalmanVars.Noise_Pos * sqrt_freq;
            }

            // так как в векторе состояния дрейфы в проекции на приборные оси, надо задавать соответственно матрицу шумов //
            KalmanVars.CovarianceMatrixNoise[(iMx_dV_12 + 0) * iMq + iMx_dV_12 + 0] = Noise_Vel_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_dV_12 + 0) * iMq + iMx_alphaBeta + 0] = SINSstate.Vx_0[1] * Noise_Angl_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_dV_12 + 1) * iMq + iMx_dV_12 + 1] = Noise_Vel_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_dV_12 + 1) * iMq + iMx_alphaBeta + 1] = SINSstate.Vx_0[0] * Noise_Angl_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_alphaBeta + 0) * iMq + iMx_alphaBeta + 0] = Noise_Angl_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_alphaBeta + 1) * iMq + iMx_alphaBeta + 1] = Noise_Angl_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_alphaBeta + 2) * iMq + iMx_alphaBeta + 2] = Noise_Angl_in_Mx[2] * sqrt_freq;

            if (SINSstate.flag_iMqDeltaNu)
            {
                KalmanVars.CovarianceMatrixNoise[(iMx_Nu0 + 0) * iMq + iMx_Nu0 + 0] = KalmanVars.Noise_Drift * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[(iMx_Nu0 + 1) * iMq + iMx_Nu0 + 1] = KalmanVars.Noise_Drift * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[(iMx_Nu0 + 2) * iMq + iMx_Nu0 + 2] = KalmanVars.Noise_Drift * sqrt_freq;
            }
            if (SINSstate.flag_iMqDeltaF)
            {
                KalmanVars.CovarianceMatrixNoise[(f0_12 + 0) * iMq + f0_12 + 0] = KalmanVars.Noise_Accel * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[(f0_12 + 1) * iMq + f0_12 + 1] = KalmanVars.Noise_Accel * sqrt_freq;

                KalmanVars.CovarianceMatrixNoise[(f0_3 + 0) * iMq + f0_3 + 0] = KalmanVars.Noise_Accel * sqrt_freq;
            }


            if (SINSstate.flag_iMx_r3_dV3)
            {
                if (SINSstate.flag_iMqDeltaR)
                    KalmanVars.CovarianceMatrixNoise[(value_iMx_dr3 + 0) * iMq + value_iMx_dr3 + 0] = KalmanVars.Noise_Pos * sqrt_freq;
                
                KalmanVars.CovarianceMatrixNoise[(value_iMx_dV3 + 0) * iMq + value_iMx_dV3 + 0] = Noise_Vel_in_Mx[2] * sqrt_freq;
            }

            if (SINSstate.flag_iMx_kappa_13_ds)
            {
                if (SINSstate.flag_iMqVarkappa3)
                    KalmanVars.CovarianceMatrixNoise[(iMx_kappa_3_ds + 0) * iMq + iMx_kappa_3_ds + 0] = KalmanVars.Noise_OdoKappa_3 * sqrt_freq;

                if (SINSstate.flag_iMqVarkappa1)
                    if (SINSstate.value_iMx_kappa_1 > 0)
                        KalmanVars.CovarianceMatrixNoise[(iMx_kappa_1 + 0) * iMq + iMx_kappa_1 + 0] = KalmanVars.Noise_OdoKappa_1 * sqrt_freq;

                if (SINSstate.flag_iMqKappa)
                    KalmanVars.CovarianceMatrixNoise[(iMx_kappa_3_ds + 1) * iMq + iMx_kappa_3_ds + 1] = KalmanVars.Noise_OdoScale * sqrt_freq;
            }


            // --- Матрица с одной ненулевой компонентой ---
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                for (int i = 0; i < SimpleData.iMx_Vertical * SimpleData.iMq_Vertical; i++)
                    KalmanVars.Vertical_CovarianceMatrixNoise[i] = 0.0;

                if (SINSstate.flag_iMqDeltaR)
                    KalmanVars.Vertical_CovarianceMatrixNoise[0 * SimpleData.iMq_Vertical + 0] = KalmanVars.Noise_Pos_Odo * sqrt_freq;

                KalmanVars.Vertical_CovarianceMatrixNoise[1 * SimpleData.iMq_Vertical + 1] = Noise_Vel_in_Mx[2] * sqrt_freq * 1.0;

                if (SINSstate.flag_iMqDeltaF)
                    KalmanVars.Vertical_CovarianceMatrixNoise[SINSstate.Vertical_f0_3 * SimpleData.iMq_Vertical + SINSstate.Vertical_f0_3] = KalmanVars.Noise_Accel * sqrt_freq;
            }
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
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_kappa_1 = SINSstate.value_iMx_kappa_1,
                iMx_r12_odo = SINSstate.value_iMx_r_odo_12, iMx_dr3 = SINSstate.value_iMx_dr3, iMx_dV3 = SINSstate.value_iMx_dV3;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3
                ;

            for (int i = 0; i < iMx * iMx; i++)
                KalmanVars.Matrix_A[i] = 0;

            SINSstate.W_x[0] = SINSstate.Omega_x[0];
            SINSstate.W_x[1] = SINSstate.Omega_x[1] + SimpleData.U * Math.Cos(SINSstate.Latitude);
            SINSstate.W_x[2] = SINSstate.Omega_x[2] + SimpleData.U * Math.Sin(SINSstate.Latitude);

            KalmanVars.Matrix_A[0 * iMx + 1] = SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[0 * iMx + (iMx_dV_12 + 0)] = 1.0;
            KalmanVars.Matrix_A[0 * iMx + (iMx_alphaBeta + 2)] = SINSstate.Vx_0[1];

            KalmanVars.Matrix_A[1 * iMx + 0] = -SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[1 * iMx + (iMx_dV_12 + 1)] = 1.0;
            KalmanVars.Matrix_A[1 * iMx + (iMx_alphaBeta + 2)] = -SINSstate.Vx_0[0];

            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + 1] = SINSstate.u_x[1] * SINSstate.Vx_0[1] / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_dV_12 + 1)] = SINSstate.Omega_x[2] + 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_alphaBeta + 0)] = SINSstate.u_x[1] * SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_alphaBeta + 1)] = -SINSstate.g;
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 0)] = -SINSstate.Vx_0[1] * SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 1)] = -SINSstate.Vx_0[1] * SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (f0_12 + 0)] = SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (f0_12 + 1)] = SINSstate.A_x0s[0, 1];

            if (SINSstate.existRelationHoriz_VS_Vertical || ! SINSstate.flag_iMx_r3_dV3)
                if (SINSstate.value_iMx_f0_3 > 0)
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (f0_3 + 0)] = SINSstate.A_x0s[0, 2];


            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + 1] = -SINSstate.u_x[1] * SINSstate.Vx_0[0] / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_dV_12 + 0)] = -SINSstate.Omega_x[2] - 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_alphaBeta + 0)] = -SINSstate.u_x[1] * SINSstate.Vx_0[0] + SINSstate.g;
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 0)] = SINSstate.Vx_0[0] * SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 1)] = SINSstate.Vx_0[0] * SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (f0_12 + 0)] = SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (f0_12 + 1)] = SINSstate.A_x0s[1, 1];

            if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                if (SINSstate.value_iMx_f0_3 > 0)
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (f0_3 + 0)] = SINSstate.A_x0s[1, 2];


            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + 0] = -SINSstate.u_x[2] / SINSstate.R_e;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_dV_12 + 1)] = -1.0 / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 1)] = SINSstate.u_x[2];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 2)] = -SINSstate.u_x[1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_Nu0 + 0)] = -SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_Nu0 + 1)] = -SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_Nu0 + 2)] = -SINSstate.A_x0s[0, 2];

            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + 1] = -SINSstate.u_x[2] / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_dV_12 + 0)] = 1.0 / SINSstate.R_e;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_alphaBeta + 0)] = -SINSstate.u_x[2];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_Nu0 + 0)] = -SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_Nu0 + 1)] = -SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_Nu0 + 2)] = -SINSstate.A_x0s[1, 2];

            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + 0] = SINSstate.Omega_x[0] / SINSstate.R_e;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + 1] = (SINSstate.Omega_x[1] + SINSstate.u_x[1]) / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 0)] = SINSstate.Omega_x[1] + SINSstate.u_x[1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 1)] = -SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_Nu0 + 0)] = -SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_Nu0 + 1)] = -SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_Nu0 + 2)] = -SINSstate.A_x0s[2, 2];

            if (SINSstate.flag_iMx_r3_dV3)
            {
                if (SINSstate.existRelationHoriz_VS_Vertical)
                {
                    KalmanVars.Matrix_A[0 * iMx + 0] += SINSstate.Vx_0[2] / SINSstate.R_e;
                    KalmanVars.Matrix_A[0 * iMx + (iMx_alphaBeta + 1)] += -SINSstate.Vx_0[2];
                    KalmanVars.Matrix_A[0 * iMx + iMx_dr3] = -SINSstate.Omega_x[1];

                    KalmanVars.Matrix_A[1 * iMx + 1] += SINSstate.Vx_0[2] / SINSstate.R_n;
                    KalmanVars.Matrix_A[1 * iMx + (iMx_alphaBeta + 0)] += SINSstate.Vx_0[2];
                    KalmanVars.Matrix_A[1 * iMx + iMx_dr3] = SINSstate.Omega_x[0];


                    KalmanVars.Matrix_A[iMx_dr3 * iMx + 0] = SINSstate.Omega_x[1] - SINSstate.Vx_0[0] / SINSstate.R_e;
                    KalmanVars.Matrix_A[iMx_dr3 * iMx + 1] = -SINSstate.Omega_x[0] - SINSstate.Vx_0[1] / SINSstate.R_n;

                    KalmanVars.Matrix_A[iMx_dr3 * iMx + (iMx_alphaBeta + 0)] = -SINSstate.Vx_0[1];
                    KalmanVars.Matrix_A[iMx_dr3 * iMx + (iMx_alphaBeta + 1)] = SINSstate.Vx_0[0];
                }

                KalmanVars.Matrix_A[iMx_dr3 * iMx + iMx_dV3] = 1.0;


                if (SINSstate.existRelationHoriz_VS_Vertical)
                {
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + 1] += SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_n;
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_alphaBeta + 0)] += SINSstate.u_x[2] * SINSstate.Vx_0[2];

                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 0)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 0];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 1)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 1];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 2)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 2];

                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + iMx_dV3] = -SINSstate.Omega_x[1] - 2 * SINSstate.u_x[1];


                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + 0] += -SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_e;
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_alphaBeta + 1)] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_alphaBeta + 2)] += -SINSstate.u_x[1] * SINSstate.Vx_0[2];

                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 0)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 0];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 1)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 1];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 2)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 2];

                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + iMx_dV3] = SINSstate.Omega_x[0];


                    KalmanVars.Matrix_A[iMx_dV3 * iMx + 0] = SINSstate.u_x[2] * SINSstate.Vx_0[1] / SINSstate.R_e;
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + 1] = -SINSstate.u_x[2] * SINSstate.Vx_0[0] / SINSstate.R_n;
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (iMx_dV_12 + 0)] = SINSstate.Omega_x[1] + 2 * SINSstate.u_x[1];
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (iMx_dV_12 + 1)] = -SINSstate.Omega_x[0];

                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (iMx_alphaBeta + 0)] = -SINSstate.u_x[2] * SINSstate.Vx_0[0];
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (iMx_alphaBeta + 1)] = -SINSstate.u_x[2] * SINSstate.Vx_0[1];
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (iMx_alphaBeta + 2)] = SINSstate.u_x[1] * SINSstate.Vx_0[1];

                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (iMx_Nu0 + 0)] = -SINSstate.Vx_0[0] * SINSstate.A_x0s[1, 0] + SINSstate.Vx_0[1] * SINSstate.A_x0s[0, 0];
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (iMx_Nu0 + 1)] = -SINSstate.Vx_0[0] * SINSstate.A_x0s[1, 1] + SINSstate.Vx_0[1] * SINSstate.A_x0s[0, 1];
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (iMx_Nu0 + 2)] = -SINSstate.Vx_0[0] * SINSstate.A_x0s[1, 2] + SINSstate.Vx_0[1] * SINSstate.A_x0s[0, 2];
                }

                if (SINSstate.existRelationHoriz_VS_Vertical)
                {
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (f0_12 + 0)] = SINSstate.A_x0s[2, 0];
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (f0_12 + 1)] = SINSstate.A_x0s[2, 1];

                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (f0_3 + 0)] = SINSstate.A_x0s[2, 2];
                }
                else
                {
                    KalmanVars.Matrix_A[iMx_dV3 * iMx + (f0_3 + 0)] = 1.0;
                }

                KalmanVars.Matrix_A[iMx_dV3 * iMx + iMx_dr3] = 2 * 0.000001538;
            }




            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                int iMxV = SimpleData.iMx_Vertical
                    ;

                for (int i = 0; i < iMxV * iMxV; i++)
                    KalmanVars.Vertical_Matrix_A[i] = 0;

                KalmanVars.Vertical_Matrix_A[0 * iMxV + 1] = 1.0;
                KalmanVars.Vertical_Matrix_A[1 * iMxV + 0] = 2 * 0.000001538;

                if (SINSstate.Vertical_f0_12 > 0)
                {
                    KalmanVars.Vertical_Matrix_A[1 * iMxV + SINSstate.Vertical_f0_12 + 0] = SINSstate.A_x0s[2, 0];
                    KalmanVars.Vertical_Matrix_A[1 * iMxV + SINSstate.Vertical_f0_12 + 1] = SINSstate.A_x0s[2, 1];
                }

                if (SINSstate.Vertical_f0_3 > 0)
                    KalmanVars.Vertical_Matrix_A[1 * iMxV + SINSstate.Vertical_f0_3] = SINSstate.A_x0s[2, 2];


                // ----------------------------------------------------------//
                // -------------Дополняем горизонтальный канал--------------- //
                if (SINSstate.existRelationHoriz_VS_Vertical)
                {
                    KalmanVars.Matrix_A[0 * iMx + 0] += SINSstate.Vx_0[2] / SINSstate.R_e;
                    KalmanVars.Matrix_A[0 * iMx + (iMx_alphaBeta + 1)] += -SINSstate.Vx_0[2];
                    KalmanVars.Matrix_A[1 * iMx + 1] += SINSstate.Vx_0[2] / SINSstate.R_n;
                    KalmanVars.Matrix_A[1 * iMx + (iMx_alphaBeta + 0)] += SINSstate.Vx_0[2];

                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + 1] += SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_n;
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_alphaBeta + 0)] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 0)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 0];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 1)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 1];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 2)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 2];

                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + 0] += -SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_e;
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_alphaBeta + 1)] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_alphaBeta + 2)] += -SINSstate.u_x[1] * SINSstate.Vx_0[2];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 0)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 0];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 1)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 1];
                    KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 2)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 2];

                    KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + 0] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_e;
                    KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + (iMx_alphaBeta + 1)] += -SINSstate_OdoMod.Vx_0[2];
                    KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 1] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_n;
                    KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + (iMx_alphaBeta + 0)] += SINSstate_OdoMod.Vx_0[2];
                }
            }
        }





        public static void nullingCovVerticalPart(SINS_State SINSstate, Kalman_Vars KalmanVars, int j1)
        {
            int iMx = SimpleData.iMx;
            KalmanVars.CovarianceMatrixS_m[j1 * iMx + SINSstate.value_iMx_dr3] = KalmanVars.CovarianceMatrixS_p[j1 * iMx + SINSstate.value_iMx_dr3] = 0.0;
            KalmanVars.CovarianceMatrixS_m[j1 * iMx + SINSstate.value_iMx_dV3] = KalmanVars.CovarianceMatrixS_p[j1 * iMx + SINSstate.value_iMx_dV3] = 0.0;
            KalmanVars.CovarianceMatrixS_m[j1 * iMx + SINSstate.value_iMx_f0_3] = KalmanVars.CovarianceMatrixS_p[j1 * iMx + SINSstate.value_iMx_f0_3] = 0.0;
            KalmanVars.CovarianceMatrixS_m[j1 * iMx + SINSstate.value_iMx_r_odo_3] = KalmanVars.CovarianceMatrixS_p[j1 * iMx + SINSstate.value_iMx_r_odo_3] = 0.0;

            KalmanVars.CovarianceMatrixS_m[iMx * SINSstate.value_iMx_dr3 + j1] = KalmanVars.CovarianceMatrixS_p[iMx * SINSstate.value_iMx_dr3 + j1] = 0.0;
            KalmanVars.CovarianceMatrixS_m[iMx * SINSstate.value_iMx_dV3 + j1] = KalmanVars.CovarianceMatrixS_p[iMx * SINSstate.value_iMx_dV3 + j1] = 0.0;
            KalmanVars.CovarianceMatrixS_m[iMx * SINSstate.value_iMx_f0_3 + j1] = KalmanVars.CovarianceMatrixS_p[iMx * SINSstate.value_iMx_f0_3 + j1] = 0.0;
            KalmanVars.CovarianceMatrixS_m[iMx * SINSstate.value_iMx_r_odo_3 + j1] = KalmanVars.CovarianceMatrixS_p[iMx * SINSstate.value_iMx_r_odo_3 + j1] = 0.0;
        }
        public static void DeletePerevyazkaVertikalToHorizontal(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            int iMx = SimpleData.iMx;

            for (int j1 = 0; j1 < 2; j1++)
                nullingCovVerticalPart(SINSstate, KalmanVars, j1);

            for (int j1 = SINSstate.value_iMx_dV_12; j1 < SINSstate.value_iMx_dV_12 + 2; j1++)
                nullingCovVerticalPart(SINSstate, KalmanVars, j1);

            for (int j1 = SINSstate.value_iMx_alphaBeta; j1 < SINSstate.value_iMx_alphaBeta + 3; j1++)
                nullingCovVerticalPart(SINSstate, KalmanVars, j1);

            for (int j1 = SINSstate.value_iMx_Nu0; j1 < SINSstate.value_iMx_Nu0 + 3; j1++)
                nullingCovVerticalPart(SINSstate, KalmanVars, j1);

            for (int j1 = SINSstate.value_iMx_f0_12; j1 < SINSstate.value_iMx_f0_12 + 2; j1++)
                nullingCovVerticalPart(SINSstate, KalmanVars, j1);

            for (int j1 = SINSstate.value_iMx_r_odo_12; j1 < SINSstate.value_iMx_r_odo_12 + 2; j1++)
                nullingCovVerticalPart(SINSstate, KalmanVars, j1);

            for (int j1 = SINSstate.value_iMx_kappa_3_ds + 0; j1 < SINSstate.value_iMx_kappa_3_ds + 1; j1++)
                nullingCovVerticalPart(SINSstate, KalmanVars, j1);
            //for (int j1 = SINSstate.value_iMx_kappa_13_ds; j1 < SINSstate.value_iMx_kappa_13_ds + 3; j1++)
            //    nullingCovVerticalPart(SINSstate, KalmanVars, j1);
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
            Altitude = SINSstate.Height;
            Altitude_prev = SINSstate.Height_prev;

            fz[1] = SINSstate.F_z[1];
            fz[2] = SINSstate.F_z[2];
            fz[0] = SINSstate.F_z[0];
            Wz[1] = SINSstate.W_z[1];
            Wz[2] = SINSstate.W_z[2];
            Wz[0] = SINSstate.W_z[0];

            if (SINSstate.flag_FeedbackExist)
            {
                fz[0] -= SINSstate.Cumulative_KalmanErrorVector[(SINSstate.value_iMx_f0_12 + 0)];
                fz[1] -= SINSstate.Cumulative_KalmanErrorVector[(SINSstate.value_iMx_f0_12 + 1)];

                if (SINSstate.flag_SeparateHorizVSVertical == true)
                    fz[2] -= SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_f0_3];
                else
                    fz[2] -= SINSstate.Cumulative_KalmanErrorVector[(SINSstate.value_iMx_f0_3 + 0)];

                for (int i = 0; i < 3; i++)
                    Wz[i] += SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_Nu0 + i];
            }

            if (SINSstate.flag_UseAlgebraDrift)
            {
                for (int i = 0; i < 3; i++)
                    fz[i] = fz[i] - SINSstate.AlignAlgebraZeroF[i];
                for (int i = 0; i < 3; i++)
                    Wz[i] = Wz[i] + SINSstate.AlignAlgebraDrifts[i];
            }


            CopyArray(SINSstate.F_z, fz);
            CopyArray(SINSstate.W_z, Wz);
            CopyArray(Vx_0, SINSstate.Vx_0);
            CopyArray(Vx_0_prev, SINSstate.Vx_0_prev);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);

            SINSstate.u_x = U_x0(SINSstate.Latitude);

            u[0] = 0.0;
            u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude);
            u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);


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
            if (SINSstate.flag_SeparateHorizVSVertical || SINSstate.flag_iMx_r3_dV3)
            {
                dVh = SINSstate.F_x[2] - SINSstate.g + (Vx_0[0] + Vx_0_prev[0]) / 2.0 * (2 * u[1] + SINSstate.Omega_x[1]) - (Vx_0[1] + Vx_0_prev[1]) / 2.0 * (2 * u[0] + SINSstate.Omega_x[0]);
                Vx_0[2] += dVh * SINSstate.timeStep;

                dh = (Vx_0[2] + Vx_0_prev[2]) / 2.0;
                Altitude += dh * SINSstate.timeStep;

                //SINSstate.tmp_dh_timeStep = dh * SINSstate.timeStep;
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

            SINSstate.Height_prev = SINSstate.Height;
            SINSstate.Height = Altitude;


            //SINSstate.Heading = gkurs - Azimth;
            SINSstate.Heading = Math.Atan2(SINSstate.A_sx0[1, 0], SINSstate.A_sx0[1, 1]);
            SINSstate.Roll = -Math.Atan2(SINSstate.A_sx0[0, 2], SINSstate.A_sx0[2, 2]);
            SINSstate.Pitch = Math.Atan2(SINSstate.A_sx0[1, 2], Math.Sqrt(SINSstate.A_sx0[0, 2] * SINSstate.A_sx0[0, 2] + SINSstate.A_sx0[2, 2] * SINSstate.A_sx0[2, 2]));
            //SINSstate.Azimth = Azimth;

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);
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
                    SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_sx0, SINSstate_OdoMod.A_x0s.Transpose());

                    SINSstate_OdoMod.Heading = Math.Atan2(SINSstate_OdoMod.A_sx0[1, 0], SINSstate_OdoMod.A_sx0[1, 1]);
                    SINSstate_OdoMod.Roll = -Math.Atan2(SINSstate_OdoMod.A_sx0[0, 2], SINSstate_OdoMod.A_sx0[2, 2]);
                    SINSstate_OdoMod.Pitch = Math.Atan2(SINSstate_OdoMod.A_sx0[1, 2], Math.Sqrt(SINSstate_OdoMod.A_sx0[0, 2] * SINSstate_OdoMod.A_sx0[0, 2] + SINSstate_OdoMod.A_sx0[2, 2] * SINSstate_OdoMod.A_sx0[2, 2]));

                    double[] dS_x = new double[3];
                    SimpleOperations.CopyArray(dS_x, SINSstate_OdoMod.A_x0s * SINSstate.OdometerVector);

                    // -- Используются средние значения углов ориентации между двумя моментами съема показаний одометра -- //
                    //SimpleOperations.CopyArray(dS_x, SimpleOperations.A_sx0((SINSstate_OdoMod.Heading + SINSstate_OdoMod.Heading_prev) / 2.0, 
                    //    (SINSstate_OdoMod.Roll + SINSstate_OdoMod.Roll_prev) / 2.0, (SINSstate_OdoMod.Pitch + SINSstate_OdoMod.Pitch_prev) / 2.0).Transpose() * SINSstate.OdometerVector);

                    SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude + dS_x[1] / SimpleOperations.RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height);
                    SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude + dS_x[0] / SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height) / Math.Cos(SINSstate_OdoMod.Latitude);
                    SINSstate_OdoMod.Height = SINSstate_OdoMod.Height + dS_x[2];
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

            SINSstate_OdoMod.R_e = RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height);
            SINSstate_OdoMod.R_n = RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height);
            //--------------------------------------------------------------------------------------
        }










        public static void StateIntegration_AT_ForImitator(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate2, SINS_State SINSstate_OdoMod)
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
            Altitude = SINSstate.Height;
            Altitude_prev = SINSstate.Height_prev;

            fz[1] = SINSstate.F_z[1];
            fz[2] = SINSstate.F_z[2];
            fz[0] = SINSstate.F_z[0];
            Wz[1] = SINSstate.W_z[1];
            Wz[2] = SINSstate.W_z[2];
            Wz[0] = SINSstate.W_z[0];

            CopyArray(SINSstate.F_z, fz);
            CopyArray(SINSstate.W_z, Wz);
            CopyArray(Vx_0, SINSstate.Vx_0);
            CopyArray(Vx_0_prev, SINSstate.Vx_0_prev);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);

            SINSstate.u_x = U_x0(SINSstate.Latitude);

            u[0] = 0.0;
            u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude);
            u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);




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
            if (SINSstate.flag_iMx_r3_dV3)
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

            SINSstate.Height_prev = SINSstate.Height;
            SINSstate.Height = Altitude;


            //SINSstate.Heading = gkurs - Azimth;
            SINSstate.Heading = Math.Atan2(SINSstate.A_sx0[1, 0], SINSstate.A_sx0[1, 1]);
            SINSstate.Roll = -Math.Atan2(SINSstate.A_sx0[0, 2], SINSstate.A_sx0[2, 2]);
            SINSstate.Pitch = Math.Atan2(SINSstate.A_sx0[1, 2], Math.Sqrt(SINSstate.A_sx0[0, 2] * SINSstate.A_sx0[0, 2] + SINSstate.A_sx0[2, 2] * SINSstate.A_sx0[2, 2]));
            //SINSstate.Azimth = Azimth;

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);
            SINSstate.u_x = U_x0(SINSstate.Latitude);

            CopyArray(SINSstate.Vx_0_prev, SINSstate.Vx_0);
            CopyArray(SINSstate.Vx_0, Vx_0);

            CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
            CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
            CopyArray(SINSstate.W_x, SINSstate.A_x0s * Wz);
        }






        //------------------ Функции для СГЛАЖИВАНИЯ --------------------//
        public static void FuncSmoothing_BackwardAndSmooth(SINS_State SINSstate, SINS_State SINSstate_Smooth, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, Proc_Help ProcHelp, StreamReader Back_Input_X, StreamReader Back_Input_P, StreamWriter ForHelpSmoothed)
        {
            string[] BackInputX_LineArray = Back_Input_X.ReadLine().Split(' ');
            string[] BackInputP_LineArray = Back_Input_P.ReadLine().Split(' ');

            int dim_shift_for_P = 0, dim_shift_for_X = 0;

            if (SimpleData.iMxSmthd >= 2)
            {
                int u2 = 0;
                int tmp_dim = 2
                    , dim_shift = dim_shift_for_X
                    ;

                if (SINSstate.flag_iMSmthd_Is_2_plus_Odo == true)
                    tmp_dim = 4;

                double Time_Back = SINSstate.Count;
                double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                {
                    KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                    dim_shift_for_X = u + 1;
                }


                SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                for (int u = 0; u < tmp_dim; u++)
                {
                    for (int u1 = u; u1 < tmp_dim; u1++)
                    {
                        KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                        u2++;
                    }
                }
                dim_shift_for_P += u2;

                KalmanVars.ErrorVector_m[0] = SINSstate.Latitude;
                KalmanVars.ErrorVector_m[1] = SINSstate.Longitude;

                if (SINSstate.flag_iMSmthd_Is_2_plus_Odo == true)
                {
                    KalmanVars.ErrorVector_m[2] = SINSstate_OdoMod.Latitude;
                    KalmanVars.ErrorVector_m[3] = SINSstate_OdoMod.Longitude;
                }

                Matrix MatrixS_ForNavDeltas = SimpleOperations.C_convultion_HorizontalCoordinate(SINSstate)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                        * SimpleOperations.C_convultion_HorizontalCoordinate(SINSstate).Transpose()
                                        ;
                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas), tmp_dim));

                //==============================================================//
                KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                //==============================================================//

                SINSstate_Smooth.Latitude = KalmanVars.ErrorVector_Smoothed[0];
                SINSstate_Smooth.Longitude = KalmanVars.ErrorVector_Smoothed[1];


                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    ForHelpSmoothed.WriteLine("Ошибка сглаживания в выколотой точке в момент времени " + SINSstate.Count + " = " +
                        Math.Sqrt(
                            Math.Pow((ProcHelp.LatSNS * SimpleData.ToRadian - KalmanVars.ErrorVector_m[0]) * SINSstate.R_n, 2)
                            + Math.Pow((ProcHelp.LongSNS * SimpleData.ToRadian - KalmanVars.ErrorVector_m[1]) * SINSstate.R_e * Math.Cos(KalmanVars.ErrorVector_m[0]), 2)
                            )
                        );
                }
            }

            if (SimpleData.iMxSmthd >= 4)
            {
                int u2 = 0;
                int tmp_dim = 2
                    , dim_shift = dim_shift_for_X
                    ;
                double Time_Back = SINSstate.Count;
                double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                {
                    KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                    dim_shift_for_X = u + 1;
                }

                SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                for (int u = 0; u < tmp_dim; u++)
                {
                    for (int u1 = u; u1 < tmp_dim; u1++)
                    {
                        KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                        u2++;
                    }
                }
                dim_shift_for_P += u2;

                KalmanVars.ErrorVector_m[0] = SINSstate.Vx_0[0];
                KalmanVars.ErrorVector_m[1] = SINSstate.Vx_0[1];

                Matrix MatrixS_ForNavDeltas = new Matrix(tmp_dim, tmp_dim);
                MatrixS_ForNavDeltas = SimpleOperations.C_convultion_HorizontalVelocity(SINSstate)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                        * SimpleOperations.C_convultion_HorizontalVelocity(SINSstate).Transpose()
                                        ;
                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas), tmp_dim));

                //==============================================================//
                KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                //==============================================================//

                SINSstate_Smooth.Vx_0[0] = KalmanVars.ErrorVector_Smoothed[0];
                SINSstate_Smooth.Vx_0[1] = KalmanVars.ErrorVector_Smoothed[1];
            }


            if (SimpleData.iMxSmthd >= 7)
            {
                int u2 = 0;
                int tmp_dim = 3
                    , dim_shift = dim_shift_for_X
                    ;
                double Time_Back = SINSstate.Count;
                double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                {
                    KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                    dim_shift_for_X = u + 1;
                }


                SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                for (int u = 0; u < tmp_dim; u++)
                {
                    for (int u1 = u; u1 < tmp_dim; u1++)
                    {
                        KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                        u2++;
                    }
                }
                dim_shift_for_P += u2;

                KalmanVars.ErrorVector_m[0] = SINSstate.Pitch;
                KalmanVars.ErrorVector_m[1] = SINSstate.Roll;
                KalmanVars.ErrorVector_m[2] = SINSstate.Heading;

                if (SINSstate.Pitch - KalmanVars.ErrorVector_Straight[0] > Math.PI) KalmanVars.ErrorVector_Straight[0] += 2 * Math.PI;
                if (SINSstate.Pitch - KalmanVars.ErrorVector_Straight[0] < -Math.PI) KalmanVars.ErrorVector_Straight[0] -= 2 * Math.PI;
                if (SINSstate.Roll - KalmanVars.ErrorVector_Straight[1] > Math.PI) KalmanVars.ErrorVector_Straight[1] += 2 * Math.PI;
                if (SINSstate.Roll - KalmanVars.ErrorVector_Straight[1] < -Math.PI) KalmanVars.ErrorVector_Straight[1] -= 2 * Math.PI;
                if (SINSstate.Heading - KalmanVars.ErrorVector_Straight[2] > Math.PI) KalmanVars.ErrorVector_Straight[2] += 2 * Math.PI;
                if (SINSstate.Heading - KalmanVars.ErrorVector_Straight[2] < -Math.PI) KalmanVars.ErrorVector_Straight[2] -= 2 * Math.PI;

                Matrix MatrixS_ForNavDeltas = new Matrix(tmp_dim, tmp_dim);
                MatrixS_ForNavDeltas = SimpleOperations.C_convultion_Angles(SINSstate)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                        * SimpleOperations.C_convultion_Angles(SINSstate).Transpose()
                                        ;
                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas), tmp_dim));

                //==============================================================//
                KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                //==============================================================//

                SINSstate_Smooth.Pitch = KalmanVars.ErrorVector_Smoothed[0];
                SINSstate_Smooth.Roll = KalmanVars.ErrorVector_Smoothed[1];
                SINSstate_Smooth.Heading = KalmanVars.ErrorVector_Smoothed[2];
            }


            //-----------------------------------------------------------------
            //------------------СГЛАЖИВАНИЕ KAPPA_1----------------------------
            //-----------------------------------------------------------------
            {
                int u2 = 0;
                int tmp_dim = 1, dim_shift = dim_shift_for_X;

                double Time_Back = SINSstate.Count;
                double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                {
                    KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                    dim_shift_for_X = u + 1;
                }

                SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                for (int u = 0; u < tmp_dim; u++)
                {
                    for (int u1 = u; u1 < tmp_dim; u1++)
                    {
                        KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                        u2++;
                    }
                }
                dim_shift_for_P += u2;

                if (SINSstate.flag_SeparateHorizVSVertical == true)
                    KalmanVars.ErrorVector_m[0] = SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_kappa1];
                else
                    KalmanVars.ErrorVector_m[0] = SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_1];

                Matrix MatrixS_For_VerticalCoordinate = new Matrix(tmp_dim, tmp_dim);
                if (SINSstate.flag_SeparateHorizVSVertical == false)
                    MatrixS_For_VerticalCoordinate = SimpleOperations.C_convultion_OdoKappa_1(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_OdoKappa_1(SINSstate).Transpose();
                else
                    MatrixS_For_VerticalCoordinate = SimpleOperations.C_convultion_OdoKappa_1(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p)
                            * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_OdoKappa_1(SINSstate).Transpose();

                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_VerticalCoordinate), tmp_dim));

                //==============================================================//
                KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                //==============================================================//

                SINSstate_Smooth.Cumulative_KalmanErrorVector[Math.Max(SINSstate.value_iMx_kappa_1, SINSstate.Vertical_kappa1)] = KalmanVars.ErrorVector_Smoothed[0];
            }

            //-----------------------------------------------------------------
            //------------------СГЛАЖИВАНИЕ KAPPA_3----------------------------
            //-----------------------------------------------------------------
            {
                int u2 = 0;
                int tmp_dim = 1, dim_shift = dim_shift_for_X;

                double Time_Back = SINSstate.Count;
                double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                {
                    KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                    dim_shift_for_X = u + 1;
                }

                SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                for (int u = 0; u < tmp_dim; u++)
                {
                    for (int u1 = u; u1 < tmp_dim; u1++)
                    {
                        KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                        u2++;
                    }
                }
                dim_shift_for_P += u2;

                KalmanVars.ErrorVector_m[0] = SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_3_ds + 0];

                Matrix MatrixS_For_VerticalCoordinate = new Matrix(tmp_dim, tmp_dim);
                MatrixS_For_VerticalCoordinate = SimpleOperations.C_convultion_OdoKappa_3(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_OdoKappa_3(SINSstate).Transpose();

                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_VerticalCoordinate), tmp_dim));

                //==============================================================//
                KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                //==============================================================//

                SINSstate_Smooth.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_3_ds + 0] = KalmanVars.ErrorVector_Smoothed[0];
            }

            //-----------------------------------------------------------------
            //------------------СГЛАЖИВАНИЕ SCALE FACTOR----------------------------
            //-----------------------------------------------------------------
            {
                int u2 = 0;
                int tmp_dim = 1, dim_shift = dim_shift_for_X;

                double Time_Back = SINSstate.Count;
                double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                {
                    KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                    dim_shift_for_X = u + 1;
                }

                SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                for (int u = 0; u < tmp_dim; u++)
                {
                    for (int u1 = u; u1 < tmp_dim; u1++)
                    {
                        KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                        u2++;
                    }
                }
                dim_shift_for_P += u2;

                KalmanVars.ErrorVector_m[0] = SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_3_ds + 1];

                Matrix MatrixS_For_VerticalCoordinate = new Matrix(tmp_dim, tmp_dim);
                MatrixS_For_VerticalCoordinate = SimpleOperations.C_convultion_ScaleError(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_ScaleError(SINSstate).Transpose();

                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_VerticalCoordinate), tmp_dim));

                //==============================================================//
                KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                //==============================================================//

                SINSstate_Smooth.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_3_ds + 1] = KalmanVars.ErrorVector_Smoothed[0];
            }




            //===================Vertical Channel===================//
            if (SINSstate.flag_iMx_r3_dV3 == true || SINSstate.flag_SeparateHorizVSVertical == true)
            {
                if (SimpleData.iMxSmthd >= 2)
                {
                    int u2 = 0;
                    int tmp_dim = 1,
                        dim_shift = dim_shift_for_X
                        ;

                    double Time_Back = SINSstate.Count;
                    double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                    for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                    {
                        KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                        dim_shift_for_X = u + 1;
                    }


                    SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                    for (int u = 0; u < tmp_dim; u++)
                    {
                        for (int u1 = u; u1 < tmp_dim; u1++)
                        {
                            KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                            u2++;
                        }
                    }
                    dim_shift_for_P += u2;

                    KalmanVars.ErrorVector_m[0] = SINSstate.Height;

                    Matrix MatrixS_ForNavDeltas = new Matrix(tmp_dim, tmp_dim);
                    if (SINSstate.flag_SeparateHorizVSVertical == false)
                        MatrixS_ForNavDeltas = SimpleOperations.C_convultion_VerticalCoordinate(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_VerticalCoordinate(SINSstate).Transpose();
                    else
                        MatrixS_ForNavDeltas = SimpleOperations.C_convultion_VerticalCoordinate(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_VerticalCoordinate(SINSstate).Transpose();

                    SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas), tmp_dim));

                    //==============================================================//
                    KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                    //==============================================================//

                    SINSstate_Smooth.Height = KalmanVars.ErrorVector_Smoothed[0];
                }

                if (SimpleData.iMxSmthd >= 4)
                {
                    int u2 = 0;
                    int tmp_dim = 1,
                        dim_shift = dim_shift_for_X
                        ;

                    double Time_Back = SINSstate.Count;
                    double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                    for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                    {
                        KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                        dim_shift_for_X = u + 1;
                    }


                    SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                    for (int u = 0; u < tmp_dim; u++)
                    {
                        for (int u1 = u; u1 < tmp_dim; u1++)
                        {
                            KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                            u2++;
                        }
                    }
                    dim_shift_for_P += u2;

                    KalmanVars.ErrorVector_m[0] = SINSstate.Vx_0[2];

                    Matrix MatrixS_ForNavDeltas = new Matrix(tmp_dim, tmp_dim);
                    if (SINSstate.flag_SeparateHorizVSVertical == false)
                        MatrixS_ForNavDeltas = SimpleOperations.C_convultion_VerticalVelocity(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_VerticalVelocity(SINSstate).Transpose();
                    else
                        MatrixS_ForNavDeltas = SimpleOperations.C_convultion_VerticalVelocity(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_VerticalVelocity(SINSstate).Transpose();

                    SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_ForNavDeltas), tmp_dim));

                    //==============================================================//
                    KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                    //==============================================================//

                    SINSstate_Smooth.Vx_0[2] = KalmanVars.ErrorVector_Smoothed[0];
                }


                //-----------------------------------------------------------------
                //------------------СГЛАЖИВАНИЕ dF0_3----------------------------
                //-----------------------------------------------------------------
                {
                    int u2 = 0;
                    int tmp_dim = 1, dim_shift = dim_shift_for_X;

                    double Time_Back = SINSstate.Count;
                    double Time_Streight = Convert.ToDouble(BackInputX_LineArray[0]);
                    for (int u = 0 + dim_shift; u < tmp_dim + dim_shift; u++)
                    {
                        KalmanVars.ErrorVector_Straight[u - dim_shift] = Convert.ToDouble(BackInputX_LineArray[u + 1]);
                        dim_shift_for_X = u + 1;
                    }

                    SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrix_SP_Straight);
                    for (int u = 0; u < tmp_dim; u++)
                    {
                        for (int u1 = u; u1 < tmp_dim; u1++)
                        {
                            KalmanVars.CovarianceMatrix_SP_Straight[u * tmp_dim + u1] = Convert.ToDouble(BackInputP_LineArray[u2 + dim_shift_for_P]);
                            u2++;
                        }
                    }
                    dim_shift_for_P += u2;

                    if (SINSstate.flag_SeparateHorizVSVertical == true)
                        KalmanVars.ErrorVector_m[0] = SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_f0_3];
                    else
                        KalmanVars.ErrorVector_m[0] = SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_f0_3];

                    Matrix MatrixS_For_VerticalCoordinate = new Matrix(tmp_dim, tmp_dim);
                    if (SINSstate.flag_SeparateHorizVSVertical == false)
                        MatrixS_For_VerticalCoordinate = SimpleOperations.C_convultion_dF0_3(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_dF0_3(SINSstate).Transpose();
                    else
                        MatrixS_For_VerticalCoordinate = SimpleOperations.C_convultion_dF0_3(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_dF0_3(SINSstate).Transpose();

                    SimpleOperations.CopyArray(KalmanVars.CovarianceMatrix_SP_m, KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_VerticalCoordinate), tmp_dim));

                    //==============================================================//
                    KalmanProcs.Smoothing(KalmanVars, SINSstate, tmp_dim);
                    //==============================================================//

                    SINSstate_Smooth.Cumulative_KalmanErrorVector[Math.Max(SINSstate.value_iMx_f0_3, SINSstate.Vertical_f0_3)] = KalmanVars.ErrorVector_Smoothed[0];
                }
            }
        }



        public static void FuncSmoothing_Forward(int i, SINS_State SINSstate, SINS_State SINSstate_Smooth, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, Proc_Help ProcHelp
            , StreamWriter Smthing_X, StreamWriter Smthing_P, StreamWriter Smthing_Backward)
        {
            int EachN_CountOutput = 100;

            //------------------------------------------------------------------
            // --- Расчет параметров на прямом прогоне и их вывод в файлы для обратного
            string str_P = "";
            double[] CovarianceMatrix_SP_Straight_1 = new double[1],
                CovarianceMatrix_SP_Straight_2 = new double[2],
                CovarianceMatrix_SP_Straight_3 = new double[3],
                CovarianceMatrix_SP_Straight_4 = new double[4]
                ;

            //------------------------------------------------------------------
            //---Делаем свертку до S_X по плановым координатам---
            if (SimpleData.iMxSmthd >= 2)
            {
                int tmp_dim = 2;

                if (SINSstate.flag_iMSmthd_Is_2_plus_Odo == true) 
                    tmp_dim = 4;

                Matrix MatrixS_For_HorizontalCoordinate = SimpleOperations.C_convultion_HorizontalCoordinate(SINSstate)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                            * SimpleOperations.C_convultion_HorizontalCoordinate(SINSstate).Transpose();

                double[] CovarianceMatrix_SP_Straight = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_HorizontalCoordinate), tmp_dim);

                for (int ii = 0; ii < tmp_dim; ii++)
                    for (int ji = ii; ji < tmp_dim; ji++)
                        str_P += CovarianceMatrix_SP_Straight[ii * tmp_dim + ji].ToString() + " ";
            }

            //------------------------------------------------------------------
            // --- Делаем свертку до S_X по плановым скоростям---
            if (SimpleData.iMxSmthd >= 4)
            {
                int tmp_dim = 2;
                Matrix MatrixS_For_HorizontalVelocity = new Matrix(tmp_dim, tmp_dim);
                MatrixS_For_HorizontalVelocity = SimpleOperations.C_convultion_HorizontalVelocity(SINSstate)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                        * SimpleOperations.C_convultion_HorizontalVelocity(SINSstate).Transpose();

                SimpleOperations.NullingOfArray(CovarianceMatrix_SP_Straight_2);
                CovarianceMatrix_SP_Straight_2 = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_HorizontalVelocity), tmp_dim);

                for (int ii = 0; ii < tmp_dim; ii++)
                    for (int ji = ii; ji < tmp_dim; ji++)
                        str_P += CovarianceMatrix_SP_Straight_2[ii * tmp_dim + ji].ToString() + " ";
            }

            //------------------------------------------------------------------
            // --- Делаем свертку до S_X по углам ориентации---
            if (SimpleData.iMxSmthd >= 7)
            {
                int tmp_dim = 3;
                Matrix MatrixS_For_Angles = new Matrix(tmp_dim, tmp_dim);
                MatrixS_For_Angles = SimpleOperations.C_convultion_Angles(SINSstate)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                        * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose()
                                        * SimpleOperations.C_convultion_Angles(SINSstate).Transpose();

                SimpleOperations.NullingOfArray(CovarianceMatrix_SP_Straight_3);
                CovarianceMatrix_SP_Straight_3 = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_Angles), tmp_dim);

                for (int ii = 0; ii < tmp_dim; ii++)
                    for (int ji = ii; ji < tmp_dim; ji++)
                        str_P += CovarianceMatrix_SP_Straight_3[ii * tmp_dim + ji].ToString() + " ";
            }

            //------------------------------------------------------------------
            // --- Делаем свертку до S_X по Kappa_1 угловой ошибке установки БИНС ---
            {
                int tmp_dim = 1;
                Matrix MatrixS_For_Kappa_1 = new Matrix(tmp_dim, tmp_dim);
                if (SINSstate.flag_SeparateHorizVSVertical == true)
                    MatrixS_For_Kappa_1 = SimpleOperations.C_convultion_OdoKappa_1(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p)
                            * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_OdoKappa_1(SINSstate).Transpose();
                else
                    MatrixS_For_Kappa_1 = SimpleOperations.C_convultion_OdoKappa_1(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_OdoKappa_1(SINSstate).Transpose();

                SimpleOperations.NullingOfArray(CovarianceMatrix_SP_Straight_1);
                CovarianceMatrix_SP_Straight_1 = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_Kappa_1), tmp_dim);

                str_P += CovarianceMatrix_SP_Straight_1[0].ToString() + " ";
            }
            // --- Делаем свертку до S_X по Kappa_3 угловой ошибке установки БИНС ---
            {
                int tmp_dim = 1;
                Matrix MatrixS_For_Kappa_3 = new Matrix(tmp_dim, tmp_dim);
                MatrixS_For_Kappa_3 = SimpleOperations.C_convultion_OdoKappa_3(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_OdoKappa_3(SINSstate).Transpose();

                SimpleOperations.NullingOfArray(CovarianceMatrix_SP_Straight_1);
                CovarianceMatrix_SP_Straight_1 = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_Kappa_3), tmp_dim);

                str_P += CovarianceMatrix_SP_Straight_1[0].ToString() + " ";
            }
            // --- Делаем свертку до S_X по ошибке масштаба одометр---
            {
                int tmp_dim = 1;
                Matrix MatrixS_For_Scale = new Matrix(tmp_dim, tmp_dim);
                MatrixS_For_Scale = SimpleOperations.C_convultion_ScaleError(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                            * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_ScaleError(SINSstate).Transpose();

                SimpleOperations.NullingOfArray(CovarianceMatrix_SP_Straight_1);
                CovarianceMatrix_SP_Straight_1 = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_Scale), tmp_dim);

                str_P += CovarianceMatrix_SP_Straight_1[0].ToString() + " ";
            }
            //------------------------------------------------------------------



            //------------------------------------------------------------------
            // --- Делаем свертку до S_X по высоте---
            if (SINSstate.flag_iMx_r3_dV3 == true || SINSstate.flag_SeparateHorizVSVertical == true)
            {
                if (SimpleData.iMxSmthd >= 2)
                {
                    int tmp_dim = 1;
                    Matrix MatrixS_For_VerticalCoordinate = new Matrix(tmp_dim, tmp_dim);
                    if (SINSstate.flag_SeparateHorizVSVertical == false)
                        MatrixS_For_VerticalCoordinate = SimpleOperations.C_convultion_VerticalCoordinate(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_VerticalCoordinate(SINSstate).Transpose();
                    else
                        MatrixS_For_VerticalCoordinate = SimpleOperations.C_convultion_VerticalCoordinate(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_VerticalCoordinate(SINSstate).Transpose();

                    SimpleOperations.NullingOfArray(CovarianceMatrix_SP_Straight_1);
                    CovarianceMatrix_SP_Straight_1 = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_VerticalCoordinate), tmp_dim);

                    str_P += CovarianceMatrix_SP_Straight_1[0].ToString() + " ";
                }
                // --- Делаем свертку до S_X по вертикальной скорости---
                if (SimpleData.iMxSmthd >= 4)
                {
                    int tmp_dim = 1;
                    Matrix MatrixS_For_VerticalVelocity = new Matrix(tmp_dim, tmp_dim);
                    if (SINSstate.flag_SeparateHorizVSVertical == false)
                        MatrixS_For_VerticalVelocity = SimpleOperations.C_convultion_VerticalVelocity(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_VerticalVelocity(SINSstate).Transpose();
                    else
                        MatrixS_For_VerticalVelocity = SimpleOperations.C_convultion_VerticalVelocity(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_VerticalVelocity(SINSstate).Transpose();

                    SimpleOperations.NullingOfArray(CovarianceMatrix_SP_Straight_1);
                    CovarianceMatrix_SP_Straight_1 = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_VerticalVelocity), tmp_dim);

                    str_P += CovarianceMatrix_SP_Straight_1[0].ToString() + " ";
                }

                // --- Делаем свертку до S_X по нулю вертикального ньютонометра ---
                {
                    int tmp_dim = 1;
                    Matrix MatrixS_For_Vertical_F0 = new Matrix(tmp_dim, tmp_dim);
                    if (SINSstate.flag_SeparateHorizVSVertical == false)
                        MatrixS_For_Vertical_F0 = SimpleOperations.C_convultion_dF0_3(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_dF0_3(SINSstate).Transpose();
                    else
                        MatrixS_For_Vertical_F0 = SimpleOperations.C_convultion_dF0_3(SINSstate) * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p)
                                * SimpleOperations.ArrayToMatrix(KalmanVars.Vertical_CovarianceMatrixS_p).Transpose() * SimpleOperations.C_convultion_dF0_3(SINSstate).Transpose();

                    SimpleOperations.NullingOfArray(CovarianceMatrix_SP_Straight_1);
                    CovarianceMatrix_SP_Straight_1 = KalmanProcs.rsb_rsb(SimpleOperations.MatrixToArray(MatrixS_For_Vertical_F0), tmp_dim);

                    str_P += CovarianceMatrix_SP_Straight_1[0].ToString() + " ";
                }
            }


            if (i % EachN_CountOutput == 0)
            {
                SINSstate.SmoothingOutput_str_P += str_P;
                Smthing_P.WriteLine(SINSstate.SmoothingOutput_str_P);
                SINSstate.SmoothingOutput_str_P = "";
            }
            else
                SINSstate.SmoothingOutput_str_P += str_P + "\n";
            //-----------------------



            string str_X = "";
            if (SimpleData.iMxSmthd >= 2)
            {
                str_X = SINSstate.Count + " " + SINSstate.Latitude + " " + SINSstate.Longitude;

                if (SINSstate.flag_iMSmthd_Is_2_plus_Odo == true)
                    str_X += " " + SINSstate_OdoMod.Latitude + " " + SINSstate_OdoMod.Longitude;
            }
            if (SimpleData.iMxSmthd >= 4)
                str_X = str_X + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[1];
            if (SimpleData.iMxSmthd >= 7)
                str_X = str_X + " " + SINSstate.Pitch + " " + SINSstate.Roll + " " + SINSstate.Heading;

            // --- Вывод комулятивных оценок модели ошибок одометра ТОЛЬКО ДЛЯ ОБРЫТНЫХ СВЯЗЕЙ --- //
            if (SINSstate.flag_SeparateHorizVSVertical == true)
                str_X += " " + SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_kappa1];
            else
                str_X += " " + SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_1];
            str_X += " " + SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_3_ds + 0];
            str_X += " " + SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_kappa_3_ds + 1];

            if (SINSstate.flag_iMx_r3_dV3 == true || SINSstate.flag_SeparateHorizVSVertical == true)
            {
                if (SimpleData.iMxSmthd >= 2)
                    str_X = str_X + " " + SINSstate.Height;

                if (SimpleData.iMxSmthd >= 4)
                    str_X = str_X + " " + SINSstate.Vx_0[2];

                // --- Вывод комулятивной оценки нуля вертикального ньютоноетра ТОЛЬКО ДЛЯ ОБРЫТНЫХ СВЯЗЕЙ --- //
                if (SINSstate.flag_SeparateHorizVSVertical == true)
                    str_X += " " + SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_f0_3];
                else
                    str_X += " " + SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_f0_3];
            }

            if (i % EachN_CountOutput == 0)
            {
                SINSstate.SmoothingOutput_str_X += str_X;
                Smthing_X.WriteLine(SINSstate.SmoothingOutput_str_X);
                SINSstate.SmoothingOutput_str_X = "";
            }
            else
                SINSstate.SmoothingOutput_str_X += str_X + "\n";
            //-----------------------


            //-----------------------------------------------
            // --- Важный момент - на обратном проходу нужно использовать именно те координаты одометрического счисления, которые были расчитаны на прямом прогоне
            string StringForBack = "";
            StringForBack = ProcHelp.datastring + " " + SINSstate_OdoMod.Latitude.ToString() + " " + SINSstate_OdoMod.Longitude.ToString();

            if (i % EachN_CountOutput == 0)
            {
                SINSstate.SmoothingOutput_strForBack += StringForBack;
                Smthing_Backward.WriteLine(SINSstate.SmoothingOutput_strForBack);
                SINSstate.SmoothingOutput_strForBack = "";
            }
            else
                SINSstate.SmoothingOutput_strForBack += StringForBack + "\n";
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
