using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class SINSprocessing : SimpleOperations
    {
        public static double dVh_global = 0.0;
        public static int Can = 0;

        public static void Redifinition_OdoCounts(SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod, ParamsForModel OdoModel)
        {
            if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                SINSstate.OdometerLeftPrev_2 = SINSstate.OdometerData.odometer_left.Value;
                SINSstate.OdometerRightPrev_2 = SINSstate.OdometerData.odometer_right.Value;
                SINSstate.OdoSpeedPrev_2 = OdoModel.V_odo;
                SINSstate.OdoTimeStepCount_2 = 0;

                

                if (SINSstate.flag_UsingCorrection == true || SINSstate.flag_Odometr_SINS_case == true || SINSstate.flag_UsingOdoPosition == true || SINSstate.flag_UseOnlyStops == true || SINSstate.flag_slipping == true)
                {
                    SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                    SINSstate.OdoSpeedPrev = OdoModel.V_odo;
                    SINSstate.OdoTimeStepCount = 0;

                    SINSstate.odotime_prev = SINSstate.Time;

                    SINSstate.Latitude_prev = SINSstate.Latitude; SINSstate2.Latitude_prev = SINSstate2.Latitude;
                    SINSstate.Longitude_prev = SINSstate.Longitude; SINSstate2.Longitude_prev = SINSstate2.Longitude;
                    SINSstate.Altitude_prev = SINSstate.Altitude; SINSstate2.Altitude_prev = SINSstate2.Altitude;
                }
            }

            if (SINSstate.flag_FeedbackExist == false) 
                SimpleOperations.CopyArray(SINSstate_OdoMod.Vx_0_prev, SINSstate_OdoMod.Vx_0);
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

            //SimpleOperations.CopyArray(SINSstate.Omega_x, SINSstate.Omega_x);
            SINSstate.Omega_x[0] = -SINSstate.Vx_0[1] / SINSstate.R_n;
            SINSstate.Omega_x[1] = SINSstate.Vx_0[0] / SINSstate.R_e;
            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * SINSstate.Omega_x[1];

            SINSstate.g = 9.78049 * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014;
            SINSstate.g -= 2 * 0.000001538 *SINSstate.Altitude;
        }


        public static void CalcStateErrorsEasySINS(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate_OdoMod, SINS_State SINSstateDinamOdo)
        {
            SINSstate.DeltaLatitude = ErrorVector[1] / SINSstate.R_n;
            SINSstate.DeltaLongitude = ErrorVector[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

            SINSstate.DeltaV_1 = ErrorVector[2] + SINSstate.Vx_0[1] * ErrorVector[6];// + SINSstate.Vx_0[1] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
            SINSstate.DeltaV_2 = ErrorVector[3] - SINSstate.Vx_0[0] * ErrorVector[6];// -SINSstate.Vx_0[0] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);

            if (SINSstate.flag_iMx_r3_dV3)
            {
                SINSstate.DeltaAltitude = ErrorVector[SINSstate.iMx_r3_dV3];
                SINSstate.DeltaV_3 = ErrorVector[SINSstate.iMx_r3_dV3 + 1] + SINSstate.Vx_0[0] * (ErrorVector[5] - ErrorVector[0] / SINSstate.R_e) - SINSstate.Vx_0[1] * (ErrorVector[4] + ErrorVector[1] / SINSstate.R_n);
            }

            SINSstate.DeltaRoll = -(ErrorVector[4] * Math.Sin(SINSstate.Heading) + ErrorVector[5] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
            SINSstate.DeltaPitch = -ErrorVector[4] * Math.Cos(SINSstate.Heading) + ErrorVector[5] * Math.Sin(SINSstate.Heading);
            SINSstate.DeltaHeading = ErrorVector[6] + SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude) + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);
        }


        public static void CalcStateErrors(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate_OdoMod, SINS_State SINSstateDinamOdo)
        {
            SINSstate.DeltaLatitude = ErrorVector[1] / SINSstate.R_n;
            SINSstate.DeltaLongitude = ErrorVector[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

            SINSstate.DeltaV_1 = ErrorVector[2] + SINSstate.Vx_0[1] * ErrorVector[6];// + SINSstate.Vx_0[1] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);
            SINSstate.DeltaV_2 = ErrorVector[3] - SINSstate.Vx_0[0] * ErrorVector[6];// -SINSstate.Vx_0[0] * SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude);

            //SimpleOperations.PrintVectorToFile(ErrorVector, SimpleData.iMx);

            if (SINSstate.flag_iMx_r3_dV3)
            {
                SINSstate.DeltaAltitude = ErrorVector[SINSstate.iMx_r3_dV3];

                SINSstate.DeltaV_1 += SINSstate.Vx_0[2] * (ErrorVector[0] / SINSstate.R_e - ErrorVector[5]);
                SINSstate.DeltaV_2 += SINSstate.Vx_0[2] * (ErrorVector[1] / SINSstate.R_n - ErrorVector[4]);
                SINSstate.DeltaV_3 = ErrorVector[SINSstate.iMx_r3_dV3 + 1] + SINSstate.Vx_0[0] * (ErrorVector[5] - ErrorVector[0] / SINSstate.R_e) - SINSstate.Vx_0[1] * (ErrorVector[4] + ErrorVector[1] / SINSstate.R_n);
            }

            SINSstate.DeltaRoll = -(ErrorVector[4] * Math.Sin(SINSstate.Heading) + ErrorVector[5] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
            SINSstate.DeltaPitch = -ErrorVector[4] * Math.Cos(SINSstate.Heading) + ErrorVector[5] * Math.Sin(SINSstate.Heading);
            SINSstate.DeltaHeading = ErrorVector[6] + SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude) + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);

            if (SINSstate.flag_FeedbackExist && SINSstate.flag_iMx_kappa_13_ds)
            {
                SINSstate.ComulativeKappaEst[0] += ErrorVector[SINSstate.iMx_odo_model + 0];
                SINSstate.ComulativeKappaEst[2] += ErrorVector[SINSstate.iMx_odo_model + 1];
                SINSstate.ComulativeKappaEst[1] += ErrorVector[SINSstate.iMx_odo_model + 2];

                //--- Коррекция весовых матриц в случае обрытных связей без уравнений ошибок одометра ---//
                if (SINSstate.flag_Odometr_SINS_case == false)
                {
                    double[] d_2 = new double[3];
                    Matrix matr_1 = new Matrix(3, 3), matr_2 = new Matrix(3, 3);
                    double[] vect_1 = new double[3], vect_2 = new double[3], vect_3 = new double[3], vect_result = new double[3];

                    for (int u = 0; u < 3; u++) 
                        d_2[u] = SINSstate.A_x0s[u, 1];

                    vect_1[0] = ErrorVector[4]; vect_1[1] = ErrorVector[5]; vect_1[2] = ErrorVector[6] + ErrorVector[0] / SINSstate.R_e * Math.Tan(SINSstate.Latitude);
                    vect_2[0] = ErrorVector[SINSstate.iMx_odo_model + 1]; vect_2[1] = ErrorVector[SINSstate.iMx_odo_model + 2]; vect_2[2] = -ErrorVector[SINSstate.iMx_odo_model + 0];

                    CopyMatrix(matr_1, ErrorVector[SINSstate.iMx_odo_model + 2] * SINSstate.OdometerVector[1] * SINSstate.A_x0s + ErrorVector[SINSstate.iMx_odo_model + 2] * SINSstate.Ds_ComulativeByOdoTrack);
                    CopyMatrix(matr_2, SINSstate.OdometerVector[1] * Matrix.SkewSymmetricMatrix(vect_1) * SINSstate.A_x0s + Matrix.SkewSymmetricMatrix(vect_1) * SINSstate.Ds_ComulativeByOdoTrack);
                    //--- Корректируем весовую матрицу Ds_ComulativeByOdoTrack ---//
                    CopyMatrix(SINSstate.Ds_ComulativeByOdoTrack, SINSstate.Ds_ComulativeByOdoTrack - matr_1 - matr_2);

                    CopyMatrix(matr_1, ErrorVector[SINSstate.iMx_odo_model + 2] * SINSstate.OdometerVector[1] * Matrix.SkewSymmetricMatrix(d_2) + ErrorVector[SINSstate.iMx_odo_model + 2] * SINSstate.Ds2_ComulativeByOdoTrack);
                    CopyMatrix(matr_2, SINSstate.OdometerVector[1] * Matrix.SkewSymmetricMatrix(Matrix.SkewSymmetricMatrix(vect_1) * d_2) + Matrix.SkewSymmetricMatrix(vect_1) * SINSstate.Ds2_ComulativeByOdoTrack);
                    //--- Корректируем весовую матрицу Ds2_ComulativeByOdoTrack ---//
                    CopyMatrix(SINSstate.Ds2_ComulativeByOdoTrack, SINSstate.Ds2_ComulativeByOdoTrack - matr_1 - matr_2);


                    SimpleOperations.CopyArray(vect_result, SINSstate.Ds2_ComulativeByOdoTrack * vect_1);
                    SimpleOperations.CopyArray(vect_3, SINSstate.Ds_ComulativeByOdoTrack * vect_2);

                    //------------------------------------------------------------//

                    SINSstate_OdoMod.DeltaLatitude = (-vect_result[1] + vect_3[1]) / SINSstate.R_n;
                    SINSstate_OdoMod.DeltaLongitude = (-vect_result[0] + vect_3[0]) / SINSstate.R_e / Math.Cos(SINSstate_OdoMod.Latitude);
                }
            }



             //---Случай Одометр+БИНС. Обратная связть.---
            if (SINSstate.flag_Odometr_SINS_case == true)
            {
                SINSstateDinamOdo.DeltaLatitude = ErrorVector[SINSstate.iMx_r12_odo + 1] / SINSstateDinamOdo.R_n;
                SINSstateDinamOdo.DeltaLongitude = ErrorVector[SINSstate.iMx_r12_odo + 0] / SINSstateDinamOdo.R_e / Math.Cos(SINSstateDinamOdo.Latitude);

                if (SINSstate.flag_Using_iMx_r_odo_3)
                {
                    SINSstateDinamOdo.DeltaAltitude = ErrorVector[SINSstate.iMx_r12_odo + 2];
                }

                SINSstateDinamOdo.DeltaRoll = -(ErrorVector[4] * Math.Sin(SINSstateDinamOdo.Heading) + ErrorVector[5] * Math.Cos(SINSstateDinamOdo.Heading)) / Math.Cos(SINSstateDinamOdo.Pitch);
                SINSstateDinamOdo.DeltaPitch = -ErrorVector[4] * Math.Cos(SINSstateDinamOdo.Heading) + ErrorVector[5] * Math.Sin(SINSstateDinamOdo.Heading);
                SINSstateDinamOdo.DeltaHeading = ErrorVector[6] + SINSstateDinamOdo.DeltaLongitude * Math.Sin(SINSstateDinamOdo.Latitude) + SINSstateDinamOdo.DeltaRoll * Math.Sin(SINSstateDinamOdo.Pitch);
            }       
        }



        public static void StateCorrection(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod, SINS_State SINSstateDinamOdo)
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



            //---Суммируем инструментальные ошибки---
            if (SINSstate.flag_FeedbackExist && SINSstate.flag_DoFeedBackDeltaFW)
            {
                for (int i = 0; i < 3; i++)
                {
                    SINSstate2.ComulativeInstrumental_Fz[i] += ErrorVector[7 + i];
                    SINSstate2.ComulativeInstrumental_Wz[i] += ErrorVector[10 + i];
                }
            }



            //---Случай Одометр+БИНС. Обратная связть.---
            if (SINSstate.flag_Odometr_SINS_case == true)
            {
                SINSstateDinamOdo.Latitude_Corr = SINSstateDinamOdo.Latitude - SINSstateDinamOdo.DeltaLatitude;
                SINSstateDinamOdo.Longitude_Corr = SINSstateDinamOdo.Longitude - SINSstateDinamOdo.DeltaLongitude;
                if (SINSstate.flag_Using_iMx_r_odo_3)
                    SINSstateDinamOdo.Altitude_Corr = SINSstateDinamOdo.Altitude - SINSstateDinamOdo.DeltaAltitude;

                if (SINSstate.flag_FeedbackExist == true)
                {
                    SINSstateDinamOdo.Latitude = SINSstateDinamOdo.Latitude - SINSstateDinamOdo.DeltaLatitude;
                    SINSstateDinamOdo.Longitude = SINSstateDinamOdo.Longitude - SINSstateDinamOdo.DeltaLongitude;
                    if (SINSstate.flag_Using_iMx_r_odo_3)
                        SINSstateDinamOdo.Altitude = SINSstateDinamOdo.Altitude - SINSstateDinamOdo.DeltaAltitude;

                    //---Обратная связь для СЛАБОСВЯЗНОГО случая---
                    if (SINSstate.flag_OdoSINSWeakConnect)
                    {
                        SINSstateDinamOdo.Vx_0[0] = SINSstateDinamOdo.Vx_0[0] - SINSstate.DeltaV_1;
                        SINSstateDinamOdo.Vx_0[1] = SINSstateDinamOdo.Vx_0[1] - SINSstate.DeltaV_2;
                        if (SINSstate.flag_Using_iMx_r_odo_3)
                            SINSstateDinamOdo.Vx_0[2] = SINSstateDinamOdo.Vx_0[2] - SINSstate.DeltaV_3;

                        SINSstateDinamOdo.Roll -= SINSstate.DeltaRoll;
                        SINSstateDinamOdo.Pitch -= SINSstate.DeltaPitch;
                        SINSstateDinamOdo.Heading -= SINSstate.DeltaHeading;
                    }
                    //---Обратная связь для МОДИФИЦИРОВАННОГО случая---
                    if (SINSstate.flag_OdoSINSWeakConnect_MODIF)
                    {
                        SINSstateDinamOdo.Roll -= SINSstateDinamOdo.DeltaRoll;
                        SINSstateDinamOdo.Pitch -= SINSstateDinamOdo.DeltaPitch;
                        SINSstateDinamOdo.Heading -= SINSstateDinamOdo.DeltaHeading;
                    }

                    SINSprocessing.ApplyCompensatedErrorsToSolution(SINSstateDinamOdo);
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
                //---зануляем только если имеется соответсвующая настройка запуска---
                if (SINSstate.flag_DoFeedBackDeltaFW == false && i >= 7 && i <= 12)
                    continue;

                //if (SINSstate.flag_Odometr_SINS_case == false && (i == SINSstate.iMx_odo_model || i == SINSstate.iMx_odo_model + 1 || i == SINSstate.iMx_odo_model + 2) && SINSstate.flag_iMx_kappa_13_ds)
                //    continue;

                //if ((i == SINSstate.iMx_r12_odo || i == SINSstate.iMx_r12_odo + 1 || (i == SINSstate.iMx_r12_odo + 2 && SINSstate.flag_Using_iMx_r_odo_3)) && SINSstate.flag_Odometr_SINS_case)
                //    continue;

                double tmp = KalmanVars.ErrorConditionVector_p[i];
                KalmanVars.ErrorConditionVector_p[i] = 0.0;
                KalmanVars.ErrorConditionVector_m[i] = 0.0;
            }
        }








        public static void Gauss_Kruger(SINS_State SINSstate)
        {
            double[] X_et = new double[5], Y_et = new double[5], H_et = new double[5];
            X_et[0] = 5978265.75; X_et[1] = 5979838.67; X_et[2] = 5979140.272;
            Y_et[0] = 5555526.02; Y_et[1] = 5555406.746; Y_et[2] = 5555362.311;
            H_et[0] = 195.0; H_et[1] = 217.084; H_et[2] = 210.397;

            int i, iN;
            for (i = 0; i < 3; i++)
            {
                Phi_Lambda_GAUSS_KRUGER(X_et[i], Y_et[i], SINSstate, i);
            }

            iN = (int)(Y_et[0] * 1.0e-6);
            SINSstate.DirectionalAngle = 280 * 0.06 + (SINSstate.Longitude / 3.141592 * 180.0 - 6.0 * (iN - 0.5)) * Math.Sin(SINSstate.Latitude);
        }

        public static void Phi_Lambda_GAUSS_KRUGER(double dX, double dY, SINS_State SINSstate, int i)
        {
            // INPUT:  dX -> X (Gauss-Kruger) Coordinate
            // INPUT:  dY -> X (Gauss-Kruger) Coordinate
            // OUTPUT:  *dPhi -> Latitude, *dLambda -> Longitude
            //    dPhi and dLambda in Radians !!!
            int iN;
            double dSinPhi, dCosPhi, dSin2Phi, dSin4Phi, dSin6Phi;
            double dBeta, dSinBeta, dSin2Beta, dSin4Beta, dZ0, dZ02, dPhi0;
            double dDeltaPhi, dL;
            iN = (int)(dY * 1.0e-6);
            dBeta = dX / 6367558.4968;
            dSinBeta = Math.Sin(dBeta);
            dSin2Beta = dSinBeta * dSinBeta;
            dSin4Beta = dSin2Beta * dSin2Beta;
            dPhi0 = dBeta + Math.Sin(2.0 * dBeta) * (0.00252588685 -
            0.00001491860 * dSin2Beta + 0.00000011904 * dSin4Beta);

            dCosPhi = Math.Cos(dPhi0);
            dSinPhi = Math.Sin(dPhi0);
            dSin2Phi = dSinPhi * dSinPhi;
            dSin4Phi = dSin2Phi * dSin2Phi;
            dSin6Phi = dSin4Phi * dSin2Phi;

            dZ0 = (dY - (10.0 * iN + 5.0) * 1.0e5) / (6378245.0 * dCosPhi);
            dZ02 = dZ0 * dZ0;

            dDeltaPhi = -dZ02 * Math.Sin(2.0 * dPhi0) * (0.251684631 -
            0.003369263 * dSin2Phi + 0.000011276 * dSin4Phi -
            dZ02 * (0.10500614 - 0.04559916 * dSin2Phi + 0.00228901 * dSin4Phi - 0.00002987 * dSin6Phi -
            dZ02 * (0.042858 - 0.025318 * dSin2Phi + 0.014346 * dSin4Phi - 0.001264 * dSin6Phi -
            dZ02 * (0.01672 - 0.00630 * dSin2Phi + 0.01188 * dSin4Phi - 0.00328 * dSin6Phi))));

            dL = dZ0 * (1.0 - 0.0033467108 * dSin2Phi - 0.0000056002 * dSin4Phi - 0.0000000187 * dSin6Phi -
            dZ02 * (0.16778975 + 0.16273586 * dSin2Phi - 0.00052490 * dSin4Phi - 0.00000846 * dSin6Phi -
            dZ02 * (0.0420025 + 0.1487407 * dSin2Phi + 0.0059420 * dSin4Phi - 0.0000150 * dSin6Phi -
            dZ02 * (0.01225 + 0.09477 * dSin2Phi + 0.03282 * dSin4Phi - 0.00034 * dSin6Phi -
            dZ02 * (0.0038 + 0.0524 * dSin2Phi + 0.0482 * dSin4Phi + 0.0032 * dSin6Phi)))));

            SINSstate.GK_Latitude[i] = dPhi0 + dDeltaPhi;
            //*dLambda = 6.0*( iN - 0.0)/ 57.29577951 + dL;
            SINSstate.GK_Longitude[i] = 6.0 * (iN - 0.5) / 57.29577951 + dL;
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

            KalmanVars.CovarianceMatrixS_m[4 * SimpleData.iMx + 4] = KalmanVars.CovarianceMatrixS_p[4 * SimpleData.iMx + 4] = Math.Max(SINSstate.stdAlpha12, 1E-3);  // 5 угл. минут
            KalmanVars.CovarianceMatrixS_m[5 * SimpleData.iMx + 5] = KalmanVars.CovarianceMatrixS_p[5 * SimpleData.iMx + 5] = Math.Max(SINSstate.stdAlpha12, 1E-3);
            KalmanVars.CovarianceMatrixS_m[6 * SimpleData.iMx + 6] = KalmanVars.CovarianceMatrixS_p[6 * SimpleData.iMx + 6] = Math.Max(SINSstate.stdBeta3, 1E-3);

            KalmanVars.CovarianceMatrixS_m[7 * SimpleData.iMx + 7] = KalmanVars.CovarianceMatrixS_p[7 * SimpleData.iMx + 7] = Math.Max(SINSstate.stdNu * SimpleData.ToRadian / 3600.0, 1E-6); //0.2 * SimpleData.ToRadian / 3600.0; // 0.2 град/час
            KalmanVars.CovarianceMatrixS_m[8 * SimpleData.iMx + 8] = KalmanVars.CovarianceMatrixS_p[8 * SimpleData.iMx + 8] = Math.Max(SINSstate.stdNu * SimpleData.ToRadian / 3600.0, 1E-6);//0.2 * SimpleData.ToRadian / 3600.0;
            KalmanVars.CovarianceMatrixS_m[9 * SimpleData.iMx + 9] = KalmanVars.CovarianceMatrixS_p[9 * SimpleData.iMx + 9] = Math.Max(SINSstate.stdNu * SimpleData.ToRadian / 3600.0, 1E-6);//0.2 * SimpleData.ToRadian / 3600.0;

            KalmanVars.CovarianceMatrixS_m[10 * SimpleData.iMx + 10] = KalmanVars.CovarianceMatrixS_p[10 * SimpleData.iMx + 10] = Math.Max(SINSstate.stdF, 1E-4);    // м/с^2
            KalmanVars.CovarianceMatrixS_m[11 * SimpleData.iMx + 11] = KalmanVars.CovarianceMatrixS_p[11 * SimpleData.iMx + 11] = Math.Max(SINSstate.stdF, 1E-4);
            KalmanVars.CovarianceMatrixS_m[12 * SimpleData.iMx + 12] = KalmanVars.CovarianceMatrixS_p[12 * SimpleData.iMx + 12] = Math.Max(SINSstate.stdF, 1E-4);

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



            if (false && SINSstate.flag_Odometr_SINS_case == false)
            {
                for (int i = 0; i < SimpleData.iMx; i++)
                    pdP[i * SimpleData.iMx + i] = KalmanVars.CovarianceMatrixS_m[i * SimpleData.iMx + i] * KalmanVars.CovarianceMatrixS_m[i * SimpleData.iMx + i];

                pdP[4 * SimpleData.iMx + 4] = 1.001 * pdP[10 * SimpleData.iMx + 10] / 9.78 / 9.78;

                pdP[4 * SimpleData.iMx + 10] = pdP[10 * SimpleData.iMx + 4] = -pdP[10 * SimpleData.iMx + 10] / 9.78 * SINSstate.A_sx0[1, 1];
                pdP[4 * SimpleData.iMx + 11] = pdP[11 * SimpleData.iMx + 4] = -pdP[10 * SimpleData.iMx + 10] / 9.78 * SINSstate.A_sx0[2, 1];
                pdP[4 * SimpleData.iMx + 12] = pdP[12 * SimpleData.iMx + 4] = -pdP[10 * SimpleData.iMx + 10] / 9.78 * SINSstate.A_sx0[0, 1];
                pdP[5 * SimpleData.iMx + 5] = 1.001 * pdP[10 * SimpleData.iMx + 10] / 9.78 / 9.78;
                pdP[5 * SimpleData.iMx + 10] = pdP[10 * SimpleData.iMx + 5] = pdP[10 * SimpleData.iMx + 10] / 9.78 * SINSstate.A_sx0[1, 0];
                pdP[5 * SimpleData.iMx + 11] = pdP[11 * SimpleData.iMx + 5] = pdP[10 * SimpleData.iMx + 10] / 9.78 * SINSstate.A_sx0[2, 0];
                pdP[5 * SimpleData.iMx + 12] = pdP[12 * SimpleData.iMx + 5] = pdP[10 * SimpleData.iMx + 10] / 9.78 * SINSstate.A_sx0[0, 0];

                pdP[6 * SimpleData.iMx + 6] = 1.001 * pdP[7 * SimpleData.iMx + 7] / SimpleData.U * Math.Cos(SINSstate.Latitude) / SimpleData.U * Math.Cos(SINSstate.Latitude);
                pdP[6 * SimpleData.iMx + 7] = pdP[7 * SimpleData.iMx + 6] = pdP[7 * SimpleData.iMx + 7] / SimpleData.U * Math.Cos(SINSstate.Latitude) * SINSstate.A_sx0[1, 0];
                pdP[6 * SimpleData.iMx + 8] = pdP[8 * SimpleData.iMx + 6] = pdP[7 * SimpleData.iMx + 7] / SimpleData.U * Math.Cos(SINSstate.Latitude) * SINSstate.A_sx0[2, 0];
                pdP[6 * SimpleData.iMx + 9] = pdP[9 * SimpleData.iMx + 6] = pdP[7 * SimpleData.iMx + 7] / SimpleData.U * Math.Cos(SINSstate.Latitude) * SINSstate.A_sx0[0, 0];

                KalmanVars.CovarianceMatrixS_m = KalmanProcs.rsb_rsb(pdP, SimpleData.iMx);
                KalmanVars.CovarianceMatrixS_p = KalmanProcs.rsb_rsb(pdP, SimpleData.iMx);
            }
        }






        public static int MatrixNoise_ReDef(SINS_State SINSstate, Kalman_Vars KalmanVars, bool AlignmentFLG)
        {
            int iMx = SimpleData.iMx, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;

            double[] Noise_Vel_in_Mx = new double[3], Noise_Angl_in_Mx = new double[3];

            for (int i = 0; i < iMx * iMq; i++)
                KalmanVars.CovarianceMatrixNoise[i] = 0.0;

            for (int j = 0; j < 3; j++)
            {
                // выбираем, где для каких осей инструментальных шумов формируется матрица шумов //
                Noise_Vel_in_Mx[j] = Math.Sqrt(SINSstate.A_x0s[j, 0] * SINSstate.A_x0s[j, 0] * KalmanVars.Noise_Vel[0] * KalmanVars.Noise_Vel[0] +
                                               SINSstate.A_x0s[j, 1] * SINSstate.A_x0s[j, 1] * KalmanVars.Noise_Vel[1] * KalmanVars.Noise_Vel[1] +
                                               SINSstate.A_x0s[j, 2] * SINSstate.A_x0s[j, 2] * KalmanVars.Noise_Vel[2] * KalmanVars.Noise_Vel[2]);
                Noise_Angl_in_Mx[j] = Math.Sqrt(SINSstate.A_x0s[j, 0] * SINSstate.A_x0s[j, 0] * KalmanVars.Noise_Angl[0] * KalmanVars.Noise_Angl[0] +
                                               SINSstate.A_x0s[j, 1] * SINSstate.A_x0s[j, 1] * KalmanVars.Noise_Angl[1] * KalmanVars.Noise_Angl[1] +
                                               SINSstate.A_x0s[j, 2] * SINSstate.A_x0s[j, 2] * KalmanVars.Noise_Angl[2] * KalmanVars.Noise_Angl[2]);

            }
            double sqrt_freq = Math.Sqrt(Math.Abs(SINSstate.Freq));
            //sqrt_freq = 1.0;


            int tmpCounter = 0;
            if (SINSstate.flag_iMqDeltaR)
            {
                KalmanVars.CovarianceMatrixNoise[0 * iMq + tmpCounter + 0] = KalmanVars.Noise_Pos * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[1 * iMq + tmpCounter + 1] = KalmanVars.Noise_Pos * sqrt_freq;
                tmpCounter = tmpCounter + 2;
            }

            // так как в векторе состояния дрейфы в проекции на приборные оси, надо задавать соответственно матрицу шумов //
            KalmanVars.CovarianceMatrixNoise[2 * iMq + tmpCounter + 0] = Noise_Vel_in_Mx[0] * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[2 * iMq + tmpCounter + 2] = SINSstate.Vx_0[1] * Noise_Angl_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[3 * iMq + tmpCounter + 1] = Noise_Vel_in_Mx[1] * sqrt_freq;
            //KalmanVars.CovarianceMatrixNoise[3 * iMq + tmpCounter + 3] = SINSstate.Vx_0[0] * Noise_Angl_in_Mx[1] * sqrt_freq;
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

            double W_z_abs, Omega_x_abs, dlt, dlt2, Altitude, Altitude_prev, dh, dVx, dVy, dVh;
            double kren, tang, gkurs, Azimth;

            CopyMatrix(AT_z_xi, SINSstate.AT);
            CopyMatrix(B_x_eta, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            //C_eta_xi = Matrix.DoA_eta_xi(SINSstate.Time);
            Altitude = SINSstate.Altitude;
            Altitude_prev = SINSstate.Altitude_prev;
            //Azimth = SINSstate.Azimth;

            fz[1] = SINSstate.F_z[1];
            fz[2] = SINSstate.F_z[2];
            fz[0] = SINSstate.F_z[0];
            Wz[1] = SINSstate.W_z[1];
            Wz[2] = SINSstate.W_z[2];
            Wz[0] = SINSstate.W_z[0];

            if (SINSstate.flag_FeedbackExist && SINSstate.flag_DoFeedBackDeltaFW)
            {
                for (int i=0; i < 3; i++)
                {
                    fz[i] -= SINSstate.ComulativeInstrumental_Fz[i];
                    Wz[i] -= SINSstate.ComulativeInstrumental_Wz[i];
                }
            }


            CopyArray(SINSstate.F_z, fz);
            CopyArray(SINSstate.W_z, Wz);
            CopyArray(Vx_0, SINSstate.Vx_0);
            CopyArray(Vx_0_prev, SINSstate.Vx_0_prev);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);

            SINSstate.u_x = U_x0(SINSstate.Latitude);

            //u[0] = SimpleData.U * Math.Cos(SINSstate.Latitude) * Math.Sin(Azimth);
            //u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude) * Math.Cos(Azimth);
            //u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);

            u[0] = 0.0;
            u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude);
            u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);

            if (SINSstate.flag_UseAlgebraDrift)
                for (int i = 0; i < 3; i++)
                    Wz[i] = Wz[i] - SINSstate.AlignAlgebraDrifts[i];



            //-------------ИНТЕГРИРОВАНИЕ МАТРИЦЫ AT_Z_XI И ПЕРВОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z---------
            if (true)
            {

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
            }
            else //Эйлера с пересчетом
            {
                Hat1 = Matrix.SkewSymmetricMatrix(Wz);
                Hat2 = Matrix.SkewSymmetricMatrix(SINSstate.W_z_prev);

                CopyMatrix(dMatrix, (E + (Hat1 + Hat2) * SINSstate.timeStep / 2.0 + (Hat1 * Hat2) * SINSstate.timeStep * SINSstate.timeStep / 2.0));
                CopyMatrix(AT_z_xi, (dMatrix * AT_z_xi));

                //Ортогональность
                //AT_z_xi[0, 0] = AT_z_xi[1, 1] * AT_z_xi[2, 2] - AT_z_xi[2, 1] * AT_z_xi[1, 2];
                //AT_z_xi[1, 0] = AT_z_xi[2, 1] * AT_z_xi[0, 2] - AT_z_xi[0, 1] * AT_z_xi[2, 2];
                //AT_z_xi[2, 0] = AT_z_xi[0, 1] * AT_z_xi[1, 2] - AT_z_xi[0, 2] * AT_z_xi[1, 1];

                //Нормировка
                //for (int i = 0; i < 3; i++)
                //{
                //    tempV[i] = Math.Sqrt(AT_z_xi[i, 0] * AT_z_xi[i, 0] + AT_z_xi[i, 1] * AT_z_xi[i, 1] + AT_z_xi[i, 2] * AT_z_xi[i, 2]);
                //    for (int j = 0; j < 3; j++)
                //        AT_z_xi[i, j] = AT_z_xi[i, j] / tempV[i];
                //}

                CopyMatrix(SINSstate.AT, AT_z_xi);

                CopyMatrix(W_x_xi, B_x_eta * SINSstate.A_nxi);
                CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());
                //--------------------------------------------------------------------------------------
            }




            //---------------------------------ИНТЕГРИРОВАНИЕ СКОРОСТЕЙ----------------------------
            CopyArray(SINSstate.F_x, D_x_z * fz);

            //--- надо вычислять, используется, например в выставке ---//
            SINSstate.g = 9.78049 * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014;
            if (true)
                SINSstate.g -= 2 * 0.000001538 * Altitude;

            if (SINSstate.flag_Alignment == true)
            {
                dVx = SINSstate.F_x[0] + Vx_0[1] * (2.0 * u[2] + SINSstate.Omega_x[2]);
                dVy = SINSstate.F_x[1] - Vx_0[0] * (2.0 * u[2] + SINSstate.Omega_x[2]);
            }
            else
            {
                dVx = SINSstate.F_x[0] + Vx_0[1] * (2.0 * u[2] + SINSstate.Omega_x[2]) - Vx_0[2] * (2.0 * u[1] + SINSstate.Omega_x[1]);
                dVy = SINSstate.F_x[1] - Vx_0[0] * (2.0 * u[2] + SINSstate.Omega_x[2]) + Vx_0[2] * (2.0 * u[0] + SINSstate.Omega_x[0]);
            }
            Vx_0[0] += dVx * SINSstate.timeStep;
            Vx_0[1] += dVy * SINSstate.timeStep;

            //--------------------------------------------------------------------------------------

            //double GG = SimpleData.Gravity_Normal * (1 + 0.005317099 * Math.Sin(SINSstate.Latitude) * Math.Sin(SINSstate.Latitude)) * SimpleData.A * SimpleData.A / (SimpleData.A + SINSstate.Altitude) / (SimpleData.A + SINSstate.Altitude);

            if (SINSstate.flag_iMx_r3_dV3 && (SINSstate.flag_UsingAltitudeCorrection || SINSstate.flag_Using_SNS))
            {
                dVh = SINSstate.F_x[2] - SINSstate.g + (Vx_0[0] + Vx_0_prev[0]) / 2.0 * (2 * u[1] + SINSstate.Omega_x[1]) - (Vx_0[1] + Vx_0_prev[1]) / 2.0 * (2 * u[0] + SINSstate.Omega_x[0]);
                Vx_0[2] += dVh * SINSstate.timeStep;

                dh = (Vx_0[2] + Vx_0_prev[2]) / 2.0;
                Altitude += dh * SINSstate.timeStep;
            }





            //Сюда заходим если установлен флажок при автономном счислении ИЛИ объект есть чисто одометрическое решение.
            SimpleOperations.CopyArray(SINSstate.OdoSpeed_x0, D_x_z * SINSstate.OdoSpeed_s);
            if (SINSstate.flag_OdoSINSWeakConnect_MODIF && (SINSstate.flag_Autonomous_Solution && SINSstate.flag_autonomous_dinamic_mode))
            {
                SimpleOperations.CopyArray(Vx_0, SINSstate.OdoSpeed_x0);
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







            if (SINSstate.flag_OdoSINSWeakConnect && (SINSstate.flag_Autonomous_Solution && SINSstate.flag_autonomous_dinamic_mode))
            {
                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    double[] dS_x = new double[3];
                    SimpleOperations.CopyArray(dS_x, SINSstate.A_x0s * SINSstate.OdometerVector);

                    SINSstate.Latitude = SINSstate.Latitude + dS_x[1] / SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);
                    SINSstate.Longitude = SINSstate.Longitude + dS_x[0] / SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude) / Math.Cos(SINSstate.Latitude);
                    SINSstate.Altitude = SINSstate.Altitude + dS_x[2];
                }
            }
            else
            {
                //---ОПРЕДЕЛЕНИЕ ГЕОГРАФИЧЕСКИХ КООРДИНАТ---
                SINSstate.Longitude = Math.Atan2(B_x_eta[2, 1], B_x_eta[2, 0]);
                SINSstate.Latitude = Math.Atan2(B_x_eta[2, 2], Math.Sqrt(B_x_eta[0, 2] * B_x_eta[0, 2] + B_x_eta[1, 2] * B_x_eta[1, 2]));
                Azimth = Math.Atan2(B_x_eta[0, 2], B_x_eta[1, 2]);

                if (SINSstate.flag_OdoSINSWeakConnect_MODIF && (SINSstate.flag_Autonomous_Solution && SINSstate.flag_autonomous_dinamic_mode))
                {
                    if (SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        double[] dS_x = new double[3];
                        SimpleOperations.CopyArray(dS_x, SINSstate.A_x0s * SINSstate.OdometerVector);

                        SINSstate.Altitude = SINSstate.Altitude + dS_x[2];
                    }
                }
                else
                {
                    SINSstate.Altitude_prev = SINSstate.Altitude;
                    SINSstate.Altitude = Altitude;
                }
            }



            // определение углов курса,крена,тангажа
            kren = -Math.Atan2(SINSstate.A_sx0[0, 2], SINSstate.A_sx0[2, 2]);
            tang = Math.Atan2(SINSstate.A_sx0[1, 2], Math.Sqrt(SINSstate.A_sx0[0, 2] * SINSstate.A_sx0[0, 2] + SINSstate.A_sx0[2, 2] * SINSstate.A_sx0[2, 2]));
            gkurs = Math.Atan2(SINSstate.A_sx0[1, 0], SINSstate.A_sx0[1, 1]);

            //SINSstate.Heading = gkurs - Azimth;
            SINSstate.Heading = gkurs;
            SINSstate.Roll = kren;
            SINSstate.Pitch = tang;
            //SINSstate.Azimth = Azimth;

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.u_x = U_x0(SINSstate.Latitude);

            CopyArray(SINSstate.W_x, SINSstate.A_x0s * Wz);

            CopyArray(SINSstate.Vx_0_prev, SINSstate.Vx_0);
            CopyArray(SINSstate.Vx_0, Vx_0);
            CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
            CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
            //CopyArray(SINSstate.W_z, Wz);
            //--------------------------------------------------------------------------------------


            //---ВЫЧИСЛЯЕМ СКОРОСТНЫЕ УГЛЫ---//
            double[] Vx_0_temp = new double[3];
            if ((SINSstate.flag_FeedbackExist == true && SimpleOperations.AbsoluteVectorValue(SINSstate.Vx_0) > 2.0) ||
                (SINSstate.flag_FeedbackExist == false && SimpleOperations.AbsoluteVectorValue(SINSstate2.Vx_0) > 2.0))
            {
                if (SINSstate.flag_FeedbackExist == true) SimpleOperations.CopyArray(Vx_0_temp, SINSstate.Vx_0);
                if (SINSstate.flag_FeedbackExist == false) SimpleOperations.CopyArray(Vx_0_temp, SINSstate2.Vx_0);

                SINSstate.CoursePitch = Math.Atan2(Vx_0_temp[2], Math.Sqrt(Vx_0_temp[0] * Vx_0_temp[0] + Vx_0_temp[1] * Vx_0_temp[1]));
                SINSstate.CourseHeading = Math.Atan2(Vx_0_temp[0], Vx_0_temp[1]);
            }
            else
            {
                SINSstate.CoursePitch = SINSstate.Pitch;
                SINSstate.CourseHeading = SINSstate.Heading;
            }
            SINSstate.A_cx0 = SimpleOperations.A_cx0(SINSstate); SINSstate.A_x0c = SINSstate.A_cx0.Transpose();
            SimpleOperations.CopyMatrix(SINSstate.A_sc, SINSstate.A_sx0 * SINSstate.A_x0c);
            SINSstate.alpha_c = -Math.Atan2(SINSstate.A_sc[2, 1], SINSstate.A_sc[1, 1]);
            SINSstate.gamma_c = -Math.Atan2(SINSstate.A_sc[0, 2], SINSstate.A_sc[0, 0]);
            SINSstate.beta_c = Math.Atan2(SINSstate.A_sc[0, 1] * Math.Cos(SINSstate.gamma_c), SINSstate.A_sc[0, 0]);

        }





        public static void SlipageProcessing(SINS_State SINSstate, SINS_State SINSstate2, Kalman_Vars KalmanVars, System.IO.StreamWriter SlippageLog, int temp_cnt_V_more)
        {
            SINSstate.flag_slipping = false;
            string str;
            double V_sins_mod = 0.0, V_odo_mod = SimpleOperations.AbsoluteVectorValue(SINSstate.OdoSpeed_x0);

            if (SINSstate.flag_FeedbackExist == true) V_sins_mod = SimpleOperations.AbsoluteVectorValue(SINSstate.Vx_0);
            if (SINSstate.flag_FeedbackExist == false) V_sins_mod = SimpleOperations.AbsoluteVectorValue(SINSstate2.Vx_0);

            str = SINSstate.Count + " " + (SINSstate.Count * Math.Abs(SINSstate.timeStep)) + " " + V_sins_mod + " " + V_odo_mod;

            if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                if (Math.Abs(V_sins_mod - V_odo_mod) > KalmanVars.OdoNoise_V)
                {
                    SINSstate.flag_UsingCorrection = false;
                    SINSstate.flag_slipping = true;

                    temp_cnt_V_more++;
                    if (temp_cnt_V_more > 1) str = str + " 3";
                    else str = str + " 1";
                }
                else { temp_cnt_V_more = 0; str = str + " 0"; }
            }
            else
            {
                if (temp_cnt_V_more == 0) str = str + " 0";
                else if (temp_cnt_V_more > 1) str = str + " 3";
                else str = str + " 1";
            }
            SlippageLog.WriteLine(str);
        }
    }
}
