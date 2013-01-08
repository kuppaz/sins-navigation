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

        public static void InitOfCovarianceMatrixes(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            for (int i = 0; i < SimpleData.iMx * SimpleData.iMx; i++)
                KalmanVars.CovarianceMatrixS_m[i] = KalmanVars.CovarianceMatrixS_p[i] = 0.0;

            KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0] = KalmanVars.CovarianceMatrixS_p[0 * SimpleData.iMx + 0] = 1.0;    // позиционные ошибки
            KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1] = KalmanVars.CovarianceMatrixS_p[1 * SimpleData.iMx + 1] = 1.0;
            //

            KalmanVars.CovarianceMatrixS_m[2 * SimpleData.iMx + 2] = KalmanVars.CovarianceMatrixS_p[2 * SimpleData.iMx + 2] = 0.1;   // 0.01 м/с
            KalmanVars.CovarianceMatrixS_m[3 * SimpleData.iMx + 3] = KalmanVars.CovarianceMatrixS_p[3 * SimpleData.iMx + 3] = 0.1;
            //

            KalmanVars.CovarianceMatrixS_m[4 * SimpleData.iMx + 4] = KalmanVars.CovarianceMatrixS_p[4 * SimpleData.iMx + 4] = 5.0 * SimpleData.ToRadian_min;  // 5 угл. минут
            KalmanVars.CovarianceMatrixS_m[5 * SimpleData.iMx + 5] = KalmanVars.CovarianceMatrixS_p[5 * SimpleData.iMx + 5] = 5.0 * SimpleData.ToRadian_min;
            KalmanVars.CovarianceMatrixS_m[6 * SimpleData.iMx + 6] = KalmanVars.CovarianceMatrixS_p[6 * SimpleData.iMx + 6] = 5.0 * SimpleData.ToRadian_min;

            KalmanVars.CovarianceMatrixS_m[7 * SimpleData.iMx + 7] = KalmanVars.CovarianceMatrixS_p[7 * SimpleData.iMx + 7] = 0.00000001;//0.2 * SimpleData.ToRadian / 3600.0; // 0.2 град/час
            KalmanVars.CovarianceMatrixS_m[8 * SimpleData.iMx + 8] = KalmanVars.CovarianceMatrixS_p[8 * SimpleData.iMx + 8] = 0.00000001;//0.2 * SimpleData.ToRadian / 3600.0;
            KalmanVars.CovarianceMatrixS_m[9 * SimpleData.iMx + 9] = KalmanVars.CovarianceMatrixS_p[9 * SimpleData.iMx + 9] = 0.00000001;//0.2 * SimpleData.ToRadian / 3600.0;

            KalmanVars.CovarianceMatrixS_m[10 * SimpleData.iMx + 10] = KalmanVars.CovarianceMatrixS_p[10 * SimpleData.iMx + 10] = 0.0001;    // м/с^2
            KalmanVars.CovarianceMatrixS_m[11 * SimpleData.iMx + 11] = KalmanVars.CovarianceMatrixS_p[11 * SimpleData.iMx + 11] = 0.0001;
            KalmanVars.CovarianceMatrixS_m[12 * SimpleData.iMx + 12] = KalmanVars.CovarianceMatrixS_p[12 * SimpleData.iMx + 12] = 0.0001;

            if (SINSstate.iMx_r3_dV3)
            {
                KalmanVars.CovarianceMatrixS_m[SINSstate.value_iMx_r3_dV3 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3] = 
                    KalmanVars.CovarianceMatrixS_p[SINSstate.value_iMx_r3_dV3 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3] = 1.0;
                KalmanVars.CovarianceMatrixS_m[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + (SINSstate.value_iMx_r3_dV3 + 1)] =
                    KalmanVars.CovarianceMatrixS_p[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + (SINSstate.value_iMx_r3_dV3 + 1)] = 0.1;
            }

            if (SINSstate.iMx_r_odo_12)
            {
                KalmanVars.CovarianceMatrixS_m[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12] =
                    KalmanVars.CovarianceMatrixS_p[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12] = 1.0;
                KalmanVars.CovarianceMatrixS_m[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + (SINSstate.value_iMx_r_odo_12 + 1)] =
                    KalmanVars.CovarianceMatrixS_p[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + (SINSstate.value_iMx_r_odo_12 + 1)] = 1.1;
            }

            if (SINSstate.iMx_kappa_13_ds)
            {
                KalmanVars.CovarianceMatrixS_m[SINSstate.value_iMx_kappa_13_ds * SimpleData.iMx + SINSstate.value_iMx_kappa_13_ds] =
                    KalmanVars.CovarianceMatrixS_p[SINSstate.value_iMx_kappa_13_ds * SimpleData.iMx + SINSstate.value_iMx_kappa_13_ds] = 3.0 * SimpleData.ToRadian_min;
                KalmanVars.CovarianceMatrixS_m[(SINSstate.value_iMx_kappa_13_ds + 1) * SimpleData.iMx + (SINSstate.value_iMx_kappa_13_ds + 1)] =
                    KalmanVars.CovarianceMatrixS_p[(SINSstate.value_iMx_kappa_13_ds + 1) * SimpleData.iMx + (SINSstate.value_iMx_kappa_13_ds + 1)] = 3.0 * SimpleData.ToRadian_min;
                KalmanVars.CovarianceMatrixS_m[(SINSstate.value_iMx_kappa_13_ds + 2) * SimpleData.iMx + (SINSstate.value_iMx_kappa_13_ds + 2)] =
                    KalmanVars.CovarianceMatrixS_p[(SINSstate.value_iMx_kappa_13_ds + 2) * SimpleData.iMx + (SINSstate.value_iMx_kappa_13_ds + 2)] = 3.0 * SimpleData.ToRadian_min;
            }
  
        }

        public static void Gauss_Kruger(SINS_State SINSstate)
        {
            double[] X_et = new double[5], Y_et = new double[5], H_et = new double[5];
            X_et[0] = 6242517.022; X_et[1] = 6246022.58; X_et[2] = 6244147.105; X_et[3] = 6246022.58; X_et[4] = 6242517.022;
            Y_et[0] = 8381534.806; Y_et[1] = 8384204.691; Y_et[2] = 8383519.943; Y_et[3] = 8384204.691; Y_et[4] = 8381534.806;
            H_et[0] = 104.832; H_et[1] = 101.804; H_et[2] = 97.121; H_et[3] = 101.804; H_et[4] = 104.832;

            int i, iN;
            for (i = 0; i < 5; i++)
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
            dZ02 * (0.01672 - 0.00630 * dSin2Phi + 0.01188 * dSin4Phi - 0.00328 * dSin6Phi)
            )));


            dL = dZ0 * (1.0 - 0.0033467108 * dSin2Phi - 0.0000056002 * dSin4Phi - 0.0000000187 * dSin6Phi -
            dZ02 * (0.16778975 + 0.16273586 * dSin2Phi - 0.00052490 * dSin4Phi - 0.00000846 * dSin6Phi -
            dZ02 * (0.0420025 + 0.1487407 * dSin2Phi + 0.0059420 * dSin4Phi - 0.0000150 * dSin6Phi -
            dZ02 * (0.01225 + 0.09477 * dSin2Phi + 0.03282 * dSin4Phi - 0.00034 * dSin6Phi -
            dZ02 * (0.0038 + 0.0524 * dSin2Phi + 0.0482 * dSin4Phi + 0.0032 * dSin6Phi)
            ))));

            SINSstate.GK_Latitude[i] = dPhi0 + dDeltaPhi;
            //*dLambda = 6.0*( iN - 0.0)/ 57.29577951 + dL;
            SINSstate.GK_Longitude[i] = 6.0 * (iN - 0.5) / 57.29577951 + dL;
        }

        public static void NullingOfCorrectedErrors(Kalman_Vars KalmanVars)
        {
            for (int i = 0; i < SimpleData.iMx; i++)
            {
                KalmanVars.ErrorConditionVector_p[i] = 0.0;
                KalmanVars.ErrorConditionVector_m[i] = 0.0;
            }
        }

        public static void MatrixNoise_ReDef(SINS_State SINSstate, Kalman_Vars KalmanVars, bool AlignmentFLG)
        {
            for (int i = 0; i < SimpleData.iMx * SimpleData.iMq; i++)
                KalmanVars.CovarianceMatrixNoise[i] = 0.0;


            KalmanVars.CovarianceMatrixNoise[0 * SimpleData.iMq + 0] = KalmanVars.Noise_Pos;
            KalmanVars.CovarianceMatrixNoise[1 * SimpleData.iMq + 1] = KalmanVars.Noise_Pos;
            KalmanVars.CovarianceMatrixNoise[2 * SimpleData.iMq + 2] = KalmanVars.Noise_Vel;
            KalmanVars.CovarianceMatrixNoise[3 * SimpleData.iMq + 3] = KalmanVars.Noise_Vel;
            KalmanVars.CovarianceMatrixNoise[4 * SimpleData.iMq + 4] = KalmanVars.Noise_Angl;
            KalmanVars.CovarianceMatrixNoise[5 * SimpleData.iMq + 5] = KalmanVars.Noise_Angl;
            KalmanVars.CovarianceMatrixNoise[6 * SimpleData.iMq + 6] = KalmanVars.Noise_Angl;
            KalmanVars.CovarianceMatrixNoise[7 * SimpleData.iMq + 7] = KalmanVars.Noise_Drift;
            KalmanVars.CovarianceMatrixNoise[8 * SimpleData.iMq + 8] = KalmanVars.Noise_Drift;
            KalmanVars.CovarianceMatrixNoise[9 * SimpleData.iMq + 9] = KalmanVars.Noise_Drift;
            KalmanVars.CovarianceMatrixNoise[10 * SimpleData.iMq + 10] = KalmanVars.Noise_Accel;
            KalmanVars.CovarianceMatrixNoise[11 * SimpleData.iMq + 11] = KalmanVars.Noise_Accel;
            KalmanVars.CovarianceMatrixNoise[12 * SimpleData.iMq + 12] = KalmanVars.Noise_Accel;

            if (SINSstate.iMx_r3_dV3)
            {
                KalmanVars.CovarianceMatrixNoise[SINSstate.value_iMx_r3_dV3 * SimpleData.iMq + SINSstate.value_iMx_r3_dV3] = KalmanVars.Noise_Pos;
                KalmanVars.CovarianceMatrixNoise[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMq + (SINSstate.value_iMx_r3_dV3 + 1)] = KalmanVars.Noise_Vel;
            }

            if (SINSstate.iMx_r_odo_12)
            {
                KalmanVars.CovarianceMatrixNoise[SINSstate.value_iMx_r_odo_12 * SimpleData.iMq + SINSstate.value_iMx_r_odo_12] = KalmanVars.Noise_Pos;
                KalmanVars.CovarianceMatrixNoise[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMq + (SINSstate.value_iMx_r_odo_12 + 1)] = KalmanVars.Noise_Pos;
            }

            if (SINSstate.iMx_kappa_13_ds)
            {
                KalmanVars.CovarianceMatrixNoise[SINSstate.value_iMx_kappa_13_ds * SimpleData.iMq + SINSstate.value_iMx_kappa_13_ds] = 0.0000001;
                KalmanVars.CovarianceMatrixNoise[(SINSstate.value_iMx_kappa_13_ds + 1) * SimpleData.iMq + (SINSstate.value_iMx_kappa_13_ds + 1)] = 0.0000001;
                KalmanVars.CovarianceMatrixNoise[(SINSstate.value_iMx_kappa_13_ds + 2) * SimpleData.iMq + (SINSstate.value_iMx_kappa_13_ds + 2)] = 0.0000001;
            }

            if (AlignmentFLG == true)
            {
                if (SimpleData.iMq == 13)
                {
                    KalmanVars.CovarianceMatrixNoise[0 * SimpleData.iMq + 0] = 0.75;
                    KalmanVars.CovarianceMatrixNoise[1 * SimpleData.iMq + 1] = 0.75;
                    KalmanVars.CovarianceMatrixNoise[2 * SimpleData.iMq + 2] = 0.003 * 9.78049;
                    KalmanVars.CovarianceMatrixNoise[3 * SimpleData.iMq + 3] = 0.003 * 9.78049;
                    KalmanVars.CovarianceMatrixNoise[4 * SimpleData.iMq + 4] = 0.2 * 3.141592 / 180.0 / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[5 * SimpleData.iMq + 5] = 0.2 * 3.141592 / 180.0 / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[6 * SimpleData.iMq + 6] = 0.2 * 3.141592 / 180.0 / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[7 * SimpleData.iMq + 7] = 0.00000002;
                    KalmanVars.CovarianceMatrixNoise[8 * SimpleData.iMq + 8] = 0.00000002;
                    KalmanVars.CovarianceMatrixNoise[9 * SimpleData.iMq + 9] = 0.00000002;
                    KalmanVars.CovarianceMatrixNoise[10 * SimpleData.iMq + 10] = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[11 * SimpleData.iMq + 11] = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[12 * SimpleData.iMq + 12] = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                }
                else if (SimpleData.iMq == 7)
                {
                    KalmanVars.CovarianceMatrixNoise[0 * SimpleData.iMq + 0] = 0.75;
                    KalmanVars.CovarianceMatrixNoise[1 * SimpleData.iMq + 1] = 0.75;
                    KalmanVars.CovarianceMatrixNoise[2 * SimpleData.iMq + 2] = 0.003;
                    KalmanVars.CovarianceMatrixNoise[3 * SimpleData.iMq + 3] = 0.003;
                    KalmanVars.CovarianceMatrixNoise[4 * SimpleData.iMq + 4] = 0.02 * SimpleData.ToRadian / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[5 * SimpleData.iMq + 5] = 0.02 * SimpleData.ToRadian / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[6 * SimpleData.iMq + 6] = 0.02 * SimpleData.ToRadian / 3600.0;
                }
                else if (SimpleData.iMq == 5)
                {
                    KalmanVars.CovarianceMatrixNoise[2 * SimpleData.iMq + 0] = 0.003;
                    KalmanVars.CovarianceMatrixNoise[3 * SimpleData.iMq + 1] = 0.003;
                    KalmanVars.CovarianceMatrixNoise[4 * SimpleData.iMq + 2] = 0.02 * SimpleData.ToRadian / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[5 * SimpleData.iMq + 3] = 0.02 * SimpleData.ToRadian / 3600.0;
                    KalmanVars.CovarianceMatrixNoise[6 * SimpleData.iMq + 4] = 0.02 * SimpleData.ToRadian / 3600.0;
                }
            }
        }

        public static void StateCorrection(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate2)
        {
            //if (SINSstate.UsingAltitudeCorrection == true || SINSstate.usingSNS == true)
            //    SINSstate2.Altitude = SINSstate.Altitude - ErrorVector[2];

            SINSstate2.Latitude = SINSstate.Latitude - SINSstate.DeltaLatitude;
            SINSstate2.Longitude = SINSstate.Longitude - SINSstate.DeltaLongitude;
            
            if (SINSstate.UsingOdoVelocity || SINSstate.KNS_flg)
            {
                SINSstate2.Vx_0[0] = SINSstate.Vx_0[0] - SINSstate.DeltaV_1;
                SINSstate2.Vx_0[1] = SINSstate.Vx_0[1] - SINSstate.DeltaV_2;

                if (SINSstate.iMx_r3_dV3)
                {
                    SINSstate2.Vx_0[2] = SINSstate.Vx_0[2] - SINSstate.DeltaV_3;
                }
            }
            else if (SINSstate.UsingOdoPosition)
            {
                SINSstate2.Altitude = SINSstate.Altitude - ErrorVector[13];
                //SINSstate2.Vx_0[2] = SINSstate.Vx_0[2] - SINSstate.DeltaV_3;

                SINSstate2.Vx_0[0] = SINSstate.Vx_0[0] - SINSstate.DeltaV_1;
                SINSstate2.Vx_0[1] = SINSstate.Vx_0[1] - SINSstate.DeltaV_2;
            }

            SINSstate2.Roll = SINSstate.Roll - SINSstate.DeltaRoll;
            SINSstate2.Pitch = SINSstate.Pitch - SINSstate.DeltaPitch;
            SINSstate2.Heading = SINSstate.Heading - SINSstate.DeltaHeading;

            //корректированная матрица ориентации
            SINSstate2.A_sx0 = A_sx0(SINSstate);
            SINSstate2.A_x0s = SINSstate2.A_sx0.Transpose();
            SINSstate2.A_x0n = A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate2.A_nx0 = SINSstate2.A_x0n.Transpose();
        }

        public static void CalcStateErrors(double[] ErrorVector, SINS_State SINSstate)
        {
            SINSstate.DeltaLatitude = ErrorVector[1] / SINSstate.R_n;
            SINSstate.DeltaLongitude = ErrorVector[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

            //SINSstate.DeltaV_1 = ErrorVector[3] + SINSstate.V_x[1] * ErrorVector[8] + SINSstate.V_x[2] * (ErrorVector[0] / SimpleData.A - ErrorVector[7]);
            //SINSstate.DeltaV_2 = ErrorVector[4] - SINSstate.V_x[0] * ErrorVector[8] - SINSstate.V_x[2] * (ErrorVector[1] / SimpleData.A + ErrorVector[6]);
            //SINSstate.DeltaV_3 = ErrorVector[5] + SINSstate.V_x[0] * (-ErrorVector[0] / SimpleData.A + ErrorVector[7]) + SINSstate.V_x[1] * (-ErrorVector[1] / SimpleData.A - ErrorVector[6]);

            if (SINSstate.UsingOdoVelocity || SINSstate.KNS_flg)
            {
                SINSstate.DeltaV_1 = ErrorVector[2] +
                                     (SINSstate.Vx_0[2]*(ErrorVector[0]/SINSstate.R_e - ErrorVector[5]) +
                                      SINSstate.Vx_0[1]*ErrorVector[6]);
                SINSstate.DeltaV_2 = ErrorVector[3] +
                                     (SINSstate.Vx_0[2] * (ErrorVector[1] / SINSstate.R_n - ErrorVector[4]) -
                                      SINSstate.Vx_0[0]*ErrorVector[6]);

                if (SINSstate.iMx_r3_dV3)
                {
                    SINSstate.DeltaV_3 = ErrorVector[14] + SINSstate.Vx_0[0] * (ErrorVector[5] - ErrorVector[0] / SINSstate.R_e) - SINSstate.Vx_0[1] * (ErrorVector[4] + ErrorVector[1] / SINSstate.R_n);
                }
            }
            else if (SINSstate.UsingOdoPosition)
            {
                //SINSstate.DeltaLatitude = (ErrorVector[0]*SINSstate.A_x0s[0, 1]*SINSstate.OdometerVector[1]*
                //                           Math.Tan(SINSstate.Latitude)/SINSstate.R_e +
                //                           ErrorVector[1] -
                //                           ErrorVector[4]*SINSstate.A_x0s[2, 1]*SINSstate.OdometerVector[1] +
                //                           ErrorVector[6]*SINSstate.A_x0s[0, 1]*SINSstate.OdometerVector[1])/
                //                          SINSstate.R_n;
                //SINSstate.DeltaLongitude = (ErrorVector[0] -
                //                            ErrorVector[0]*SINSstate.A_x0s[1, 1]*SINSstate.OdometerVector[1]*
                //                            Math.Tan(SINSstate.Latitude)/SINSstate.R_e +
                //                            ErrorVector[5]*SINSstate.A_x0s[2, 1]*SINSstate.OdometerVector[1] -
                //                            ErrorVector[6]*SINSstate.A_x0s[1, 1]*SINSstate.OdometerVector[1])/
                //                           SINSstate.R_e/Math.Cos(SINSstate.Latitude);


                SINSstate.DeltaAltitude = ErrorVector[13];
                //SINSstate.DeltaV_3 = ErrorVector[14];

                SINSstate.DeltaV_1 = ErrorVector[2] +
                                     (SINSstate.Vx_0[1] * Math.Sin(SINSstate.Latitude) * SINSstate.DeltaLongitude +
                                      SINSstate.Vx_0[1] * ErrorVector[6]);
                SINSstate.DeltaV_2 = ErrorVector[3] -
                                     (SINSstate.Vx_0[0] * Math.Sin(SINSstate.Latitude) * SINSstate.DeltaLongitude +
                                      SINSstate.Vx_0[0] * ErrorVector[6]);
            }

            

            if (SINSstate.usinganglecorrection == true)
            {
                SINSstate.DeltaRoll = -(ErrorVector[4] * Math.Sin(SINSstate.Heading) + ErrorVector[5] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
                SINSstate.DeltaPitch = -ErrorVector[4] * Math.Cos(SINSstate.Heading) + ErrorVector[5] * Math.Sin(SINSstate.Heading);
                SINSstate.DeltaHeading = ErrorVector[6] + SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude) + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);
            }

            //if (SINSstate.usinganglecorrection == true)
            //{
            //    SINSstate.DeltaRoll = (ErrorVector[6] * Math.Sin(SINSstate.Heading) - ErrorVector[7] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
            //    SINSstate.DeltaPitch = -(ErrorVector[6] * Math.Cos(SINSstate.Heading) + ErrorVector[7] * Math.Sin(SINSstate.Heading));
            //    SINSstate.DeltaHeading = -ErrorVector[8] - SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude) - SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);
            //}
        }

        public static void MAKE_H_AND_CORRECTION_alignment(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            for (int i = 0; i < SimpleData.iMx * SimpleData.iMz; i++) KalmanVars.Matrix_H[i] = 0.0;

            KalmanVars.Matrix_H[0 * SimpleData.iMx + 2] = 1.0;
            KalmanVars.Matrix_H[1 * SimpleData.iMx + 3] = 1.0;

            KalmanVars.Measure[0] = SINSstate.Vx_0[0];
            KalmanVars.Measure[1] = SINSstate.Vx_0[1];

            KalmanVars.Matrix_H[2 * SimpleData.iMx + 0] = 1.0;
            KalmanVars.Matrix_H[3 * SimpleData.iMx + 1] = 1.0;

            KalmanVars.Measure[2] = (SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
            KalmanVars.Measure[3] = (SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n;


            KalmanVars.Noize_Z[0] = 0.02;
            KalmanVars.Noize_Z[1] = 0.02;
            KalmanVars.Noize_Z[2] = 0.002;
            KalmanVars.Noize_Z[3] = 0.002;

            KalmanVars.cnt_measures = 4;

            KalmanProcs.KalmanCorrection(KalmanVars);
        }

        public static void MAKE_H_AND_CORRECTION(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            for (int i = 0; i < SimpleData.iMx * SimpleData.iMz; i++) KalmanVars.Matrix_H[i] = 0.0;

            double[] tempVect = new double[3];
            if (SINSstate.UsingCorrection)
            {
                if (SINSstate.KNS_flg == false)
                {
                    if (SINSstate.iMx_r_odo_12 == true)
                    {
                        KalmanVars.Matrix_H[1 * SimpleData.iMx + 1] = 1.0;
                        KalmanVars.Matrix_H[1 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 1] = -1.0;
                        KalmanVars.Matrix_H[0 * SimpleData.iMx + 0] = 1.0;
                        KalmanVars.Matrix_H[0 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12] = -1.0;

                        KalmanVars.Measure[0] = (SINSstate.Longitude - SINSstate_OdoMod.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
                        KalmanVars.Measure[1] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n;

                        KalmanVars.Noize_Z[0] = 0.2;
                        KalmanVars.Noize_Z[1] = 0.2;

                        KalmanVars.cnt_measures = 2;
                    }
                    else if (SINSstate.iMx_r_odo_12 == false)
                    {
                        if (SINSstate.UsingOdoPosition == true)
                        {
                            SINSstate.OdometerVector[1] = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;

                            if (SINSstate.Use_First_Odo_Data_3_Measure == true)
                            {
                                //if (SINSstate.Use_First_Odo_Data_3_Measure_In_Oz == false)
                                //{
                                    KalmanVars.Matrix_H[0 * SimpleData.iMx + 0] = 1.0;
                                    //KalmanVars.Matrix_H[0 * SimpleData.iMx + 0] = 1.0 - (SINSstate.A_x0s[1, 1] * SINSstate.OdometerVector[1]*Math.Tan(SINSstate.Latitude)) / SINSstate.R_e;
                                    //KalmanVars.Matrix_H[0 * SimpleData.iMx + 5] = SINSstate.A_x0s[2, 1] * SINSstate.OdometerVector[1];
                                    //KalmanVars.Matrix_H[0 * SimpleData.iMx + 6] = - SINSstate.A_x0s[1, 1] * SINSstate.OdometerVector[1];

                                    KalmanVars.Matrix_H[1 * SimpleData.iMx + 1] = 1.0;
                                    //KalmanVars.Matrix_H[1 * SimpleData.iMx + 0] = (SINSstate.A_x0s[0, 1] * SINSstate.OdometerVector[1] * Math.Tan(SINSstate.Latitude)) / SINSstate.R_e;
                                    //KalmanVars.Matrix_H[1 * SimpleData.iMx + 4] = -SINSstate.A_x0s[2, 1] * SINSstate.OdometerVector[1];
                                    //KalmanVars.Matrix_H[1 * SimpleData.iMx + 6] = SINSstate.A_x0s[0, 1] * SINSstate.OdometerVector[1];

                                    //Формирование измерений по географическим координатам
                                    KalmanVars.Measure[0] = (SINSstate.Longitude - SINSstate_OdoMod.Longitude) * SINSstate.R_e * Math.Cos(SINSstate_OdoMod.Latitude);
                                    KalmanVars.Measure[1] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n;

                                    KalmanVars.Noize_Z[0] = SINSstate.A_x0s[0, 1] * 0.2;
                                    KalmanVars.Noize_Z[1] = SINSstate.A_x0s[1, 1] * 0.2;

                                    KalmanVars.cnt_measures = 2;

                                    if (SINSstate.iMx_r3_dV3)
                                    {
                                        KalmanVars.Matrix_H[2 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3] = 1.0;
                                        //KalmanVars.Matrix_H[2 * SimpleData.iMx + 4] = SINSstate.A_x0s[1, 1] * SINSstate.OdometerVector[1];
                                        //KalmanVars.Matrix_H[2 * SimpleData.iMx + 5] = -SINSstate.A_x0s[0, 1] * SINSstate.OdometerVector[1];

                                        KalmanVars.Measure[2] = (SINSstate.Altitude - SINSstate_OdoMod.Altitude);

                                        KalmanVars.Noize_Z[2] = SINSstate.A_x0s[2, 1] * 0.2;

                                        KalmanVars.cnt_measures = 3;
                                    }
                                //}
                                //else if (SINSstate.Use_First_Odo_Data_3_Measure_In_Oz == true)
                                //{
                                //    KalmanVars.Matrix_H[0 * SimpleData.iMx + 0] = 1.0;
                                //    KalmanVars.Matrix_H[1 * SimpleData.iMx + 1] = 1.0;

                                //    KalmanVars.Measure[0] = 0.0;
                                //    KalmanVars.Measure[1] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n;

                                //    KalmanVars.Noize_Z[0] = 0.001;
                                //    KalmanVars.Noize_Z[1] = 1.0;


                                //    if (SINSstate.iMx_r3_dV3)
                                //    {
                                //        KalmanVars.Matrix_H[2 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3] = 1.0;

                                //        KalmanVars.Measure[2] = 0.0;
                                //        KalmanVars.Noize_Z[2] = 0.001;
                                //        KalmanVars.cnt_measures = 3;
                                //    }
                                //}
                            }
                            else if (SINSstate.Use_First_Odo_Data_1_Measure == true)
                            {
                                KalmanVars.Matrix_H[0 * SimpleData.iMx + 0] = 1.0;
                                KalmanVars.Matrix_H[0 * SimpleData.iMx + 1] = 1.0;

                                KalmanVars.Measure[0] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n + (SINSstate.Longitude - SINSstate_OdoMod.Longitude) * SINSstate.R_e * Math.Cos(SINSstate_OdoMod.Latitude);

                                KalmanVars.Noize_Z[0] = 1.0;
                                KalmanVars.cnt_measures = 1;

                                if (SINSstate.iMx_r3_dV3)
                                {
                                    KalmanVars.Matrix_H[0 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3] = 1.0;

                                    KalmanVars.Measure[0] += (SINSstate.Altitude - SINSstate_OdoMod.Altitude);
                                }
                            }
                        }
                        else if (SINSstate.UsingOdoVelocity == true)
                        {
                            if (SINSstate.UseOdoVelocity_In_Oz == false)
                            {
                                if (SINSstate.UseLastMinusOneOdo == false)
                                    SINSstate.OdometerVector[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev) / SINSstate.OdoTimeStepCount / SINSstate.timeStep;
                                else
                                    SINSstate.OdometerVector[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev_2) / SINSstate.OdoTimeStepCount_2 / SINSstate.timeStep;

                                CopyArray(SINSstate.OdoSpeed, SINSstate.A_x0s * SINSstate.OdometerVector);


                                //Так какая же все таки модель
                                if (false)
                                {
                                    KalmanVars.Matrix_H[0 * SimpleData.iMx + 2] = 1.0;
                                    KalmanVars.Matrix_H[1 * SimpleData.iMx + 3] = 1.0;
                                    KalmanVars.cnt_measures = 2;

                                    if (SINSstate.iMx_r3_dV3)
                                    {
                                        KalmanVars.Matrix_H[2 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = 1.0;
                                        KalmanVars.cnt_measures = 3;
                                    }
                                }
                                else
                                {
                                    KalmanVars.Matrix_H[0 * SimpleData.iMx + 2] = 1.0;
                                    KalmanVars.Matrix_H[0 * SimpleData.iMx + 6] = SINSstate.Vx_0[1];
                                    KalmanVars.Matrix_H[1 * SimpleData.iMx + 3] = 1.0;
                                    KalmanVars.Matrix_H[1 * SimpleData.iMx + 6] = -SINSstate.Vx_0[0];

                                    KalmanVars.cnt_measures = 2;

                                    if (SINSstate.iMx_r3_dV3)
                                    {
                                        KalmanVars.Matrix_H[2 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = 1.0;
                                        KalmanVars.Matrix_H[2 * SimpleData.iMx + 5] = SINSstate.Vx_0[0];
                                        KalmanVars.Matrix_H[2 * SimpleData.iMx + 0] = -SINSstate.Vx_0[0] / SINSstate.R_e;
                                        KalmanVars.Matrix_H[2 * SimpleData.iMx + 4] = -SINSstate.Vx_0[1];
                                        KalmanVars.Matrix_H[2 * SimpleData.iMx + 1] = -SINSstate.Vx_0[1] / SINSstate.R_n;

                                        KalmanVars.Matrix_H[0 * SimpleData.iMx + 5] = -SINSstate.Vx_0[2];
                                        KalmanVars.Matrix_H[0 * SimpleData.iMx + 0] = SINSstate.Vx_0[2] / SINSstate.R_e;
                                        KalmanVars.Matrix_H[1 * SimpleData.iMx + 4] = SINSstate.Vx_0[2];
                                        KalmanVars.Matrix_H[1 * SimpleData.iMx + 1] = SINSstate.Vx_0[2] / SINSstate.R_n;

                                        KalmanVars.cnt_measures = 3;
                                    }
                                }



                                for (int i = 0; i < 3; i++)
                                    KalmanVars.Measure[i] = SINSstate.Vx_0[i] - SINSstate.OdoSpeed[i];


                                KalmanVars.Noize_Z[0] = SINSstate.A_x0s[0, 1] * KalmanVars.OdoNoise;
                                KalmanVars.Noize_Z[1] = SINSstate.A_x0s[1, 1] * KalmanVars.OdoNoise;
                                KalmanVars.Noize_Z[2] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise;

                            }
                            else if (SINSstate.UseOdoVelocity_In_Oz == true)
                            {
                                if (SINSstate.UseLastMinusOneOdo == false)
                                    SINSstate.OdometerVector[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev) / SINSstate.OdoTimeStepCount / SINSstate.timeStep;
                                else
                                    SINSstate.OdometerVector[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev_2) / SINSstate.OdoTimeStepCount_2 / SINSstate.timeStep;

                                KalmanVars.Matrix_H[0 * SimpleData.iMx + 2] = SINSstate.A_sx0[0, 0];
                                KalmanVars.Matrix_H[0 * SimpleData.iMx + 3] = SINSstate.A_sx0[0, 1];
                                KalmanVars.Matrix_H[1 * SimpleData.iMx + 2] = SINSstate.A_sx0[1, 0];
                                KalmanVars.Matrix_H[1 * SimpleData.iMx + 3] = SINSstate.A_sx0[1, 1];

                                KalmanVars.Measure[0] = SINSstate.A_sx0[0, 0] * SINSstate.Vx_0[0] + SINSstate.A_sx0[0, 1] * SINSstate.Vx_0[1] + SINSstate.A_sx0[0, 2] * SINSstate.Vx_0[2];
                                KalmanVars.Measure[1] = SINSstate.A_sx0[1, 0] * SINSstate.Vx_0[0] + SINSstate.A_sx0[1, 1] * SINSstate.Vx_0[1] + SINSstate.A_sx0[1, 2] * SINSstate.Vx_0[2] - SINSstate.OdometerVector[1];

                                KalmanVars.Noize_Z[0] = 0.001;
                                KalmanVars.Noize_Z[1] = 1.0;


                                if (SINSstate.iMx_r3_dV3)
                                {
                                    KalmanVars.Matrix_H[2 * SimpleData.iMx + 2] = SINSstate.A_sx0[2, 0];
                                    KalmanVars.Matrix_H[2 * SimpleData.iMx + 3] = SINSstate.A_sx0[2, 1];
                                    KalmanVars.Matrix_H[2 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = SINSstate.A_sx0[2, 2];

                                    KalmanVars.Matrix_H[0 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = SINSstate.A_sx0[0, 2];
                                    KalmanVars.Matrix_H[1 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = SINSstate.A_sx0[1, 2];

                                    KalmanVars.Measure[2] = SINSstate.A_sx0[2, 0] * SINSstate.Vx_0[0] + SINSstate.A_sx0[2, 1] * SINSstate.Vx_0[1] + SINSstate.A_sx0[2, 2] * SINSstate.Vx_0[2];
                                    KalmanVars.Noize_Z[2] = 0.001;
                                    KalmanVars.cnt_measures = 3;
                                }
                            }
                        }
                    }


                    KalmanProcs.KalmanCorrection(KalmanVars);
                }
                else
                {
                    if (SINSstate.UseLastMinusOneOdo == false)
                         SINSstate.OdometerVector[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev) / SINSstate.OdoTimeStepCount / SINSstate.timeStep;
                    else
                         SINSstate.OdometerVector[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev_2) / SINSstate.OdoTimeStepCount_2 / SINSstate.timeStep;

                    CopyArray(SINSstate.OdoSpeed, SINSstate.A_x0s * SINSstate.OdometerVector);

                    KalmanVars.Matrix_H[0 * SimpleData.iMx + 2] = 1.0;
                    KalmanVars.Matrix_H[0 * SimpleData.iMx + 6] = SINSstate.Vx_0[1];
                    KalmanVars.Matrix_H[1 * SimpleData.iMx + 3] = 1.0;
                    KalmanVars.Matrix_H[1 * SimpleData.iMx + 6] = -SINSstate.Vx_0[0];

                    KalmanVars.cnt_measures = 2;

                    if (SINSstate.iMx_r3_dV3)
                    {
                        KalmanVars.Matrix_H[2 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = 1.0;
                        KalmanVars.Matrix_H[2 * SimpleData.iMx + 5] = SINSstate.Vx_0[0];
                        KalmanVars.Matrix_H[2 * SimpleData.iMx + 0] = -SINSstate.Vx_0[0] / SINSstate.R_e;
                        KalmanVars.Matrix_H[2 * SimpleData.iMx + 4] = -SINSstate.Vx_0[1];
                        KalmanVars.Matrix_H[2 * SimpleData.iMx + 1] = -SINSstate.Vx_0[1] / SINSstate.R_n;

                        KalmanVars.Matrix_H[0 * SimpleData.iMx + 5] = -SINSstate.Vx_0[2];
                        KalmanVars.Matrix_H[0 * SimpleData.iMx + 0] = SINSstate.Vx_0[2] / SINSstate.R_e;
                        KalmanVars.Matrix_H[1 * SimpleData.iMx + 4] = SINSstate.Vx_0[2];
                        KalmanVars.Matrix_H[1 * SimpleData.iMx + 1] = SINSstate.Vx_0[2] / SINSstate.R_n;

                        KalmanVars.cnt_measures = 3;
                    }

                    for (int i = 0; i < 3; i++)
                        KalmanVars.Measure[i] = SINSstate.Vx_0[i];

                    KalmanVars.Noize_Z[0] = KalmanVars.OdoNoise_STOP;
                    KalmanVars.Noize_Z[1] = KalmanVars.OdoNoise_STOP;
                    KalmanVars.Noize_Z[2] = KalmanVars.OdoNoise_STOP;

                    //if (SINSstate.iMx_r_odo_12)
                    //{
                    //    KalmanVars.Matrix_H[3 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12] = 1.0;
                    //    KalmanVars.Matrix_H[4 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 1] = 1.0;

                    //    KalmanVars.Measure[3] = 0;
                    //    KalmanVars.Measure[4] = 0;

                    //    KalmanVars.Noize_Z[3] = 0.1;
                    //    KalmanVars.Noize_Z[4] = 0.1;
                    //    KalmanVars.cnt_measures += 2;
                    //}

                    KalmanProcs.KalmanCorrection(KalmanVars);
                }

            }

            //****!!!!***** ПЕРЕДЕЛАТЬ ПОД iM = 13

            if (SINSstate.usingSNS == true && SINSstate.GPS_Data.gps_Altitude.isReady == 1)
            {
                for (int i = 0; i < SimpleData.iMx * SimpleData.iMz; i++)
                    KalmanVars.Matrix_H[i] = 0.0;



                KalmanVars.Matrix_H[0 * SimpleData.iMx + 2] = 1.0;
                KalmanVars.Measure[0] = SINSstate.Altitude - SINSstate.GPS_Data.gps_Altitude.Value;
                KalmanVars.Noize_Z[0] = 0.1;
                KalmanVars.cnt_measures = 1;

                KalmanVars.Matrix_H[1 * SimpleData.iMx + 0] = 1.0 / RadiusE(SINSstate.Latitude, SINSstate.Altitude) / Math.Cos(SINSstate.GPS_Data.gps_Latitude.Value);
                KalmanVars.Measure[1] = SINSstate.Longitude - SINSstate.GPS_Data.gps_Longitude.Value;
                KalmanVars.Noize_Z[1] = 0.000001;
                KalmanVars.cnt_measures++;

                KalmanVars.Matrix_H[2 * SimpleData.iMx + 1] = 1.0 / RadiusN(SINSstate.Latitude, SINSstate.Altitude);
                KalmanVars.Measure[2] = SINSstate.Latitude - SINSstate.GPS_Data.gps_Latitude.Value;
                KalmanVars.Noize_Z[2] = 0.000001;
                KalmanVars.cnt_measures++;


                KalmanVars.Matrix_H[3 * SimpleData.iMx + 3] = 1.0;
                KalmanVars.Measure[3] = SINSstate.Vx_0[0] - SINSstate.GPS_Data.gps_Ve.Value;
                KalmanVars.Noize_Z[3] = 0.1;
                KalmanVars.cnt_measures++;

                KalmanVars.Matrix_H[4 * SimpleData.iMx + 4] = 1.0;
                KalmanVars.Measure[4] = SINSstate.Vx_0[1] - SINSstate.GPS_Data.gps_Vn.Value;
                KalmanVars.Noize_Z[4] = 0.1;
                KalmanVars.cnt_measures++;

                KalmanProcs.KalmanCorrection(KalmanVars);
            }
        }

        public static void Make_A(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            for (int i = 0; i < SimpleData.iMx * SimpleData.iMx; i++)
                KalmanVars.Matrix_A[i] = 0;

            KalmanVars.Matrix_A[0 * SimpleData.iMx + 1] = SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[0 * SimpleData.iMx + 2] = 1.0;
            KalmanVars.Matrix_A[0 * SimpleData.iMx + 6] = SINSstate.Vx_0[1];

            KalmanVars.Matrix_A[1 * SimpleData.iMx + 0] = -SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[1 * SimpleData.iMx + 3] = 1.0;
            KalmanVars.Matrix_A[1 * SimpleData.iMx + 6] = -SINSstate.Vx_0[0];

            KalmanVars.Matrix_A[2 * SimpleData.iMx + 0] = SINSstate.u_x[0] * SINSstate.Vx_0[1] / SINSstate.R_e;
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 1] = SINSstate.u_x[1] * SINSstate.Vx_0[1] / SINSstate.R_n;
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 3] = SINSstate.Omega_x[2] + 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 4] = SINSstate.u_x[1] * SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 5] = -SINSstate.g - SINSstate.u_x[0] * SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 9] = -SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[2 * SimpleData.iMx + 10] = 1.0;

            KalmanVars.Matrix_A[3 * SimpleData.iMx + 0] = -SINSstate.u_x[0] * SINSstate.Vx_0[0] / SINSstate.R_e;
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 1] = -SINSstate.u_x[1] * SINSstate.Vx_0[0] / SINSstate.R_n;
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 2] = -SINSstate.Omega_x[2] - 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 4] = -SINSstate.u_x[1] * SINSstate.Vx_0[0] + SINSstate.g;
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 5] = SINSstate.u_x[0] * SINSstate.Vx_0[0];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 9] = SINSstate.Vx_0[0];
            KalmanVars.Matrix_A[3 * SimpleData.iMx + 11] = 1.0;

            KalmanVars.Matrix_A[4 * SimpleData.iMx + 0] = -SINSstate.u_x[2] / SINSstate.R_e;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 3] = -1.0 / SINSstate.R_n;
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 5] = SINSstate.u_x[2];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 6] = -SINSstate.u_x[1];
            KalmanVars.Matrix_A[4 * SimpleData.iMx + 7] = 1.0;

            KalmanVars.Matrix_A[5 * SimpleData.iMx + 1] = -SINSstate.u_x[2] / SINSstate.R_n;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 2] = 1.0 / SINSstate.R_e;
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 4] = -SINSstate.u_x[2];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 6] = SINSstate.u_x[0];
            KalmanVars.Matrix_A[5 * SimpleData.iMx + 8] = 1.0;

            KalmanVars.Matrix_A[6 * SimpleData.iMx + 0] = (SINSstate.Omega_x[0] + SINSstate.u_x[0]) / SINSstate.R_e;
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 1] = (SINSstate.Omega_x[1] + SINSstate.u_x[1]) / SINSstate.R_n;
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 4] = SINSstate.Omega_x[1] + SINSstate.u_x[1];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 5] = -SINSstate.Omega_x[0] - SINSstate.u_x[0];
            KalmanVars.Matrix_A[6 * SimpleData.iMx + 9] = 1.0;

            if (SINSstate.iMx_r3_dV3)
            {
                KalmanVars.Matrix_A[0 * SimpleData.iMx + 0] += SINSstate.Vx_0[2] / SINSstate.R_e;
                KalmanVars.Matrix_A[0 * SimpleData.iMx + 5] += -SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[0 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3] = -SINSstate.Omega_x[1];

                KalmanVars.Matrix_A[1 * SimpleData.iMx + 1] += SINSstate.Vx_0[2] / SINSstate.R_n;
                KalmanVars.Matrix_A[1 * SimpleData.iMx + 4] += SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[1 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3] = SINSstate.Omega_x[0];

                KalmanVars.Matrix_A[SINSstate.value_iMx_r3_dV3 * SimpleData.iMx + 0] = SINSstate.Omega_x[1] - SINSstate.Vx_0[0] / SINSstate.R_e;
                KalmanVars.Matrix_A[SINSstate.value_iMx_r3_dV3 * SimpleData.iMx + 1] = -SINSstate.Omega_x[0] - SINSstate.Vx_0[1] / SINSstate.R_n;
                KalmanVars.Matrix_A[SINSstate.value_iMx_r3_dV3 * SimpleData.iMx + 4] = -SINSstate.Vx_0[1];
                KalmanVars.Matrix_A[SINSstate.value_iMx_r3_dV3 * SimpleData.iMx + 5] = SINSstate.Vx_0[0];
                KalmanVars.Matrix_A[SINSstate.value_iMx_r3_dV3 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = 1.0;


                KalmanVars.Matrix_A[2 * SimpleData.iMx + 1] += SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_n;
                KalmanVars.Matrix_A[2 * SimpleData.iMx + 4] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[2 * SimpleData.iMx + 6] += -SINSstate.u_x[0] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[2 * SimpleData.iMx + 8] += SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[2 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = -SINSstate.Omega_x[1] - 2 * SINSstate.u_x[1];

                KalmanVars.Matrix_A[3 * SimpleData.iMx + 0] += -SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_e;
                KalmanVars.Matrix_A[3 * SimpleData.iMx + 5] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[3 * SimpleData.iMx + 6] += -SINSstate.u_x[1] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[3 * SimpleData.iMx + 7] += -SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[3 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = SINSstate.Omega_x[0] + 2 * SINSstate.u_x[0];

                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 0] = SINSstate.u_x[2] * SINSstate.Vx_0[1] / SINSstate.R_e;
                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 1] = -SINSstate.u_x[2] * SINSstate.Vx_0[0] / SINSstate.R_n;
                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 2] = SINSstate.Omega_x[1] + 2 * SINSstate.u_x[1];
                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 3] = -SINSstate.Omega_x[0] - 2 * SINSstate.u_x[0];
                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 4] = -SINSstate.u_x[2] * SINSstate.Vx_0[0];
                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 5] = -SINSstate.u_x[2] * SINSstate.Vx_0[1];
                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 6] = SINSstate.u_x[0] * SINSstate.Vx_0[0] + SINSstate.u_x[1] * SINSstate.Vx_0[1];
                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 7] = SINSstate.Vx_0[1];
                KalmanVars.Matrix_A[(SINSstate.value_iMx_r3_dV3 + 1) * SimpleData.iMx + 8] = -SINSstate.Vx_0[0];
          }



            if (SINSstate.iMx_r_odo_12)
            {
                if (SINSstate.iMx_kappa_13_ds)
                {
                    double[] u_z = new double[3];
                    CopyArray(u_z, SINSstate.A_sx0 * SINSstate.u_x);
                    double[] Omega_z = new double[3];
                    for (int i =0; i<3; i++)
                        Omega_z[i] = SINSstate.W_z[i] - u_z[i];

                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 0) * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 0] = -SINSstate.A_x0s[0, 2] * SINSstate.OdoSpeed[1] - 
                            SINSstate.A_x0s[0, 1] * Omega_z[1] * SINSstate.OdometerVector[1] + SINSstate.A_x0s[0, 0] * Omega_z[0] * SINSstate.OdometerVector[1];
                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 0) * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 1] = SINSstate.A_x0s[0, 0] * SINSstate.OdoSpeed[1] + 
                            SINSstate.A_x0s[0, 2] * Omega_z[0] * SINSstate.OdometerVector[1] - SINSstate.A_x0s[0, 1] * Omega_z[2] * SINSstate.OdometerVector[1];
                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 0) * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 2] = SINSstate.A_x0s[0, 1] * SINSstate.OdoSpeed[1] + 
                            SINSstate.A_x0s[0, 0] * Omega_z[2] * SINSstate.OdometerVector[1] - SINSstate.A_x0s[0, 2] * Omega_z[1] * SINSstate.OdometerVector[1];

                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 0] = -SINSstate.A_x0s[1, 2] * SINSstate.OdoSpeed[1] -
                            SINSstate.A_x0s[1, 1] * Omega_z[1] * SINSstate.OdometerVector[1] + SINSstate.A_x0s[1, 0] * Omega_z[0] * SINSstate.OdometerVector[1];
                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 1] = SINSstate.A_x0s[1, 0] * SINSstate.OdoSpeed[1] +
                            SINSstate.A_x0s[1, 2] * Omega_z[0] * SINSstate.OdometerVector[1] - SINSstate.A_x0s[1, 1] * Omega_z[2] * SINSstate.OdometerVector[1];
                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 2] = SINSstate.A_x0s[1, 1] * SINSstate.OdoSpeed[1] +
                            SINSstate.A_x0s[1, 0] * Omega_z[2] * SINSstate.OdometerVector[1] - SINSstate.A_x0s[1, 2] * Omega_z[1] * SINSstate.OdometerVector[1];
                }
                else
                {
                    KalmanVars.Matrix_A[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + 0] = -SINSstate.W_x[0] * Math.Tan(SINSstate.Latitude);
                    KalmanVars.Matrix_A[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + 1] = SINSstate.Omega_x[2] + SINSstate.R_e / SINSstate.R_n * (SINSstate.W_x[2] + SimpleData.U * Math.Sin(SINSstate.Latitude));
                    //KalmanVars.Matrix_A[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + 2] = 1.0;
                    KalmanVars.Matrix_A[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + 2] = 2.0;
                    KalmanVars.Matrix_A[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + 6] = SINSstate.Vx_0[1];
                    KalmanVars.Matrix_A[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + 8] = -SINSstate.R_e;
                    KalmanVars.Matrix_A[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12] = SINSstate.W_x[0] * Math.Tan(SINSstate.Latitude);
                    KalmanVars.Matrix_A[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12 + 1] = -SINSstate.R_e / SINSstate.R_n * (SINSstate.W_x[2] + SimpleData.U * Math.Sin(SINSstate.Latitude));

                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + 0] = -SINSstate.Omega_x[2] - SINSstate.R_n / SINSstate.R_e * (SINSstate.W_x[1] * Math.Tan(SINSstate.Latitude) - SINSstate.W_x[2]);
                    //KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + 3] = 1.0;
                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + 3] = 2.0;
                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + 6] = -SINSstate.Vx_0[0];
                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + 7] = SINSstate.R_n;
                    KalmanVars.Matrix_A[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + SINSstate.value_iMx_r_odo_12] = SINSstate.R_n / SINSstate.R_e * (SINSstate.W_x[1] * Math.Tan(SINSstate.Latitude) - SINSstate.W_x[2]);
                }
            }

            


            KalmanProcs.Make_F(SINSstate.timeStep, KalmanVars);
            SINSprocessing.MatrixNoise_ReDef(SINSstate, KalmanVars, SINSstate.Alignment);
        }


        public static void StateIntegration_AT(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate2, SINS_State SINSstate_OdoMod)
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

            if (SINSstate.Odometr_SINS)
                SimpleOperations.CopyMatrix(SINSstate.A_x0n, SINSstate_OdoMod.A_x0n_prev);

            CopyMatrix(AT_z_xi, SINSstate.AT);
            CopyMatrix(B_x_eta, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            Altitude = SINSstate.Altitude;
            Altitude_prev = SINSstate.Altitude_prev;
            Azimth = SINSstate.Azimth;

            fz[1] = SINSstate.F_z[1]; Wz[1] = SINSstate.W_z[1];
            fz[2] = SINSstate.F_z[2]; Wz[2] = SINSstate.W_z[2];
            fz[0] = SINSstate.F_z[0]; Wz[0] = SINSstate.W_z[0];
            CopyArray(SINSstate.F_z, fz);
            CopyArray(SINSstate.W_z, Wz);
            CopyArray(Vx_0, SINSstate.Vx_0);

            Vx[0] = Vx_0[0] * Math.Cos(Azimth) + Vx_0[1] * Math.Sin(Azimth);
            Vx[1] = -Vx_0[0] * Math.Sin(Azimth) + Vx_0[1] * Math.Cos(Azimth);
            Vx[2] = Vx_0[2];

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);

            SINSstate.u_x = U_x0(SINSstate.Latitude);

            u[0] = SimpleData.U * Math.Cos(SINSstate.Latitude) * Math.Sin(Azimth);
            u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude) * Math.Cos(Azimth);
            u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);


            //-------------Интегрирование матрицы AT_z_xi и первое вычисление матрицы D_x_z---------
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




            //---------------------------------Интегрирование скоростей----------------------------
            CopyArray(SINSstate.F_x, D_x_z * fz);

            SINSstate.g = 9.78049 * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014;
            if (true)
                SINSstate.g -= 2 * 0.000001538 * Altitude;

            //SINSstate.Omega_x[0] = -Vx[1] / R_n;
            //SINSstate.Omega_x[1] = Vx[0] / R_e;

            SINSstate.Omega_x[0] = -Vx_0[1] / SINSstate.R_n * Math.Cos(Azimth) + Vx_0[0] / SINSstate.R_e * Math.Sin(Azimth);
            SINSstate.Omega_x[1] = Vx_0[1] / SINSstate.R_n * Math.Sin(Azimth) + Vx_0[0] / SINSstate.R_e * Math.Cos(Azimth);

            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * Vx_0[0] / SINSstate.R_e;
            //Vx[2] = 0.0;

            if (SINSstate.Odometr_SINS == true)
            {
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
            else
            {
                if (SINSstate.Alignment == true)
                {
                    dVx = SINSstate.F_x[0] + Vx[1] * (2.0 * u[2] + SINSstate.Omega_x[2]);
                    dVy = SINSstate.F_x[1] - Vx[0] * (2.0 * u[2] + SINSstate.Omega_x[2]);
                }
                else
                {
                    dVx = SINSstate.F_x[0] + Vx[1] * (2.0 * u[2] + SINSstate.Omega_x[2]) - Vx[2] * (2.0 * u[1] + SINSstate.Omega_x[1]);
                    dVy = SINSstate.F_x[1] - Vx[0] * (2.0 * u[2] + SINSstate.Omega_x[2]) + Vx[2] * (2.0 * u[0] + SINSstate.Omega_x[0]);
                }
                Vx[0] += dVx * SINSstate.timeStep;
                Vx[1] += dVy * SINSstate.timeStep;

                //--------------------------------------------------------------------------------------

                //double GG = SimpleData.Gravity_Normal * (1 + 0.005317099 * Math.Sin(SINSstate.Latitude) * Math.Sin(SINSstate.Latitude)) * SimpleData.A * SimpleData.A / (SimpleData.A + SINSstate.Altitude) / (SimpleData.A + SINSstate.Altitude);
                if (SINSstate.UsingAltitudeCorrection == true || SINSstate.usingSNS == true)
                {
                    dVh = SINSstate.F_x[2] - SINSstate.g + (Vx[0] + SINSstate.Vx_0_prev[0]) / 2.0 * (2 * u[1] + SINSstate.Omega_x[1]) - (Vx[1] + SINSstate.Vx_0_prev[1]) / 2.0 * (2 * u[0] + SINSstate.Omega_x[0]);
                    Vx[2] += dVh * SINSstate.timeStep;

                    dh = (Vx[2] + SINSstate.Vx_0_prev[2]) / 2.0;
                    Altitude = (Altitude + Altitude_prev) / 2.0 + dh * SINSstate.timeStep;
                }
            }

            //if (Can < 10)
            //{
            //    Can++;
            //    dVh_global += SINSstate.F_x[2] - SINSstate.g + (Vx[0] + SINSstate.V_x_prev[0]) / 2.0 * (2.0 * u[1] + SINSstate.Omega_x[1]) - (Vx[1] + SINSstate.V_x_prev[1]) / 2.0 * (2.0 * u[0] + SINSstate.Omega_x[0]);
            //}
            //else
            //{
            //    dVh_global = dVh_global / Can;
            //    Vx[2] += dVh_global * SINSstate.timeStep;
            //    Can = 0;
            //    dVh_global = 0.0;
            //}


            //---------Интегрирование матрицы B_x_eta и второе вычисление матрицы D_x_z--------------
            //SINSstate.Omega_x[0] = -(Vx[1] + SINSstate.V_x_prev[1]) / 2.0 / R_n;
            //SINSstate.Omega_x[1] = (Vx[0] + SINSstate.V_x_prev[0]) / 2.0 / R_e;

            SINSstate.Omega_x[0] = -(Vx_0[1] + SINSstate.Vx_0_prev[1]) / 2.0 / SINSstate.R_n * Math.Cos(Azimth) + (Vx_0[0] + SINSstate.Vx_0_prev[0]) / 2.0 / SINSstate.R_e * Math.Sin(Azimth);
            SINSstate.Omega_x[1] = (Vx_0[1] + SINSstate.Vx_0_prev[1]) / 2.0 / SINSstate.R_n * Math.Sin(Azimth) + (Vx_0[0] + SINSstate.Vx_0_prev[0]) / 2.0 / SINSstate.R_e * Math.Cos(Azimth);
            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * (Vx[0] + SINSstate.Vx_0_prev[0]) / 2.0 / SINSstate.R_e;

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

            // определение географических координат
            SINSstate.Longitude = Math.Atan2(B_x_eta[2, 1], B_x_eta[2, 0]);
            SINSstate.Latitude = Math.Atan2(B_x_eta[2, 2], Math.Sqrt(B_x_eta[0, 2] * B_x_eta[0, 2] + B_x_eta[1, 2] * B_x_eta[1, 2]));
            Azimth = Math.Atan2(B_x_eta[0, 2], B_x_eta[1, 2]);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);

            u[0] = SimpleData.U * Math.Cos(SINSstate.Latitude) * Math.Sin(Azimth);
            u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude) * Math.Cos(Azimth);
            u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);

            CopyMatrix(W_x_xi, B_x_eta * SINSstate.A_nxi);
            CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());



            //----------------Вычисление углов и переприсвоение матриц---------------------------
            CopyMatrix(SINSstate.A_sx0, D_x_z.Transpose());
            CopyMatrix(SINSstate.A_x0s, D_x_z);
            CopyMatrix(SINSstate.A_x0n, B_x_eta);
            CopyMatrix(SINSstate.A_nx0, B_x_eta.Transpose());

            // определение географических координат
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

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.u_x = U_x0(SINSstate.Latitude);

            CopyArray(SINSstate.W_x, SINSstate.A_x0s * SINSstate.W_z);

            CopyArray(SINSstate.Vx_0_prev, SINSstate.Vx_0);
            CopyArray(SINSstate.Vx_0, Vx_0);
            CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
            CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
            //--------------------------------------------------------------------------------------
        }
    }
}
