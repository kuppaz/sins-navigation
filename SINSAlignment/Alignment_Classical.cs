using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Common_Namespace;

namespace SINSAlignment
{
    public class Alignment_Classical
    {
        public static void InitOfCovarianceMatrixes(Kalman_Align KalmanAlign)
        {
            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 0 + 0] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 0 + 0] = 20 * SimpleData.ToRadian / 60.0;
            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 1 + 1] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 1 + 1] = 20 * SimpleData.ToRadian / 60.0;
            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 2 + 2] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 2 + 2] = 20 * SimpleData.ToRadian / 60.0;

            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 3 + 3] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 3 + 3] = 0.01;
            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 4 + 4] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 4 + 4] = 0.01;
            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 5 + 5] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 5 + 5] = 0.01;

            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 6 + 6] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 6 + 6] = 0.05 * SimpleData.ToRadian / 3600.0;
            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 7 + 7] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 7 + 7] = 0.05 * SimpleData.ToRadian / 3600.0;
            KalmanAlign.CovarianceMatrixS_m[SimpleData.iMx_Align * 8 + 8] = KalmanAlign.CovarianceMatrixS_p[SimpleData.iMx_Align * 8 + 8] = 0.05 * SimpleData.ToRadian / 3600.0;
        }

        public static void Make_A(SINS_State SINSstate, Kalman_Align KalmanAlign)
        {
            KalmanAlign.Matrix_A[0 * SimpleData.iMx_Align + 1] = SimpleData.U * Math.Sin(SINSstate.Latitude);
            KalmanAlign.Matrix_A[0 * SimpleData.iMx_Align + 2] = -SimpleData.U * Math.Cos(SINSstate.Latitude);
            KalmanAlign.Matrix_A[1 * SimpleData.iMx_Align + 0] = -SimpleData.U * Math.Sin(SINSstate.Latitude);
            KalmanAlign.Matrix_A[2 * SimpleData.iMx_Align + 0] = SimpleData.U * Math.Cos(SINSstate.Latitude);

            KalmanAlign.Matrix_A[0 * SimpleData.iMx_Align + 6] = SINSstate.A_x0s[0, 0];
            KalmanAlign.Matrix_A[0 * SimpleData.iMx_Align + 7] = SINSstate.A_x0s[0, 1];
            KalmanAlign.Matrix_A[0 * SimpleData.iMx_Align + 8] = SINSstate.A_x0s[0, 2];
            KalmanAlign.Matrix_A[1 * SimpleData.iMx_Align + 6] = SINSstate.A_x0s[1, 0];
            KalmanAlign.Matrix_A[1 * SimpleData.iMx_Align + 7] = SINSstate.A_x0s[1, 1];
            KalmanAlign.Matrix_A[1 * SimpleData.iMx_Align + 8] = SINSstate.A_x0s[1, 2];
            KalmanAlign.Matrix_A[2 * SimpleData.iMx_Align + 6] = SINSstate.A_x0s[2, 0];
            KalmanAlign.Matrix_A[2 * SimpleData.iMx_Align + 7] = SINSstate.A_x0s[2, 1];
            KalmanAlign.Matrix_A[2 * SimpleData.iMx_Align + 8] = SINSstate.A_x0s[2, 2];

            //SimpleOperations.PrintMatrixToFile(KalmanAlign.Matrix_A, SimpleData.iMx_Align, SimpleData.iMx_Align);
        }




        public static void Make_H_and_Correction(SINS_State SINSstate, Kalman_Align KalmanAlign)
        {
            KalmanAlign.Matrix_H[0 * SimpleData.iMx_Align + 1] = -SINSstate.g;
            KalmanAlign.Matrix_H[1 * SimpleData.iMx_Align + 0] = SINSstate.g;

            KalmanAlign.Matrix_H[0 * SimpleData.iMx_Align + 3] = SINSstate.A_x0s[0, 0];
            KalmanAlign.Matrix_H[0 * SimpleData.iMx_Align + 4] = SINSstate.A_x0s[0, 1];
            KalmanAlign.Matrix_H[0 * SimpleData.iMx_Align + 5] = SINSstate.A_x0s[0, 2];
            KalmanAlign.Matrix_H[1 * SimpleData.iMx_Align + 3] = SINSstate.A_x0s[1, 0];
            KalmanAlign.Matrix_H[1 * SimpleData.iMx_Align + 4] = SINSstate.A_x0s[1, 1];
            KalmanAlign.Matrix_H[1 * SimpleData.iMx_Align + 5] = SINSstate.A_x0s[1, 2];
            KalmanAlign.Matrix_H[2 * SimpleData.iMx_Align + 3] = SINSstate.A_x0s[2, 0];
            KalmanAlign.Matrix_H[2 * SimpleData.iMx_Align + 4] = SINSstate.A_x0s[2, 1];
            KalmanAlign.Matrix_H[2 * SimpleData.iMx_Align + 5] = SINSstate.A_x0s[2, 2];

            SimpleOperations.CopyArray(SINSstate.F_x, SINSstate.A_x0s * SINSstate.F_z);

            KalmanAlign.Measure[0] = SINSstate.F_x[0];
            KalmanAlign.Measure[1] = SINSstate.F_x[1];
            KalmanAlign.Measure[2] = SINSstate.F_x[2] - SINSstate.g;

            //KalmanAlign.Noize_Z[0] = 0.0003;
            //KalmanAlign.Noize_Z[1] = 0.0003;
            //KalmanAlign.Noize_Z[2] = 0.0003;

            KalmanAlign.Noize_Z[0] = Math.Abs(SINSstate.A_x0s[0, 0] * KalmanAlign.Noise_Vel[0] + SINSstate.A_x0s[0, 1] * KalmanAlign.Noise_Vel[1] + SINSstate.A_x0s[0, 2] * KalmanAlign.Noise_Vel[2]);
            KalmanAlign.Noize_Z[1] = Math.Abs(SINSstate.A_x0s[1, 0] * KalmanAlign.Noise_Vel[0] + SINSstate.A_x0s[1, 1] * KalmanAlign.Noise_Vel[1] + SINSstate.A_x0s[1, 2] * KalmanAlign.Noise_Vel[2]);
            KalmanAlign.Noize_Z[2] = Math.Abs(SINSstate.A_x0s[2, 0] * KalmanAlign.Noise_Vel[0] + SINSstate.A_x0s[2, 1] * KalmanAlign.Noise_Vel[1] + SINSstate.A_x0s[2, 2] * KalmanAlign.Noise_Vel[2]);

            KalmanAlign.cnt_measures = 3;

            if (false)
            {
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 3] = SINSstate.W_z[0];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 4] = SINSstate.W_z[1];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 5] = SINSstate.W_z[2];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 6] = SINSstate.F_z[0];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 7] = SINSstate.F_z[1];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 8] = SINSstate.F_z[2];

                KalmanAlign.Measure[KalmanAlign.cnt_measures] = SimpleOperations.SkalyarProduct(SINSstate.F_z, SINSstate.W_z) - SimpleData.U * SINSstate.g * Math.Sin(SINSstate.Latitude);
                //KalmanAlign.Noize_Z[KalmanAlign.cnt_measures] = SimpleOperations.SkalyarProduct(SINSstate.F_z, KalmanAlign.Noise_Angl);
                KalmanAlign.Noize_Z[KalmanAlign.cnt_measures] = 0.05;

                KalmanAlign.cnt_measures++;
            }

            if (false)
            {
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 6] = 2.0 * SINSstate.W_z[0];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 7] = 2.0 * SINSstate.W_z[1];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 8] = 2.0 * SINSstate.W_z[2];

                KalmanAlign.Measure[KalmanAlign.cnt_measures] = SimpleOperations.SkalyarProduct(SINSstate.W_z, SINSstate.W_z) - SimpleData.U * SimpleData.U;
                //KalmanAlign.Noize_Z[KalmanAlign.cnt_measures] = 2.0 * SimpleOperations.SkalyarProduct(SINSstate.W_z, KalmanAlign.Noise_Angl);
                KalmanAlign.Noize_Z[KalmanAlign.cnt_measures] = 0.00000000001;

                KalmanAlign.cnt_measures++;
            }

            if (false)
            {
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 3] = 2.0 * SINSstate.F_z[0];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 4] = 2.0 * SINSstate.F_z[1];
                KalmanAlign.Matrix_H[KalmanAlign.cnt_measures * SimpleData.iMx_Align + 5] = 2.0 * SINSstate.F_z[2];

                KalmanAlign.Measure[KalmanAlign.cnt_measures] = SimpleOperations.SkalyarProduct(SINSstate.F_z, SINSstate.F_z) - SINSstate.g * SINSstate.g;
                //KalmanAlign.Noize_Z[KalmanAlign.cnt_measures] = 2.0 * SimpleOperations.SkalyarProduct(SINSstate.F_z, KalmanAlign.Noise_Vel);
                KalmanAlign.Noize_Z[KalmanAlign.cnt_measures] = 0.01;

                KalmanAlign.cnt_measures++;
            }

            //SimpleOperations.PrintMatrixToFile(KalmanAlign.Matrix_H, KalmanAlign.cnt_measures, SimpleData.iMx_Align);

            KalmanProcs.KalmanCorrection_Align(KalmanAlign);
        }




        public static void MatrixNoise_ReDef(SINS_State SINSstate, Kalman_Align KalmanAlign)
        {
            for (int i = 0; i < SimpleData.iMx_Align * SimpleData.iMq_Align; i++)
                KalmanAlign.CovarianceMatrixNoise[i] = 0.0;

            //KalmanAlign.CovarianceMatrixNoise[0 * SimpleData.iMq_Align + 0] = KalmanAlign.Noise_Angl[0];
            //KalmanAlign.CovarianceMatrixNoise[1 * SimpleData.iMq_Align + 1] = KalmanAlign.Noise_Angl[1];
            //KalmanAlign.CovarianceMatrixNoise[2 * SimpleData.iMq_Align + 2] = KalmanAlign.Noise_Angl[2];
            KalmanAlign.CovarianceMatrixNoise[0 * SimpleData.iMq_Align + 0] = 1e-7;
            KalmanAlign.CovarianceMatrixNoise[1 * SimpleData.iMq_Align + 1] = 1e-7;
            KalmanAlign.CovarianceMatrixNoise[2 * SimpleData.iMq_Align + 2] = 1e-7;

            //SimpleOperations.PrintMatrixToFile(KalmanAlign.CovarianceMatrixNoise, SimpleData.iMx_Align, SimpleData.iMq_Align);
        }




        public static void AlignStateIntegration_AT(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate2, SINS_State SINSstate_OdoMod)
        {
            double[] fz = new double[3], Wz = new double[3], tempV = new double[3], Wz_avg = new double[3];
            double[] Vx_0 = new double[3], Vx_0_prev = new double[3];

            Matrix AT_z_xi = new Matrix(3, 3); Matrix B_x_eta = new Matrix(3, 3);
            Matrix dAT = new Matrix(3, 3); Matrix D_x_z = new Matrix(3, 3);
            Matrix W_x_xi = new Matrix(3, 3); Matrix C_eta_xi = new Matrix(3, 3);

            Matrix Hat1 = new Matrix(3, 3);
            Matrix Hat2 = new Matrix(3, 3);
            Matrix E = Matrix.UnitMatrix(3);
            Matrix dMatrix = new Matrix(3, 3);

            double W_z_abs, dlt, dlt2, kren, tang, gkurs;

            SimpleOperations.CopyMatrix(AT_z_xi, SINSstate.AT);
            SimpleOperations.CopyMatrix(B_x_eta, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            //C_eta_xi = Matrix.DoA_eta_xi(SINSstate.Time);

            SimpleOperations.CopyArray(fz, SINSstate.F_z);
            SimpleOperations.CopyArray(Wz, SINSstate.W_z);

            SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Height);

            SINSstate.u_x = SimpleOperations.U_x0(SINSstate.Latitude);

            //-------------ИНТЕГРИРОВАНИЕ МАТРИЦЫ AT_Z_XI И ПЕРВОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z---------

            if (SINSstate.flag_UsingAvegering == true)
                for (int i = 0; i < 3; i++)
                {
                    fz[i] = (fz[i] + SINSstate.F_z_prev[i]) / 2.0;
                    Wz[i] = (Wz[i] + SINSstate.W_z_prev[i]) / 2.0;
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
            SimpleOperations.CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());
            //--------------------------------------------------------------------------------------


            //--- надо вычислять, используется, например в выставке ---//
            SINSstate.g = 9.78049 * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014;
            if (true)
                SINSstate.g -= 2 * 0.000001538 * SINSstate.Height;


            //----------------Вычисление углов и переприсвоение матриц---------------------------
            SimpleOperations.CopyMatrix(SINSstate.A_sx0, D_x_z.Transpose());
            SimpleOperations.CopyMatrix(SINSstate.A_x0s, D_x_z);

            SimpleOperations.CopyArray(SINSstate.W_x, SINSstate.A_x0s * Wz);

            SimpleOperations.CopyArray(SINSstate.F_z_prev, fz);
            SimpleOperations.CopyArray(SINSstate.W_z_prev, Wz);
            SimpleOperations.CopyArray(SINSstate.W_z, Wz);
            //--------------------------------------------------------------------------------------
        }
    }
}
