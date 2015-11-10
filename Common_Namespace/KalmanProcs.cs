using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class KalmanProcs
    {

        public static void Smoothing(Kalman_Vars KalmanVars, SINS_State SINSstate, int iMx)
        {
            double[] temp_array_1 = new double[iMx * iMx], temp_array_2 = new double[iMx * iMx];

            if (iMx == 1)
            {
                KalmanVars.ErrorVector_Smoothed[0] =
                    (Math.Pow(KalmanVars.CovarianceMatrix_SP_m[0], 2) * KalmanVars.ErrorVector_Straight[0]
                    + Math.Pow(KalmanVars.CovarianceMatrix_SP_Straight[0], 2) * KalmanVars.ErrorVector_m[0]
                    ) /
                    (Math.Pow(KalmanVars.CovarianceMatrix_SP_m[0], 2) + Math.Pow(KalmanVars.CovarianceMatrix_SP_Straight[0], 2))
                    ;
            }
            else
            {
                unsafe
                {
                    fixed (double* Xb = KalmanVars.ErrorVector_m, Sb = KalmanVars.CovarianceMatrix_SP_m, Xf = KalmanVars.ErrorVector_Straight, Sf = KalmanVars.CovarianceMatrix_SP_Straight,
                                   Xsgl = KalmanVars.ErrorVector_Smoothed, Ssgl = KalmanVars.CovarianceMatrix_SP_Smoothed, _temp_1 = temp_array_1, _temp_2 = temp_array_2)
                    {
                        smooth_sub(Xf, Sf, Xb, Sb, Xsgl, Ssgl, iMx, _temp_1, _temp_2);
                    }
                }
            }

        }





        public static void Make_F(double timeStep, Kalman_Vars KalmanVars, SINS_State SINSstate)
        {
            unsafe
            {
                fixed (double* _a = KalmanVars.Matrix_A, _f = KalmanVars.TransitionMatrixF)
                {
                    func(timeStep, _a, _f, SimpleData.iMx);
                }
            }
        }
        public static void Make_F_NB(double timeStep, Kalman_Vars KalmanVars)
        {
            unsafe
            {
                fixed (double* _a = KalmanVars.Matrix_A, _f = KalmanVars.TransitionMatrixF)
                {
                    Make_F_nb(timeStep, _a, _f, SimpleData.iMx);
                }
            }
        }



        public static double[] rsb_rsb(double[] pdP, int iMx)
        {
            double[] CovarianceMatrixS = new double[iMx * iMx];
            unsafe
            {
                fixed (double* _pdP = pdP, _CovarianceMatrixS = CovarianceMatrixS)
                {
                    rsb(_pdP, _CovarianceMatrixS, iMx);
                }
            }

            return CovarianceMatrixS;
        }

        public static double Sigmf_Disp(int numb_i, Kalman_Vars KalmanVars)
        {
            double _out = 0.0;

            unsafe
            {
                fixed (double* _s = KalmanVars.CovarianceMatrixS_m)
                {
                    _out = sigmbi(_s, numb_i, SimpleData.iMx);
                }
            }
            return _out;
        }

        public static double Sigmf_Disp(int numb_i, Kalman_Align KalmanAlign)
        {
            double _out = 0.0;

            unsafe
            {
                fixed (double* _s = KalmanAlign.CovarianceMatrixS_m)
                {
                    _out = sigmbi(_s, numb_i, SimpleData.iMx_Align);
                }
            }
            return _out;
        }

        unsafe static double sigmbi(double* s, int i, int m)
        {
            int j;
            double y;

            y = 0.0;
            for (j = i; j < m; j++) y += s[i * m + j] * s[i * m + j];

            return (Math.Sqrt(y));
        }

        /*           sigmba               */
        /*              T      2          */
        /* sigmba =  ||s * a ||           */
        unsafe static double sigmba(double* a, double* s, int m)
        {
            int i, j;
            double c, y;
            c = 0.0;
            for (i = 0; i < m; i++)
            {
                y = 0.0;
                for (j = 0; j <= i; j++) y += *(s + j * m + i) * *(a + j);
                c += (y * y);
            }
            return (c);
        }

        public static void Check_Measurement(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            for (int i = 0; i < SimpleData.iMx; i++)
                KalmanVars.StringOfMeasure[i] = 0.0;

            for (int i = 0; i < SimpleData.iMz; i++)
            {
                KalmanVars.pdResidual[i] = 0.0;
                KalmanVars.pdSigmaApriori[i] = 0.0;
            }

            for (int i = 0; i < KalmanVars.cnt_measures; i++)
            {
                KalmanVars.pdResidual[i] = KalmanVars.Measure[i];
                for (int j = 0; j < SimpleData.iMx; j++)
                {
                    KalmanVars.StringOfMeasure[j] = KalmanVars.Matrix_H[i * SimpleData.iMx + j];
                    KalmanVars.pdResidual[i] -= KalmanVars.StringOfMeasure[j];
                }

                unsafe
                {
                    fixed (double* StringOfMeasure = KalmanVars.StringOfMeasure, CovarianceMatrixS_m = KalmanVars.CovarianceMatrixS_m)
                    {
                        KalmanVars.pdSigmaApriori[i] = Math.Sqrt(sigmba(StringOfMeasure, CovarianceMatrixS_m, SimpleData.iMx)) + KalmanVars.Noize_Z[i] * KalmanVars.Noize_Z[i];
                    }
                }
            }
        }

        public static void KalmanCorrection(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            for (int i = 0; i < SimpleData.iMx; i++)
            {
                KalmanVars.KalmanFactor[i] = 0.0;
                KalmanVars.StringOfMeasure[i] = 0.0;
            }

            if (!SINSstate.MyOwnKalman_Korrection)
            {
                unsafe
                {
                    fixed (double* _xm = KalmanVars.ErrorConditionVector_m, _xp = KalmanVars.ErrorConditionVector_p, _sm = KalmanVars.CovarianceMatrixS_m, _sp = KalmanVars.CovarianceMatrixS_p, _kf = KalmanVars.KalmanFactor)
                    {
                        for (int t = 0; t < KalmanVars.cnt_measures; t++)
                        {
                            for (int i = 0; i < SimpleData.iMx; i++)
                                KalmanVars.StringOfMeasure[i] = KalmanVars.Matrix_H[t * SimpleData.iMx + i];

                            fixed (double* _h = KalmanVars.StringOfMeasure)
                            {
                                //Коррекция по измерениям
                                f0b(KalmanVars.Measure[t], _xm, _sm, _h, KalmanVars.Noize_Z[t] * KalmanVars.Noize_Z[t], _xp, _sp, _kf, SimpleData.iMx);

                                if (t < KalmanVars.cnt_measures - 1)
                                {
                                    for (int i = 0; i < SimpleData.iMx; i++)
                                    {
                                        _xm[i] = _xp[i];
                                        for (int j = 0; j < SimpleData.iMx; j++)
                                            _sm[i * SimpleData.iMx + j] = _sp[i * SimpleData.iMx + j];
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                double d_k = 0.0, b_k = 0.0, c_k = 0.0;
                double[] f, e, S_k_m, S_k_p;
                Matrix CovarianceMatrixS_p = new Matrix(SimpleData.iMx, SimpleData.iMx), 
                    CovarianceMatrixS_m = new Matrix(SimpleData.iMx, SimpleData.iMx);

                for (int t = 0; t < KalmanVars.cnt_measures; t++)
                {
                    d_k = KalmanVars.Noize_Z[t] * KalmanVars.Noize_Z[t];

                    e = new double[SimpleData.iMx];
                    S_k_m = new double[SimpleData.iMx];
                    f = new double[SimpleData.iMx];
                    S_k_p = new double[SimpleData.iMx];

                    SimpleOperations.MakeMatrixFromVector(CovarianceMatrixS_p, KalmanVars.CovarianceMatrixS_p, SimpleData.iMx);
                    SimpleOperations.MakeMatrixFromVector(CovarianceMatrixS_m, KalmanVars.CovarianceMatrixS_m, SimpleData.iMx);

                    for (int i = 0; i < SimpleData.iMx; i++)
                        KalmanVars.StringOfMeasure[i] = KalmanVars.Matrix_H[t * SimpleData.iMx + i];

                    SimpleOperations.CopyArray(f, CovarianceMatrixS_m.Transpose() * KalmanVars.StringOfMeasure);

                    for (int i = 0; i < SimpleData.iMx; i++)
                    {
                        b_k = Math.Sqrt(d_k / (d_k + f[i] * f[i]));
                        c_k = f[i] / (Math.Sqrt(d_k * (d_k + f[i] * f[i])));
                        d_k = d_k + f[i] * f[i];

                        for (int j = 0; j < SimpleData.iMx; j++)
                            S_k_m[j] = KalmanVars.CovarianceMatrixS_m[j * SimpleData.iMx + i];

                        for (int j = 0; j < SimpleData.iMx; j++)
                        {
                            S_k_p[j] = S_k_m[j] * b_k - e[j] * c_k;
                            KalmanVars.CovarianceMatrixS_p[j * SimpleData.iMx + i] = S_k_p[j];
                        }

                        for (int j = 0; j < SimpleData.iMx; j++)
                            e[j] = e[j] + S_k_m[j] * f[i];
                    }

                    double h_x_ = 0.0;
                    for (int j = 0; j < SimpleData.iMx; j++)
                        h_x_ += KalmanVars.StringOfMeasure[j] * KalmanVars.ErrorConditionVector_m[j];

                    for (int j = 0; j < SimpleData.iMx; j++)
                        KalmanVars.ErrorConditionVector_p[j] = KalmanVars.ErrorConditionVector_m[j] + (KalmanVars.Measure[t] - h_x_) * e[j] / d_k;


                    if (t < KalmanVars.cnt_measures - 1)
                    {
                        SimpleOperations.CopyArray(KalmanVars.ErrorConditionVector_m, KalmanVars.ErrorConditionVector_p);
                        SimpleOperations.CopyArray(KalmanVars.CovarianceMatrixS_m, KalmanVars.CovarianceMatrixS_p);
                    }
                }
            }
            //else
            //{
            //    double[] f;
            //    Matrix CovarianceMatrixS_p = new Matrix(SimpleData.iMx, SimpleData.iMx), 
            //        CovarianceMatrixS_m = new Matrix(SimpleData.iMx, SimpleData.iMx);

            //    for (int t = 0; t < KalmanVars.cnt_measures; t++)
            //    {
            //        f = new double[SimpleData.iMx];

            //        SimpleOperations.MakeMatrixFromVector(CovarianceMatrixS_m, KalmanVars.CovarianceMatrixS_m, SimpleData.iMx);

            //        for (int i = 0; i < SimpleData.iMx; i++)
            //            KalmanVars.StringOfMeasure[i] = KalmanVars.Matrix_H[t * SimpleData.iMx + i];

            //        double[] Sm_f = new double[SimpleData.iMx];
            //        double[] E_ffalpha = new double[SimpleData.iMx * SimpleData.iMx],
            //            tmpArray = new double[SimpleData.iMx * SimpleData.iMx],
            //            tmpArray2 = new double[SimpleData.iMx * SimpleData.iMx];

            //        SimpleOperations.CopyArray(f, CovarianceMatrixS_m.Transpose() * KalmanVars.StringOfMeasure);

            //        double alpha = KalmanVars.Noize_Z[t] * KalmanVars.Noize_Z[t];
            //        for (int j = 0; j < SimpleData.iMx; j++)
            //            alpha += f[j] * f[j];

            //        // --- Матрица E - ff/alpha ---
            //        for (int i = 0; i < SimpleData.iMx; i++)
            //        {
            //            E_ffalpha[i * SimpleData.iMx + i] = 1.0;
            //            for (int j = 0; j < SimpleData.iMx; j++)
            //                E_ffalpha[i * SimpleData.iMx + j] -= f[i] * f[j] / alpha;
            //        }
            //        // --- Корень из (E - ff/alpha) ---
            //        // -------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!----------
            //        SimpleOperations.CopyArray(tmpArray, rsb_rsb(E_ffalpha, SimpleData.iMx));
            //        // -------!!!!!!!ТУТ НУЖНО ПО ЧЕСТНОМУ СЧИТАТЬ КОРЕНЬ!!!!!!!!!!!!!!----------
            //        // -------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!----------

            //        // --- Получаем S+= S- * sqrt(E - ff/alpha) ---
            //        for (int i = 0; i < SimpleData.iMx; i++)
            //            for (int j = 0; j < SimpleData.iMx; j++)
            //                for (int k = 0; k < SimpleData.iMx; k++)
            //                    tmpArray2[i * SimpleData.iMx + j] += KalmanVars.CovarianceMatrixS_m[i * SimpleData.iMx + k] * tmpArray[k * SimpleData.iMx + j];
            //        SimpleOperations.CopyArray(KalmanVars.CovarianceMatrixS_p, tmpArray2);

            //        SimpleOperations.CopyArray(Sm_f, CovarianceMatrixS_m * f);

            //        // --- //
            //        double h_x_ = 0.0;
            //        for (int j = 0; j < SimpleData.iMx; j++)
            //            h_x_ += KalmanVars.StringOfMeasure[j] * KalmanVars.ErrorConditionVector_m[j];

            //        for (int j = 0; j < SimpleData.iMx; j++)
            //            KalmanVars.ErrorConditionVector_p[j] = KalmanVars.ErrorConditionVector_m[j] + Sm_f[j] * (KalmanVars.Measure[t] - h_x_) / alpha;


            //        if (t < KalmanVars.cnt_measures - 1)
            //        {
            //            SimpleOperations.CopyArray(KalmanVars.ErrorConditionVector_m, KalmanVars.ErrorConditionVector_p);
            //            SimpleOperations.CopyArray(KalmanVars.CovarianceMatrixS_m, KalmanVars.CovarianceMatrixS_p);
            //        }
            //    }
            //}

        }



        public static void KalmanForecast(Kalman_Vars KalmanVars, SINS_State SINSstate)
        {
            if (!SINSstate.MyOwnKalman_Forecast)
            {
                unsafe
                {
                    fixed (double* _xm = KalmanVars.ErrorConditionVector_m, _xp = KalmanVars.ErrorConditionVector_p, _sm = KalmanVars.CovarianceMatrixS_m, _sp = KalmanVars.CovarianceMatrixS_p,
                        _f = KalmanVars.TransitionMatrixF, _sq = KalmanVars.CovarianceMatrixNoise)
                    {
                        dgq0b(_xp, _sp, _f, _sq, _xm, _sm, SimpleData.iMx, SimpleData.iMq);
                        //dg0b(_xp, _sp, _f,_xm, _sm, SimpleData.iMx);
                    }
                }
                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrixS_p, KalmanVars.CovarianceMatrixS_m);
                SimpleOperations.CopyArray(KalmanVars.ErrorConditionVector_p, KalmanVars.ErrorConditionVector_m);
            }
            else
            {
                int datetimeCounter = 0;
                //конец 0
                //SINSstate.endDt[datetimeCounter] = DateTime.Now;
                //SINSstate.startDt[datetimeCounter + 1] = DateTime.Now;
                //datetimeCounter++;

                Matrix tmpMatrix = new Matrix(SimpleData.iMx, SimpleData.iMx),
                    tmpMatrix_2 = new Matrix(SimpleData.iMx, SimpleData.iMx),
                    tmpMatrix_3 = new Matrix(SimpleData.iMx, SimpleData.iMq);

                SimpleOperations.CopyArray(KalmanVars.ErrorConditionVector_m, SimpleOperations.ArrayToMatrix(KalmanVars.TransitionMatrixF) * KalmanVars.ErrorConditionVector_p);

                //SimpleOperations.CopyMatrix(tmpMatrix_3, SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixNoise, SimpleData.iMx, SimpleData.iMq));
                double[] diagonal_sqared_CovarianceMatrixNoise = new double[SimpleData.iMx];
                for (int i = 0; i < SimpleData.iMx; i++)
                    diagonal_sqared_CovarianceMatrixNoise[i] = Math.Pow(KalmanVars.CovarianceMatrixNoise[i * SimpleData.iMx + i], 2);


                SINSstate.startDt[datetimeCounter] = DateTime.Now;

                SimpleOperations.CopyMatrix(tmpMatrix, 
                    SimpleOperations.MultiplyRightUpperMatrix(SimpleOperations.ArrayToMatrix(KalmanVars.TransitionMatrixF), SimpleOperations.ArrayToMatrix(KalmanVars.CovarianceMatrixS_p))
                    );

                SimpleOperations.PrintMatrixToFile(SimpleOperations.MatrixToArray(tmpMatrix), SimpleData.iMx, SimpleData.iMx, "tmpMatrix");

                SINSstate.endDt[datetimeCounter] = DateTime.Now;
                SINSstate.startDt[datetimeCounter + 1] = DateTime.Now;
                datetimeCounter++;

                //SimpleOperations.CopyMatrix(tmpMatrix_2, tmpMatrix * tmpMatrix.Transpose());
                SimpleOperations.CopyMatrix(tmpMatrix_2, SimpleOperations.MultiplyUpperMatrix(tmpMatrix, tmpMatrix.Transpose()));

                SimpleOperations.CopyMatrix(tmpMatrix, tmpMatrix_2 + SimpleOperations.ArrayToDiagonalMatrix(diagonal_sqared_CovarianceMatrixNoise));

                SINSstate.endDt[datetimeCounter] = DateTime.Now;


                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrixS_m, rsb_rsb(SimpleOperations.MatrixToArray(tmpMatrix), SimpleData.iMx));

                SimpleOperations.CopyArray(KalmanVars.CovarianceMatrixS_p, KalmanVars.CovarianceMatrixS_m);
                SimpleOperations.CopyArray(KalmanVars.ErrorConditionVector_p, KalmanVars.ErrorConditionVector_m);
            }

        }







        public static void KalmanCorrection_Align(Kalman_Align KalmanAlign)
        {
            for (int i = 0; i < SimpleData.iMx_Align; i++)
                KalmanAlign.KalmanFactor[i] = 0.0;

            unsafe
            {
                fixed (double* _xm = KalmanAlign.ErrorConditionVector_m, _xp = KalmanAlign.ErrorConditionVector_p, _sm = KalmanAlign.CovarianceMatrixS_m, _sp = KalmanAlign.CovarianceMatrixS_p, _kf = KalmanAlign.KalmanFactor)
                {
                    for (int t = 0; t < KalmanAlign.cnt_measures; t++)
                    {
                        for (int i = 0; i < SimpleData.iMx_Align; i++)
                            KalmanAlign.StringOfMeasure[i] = KalmanAlign.Matrix_H[t * SimpleData.iMx_Align + i];

                        fixed (double* _h = KalmanAlign.StringOfMeasure)
                        {
                            //Коррекция по измерениям
                            f0b(KalmanAlign.Measure[t], _xm, _sm, _h, KalmanAlign.Noize_Z[t] * KalmanAlign.Noize_Z[t], _xp, _sp, _kf, SimpleData.iMx_Align);

                            if (t < KalmanAlign.cnt_measures - 1)
                            {
                                for (int i = 0; i < SimpleData.iMx_Align; i++)
                                {
                                    _xm[i] = _xp[i];
                                    for (int j = 0; j < SimpleData.iMx_Align; j++)
                                        _sm[i * SimpleData.iMx_Align + j] = _sp[i * SimpleData.iMx_Align + j];
                                }
                            }
                        }
                    }
                }
            }
        }
        public static void KalmanForecast_Align(Kalman_Align KalmanAlign)
        {
            unsafe
            {
                fixed (double* _xm = KalmanAlign.ErrorConditionVector_m, _xp = KalmanAlign.ErrorConditionVector_p, _sm = KalmanAlign.CovarianceMatrixS_m, _sp = KalmanAlign.CovarianceMatrixS_p, _f = KalmanAlign.TransitionMatrixF, _sq = KalmanAlign.CovarianceMatrixNoise)
                {
                    KalmanProcs.dgq0b(_xp, _sp, _f, _sq, _xm, _sm, SimpleData.iMx_Align, SimpleData.iMq_Align);
                }
            }
            SimpleOperations.CopyArray(KalmanAlign.CovarianceMatrixS_p, KalmanAlign.CovarianceMatrixS_m);
            SimpleOperations.CopyArray(KalmanAlign.ErrorConditionVector_p, KalmanAlign.ErrorConditionVector_m);
        }
        public static void Make_F_Align(double timeStep, Kalman_Align KalmanAlign)
        {
            unsafe
            {
                fixed (double* _a = KalmanAlign.Matrix_A, _f = KalmanAlign.TransitionMatrixF)
                {
                    func(timeStep, _a, _f, SimpleData.iMx_Align);
                }
            }
        }








        public static void Mult_ss_T(double[] CovarianceMatrixS, double[] CovarianceMatrix_P)
        {
            unsafe
            {
                fixed (double* s = CovarianceMatrixS, c = CovarianceMatrix_P)
                {
                    mult_ss_T(s, c, SimpleData.iMx);
                }
            }
        }
        public static void Mult_ss_T(int i, double[] CovarianceMatrixS, double[,] CovarianceMatrix_P)
        {
            double[] temp_vect = new double[SimpleData.iMx * SimpleData.iMx];
            for (int j = 0; j < SimpleData.iMx * SimpleData.iMx; j++)
                temp_vect[j] = CovarianceMatrix_P[i, j];

            unsafe
            {
                fixed (double* s = CovarianceMatrixS, c = temp_vect)
                {
                    mult_ss_T(s, c, SimpleData.iMx);
                }
            }

            for (int j = 0; j < SimpleData.iMx * SimpleData.iMx; j++)
                CovarianceMatrix_P[i, j] = temp_vect[j];
        }
        /*                                */
        /*           mult_ss_T            */
        /*                                */
        /*                                */
        /*            T                   */
        /*   c = s * s                    */
        /*                                */
        /*                                */
        unsafe static void mult_ss_T(double* s, double* c, int m)
        {
            int i, j, im, jm, k;
            double y;
            for (i = 0; i < m; i++)
            {
                im = i * m;
                for (j = i; j < m; j++)
                {
                    y = 0.0;
                    jm = j * m;
                    for (k = j; k < m; k++) y += s[im + k] * s[jm + k];
                    c[im + j] = y;
                }
            }
        }



        /*                                */
        /*           rsb                  */
        /*                                */
        unsafe static void rsb(double* p, double* s, int m)
        {
            int i, j, k, j1, jj, ij, jm, im;
            double ds, y, c;
            j1 = m * m - 1;
            *(s + j1) = Math.Sqrt(*(p + j1));
            ds = 1.0 / (*(s + j1));
            for (i = 0; i < m - 1; i++)
            {
                j1 = (i + 1) * m - 1;
                *(s + j1) = *(p + j1) * ds;
            }
            if (m > 2)
            {
                for (j = m - 2; j > 0; j--)
                {
                    y = 0.0;
                    j1 = j + 1;
                    jm = j * m;
                    for (k = j1; k < m; k++)
                    {
                        c = *(s + jm + k);
                        y += c * c;
                    }
                    jj = jm + j;
                    *(s + jj) = Math.Sqrt(*(p + jj) - y);
                    ds = 1.0 / (*(s + jj));
                    for (i = j - 1; i >= 0; i--)
                    {
                        im = i * m;
                        y = 0.0;
                        for (k = j1; k < m; k++) y += *(s + im + k) * *(s + jm + k);
                        ij = im + j;
                        *(s + ij) = (*(p + ij) - y) * ds;
                    }
                }
            }
            y = 0.0;
            for (i = 1; i < m; i++)
            {
                c = *(s + i);
                y += c * c;
            }
            *s = Math.Sqrt(*p - y);
            if (*p - y < 0)
            {
                // *s=sqrt(*p);
            }
        }

        /*
*
*_________________________________________________________________
*
*                 *********
*                 *  f0b  *
*                 *********
*
*   f0b(z,xm,sm,h,r,xp,sp,kf,m) - подпрограмма         реализующа
*                                 алгоритм фильтра   Калмана   дл
*                                 этапа    коррекции    измерений.
*                                 (Оценка   плюс    ковариационные
*                                 соотношения).
*
*   Входные параметры:
*
*                m  - размерность системы (вектора состояния);
*                z  - измерение  z (скаляр);         _
*                xm - идентификатор априорной оценки Х(-) (m x 1);
*                sm - идентификатор      априорного     значени
*                     квадратного корня S(-) (m x m), хранящийся в форме вектора;
*                h  - идентификатор вектора измерения h (m x 1);
*                r  - идентификатор  дисперсии шума измерения  z
*                     (скаляр).
*
*   Выходные параметры:
*                                                             _
*                  xp - идентификатор апостериорной  оценки   Х(+)
*                       (m x 1);
*                  sp - идентификатор   апостериорного    значени
*                       квадратного корня S(+) (m x m), хранящийся в форме вектора;
*                  kf - идентификатор  калмановского  коэффициента
*                       усиления (m x 1).
*
*   ВНИМАНИЕ!!!
*   ВНИМАНИЕ!!!   Для  массивов   sm   и   sp   значимыми являютс
*   ВНИМАНИЕ!!!   только их верхнетреугольные области.
*   ВНИМАНИЕ!!!
*
*
*/

        unsafe static void f0b(double z, double* xm, double* sm, double* h, double r, double* xp, double* sp, double* kf, int m)
        {
            int i, j, ji, ii, m1;
            double y1, c, y, zm, al, al1, a;
            m1 = m + 1;
            y1 = *sm;
            c = *h;
            y = c * y1;
            zm = c * *xm;
            al = r + y * y;
            *kf = y1 * y;
            *sp = y1 * Math.Sqrt(r / al);
            for (i = 1; i < m; i++)
            {
                y = *(h + i);
                zm += y * *(xm + i);
                ii = m1 * i;
                y *= *(sm + ii);
                for (j = 0; j < i; j++) y += *(sm + j * m + i) * *(h + j);
                al1 = al + y * y;
                a = 1.0 / Math.Sqrt(al * al1);
                c = y * a;
                a *= al;
                al = al1;
                for (j = 0; j < i; j++)
                {
                    y1 = *(kf + j);
                    ji = j * m + i;
                    al1 = *(sm + ji);
                    *(sp + ji) = al1 * a - y1 * c;
                    *(kf + j) = y1 + al1 * y;
                }
                y1 = *(sm + ii);
                *(sp + ii) = y1 * a;
                *(kf + i) = y1 * y;
            }
            y = 1.0 / al;
            y1 = (z - zm) * y;
            for (i = 0; i < m; i++)
            {
                a = *(kf + i);
                *(xp + i) = *(xm + i) + a * y1;
                *(kf + i) = a * y;
            }
        }


        /*                                */
        /*           f0b НИНЫ БОРИСОВНЫ                  */
        /*                                */
        unsafe static void f0b_nb(double z, double* xm, double* sm, double* h, double r, double* xp, double* sp, double* kf, int m)
        {
            int i, j, ji, ii, m1;
            double y1, c, y, zm, al, al1, a;
            m1 = m + 1;
            y1 = *sm;
            c = *h;
            y = c * y1;
            zm = c * *xm;
            al = r + y * y;
            *kf = y1 * y;
            /*
              if((r/al) < 0.0) {
                  _setcursor(1,1,0);
                  printf(" sqrt < 0 in f0b"); getch();

              }
              */
            *sp = y1 * Math.Sqrt(r / al);
            for (i = 1; i < m; i++)
            {
                y = *(h + i);
                zm += y * *(xm + i);
                ii = m1 * i;
                y *= *(sm + ii);
                for (j = 0; j < i; j++) y += *(sm + j * m + i) * *(h + j);
                al1 = al + y * y;
                /*
             if((al*al1) < 0.0) {
                 _setcursor(1,1,0);
                 printf(" sqrt < 0 in f0b"); getch(); }
                 */
                a = 1.0 / Math.Sqrt(al * al1);
                c = y * a;
                a *= al;
                al = al1;
                for (j = 0; j < i; j++)
                {
                    y1 = *(kf + j);
                    ji = j * m + i;
                    al1 = *(sm + ji);
                    *(sp + ji) = al1 * a - y1 * c;
                    *(kf + j) = y1 + al1 * y;
                }
                y1 = *(sm + ii);
                *(sp + ii) = y1 * a;
                *(kf + i) = y1 * y;
            }
            y = 1.0 / al;
            y1 = (z - zm) * y;
            for (i = 0; i < m; i++)
            {
                a = *(kf + i);
                *(xp + i) = *(xm + i) + a * y1;
                *(kf + i) = a * y;
            }
        }

        /*
*
*_________________________________________________________________
*
*                 ***********
*                 *  dgq0b  *
*                 ***********
*
*   dgq0b(xp,sp,f,sq,xm,sm,m,w) - подпрограмма         реализующа
*                                 алгоритм  фильтра   Калмана  дл
*                                 этапа прогноза.
*                                 (Оценка   плюс    ковариационные
*                                 соотношения).
*                                 Основу вычислительного алгоритма
*                                 составляет             процедура
*                                 ортогонализации  Грамма-Шмидта.
*
*
*   ВНИМАНИЕ!!!
*   ВНИМАНИЕ!!!   В модели задачи шум системы  q=B*u присутствует.
*   ВНИМАНИЕ!!!
*
*   Входные параметры:
*
*                m  - размерность системы (вектора состояния);
*                xp - идентификатор оценки вектора состояния  Х(+)
*                     (m x 1) до прогноза;
*                sp - идентификатор  значения  квадратного корн
*                     S(+) (m x m) до  прогноза, хранящийся в форме вектора;
*                f  - идентификатор переходной матрицы Ф(m x m), хранящийся в форме вектора.
*                sq - идентификатор    корня   sq(m x mq)     из
*                     ковариационной   матрицы  Q(m x m)    шума
*                     системы q(j)=B(j)*u(j), хранящийся в форме вектора:
*                                                       T
*                       sq(j) = B(j)*Sq(j) : sq(j)*sq(j) = Q(j).
*
*   Выходные параметры:
*
*                  xm - идентификатор  оценки  Х(-) (m x 1)  после
*                       прогноза;
*                  sm - идентификатор значения  квадратного  корн
*                       S(-)  (m x m) после  прогноза, хранящийся в форме вектора.
*
*
*   Служебные переменные:
*
*                  w  - одномерный (m x 1) массив.
*
*   ВНИМАНИЕ!!!
*   ВНИМАНИЕ!!!   Для  массивов   sm   и   sp   значимыми являютс
*   ВНИМАНИЕ!!!   только их верхнетреугольные области.
*   ВНИМАНИЕ!!!
*
*   ВНИМАНИЕ!!!
*   ВНИМАНИЕ!!!   В  процессе   работы    подпрограммы  массив  sq
*   ВНИМАНИЕ!!!   портится.
*   ВНИМАНИЕ!!!
*
*/
        unsafe static void dgq0b(double* xp, double* sp, double* f, double* sq, double* xm, double* sm, int m, int mq)
        {
            int i, j, k, ij, im, jq, kq, m1;
            double c, y, y1, dy;
            double[] w = new double[m];
            for (i = 0; i < m; i++)
                w[i] = 0.0;

            fixed (double* _w = w)
            {
                m1 = m + 1;
                for (i = 0; i < m; i++)
                {
                    c = 0.0;
                    im = i * m;
                    for (j = 0; j < m; j++)
                    {
                        y = 0.0;
                        jq = j * m;
                        for (k = 0; k <= i; k++)
                            y += *(sp + k * m + i) * *(f + jq + k);
                        ij = im + j;
                        c += *(f + ij) * *(xp + j);
                        *(sm + ij) = y;
                    }
                    *(xm + i) = c;
                }
                for (k = m - 1; k > 0; k--)
                {
                    y = 0.0;
                    kq = k * mq;

                    for (i = 0; i < m; i++)
                        _w[i] = 0.0;

                    for (i = 0; i < m; i++)
                    {
                        y1 = *(sm + i * m + k);
                        y += y1 * y1;
                    }
                    for (i = 0; i < mq; i++)
                    {
                        y1 = *(sq + kq + i);
                        y += y1 * y1;
                    }
                    y = Math.Sqrt(y);
                    dy = 1.0 / y;
                    for (j = 0; j < k; j++)
                    {
                        y1 = 0.0;
                        jq = j * mq;
                        for (i = 0; i < m; i++)
                        {
                            im = i * m;
                            y1 += *(sm + im + j) * *(sm + im + k);
                        }
                        for (i = 0; i < mq; i++)
                            y1 += *(sq + jq + i) * *(sq + kq + i);
                        y1 *= dy;
                        *(_w + j) = y1;
                        c = y1 * dy;
                        for (i = 0; i < m; i++)
                        {
                            im = i * m;
                            *(sm + im + j) -= (*(sm + im + k)) * c;
                        }
                        for (i = 0; i < mq; i++)
                            *(sq + jq + i) -= (*(sq + kq + i)) * c;
                    }
                    for (i = 0; i < k; i++)
                        *(sm + i * m + k) = *(_w + i);
                    *(sm + m1 * k) = y;
                }
                c = 0.0;
                for (i = 0; i < m; i++)
                {
                    y = *(sm + i * m);
                    c += y * y;
                }
                for (i = 0; i < mq; i++)
                {
                    y = *(sq + i);
                    c += y * y;
                }
                *sm = Math.Sqrt(c);
            }
        }



        /*
        *
        *_________________________________________________________________
        *
        *                 **********
        *                 *  dg0b  *
        *                 **********
        *
        *   dg0b(xp,sp,f,xm,sm,m,w) - подпрограмма   реализующая  алгоритм
        *                             фильтра  Калмана для этапа прогноза.
        *                             (Оценка     плюс      ковариационные
        *                             соотношения).
        *                             Основу    вычислительного  алгоритма
        *                             составляет процедура ортогонализации
        *                             Грамма-Шмидта.
        *
        *   ВНИМАНИЕ!!!
        *   ВНИМАНИЕ!!!   В модели задачи шум системы  q=B*u  отсутствует.
        *   ВНИМАНИЕ!!!
        *
        *   Входные параметры:
        *
        *                m  - размерность системы (вектора состояния);_
        *                xp - идентификатор оценки вектора состояния  Х(+)
        *                     (m x 1) до прогноза;
        *                sp - идентификатор  значения  квадратного корн
        *                     S(+) (m x m) до  прогноза, хранящийся в форме вектора;
        *                f  - идентификатор переходной матрицы Ф(m x m),
        *                     хранящийся в форме вектора.
        *
        *   Выходные параметры:
        *                                              _
        *                  xm - идентификатор  оценки  Х(-) (m x 1)  после
        *                       прогноза;
        *                  sm - идентификатор значения  квадратного  корн
        *                       S(-)  (m x m) после  прогноза, хранящийся в форме вектора.
        *
        *   Служебные переменные:
        *
        *                  w  - одномерный (m x 1) массив.
        *
        *   ВНИМАНИЕ!!!
        *   ВНИМАНИЕ!!!   Для  массивов   sm   и   sp   значимыми являютс
        *   ВНИМАНИЕ!!!   только их верхнетреугольные области.
        *   ВНИМАНИЕ!!!
        *
        */
        //unsafe static void dgq0b(double* xp, double* sp, double* f, double* sq, double* xm, double* sm, int m, int mq)
        unsafe static void dg0b(double* xp, double* sp, double* f, double* xm, double* sm, int m)
        {
            int i, j, k, ij, im, jm, m1;
            double c, y, y1, dy;
            double[] w = new double[100];

            fixed (double* _w = w)
            {
                for (i = 0; i < m; i++) _w[i] = 0.0;
                m1 = m + 1;
                for (i = 0; i < m; i++)
                {
                    c = 0.0;
                    im = i * m;
                    for (j = 0; j < m; j++)
                    {
                        y = 0.0;
                        jm = j * m;
                        for (k = 0; k <= i; k++) y += *(sp + k * m + i) * *(f + jm + k);
                        ij = im + j;
                        c += *(f + ij) * *(xp + j);
                        *(sm + ij) = y;
                    }
                    *(xm + i) = c;
                }
                for (k = m - 1; k > 0; k--)
                {
                    y = 0.0;
                    for (i = 0; i < m; i++) _w[i] = 0.0;
                    for (i = 0; i < m; i++)
                    {
                        y1 = *(sm + i * m + k);
                        y += y1 * y1;
                    }
                    y = Math.Sqrt(y);
                    dy = 1.0 / y;
                    for (j = 0; j < k; j++)
                    {
                        y1 = 0.0;
                        for (i = 0; i < m; i++)
                        {
                            im = i * m;
                            y1 += *(sm + im + j) * *(sm + im + k);
                        }
                        y1 *= dy;
                        *(_w + j) = y1;
                        c = y1 * dy;
                        for (i = 0; i < m; i++)
                        {
                            im = i * m;
                            *(sm + im + j) -= (*(sm + im + k)) * c;
                        }
                    }
                    for (i = 0; i < k; i++) *(sm + i * m + k) = *(_w + i);
                    *(sm + m1 * k) = y;
                }
                c = 0.0;
                for (i = 0; i < m; i++)
                {
                    y = *(sm + i * m);
                    c += y * y;
                }
                *sm = Math.Sqrt(c);
            }
        }




        /*

     smooth_sub


               |    -1             -1       |
     x = P(*)* | P(+) *x(+) + іPb(-) *xb(-) |
               |                            |

            |    -1         -1 |-1
     P(*) = | P(+)   +  Pb(-)  |
            |                  |

                  T
     P(*) = S(*)*S (*)  

     S(*) -  s

                  T
     P(+) = S(+)*S (+)  

     S(+) - sp

                     T
     Pb(-) = Sb(-)*Sb (-)  

     Sb(-)  - sm

     xb(-) -  xm

     x(+) -  xp

     x -  x

     m - dimension

     dsp,dsm - (m x m)


*/

        unsafe static void smooth_sub(double* xp, double* sp, double* xm, double* sm, double* x, double* s, int m, double* dsp, double* dsm)
        {
            double y, c, ds, yp, ym;
            int i, j, k, im, ij, jm, km, ik, kj, jj, ji, ki, j1;
            /* _________________________ 1 _________________________________

                      Ъ    ї-1
                dsp = іS(+)і
                      А    Щ
                      Ъ     ї-1
                s =   іSb(-)і
                      А     Щ
            */
            for (i = 0; i < m; i++)
            {
                im = i * m + i;
                dsp[im] = 1.0 / sp[im];
                s[im] = 1.0 / sm[im];
            }
            if (m >= 2)
            {
                for (i = 0; i < m - 1; i++)
                {
                    im = i * m;
                    for (j = i + 1; j < m; j++)
                    {
                        yp = 0.0;
                        ym = 0.0;
                        ij = im + j;
                        jj = j * m + j;
                        for (k = i; k <= j - 1; k++)
                        {
                            ik = im + k;
                            kj = k * m + j;
                            yp -= dsp[ik] * sp[kj];
                            ym -= s[ik] * sm[kj];
                        }
                        dsp[ij] = yp * dsp[jj];
                        s[ij] = ym * s[jj];
                    }
                }
            }
            /* _________________________ 2 _________________________________

                      Ъ    ї-TЪ    ї-1       Ъ     ї-TЪ     ї-1
                x   = іS(+)і *іS(+)і *x(+) + іSb(-)і *іSb(-)і *xb(-) = y
                      А    Щ  А    Щ         А     Щ  А     Щ
            */
            for (i = 0; i < m; i++)
            {
                yp = 0.0;
                ym = 0.0;
                im = i * m;
                for (j = i; j < m; j++)
                {
                    ij = im + j;
                    yp += dsp[ij] * xp[j];
                    ym += s[ij] * xm[j];
                }
                dsm[im] = yp;
                dsm[im + 1] = ym;
            }
            for (i = 0; i < m; i++)
            {
                yp = 0.0;
                for (j = 0; j <= i; j++)
                {
                    jm = j * m;
                    ji = jm + i;
                    yp += dsp[ji] * dsm[jm] + s[ji] * dsm[jm + 1];
                }
                x[i] = yp;
            }
            /* _________________________ 3 _________________________________

                       Ъ    ї-TЪ    ї-1   Ъ     ї-TЪ     ї-1
                dsm  = іS(+)і *іS(+)і   + іSb(-)і *іSb(-)і   = P(*)
                       А    Щ  А    Щ     А     Щ  А     Щ
            */
            for (i = 0; i < m; i++)
            {
                im = i * m;
                for (j = 0; j <= i; j++)
                {
                    yp = 0.0;
                    for (k = 0; k <= j; k++)
                    {
                        km = k * m;
                        ki = km + i;
                        kj = km + j;
                        yp += dsp[ki] * dsp[kj] + s[ki] * s[kj];
                    }
                    dsm[im + j] = yp;
                }
            }
            /* _________________________ 4 _________________________________

                             1               1
                             -               -
                       Ъ    ї2         Ъ    ї2
                dsp  = іdsm і  = S­ =  іP(*)і
                       А    Щ          А    Щ
            */
            dsp[0] = Math.Sqrt(dsm[0]);
            ds = 1.0 / dsp[0];
            for (i = 1; i < m; i++)
            {
                im = i * m;
                dsp[im] = dsm[im] * ds;
            }
            if (m >= 2)
            {
                for (j = 1; j < m - 1; j++)
                {
                    j1 = j - 1;
                    y = 0.0;
                    jm = j * m;
                    jj = jm + j;
                    for (k = 0; k <= j1; k++)
                    {
                        c = dsp[jm + k];
                        y += c * c;
                    }
                    dsp[jj] = Math.Sqrt(dsm[jj] - y);
                    ds = 1.0 / dsp[jj];
                    for (i = j + 1; i < m; i++)
                    {
                        y = 0.0;
                        im = i * m;
                        ij = im + j;
                        for (k = 0; k <= j1; k++) y += dsp[im + k] * dsp[jm + k];
                        dsp[ij] = (dsm[ij] - y) * ds;
                    }
                }
            }
            y = 0.0;
            im = m * m - 1;
            jm = m * m - m;
            for (i = 0; i < m - 1; i++)
            {
                c = dsp[jm + i];
                y += c * c;
            }
            dsp[im] = Math.Sqrt(dsm[im] - y);
            /* _________________________ 5 _________________________________

                      Ъ    ї-T         Ъ  ї-1T
                s =   іdsp і  = S(*) = іS­і
                      А    Щ           А  Щ
            */
            for (i = 0; i < m; i++)
            {
                im = i * m + i;
                s[im] = 1.0 / dsp[im];
            }
            if (m >= 2)
            {
                for (i = 1; i < m; i++)
                {
                    im = i * m;
                    for (j = i - 1; j >= 0; j--)
                    {
                        y = 0.0;
                        for (k = j + 1; k <= i; k++) y -= s[im + k] * dsp[k * m + j];
                        s[im + j] = y * s[j * m + j];
                    }
                }
                for (i = 0; i < m - 1; i++)
                {
                    im = i * m;
                    for (j = i + 1; j < m; j++) s[im + j] = s[j * m + i];
                }
            }
            /* _________________________ 6 _________________________________

                     T               T  іЪ    ї-TЪ    ї-1       Ъ     ї-TЪ     ї-1     і
              x = s*s *y = S(*)*S (*) * ііS(+)і *іS(+)і *x(+) + іSb(-)і *іSb(-)і *xb(-)і
                                        іА    Щ  А    Щ         А     Щ  А     Щ       і


            */
            for (i = 0; i < m; i++)
            {
                yp = 0.0;
                for (j = 0; j <= i; j++) yp += s[j * m + i] * x[j];
                dsm[i * m] = yp;
            }
            for (i = 0; i < m; i++)
            {
                yp = 0.0;
                im = i * m;
                for (j = i; j < m; j++) yp += s[im + j] * dsm[j * m];
                x[i] = yp;
            }
        }




        /*                                */
        /*           dgq0b   НИНЫ БОРИСОВНЫ              */
        /*                                */
        unsafe static void dgq0b_nb(double* xp, double* sp, double* f, double* sq, double* xm, double* sm, int m, int mq)
        {
            int i, j, k, ij, im, jq, kq, m1;
            double c, y, y1, dy;
            //double *w = new double [m];
            double[] w = new double[m];
            //double sqn[Mx*Mx];
            for (i = 0; i < m; i++) w[i] = 0.0;
            m1 = m + 1;
            fixed (double* _w = w)
            {
                for (i = 0; i < m; i++)
                {
                    c = 0.0;
                    im = i * m;
                    for (j = 0; j < m; j++)
                    {
                        y = 0.0;
                        jq = j * m;
                        for (k = 0; k <= i; k++) y += *(sp + k * m + i) * *(f + jq + k);
                        ij = im + j;
                        c += *(f + ij) * *(xp + j);
                        *(sm + ij) = y;
                    }
                    *(xm + i) = c;
                }
                for (k = m - 1; k > 0; k--)
                {
                    y = 0.0;
                    kq = k * mq;
                    for (i = 0; i < m; i++) _w[i] = 0.0;
                    for (i = 0; i < m; i++)
                    {
                        y1 = *(sm + i * m + k);
                        y += y1 * y1;
                    }
                    for (i = 0; i < mq; i++)
                    {
                        y1 = *(sq + kq + i);
                        y += y1 * y1;
                    }
                    y = Math.Sqrt(y);
                    dy = 1.0 / y;
                    for (j = 0; j < k; j++)
                    {
                        y1 = 0.0;
                        jq = j * mq;
                        for (i = 0; i < m; i++)
                        {
                            im = i * m;
                            y1 += *(sm + im + j) * *(sm + im + k);
                        }
                        for (i = 0; i < mq; i++) y1 += *(sq + jq + i) * *(sq + kq + i);
                        y1 *= dy;
                        *(_w + j) = y1;
                        c = y1 * dy;
                        for (i = 0; i < m; i++)
                        {
                            im = i * m;
                            *(sm + im + j) -= (*(sm + im + k)) * c;
                        }
                        for (i = 0; i < mq; i++) *(sq + jq + i) -= (*(sq + kq + i)) * c;
                    }
                    for (i = 0; i < k; i++) *(sm + i * m + k) = *(_w + i);
                    *(sm + m1 * k) = y;
                }
                c = 0.0;
                for (i = 0; i < m; i++)
                {
                    y = *(sm + i * m);
                    c += y * y;
                }
                for (i = 0; i < mq; i++)
                {
                    y = *(sq + i);
                    c += y * y;
                }
                *sm = Math.Sqrt(c);
            }
            //delete [] w;
        }

        /*
        *
        *_________________________________________________________________
        *
        *                 **********
        *                 *  func  *
        *                 **********
        *
        *   func(dt,a,f,m) - подпрограмма вычисления переходной матрицы  f
        *                    (m x m)  линейной  динамической   системы  по
        *                    значению матрицы   a(m x m)   соответствующей
        *                    непрерывной системы.
        *                    (Используется   численный    метод    второго
        *                    порядка).
        *
        *
        *                    f = E + (dt * a) + 0.5 ( dt * dt * a * a)
        *
        *                    (E - единичная матрица)
        *
        *
        *   Входные параметры:
        *
        *                 m  - размерность матриц  a  и  f;
        *                 dt - квантование времени системы;
        *                 a  - двумерный   массив  (m x m)   -     матрица
        *                      непрерывной модели системы, хранящийся в форме вектора.
        *
        *   Выходные параметры:
        *
        *                 f  - двумерный   массив   (m x m)  -  переходна
        *                      матрица системы, хранящийся в форме вектора.
        *
        */

        unsafe static void func(double dt, double* a, double* f, int m)
        {
            int i, j, k, ij, im;
            double c, y;
            c = 0.5 * dt * dt;
            for (i = 0; i < m; i++)
            {
                im = i * m;
                for (j = 0; j < m; j++)
                {
                    y = 0.0;
                    for (k = 0; k < m; k++) y += *(a + im + k) * *(a + k * m + j);
                    ij = im + j;
                    y = dt * *(a + ij) + c * y;
                    if (i == j) y = 1.0 + y;
                    *(f + ij) = y;
                }
            }
        }

        unsafe static void Make_F_nb(double dDt, double* pdA, double* pdF, int iM)
        {
            int i, j, ij, im;
            double dY;

            for (i = 0; i < (iM * iM); i++) pdF[i] = 0.0;

            for (i = 0; i < iM; i++)
            {
                im = i * iM;
                for (j = 0; j < iM; j++)
                {
                    ij = im + j;
                    dY = dDt * pdA[ij];
                    if (i == j) dY += 1.0;
                    pdF[ij] = dY;
                }
            }
        }
    }
}
