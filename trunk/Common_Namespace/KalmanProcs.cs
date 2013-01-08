using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class KalmanProcs
    {
        public static void Make_F(double timeStep, Kalman_Vars KalmanVars)
        {
            unsafe
            {
                fixed (double* _a = KalmanVars.Matrix_A, _f = KalmanVars.TransitionMatrixF)
                {
                    func(timeStep, _a, _f, SimpleData.iMx);
                }
            }
        }

        public static void KalmanCorrection(Kalman_Vars KalmanVars)
        {
            if (true)
            {
                for (int i = 0; i < KalmanVars.KalmanFactor.Length; i++)
                    KalmanVars.KalmanFactor[i] = 0.0;
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
                Matrix CovarianceMatrixS_p = new Matrix(SimpleData.iMx, SimpleData.iMx), CovarianceMatrixS_m = new Matrix(SimpleData.iMx, SimpleData.iMx);

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
                        for (int i = 0; i < SimpleData.iMx; i++)
                        {
                            KalmanVars.ErrorConditionVector_m[i] = KalmanVars.ErrorConditionVector_p[i];
                            for (int j = 0; j < SimpleData.iMx; j++)
                                KalmanVars.CovarianceMatrixS_m[i * SimpleData.iMx + j] = KalmanVars.CovarianceMatrixS_p[i * SimpleData.iMx + j];
                        }
                    }
                }
            }
            
        }



        public static void KalmanForecast(Kalman_Vars KalmanVars)
        {
            unsafe
            {
                fixed (double* _xm = KalmanVars.ErrorConditionVector_m, _xp = KalmanVars.ErrorConditionVector_p, _sm = KalmanVars.CovarianceMatrixS_m, _sp = KalmanVars.CovarianceMatrixS_p, _f = KalmanVars.TransitionMatrixF, _sq = KalmanVars.CovarianceMatrixNoise)
                {
                    dgq0b(_xp, _sp, _f, _sq, _xm, _sm, SimpleData.iMx, SimpleData.iMq);
                }
            }
            SimpleOperations.CopyArray(KalmanVars.CovarianceMatrixS_p, KalmanVars.CovarianceMatrixS_m);
            SimpleOperations.CopyArray(KalmanVars.ErrorConditionVector_p, KalmanVars.ErrorConditionVector_m);
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
                        w[i] = 0.0;

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
    }
}
