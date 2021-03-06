﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace Common_Namespace
{
    public partial class Odometr_SINS
    {
        //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------ПОЗИЦИОННАЯ КОРЕКЦИЯ-----------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        public static void Make_H_POSITION(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod, Proc_Help ProcHelp)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;


            double[] tempVect = new double[3];

            //---Разбиение на три составляющие---
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 1] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 0] = -1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo + 1] = -1.0;

            //Формирование измерений по географическим координатам
            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - SINSstate_OdoMod.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
            KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[0, 1] * KalmanVars.OdoNoise_Dist;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.A_x0s[1, 1] * KalmanVars.OdoNoise_Dist;

            KalmanVars.cnt_measures += 2;



            //double[] deltaOdoVSsins = new double[3], deltaOdoVSsins_x = new double[3];
            //if (SINSstate.Global_file == "Saratov_run_2014_07_23")
            //{
            //    deltaOdoVSsins[0] = 0.617 * Math.Cos(11.25 * SimpleData.ToRadian);
            //    deltaOdoVSsins[1] = -0.917;
            //    deltaOdoVSsins[2] = 0.617 * Math.Sin(11.25 * SimpleData.ToRadian);
            //    SimpleOperations.CopyArray(deltaOdoVSsins_x, SINSstate.A_x0s * deltaOdoVSsins);

            //    for (int i = 0; i < 3; i++)
            //        deltaOdoVSsins[i] = deltaOdoVSsins[i];

            //    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] += deltaOdoVSsins_x[0];
            //    KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] += deltaOdoVSsins_x[1];
            //}


            //---ДОПОЛНИТЕЛЬНОЕ измерение по вертикальной скорости---
            if (SINSstate.flag_iMx_r3_dV3)
            {
                if (SINSstate.add_velocity_to_position)
                {
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Vx_0[2] - SINSstate_OdoMod.Vx_0[2];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3 + 1] = 1.0;
                    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_V;

                    if (SINSstate.flag_iMx_kappa_13_ds)
                    {
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 2];
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 0];
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 1];
                    }

                    KalmanVars.cnt_measures += 1;
                }
                else if (!SINSstate.add_velocity_to_position)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3] = 1.0;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 2] = -1.0;
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Altitude - SINSstate_OdoMod.Altitude;
                    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_Dist;
                    //KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_Dist;

                    //if (SINSstate.Global_file == "Saratov_run_2014_07_23")
                    //{
                    //    KalmanVars.Measure[(KalmanVars.cnt_measures + 2)] += deltaOdoVSsins_x[2];
                    //}

                    KalmanVars.cnt_measures += 1;
                }
            }


        }




        public static void Make_H_CONTROLPOINTS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod, double Latitude_CP, double Longitude_CP, double Altitude_CP)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;

            SINSstate.flag_UsingCorrection = true;

            double[] tempVect = new double[3];


            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 1] = 1.0;
            //Формирование измерений по географическим координатам
            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - Longitude_CP) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
            KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - Latitude_CP) * SINSstate.R_n;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Imitator_GPS_PositionError;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.Imitator_GPS_PositionError;


            KalmanVars.cnt_measures += 2;

            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Altitude - Altitude_CP;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Imitator_GPS_PositionError;

                KalmanVars.cnt_measures += 1;
            }


            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 0] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo + 1] = 1.0;
            //Формирование измерений по географическим координатам
            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate_OdoMod.Longitude - Longitude_CP) * SINSstate_OdoMod.R_e * Math.Cos(SINSstate_OdoMod.Latitude);
            KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate_OdoMod.Latitude - Latitude_CP) * SINSstate_OdoMod.R_n;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Imitator_GPS_PositionError;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.Imitator_GPS_PositionError;

            KalmanVars.cnt_measures += 2;

            if (SINSstate.flag_Using_iMx_r_odo_3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 2] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate_OdoMod.Altitude - Altitude_CP;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Imitator_GPS_PositionError;

                KalmanVars.cnt_measures += 1;
            }


            //---АЛГЕБРАИЧЕСКАЯ КАЛИБРОВКА---//
            if (SINSstate.flag_using_GoCalibrInCP == true && SINSstate.flag_iMx_kappa_13_ds)
            {
                double lat_dif_calc = (SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n;
                double long_dif_calc = (SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
                double lat_dif_true = (Latitude_CP - SINSstate.Latitude_Start) * SimpleOperations.RadiusN(Latitude_CP, Altitude_CP);
                double long_dif_true = (Longitude_CP - SINSstate.Longitude_Start) * SimpleOperations.RadiusE(Latitude_CP, Altitude_CP) * Math.Cos(Latitude_CP);


                double l_calc = Math.Sqrt(Math.Pow(lat_dif_calc, 2) + Math.Pow(long_dif_calc, 2));
                double l_true = Math.Sqrt(Math.Pow(lat_dif_true, 2) + Math.Pow(long_dif_true, 2));

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = 1.0;
                if (SINSstate.flag_FeedbackExist)
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.OdometerData.odometer_left.Value / (1 + SINSstate.ComulativeKappaEst[1]) - l_true) / l_true;
                else
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.OdometerData.odometer_left.Value - l_true) / l_true;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 0.01;
                KalmanVars.cnt_measures += 1;

                double tmp = (SINSstate.OdometerData.odometer_left.Value / (1 + SINSstate.ComulativeKappaEst[1]) - l_true) / l_true;
                double tst = Math.Atan2(long_dif_calc, lat_dif_calc);
                double tst2 = Math.Atan2(long_dif_true, lat_dif_true);

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = Math.Tan(SINSstate.Latitude) / SINSstate.R_e;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 4] = -Math.Sin(SINSstate.Heading) * Math.Tan(SINSstate.Pitch);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 5] = -Math.Cos(SINSstate.Heading) * Math.Tan(SINSstate.Pitch);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 6] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = 1.0;
                if (SINSstate.flag_FeedbackExist)
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Heading - SINSstate.ComulativeKappaEst[2] - Math.Atan2(long_dif_true, lat_dif_true);
                else
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Heading - Math.Atan2(long_dif_true, lat_dif_true);
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 1.0 * SimpleData.ToRadian_min;
                KalmanVars.cnt_measures += 1;

                double tmp2 = SINSstate.Heading - SINSstate.ComulativeKappaEst[2] - Math.Atan2(long_dif_true, lat_dif_true);

                if (SINSstate.flag_Using_iMx_r_odo_3)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 4] = -Math.Cos(SINSstate.Heading);
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 5] = Math.Sin(SINSstate.Heading);
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = 1.0;
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Altitude - Altitude_CP;
                    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 0.1;
                    KalmanVars.cnt_measures += 1;
                }
            }
        }





        //--------------------------------------------------------------------------
        //--------------------------СКОРОСТНАЯ КОРЕКЦИЯ-----------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_VELOCITY(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;

            for (int i = 0; i < 3; i++)
                KalmanVars.Measure[(KalmanVars.cnt_measures + i)] = SINSstate.Vx_0[i] - SINSstate_OdoMod.Vx_0[i];

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 2] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 3] = 1.0;

            //---КОРРЕКТИРОВАНИЕ ПО СКОРОСТИ В ПРОЕКЦИИ НА ГЕОГРАФИЮ---
            if (SINSstate.flag_OdoSINSWeakConnect_MODIF)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = -SINSstate.Omega_x[0] * Math.Tan(SINSstate.Latitude);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo] = SINSstate.Omega_x[0] * Math.Tan(SINSstate.Latitude);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 0] = -SINSstate.Omega_x[2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo] = SINSstate.Omega_x[2];
            }

            if (SINSstate.flag_iMx_kappa_13_ds)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[0, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[0, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[0, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[1, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 1] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[1, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 2] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[1, 1];
            }

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[0, 1] * KalmanVars.OdoNoise_V;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.A_x0s[1, 1] * KalmanVars.OdoNoise_V;

            KalmanVars.cnt_measures += 2;


            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3 + 1] = 1.0;
                //ТУТ ПО ХОРОШЕМУ НАДО РАЗБИТЬ НА МОДИФИЦИРОВАННЫЙ И СЛАБОСВЯЗАННЫЕ ВАРИАНТЫ
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_V;

                if (SINSstate.flag_iMx_kappa_13_ds)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 2];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 0];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 1];
                }

                KalmanVars.cnt_measures += 1;
            }

        }












        public static void Make_A(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;


            SINSprocessing.Make_A(SINSstate, KalmanVars, SINSstate_OdoMod);


            /*-----------МОДИФИЦИРОВАННЫЙ СЛАБОСВЗАННЫЙ ВАРИАНТ----------------*/
            if (SINSstate.flag_OdoSINSWeakConnect_MODIF)
            {
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + 6] = SINSstate_OdoMod.Vx_0[1];
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 1] = SINSstate_OdoMod.Omega_x[2];

                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 6] = -SINSstate_OdoMod.Vx_0[0];
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r12_odo + 0] = -SINSstate_OdoMod.Omega_x[2];


                if (SINSstate.flag_iMx_r3_dV3)
                {
                    KalmanVars.Matrix_A[iMx_r12_odo * iMx + 0] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_e;
                    KalmanVars.Matrix_A[iMx_r12_odo * iMx + 5] += -SINSstate_OdoMod.Vx_0[2];
                    KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 2] = -SINSstate_OdoMod.Omega_x[1];

                    KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 1] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_n;
                    KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 4] += SINSstate_OdoMod.Vx_0[2];
                    KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r12_odo + 2] = SINSstate_OdoMod.Omega_x[0];
                }

                if (SINSstate.flag_Using_iMx_r_odo_3)
                {
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + 0] = -SINSstate_OdoMod.Vx_0[0] / SINSstate_OdoMod.R_e;
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + 1] = -SINSstate_OdoMod.Vx_0[1] / SINSstate_OdoMod.R_n;
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + 4] = -SINSstate_OdoMod.Vx_0[1];
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + 5] = SINSstate_OdoMod.Vx_0[0];
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + iMx_r12_odo + 0] = SINSstate_OdoMod.Omega_x[1];
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + iMx_r12_odo + 1] = -SINSstate_OdoMod.Omega_x[0];
                }
            }
            /*-----------СЛАБОСВЗАННЫЙ ВАРИАНТ----------------*/
            else if (SINSstate.flag_OdoSINSWeakConnect)
            {
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + 6] = SINSstate_OdoMod.Vx_0[1];
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + 0] = -SINSstate_OdoMod.Omega_x[0] * Math.Tan(SINSstate_OdoMod.Latitude);
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 0] = SINSstate_OdoMod.Omega_x[0] * Math.Tan(SINSstate_OdoMod.Latitude);
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 1] = SINSstate_OdoMod.Omega_x[2];

                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 6] = -SINSstate_OdoMod.Vx_0[0];
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 0] = -SINSstate_OdoMod.Omega_x[2];

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    KalmanVars.Matrix_A[iMx_r12_odo * iMx + 5] += -SINSstate_OdoMod.Vx_0[2];
                    KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 0] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_e;
                    KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 2] = -SINSstate_OdoMod.Omega_x[1];

                    KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 4] += SINSstate_OdoMod.Vx_0[2];
                    KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r12_odo + 1] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_n;
                    KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r12_odo + 2] = SINSstate_OdoMod.Omega_x[0];
                }

                if (SINSstate.flag_Using_iMx_r_odo_3)
                {
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + 4] = -SINSstate_OdoMod.Vx_0[1];
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + 5] = SINSstate_OdoMod.Vx_0[0];
                }
            }



            if (SINSstate.flag_iMx_kappa_13_ds)
            {
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_odo_model + 0] = -SINSstate_OdoMod.A_x0s[0, 2] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_odo_model + 1] = SINSstate_OdoMod.A_x0s[0, 0] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_odo_model + 2] = SINSstate_OdoMod.A_x0s[0, 1] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);

                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_odo_model + 0] = -SINSstate_OdoMod.A_x0s[1, 2] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_odo_model + 1] = SINSstate_OdoMod.A_x0s[1, 0] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_odo_model + 2] = SINSstate_OdoMod.A_x0s[1, 1] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);

                if (SINSstate.flag_Using_iMx_r_odo_3)
                {
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + iMx_odo_model + 0] = -SINSstate_OdoMod.A_x0s[2, 2] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + iMx_odo_model + 1] = SINSstate_OdoMod.A_x0s[2, 0] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);
                    KalmanVars.Matrix_A[(iMx_r12_odo + 2) * iMx + iMx_odo_model + 2] = SINSstate_OdoMod.A_x0s[2, 1] * SimpleOperations.AbsoluteVectorValue(SINSstate_OdoMod.OdoSpeed_x0);
                }
            }


            //if (SINSstate.Count % 1000 == 0)
            //{
            //    SimpleOperations.PrintMatrixToFile(KalmanVars.Matrix_A, SimpleData.iMx, SimpleData.iMx);
            //    SimpleOperations.PrintMatrixToFile(KalmanVars.CovarianceMatrixS_m, SimpleData.iMx, SimpleData.iMx);
            //}
        }














        public static void InitOfCovarianceMatrixes(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);

            KalmanVars.CovarianceMatrixS_m[SINSstate.iMx_r12_odo * SimpleData.iMx + SINSstate.iMx_r12_odo]
                = KalmanVars.CovarianceMatrixS_p[SINSstate.iMx_r12_odo * SimpleData.iMx + SINSstate.iMx_r12_odo] = SINSstate.stdOdoR;
            KalmanVars.CovarianceMatrixS_m[(SINSstate.iMx_r12_odo + 1) * SimpleData.iMx + (SINSstate.iMx_r12_odo + 1)]
                = KalmanVars.CovarianceMatrixS_p[(SINSstate.iMx_r12_odo + 1) * SimpleData.iMx + (SINSstate.iMx_r12_odo + 1)] = SINSstate.stdOdoR;

            if (SINSstate.flag_Using_iMx_r_odo_3)
                KalmanVars.CovarianceMatrixS_m[(SINSstate.iMx_r12_odo + 2) * SimpleData.iMx + (SINSstate.iMx_r12_odo + 2)]
                    = KalmanVars.CovarianceMatrixS_p[(SINSstate.iMx_r12_odo + 2) * SimpleData.iMx + (SINSstate.iMx_r12_odo + 2)] = SINSstate.stdOdoR;

        }



        public static void MatrixNoise_ReDef(SINS_State SINSstate, Kalman_Vars KalmanVars, bool AlignmentFLG)
        {
            int iMx = SimpleData.iMx, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;

            int tmpCounter = SINSprocessing.MatrixNoise_ReDef(SINSstate, KalmanVars, SINSstate.flag_Alignment);

            double sqrt_freq = Math.Sqrt(SINSstate.Freq);
            if (SINSstate.flag_iMqDeltaRodo)
            {
                KalmanVars.CovarianceMatrixNoise[(iMx_r12_odo + 0) * iMq + tmpCounter + 0] = KalmanVars.Noise_Pos * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[(iMx_r12_odo + 1) * iMq + tmpCounter + 1] = KalmanVars.Noise_Pos * sqrt_freq;

                if (SINSstate.flag_Using_iMx_r_odo_3)
                    KalmanVars.CovarianceMatrixNoise[(iMx_r12_odo + 2) * iMq + tmpCounter + 2] = KalmanVars.Noise_Pos * sqrt_freq;
            }

            //SimpleOperations.PrintMatrixToFile(KalmanVars.CovarianceMatrixNoise, iMx, iMq);

        }

    }
}
