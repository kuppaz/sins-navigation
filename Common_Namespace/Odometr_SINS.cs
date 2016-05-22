using System;
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
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_kappa_1 = SINSstate.value_iMx_kappa_1,
                iMx_r12_odo = SINSstate.value_iMx_r_odo_12, value_iMx_dr3 = SINSstate.value_iMx_dr3, value_iMx_dV3 = SINSstate.value_iMx_dV3;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3,
                iMx_r_odo_3 = SINSstate.value_iMx_r_odo_3
                ;


            double[] tempVect = new double[3];

            // --- шум измерения. Если объект стоит - уменьшаем
            double Noize = 1.0;
            double longOdoIncrement = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeft_ArrayOfPrev[Math.Min(20, SINSstate.OdometerLeft_ArrayOfPrev.Length)];
            double longOdoIncrement_dt = SINSstate.Time + SINSstate.Time_Alignment - SINSstate.OdometerLeft_ArrayOfPrevTime[Math.Min(20, SINSstate.OdometerLeft_ArrayOfPrev.Length)];
            if (longOdoIncrement / longOdoIncrement_dt == 0.0)
            {
                Noize = 0.01;

                // -- Блок формирования измерений по разнице координат БИНС и внешней информации
                //KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
                //KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 1] = 1.0;
                ////Формирование измерений по географическим координатам
                //KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - SINSstate_OdoMod.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
                //KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n;

                //// --- шум измерения
                //KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 0.1;
                //KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = 0.1;

                //KalmanVars.cnt_measures += 2;


                //KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + 0] = 1.0;
                //KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.Height - SINSstate_OdoMod.Height;
                //KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = 0.1;
                //KalmanVars.Vertical_cnt_measures += 1;

                //// -- Блок формирования измерений по разнице координат одометрического решения и внешней информации
                //KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 0] = 1.0;
                //KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo + 1] = 1.0;
                ////Формирование измерений по географическим координатам
                //KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate_OdoMod.Longitude - Longitude_CP) * SINSstate_OdoMod.R_e * Math.Cos(SINSstate_OdoMod.Latitude);
                //KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate_OdoMod.Latitude - Latitude_CP) * SINSstate_OdoMod.R_n;

                //// --- шум измерения
                //KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = Noize;
                //KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = Noize;

                //KalmanVars.cnt_measures += 2;

                //if (Convert.ToInt32(SINSstate.Count) % 50 == 0)
                //{
                //    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 0)] = 1.0;
                //    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + (iMx_dV_12 + 1)] = 1.0;

                //    for (int i = 0; i < 3; i++)
                //        KalmanVars.Measure[(KalmanVars.cnt_measures + i)] = SINSstate.Vx_0[i];

                //    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 0.1;
                //    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = 0.1;

                //    KalmanVars.cnt_measures += 2;
                //}
            }

            //---Разбиение на три составляющие---
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 1] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 0] = -1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo + 1] = -1.0;

            //Формирование измерений по географическим координатам
            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - SINSstate_OdoMod.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
            KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 1.0;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = 1.0;

            KalmanVars.cnt_measures += 2;


            //---ДОПОЛНИТЕЛЬНОЕ измерение по вертикальной скорости---
            if (SINSstate.flag_iMx_r3_dV3)
            {
                if (SINSstate.add_velocity_to_position)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dV3] = 1.0;
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Vx_0[2] - SINSstate_OdoMod.Vx_0[2];
                    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_V;

                    if (SINSstate.flag_iMx_kappa_13_ds)
                    {
                        if (iMx_kappa_1 > 0)
                            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_1 + 0] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 2] + SINSstate.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[2, 1];

                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 0] - SINSstate.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[2, 1];
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 1] = -SINSstate.OdoSpeed_x0[2];
                    }

                    KalmanVars.cnt_measures += 1;
                }
                else if (!SINSstate.add_velocity_to_position)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dr3] = 1.0;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r_odo_3] = -1.0;

                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Height - SINSstate_OdoMod.Height;
                    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_Dist;

                    KalmanVars.cnt_measures += 1;
                }
            }



            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                // --- Если в данный момент выставлен флаг коррекции по Vertical_ZUPT, то здесь не добавляем измерение, добавляем в спец. функции для ZUPT --- //
                if (SINSstate.flag_Vertical_ZUPT == false)
                {
                    KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + 0] = 1.0;
                    KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + SINSstate.Vertical_rOdo3] = -1.0;

                    KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.Height - SINSstate_OdoMod.Height;
                    KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = Noize * SINSstate.OdoVerticalNoiseMultiplicator;
                    //KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.OdoAcceleration_s;

                    KalmanVars.Vertical_cnt_measures += 1;
                }
            }
        }




        public static void Make_H_CONTROLPOINTS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod, double Latitude_CP, double Longitude_CP, double Altitude_CP)
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

            double[] tempVect = new double[3];


            if (Latitude_CP != 0 && Longitude_CP != 0)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 1] = 1.0;
                //Формирование измерений по географическим координатам
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - Longitude_CP) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
                KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - Latitude_CP) * SINSstate.R_n;

                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Noise_GPS_PositionError;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.Noise_GPS_PositionError;
                KalmanVars.cnt_measures += 2;

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 0] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo + 1] = 1.0;
                //Формирование измерений по географическим координатам
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate_OdoMod.Longitude - Longitude_CP) * SINSstate_OdoMod.R_e * Math.Cos(SINSstate_OdoMod.Latitude);
                KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate_OdoMod.Latitude - Latitude_CP) * SINSstate_OdoMod.R_n;

                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Noise_GPS_PositionError;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.Noise_GPS_PositionError;
                KalmanVars.cnt_measures += 2;
            }

            if (SINSstate.flag_iMx_r3_dV3 && Altitude_CP != 0.0)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dr3] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Height - Altitude_CP;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Noise_GPS_PositionError;
                KalmanVars.cnt_measures += 1;

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + SINSstate.value_iMx_r_odo_3] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate_OdoMod.Height - Altitude_CP;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Noise_GPS_PositionError;
                KalmanVars.cnt_measures += 1;
            }




            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true && Altitude_CP != 0.0)
            {
                double VerticalNoise = SINSstate.Noise_GPS_PositionError;

                if (Latitude_CP == 0 && Longitude_CP == 0)
                    VerticalNoise = 0.05;

                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + 0] = 1.0;
                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 1) * SimpleData.iMx_Vertical + SINSstate.Vertical_rOdo3] = 1.0;

                KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.Height - Altitude_CP;
                KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 1)] = SINSstate_OdoMod.Height - Altitude_CP;

                KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = VerticalNoise;
                KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 1)] = VerticalNoise;

                KalmanVars.Vertical_cnt_measures += 2;
            }


            if (Latitude_CP != 0 || Longitude_CP != 0 || Altitude_CP != 0.0)
                SINSstate.flag_UsingCorrection = true;
        }





        //--------------------------------------------------------------------------
        //--------------------------СКОРОСТНАЯ КОРЕКЦИЯ-----------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_VELOCITY(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
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


            for (int i = 0; i < 3; i++)
                KalmanVars.Measure[(KalmanVars.cnt_measures + i)] = SINSstate.Vx_0[i] - SINSstate_OdoMod.Vx_0[i];

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 0)] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + (iMx_dV_12 + 1)] = 1.0;

            //---КОРРЕКТИРОВАНИЕ ПО СКОРОСТИ В ПРОЕКЦИИ НА ГЕОГРАФИЮ---
            if (SINSstate.flag_OdoSINSWeakConnect_MODIF)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = -SINSstate_OdoMod.Omega_x[0] * Math.Tan(SINSstate_OdoMod.Latitude);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo] = SINSstate_OdoMod.Omega_x[0] * Math.Tan(SINSstate_OdoMod.Latitude);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 0] = -SINSstate_OdoMod.Omega_x[2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo] = SINSstate_OdoMod.Omega_x[2];
            }

            if (SINSstate.flag_iMx_kappa_13_ds)
            {
                if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_1 + 0] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[0, 2] + SINSstate.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[0, 1];

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[0, 0] - SINSstate.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[0, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 1] = -SINSstate.OdoSpeed_x0[0];

                if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_kappa_1 + 0] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[1, 2] + SINSstate.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[1, 1];

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_kappa_3_ds + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[1, 0] - SINSstate.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[1, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_kappa_3_ds + 1] = -SINSstate.OdoSpeed_x0[1];
            }

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[0, 1] * KalmanVars.OdoNoise_V;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.A_x0s[1, 1] * KalmanVars.OdoNoise_V;

            KalmanVars.cnt_measures += 2;


            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dV3] = 1.0;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_V;

                if (SINSstate.flag_iMx_kappa_13_ds)
                {
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_1 + 0] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 2] + SINSstate.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[2, 1];

                    if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    {
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 0] - SINSstate.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[2, 1];
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 1] = -SINSstate.OdoSpeed_x0[2];
                    }
                }

                if (SINSstate.flag_OdoSINSWeakConnect_MODIF)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures - 2) * iMx + 0] = -SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_e;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures - 2) * iMx + iMx_r12_odo] = SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_e;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures - 1) * iMx + 1] = -SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_n;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures - 1) * iMx + iMx_r12_odo + 1] = SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_n;

                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = SINSstate_OdoMod.Vx_0[0] / SINSstate_OdoMod.R_e;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo] = -SINSstate_OdoMod.Vx_0[0] / SINSstate_OdoMod.R_e;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 1] = SINSstate_OdoMod.Vx_0[1] / SINSstate_OdoMod.R_n;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 1] = -SINSstate_OdoMod.Vx_0[1] / SINSstate_OdoMod.R_n;
                }

                KalmanVars.cnt_measures += 1;
            }



            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                int iMxV = SimpleData.iMx_Vertical;

                KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.Vx_0[2] - SINSstate_OdoMod.Vx_0[2];

                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * iMxV + value_iMx_dV3] = 1.0;
                KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_V;

                if (SINSstate.Vertical_kappa1 > 0) 
                {
                    KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * iMxV + SINSstate.Vertical_kappa1] = -SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 2] + SINSstate.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[2, 1];

                    if (SINSstate.Vertical_kappa3Scale > 0)
                    {
                        KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * iMxV + SINSstate.Vertical_kappa3Scale + 0] = SINSstate.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 0] - SINSstate.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[2, 1];
                        KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * iMxV + SINSstate.Vertical_kappa3Scale + 1] = -SINSstate.OdoSpeed_x0[2];
                    }
                }
                KalmanVars.Vertical_cnt_measures += 1;
            }
        }

        public static void Make_H_VELOCITY_Mz13(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
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

            double[] Vz = new double[3];
            SimpleOperations.CopyArray(Vz, SINSstate.A_sx0 * SINSstate.Vx_0);

            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = Vz[0];

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 0)] = SINSstate.A_sx0[0, 0];
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 1)] = SINSstate.A_sx0[0, 1];
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 0.01;

            KalmanVars.cnt_measures += 1;


            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures - 1) * iMx + value_iMx_dV3] = SINSstate.A_sx0[0, 2];

                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = Vz[2];

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 0)] = SINSstate.A_sx0[2, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 1)] = SINSstate.A_sx0[2, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dV3] = SINSstate.A_sx0[2, 2];
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 0.01;

                KalmanVars.cnt_measures += 1;
            }
        }












        public static void Make_A(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate_OdoMod)
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


            SINSprocessing.Make_A(SINSstate, KalmanVars, SINSstate_OdoMod);



            /*-----------МОДИФИЦИРОВАННЫЙ СЛАБОСВЗАННЫЙ ВАРИАНТ----------------*/
            if (SINSstate.flag_OdoSINSWeakConnect_MODIF)
            {
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + (iMx_alphaBeta + 2)] = SINSstate_OdoMod.Vx_0[1];
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 1] = SINSstate_OdoMod.Omega_x[2];

                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + (iMx_alphaBeta + 2)] = -SINSstate_OdoMod.Vx_0[0];
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r12_odo + 0] = -SINSstate_OdoMod.Omega_x[2];


                if (SINSstate.flag_iMx_r3_dV3)
                {
                    if (SINSstate.existRelationHoriz_VS_Vertical)
                    {
                        KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + 0] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_e;
                        KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + (iMx_alphaBeta + 1)] += -SINSstate_OdoMod.Vx_0[2];
                        KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + iMx_r_odo_3] = -SINSstate_OdoMod.Omega_x[1];

                        KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 1] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_n;
                        KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + (iMx_alphaBeta + 0)] += SINSstate_OdoMod.Vx_0[2];
                        KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r_odo_3] = SINSstate_OdoMod.Omega_x[0];
                    }
                }

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    if (SINSstate.existRelationHoriz_VS_Vertical)
                    {
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + 0] = -SINSstate_OdoMod.Vx_0[0] / SINSstate_OdoMod.R_e;
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + 1] = -SINSstate_OdoMod.Vx_0[1] / SINSstate_OdoMod.R_n;
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + iMx_r12_odo + 0] = SINSstate_OdoMod.Omega_x[1];
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + iMx_r12_odo + 1] = -SINSstate_OdoMod.Omega_x[0];

                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + (iMx_alphaBeta + 0)] = -SINSstate_OdoMod.Vx_0[1];
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + (iMx_alphaBeta + 1)] = SINSstate_OdoMod.Vx_0[0];
                    }
                }
            }
            /*-----------СЛАБОСВЗАННЫЙ ВАРИАНТ----------------*/
            else if (SINSstate.flag_OdoSINSWeakConnect)
            {
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + (iMx_alphaBeta + 2)] = SINSstate_OdoMod.Vx_0[1];
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + 0] = -SINSstate_OdoMod.Omega_x[0] * Math.Tan(SINSstate_OdoMod.Latitude);
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 0] = SINSstate_OdoMod.Omega_x[0] * Math.Tan(SINSstate_OdoMod.Latitude);
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 1] = SINSstate_OdoMod.Omega_x[2];

                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + (iMx_alphaBeta + 2)] = -SINSstate_OdoMod.Vx_0[0];
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 0] = -SINSstate_OdoMod.Omega_x[2];

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    if (SINSstate.existRelationHoriz_VS_Vertical)
                    {
                        KalmanVars.Matrix_A[iMx_r12_odo * iMx + (iMx_alphaBeta + 1)] += -SINSstate_OdoMod.Vx_0[2];
                        KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r12_odo + 0] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_e;
                        KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_r_odo_3 + 0] = -SINSstate_OdoMod.Omega_x[1];

                        KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + (iMx_alphaBeta + 1)] += SINSstate_OdoMod.Vx_0[2];
                        KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r12_odo + 1] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_n;
                        KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r_odo_3 + 0] = SINSstate_OdoMod.Omega_x[0];
                    }
                }

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    if (SINSstate.existRelationHoriz_VS_Vertical)
                    {
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + (iMx_alphaBeta + 0)] = -SINSstate_OdoMod.Vx_0[1];
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + (iMx_alphaBeta + 1)] = SINSstate_OdoMod.Vx_0[0];
                    }
                }
            }



            if (SINSstate.flag_iMx_kappa_13_ds)
            {
                if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_kappa_1 + 0] = SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[0, 2] - SINSstate_OdoMod.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[0, 1];

                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_kappa_3_ds + 0] = -SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[0, 0] + SINSstate_OdoMod.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[0, 1];
                KalmanVars.Matrix_A[iMx_r12_odo * iMx + iMx_kappa_3_ds + 1] = SINSstate_OdoMod.OdoSpeed_x0[0];

                if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_kappa_1 + 0] = SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[1, 2] - SINSstate_OdoMod.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[1, 1];

                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_kappa_3_ds + 0] = -SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[1, 0] + SINSstate_OdoMod.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[1, 1];
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_kappa_3_ds + 1] = SINSstate_OdoMod.OdoSpeed_x0[1];

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + iMx_kappa_1 + 0] = SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 2] - SINSstate_OdoMod.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[2, 1];

                    if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    {
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + iMx_kappa_3_ds + 0] = -SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 0] + SINSstate_OdoMod.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[2, 1];
                        KalmanVars.Matrix_A[(iMx_r_odo_3) * iMx + iMx_kappa_3_ds + 1] = SINSstate_OdoMod.OdoSpeed_x0[2];
                    }
                }
            }



            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                if (SINSstate.Vertical_kappa1 > 0)
                {
                    KalmanVars.Vertical_Matrix_A[SINSstate.Vertical_rOdo3 * SimpleData.iMx_Vertical + SINSstate.Vertical_kappa1 + 0] = SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 2] - SINSstate_OdoMod.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[2, 1];

                    if (SINSstate.Vertical_kappa3Scale > 0)
                    {
                        KalmanVars.Vertical_Matrix_A[SINSstate.Vertical_rOdo3 * SimpleData.iMx_Vertical + SINSstate.Vertical_kappa3Scale + 0] = -SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 0] + SINSstate_OdoMod.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[2, 1];
                        KalmanVars.Vertical_Matrix_A[SINSstate.Vertical_rOdo3 * SimpleData.iMx_Vertical + SINSstate.Vertical_kappa3Scale + 1] = SINSstate_OdoMod.OdoSpeed_x0[2];
                    }
                }
            }
            // ----------------------------------------------------------//
        }














        public static void InitOfCovarianceMatrixes(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);

            // --- нач. ковариации горизонтальных ошибок координат одометрического счисления --- //
            KalmanVars.CovarianceMatrixS_m[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12]
                = KalmanVars.CovarianceMatrixS_p[SINSstate.value_iMx_r_odo_12 * SimpleData.iMx + SINSstate.value_iMx_r_odo_12] = SINSstate.stdOdoR;
            KalmanVars.CovarianceMatrixS_m[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + (SINSstate.value_iMx_r_odo_12 + 1)]
                = KalmanVars.CovarianceMatrixS_p[(SINSstate.value_iMx_r_odo_12 + 1) * SimpleData.iMx + (SINSstate.value_iMx_r_odo_12 + 1)] = SINSstate.stdOdoR;

            // --- нач. ковариации ошибки высоты одометрического счисления --- //
            if (SINSstate.flag_iMx_r3_dV3)
                KalmanVars.CovarianceMatrixS_m[(SINSstate.value_iMx_r_odo_3 + 0) * SimpleData.iMx + (SINSstate.value_iMx_r_odo_3 + 0)]
                    = KalmanVars.CovarianceMatrixS_p[(SINSstate.value_iMx_r_odo_3 + 0) * SimpleData.iMx + (SINSstate.value_iMx_r_odo_3 + 0)] = SINSstate.stdOdoR;


            SimpleOperations.PrintMatrixToFile(KalmanVars.CovarianceMatrixS_m, SimpleData.iMx, SimpleData.iMx, "StartCovariance");



            // ------------------ВЕРТИКАЛЬНЫЙ КАНАЛ ОТДЕЛЬНО-----------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
                KalmanVars.Vertical_CovarianceMatrixS_m[SINSstate.Vertical_rOdo3 * SimpleData.iMx_Vertical + SINSstate.Vertical_rOdo3]
                    = KalmanVars.Vertical_CovarianceMatrixS_p[SINSstate.Vertical_rOdo3 * SimpleData.iMx_Vertical + SINSstate.Vertical_rOdo3] = SINSstate.stdOdoR;
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

            SINSprocessing.MatrixNoise_ReDef(SINSstate, KalmanVars, SINSstate.flag_Alignment);

            double sqrt_freq = Math.Sqrt(SINSstate.Freq);
            sqrt_freq = Math.Sqrt(Math.Abs(SINSstate.timeStep) / SINSstate.TimeBetweenForecast);
            //sqrt_freq = 1.0;


            if (SINSstate.flag_iMqDeltaRodo)
            {
                KalmanVars.CovarianceMatrixNoise[(iMx_r12_odo + 0) * iMq + iMx_r12_odo + 0] = KalmanVars.Noise_Pos * sqrt_freq;
                KalmanVars.CovarianceMatrixNoise[(iMx_r12_odo + 1) * iMq + iMx_r12_odo + 1] = KalmanVars.Noise_Pos * sqrt_freq;

                if (SINSstate.flag_iMx_r3_dV3)
                    KalmanVars.CovarianceMatrixNoise[(iMx_r_odo_3 + 0) * iMq + iMx_r_odo_3 + 0] = KalmanVars.Noise_Pos * sqrt_freq;
            }



            // --- Считаем, что по вертикали у нас шумы только по скорости ---
            // ----------------------------------------------------------//
            if (SINSstate.flag_SeparateHorizVSVertical == true)
            {
                if (SINSstate.flag_iMqDeltaRodo)
                    KalmanVars.Vertical_CovarianceMatrixNoise[SINSstate.Vertical_rOdo3 * SimpleData.iMq_Vertical + SINSstate.Vertical_rOdo3] = KalmanVars.Noise_Pos_Odo * sqrt_freq;

                if (SINSstate.flag_iMqVarkappa1)
                    KalmanVars.Vertical_CovarianceMatrixNoise[SINSstate.Vertical_kappa1 * SimpleData.iMq_Vertical + SINSstate.Vertical_kappa1] = KalmanVars.Noise_OdoKappa_1 * sqrt_freq;
            }
        }

    }
}
