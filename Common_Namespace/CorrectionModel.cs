using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class CorrectionModel : SimpleOperations
    {
        //--------------------------------------------------------------------------
        //--------------------------ПОЗИЦИОННАЯ КОРЕКЦИЯ CHECKPOINTS-----------------------------
        //--------------------------------------------------------------------------
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

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 1] = 1.0;

            //Формирование измерений по географическим координатам
            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - Longitude_CP) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
            KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - Latitude_CP) * SINSstate.R_n;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Noise_GPS_PositionError;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.Noise_GPS_PositionError;

            KalmanVars.cnt_measures += 2;

            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dr3] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Altitude - Altitude_CP;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.Noise_GPS_PositionError;

                KalmanVars.cnt_measures += 1;
            }

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


            //---КОРРЕКТИРОВАНИЕ ПО СКОРОСТИ В ПРОЕКЦИИ НА ГЕОГРАФИЮ---
            for (int i = 0; i < 3; i++)
                KalmanVars.Measure[(KalmanVars.cnt_measures + i)] = SINSstate.Vx_0[i] - SINSstate.OdoSpeed_x0[i];

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 0)] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + (iMx_dV_12 + 1)] = 1.0;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[0, 1] * KalmanVars.OdoNoise_V * 1.0;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.A_x0s[1, 1] * KalmanVars.OdoNoise_V * 1.0;


            if (SINSstate.flag_iMx_kappa_13_ds)
            {
                if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_1 + 0] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[0, 2] + SINSstate.OdoSpeed_s[2] * SINSstate.A_x0s[0, 1];

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 0] = SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[0, 0] - SINSstate.OdoSpeed_s[0] * SINSstate.A_x0s[0, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 1] = -SINSstate.OdoSpeed_x0[0];

                if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_kappa_1 + 0] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[1, 2] + SINSstate.OdoSpeed_s[2] * SINSstate.A_x0s[1, 1];

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_kappa_3_ds + 0] = SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[1, 0] - SINSstate.OdoSpeed_s[0] * SINSstate.A_x0s[1, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_kappa_3_ds + 1] = -SINSstate.OdoSpeed_x0[1];
            }
            KalmanVars.cnt_measures += 2;


            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dV3] = 1.0;

                if (SINSstate.flag_iMx_kappa_13_ds)
                {
                    if (iMx_kappa_1 > 0)
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_1 + 0] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[2, 2] + SINSstate.OdoSpeed_s[2] * SINSstate.A_x0s[2, 1];

                    if (SINSstate.existRelationHoriz_VS_Vertical || !SINSstate.flag_iMx_r3_dV3)
                    {
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 0] = SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[2, 0] - SINSstate.OdoSpeed_s[0] * SINSstate.A_x0s[2, 1];
                        KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_kappa_3_ds + 1] = -SINSstate.OdoSpeed_x0[2];
                    }
                }
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_V * 1.0;

                KalmanVars.cnt_measures += 1;
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






        //--------------------------------------------------------------------------
        //-------------------------------КНС---------------------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_KNS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
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

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 0)] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + (iMx_dV_12 + 1)] = 1.0;

            for (int i = 0; i < 3; i++)
                KalmanVars.Measure[(KalmanVars.cnt_measures + i)] = SINSstate.Vx_0[i];

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_STOP;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = KalmanVars.OdoNoise_STOP;

            KalmanVars.cnt_measures += 2;


            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dV3] = 1.0;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_STOP;
                KalmanVars.cnt_measures += 1;
            }
        }

        public static void Make_Vertical_H_KNS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + 1] = 1.0;
            KalmanVars.Vertical_Measure[KalmanVars.Vertical_cnt_measures] = SINSstate.Vx_0[2];
            KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = KalmanVars.OdoNoise_STOP;

            KalmanVars.Vertical_cnt_measures += 1;

            // --- Корректируем также по нулевой разнице между высотами БИНС и одометра --- //
            {
                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + 0] = 1.0;
                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + SINSstate.Vertical_rOdo3] = -1.0;
                KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 0)] = 0;
                KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = KalmanVars.OdoNoise_Dist / 10.0;

                KalmanVars.Vertical_cnt_measures += 1;
            }
        }



        //--------------------------------------------------------------------------
        //-------------------------------СПУТНИК---------------------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_GPS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
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

            //---КОРРЕКЦИЯ ПО СПУТНИКОВОЙ ИНФОРМАЦИИ---
            if (SINSstate.flag_Using_SNS == true && SINSstate.GPS_Data.gps_Altitude.isReady == 1 && Math.Abs(SINSstate.GPS_Data.gps_Latitude.Value) > 0.01)
            {

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - SINSstate.GPS_Data.gps_Longitude.Value) * RadiusE(SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.Altitude) * Math.Cos(SINSstate.GPS_Data.gps_Latitude.Value);
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 3.0;
                KalmanVars.cnt_measures += 1;

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 1] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Latitude - SINSstate.GPS_Data.gps_Latitude.Value) * RadiusN(SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.Altitude);
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 3.0;
                KalmanVars.cnt_measures += 1;


                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 0)] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_alphaBeta + 2)] = SINSstate.Vx_0[1];
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Vx_0[0] - SINSstate.GPS_Data.gps_Ve.Value;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 1.0;
                KalmanVars.cnt_measures += 1;

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 1)] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_alphaBeta + 2)] = -SINSstate.Vx_0[0];
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Vx_0[1] - SINSstate.GPS_Data.gps_Vn.Value;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 1.0;
                KalmanVars.cnt_measures += 1;

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + value_iMx_dr3] = 1.0;
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Altitude - SINSstate.GPS_Data.gps_Altitude.Value;
                    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 5.0;
                    KalmanVars.cnt_measures += 1;
                }

            }
        }



    }
}
