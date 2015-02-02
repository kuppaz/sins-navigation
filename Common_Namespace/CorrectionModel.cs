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
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;

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

            if (SINSstate.flag_UsingOdoPosition && SINSstate.flag_iMx_kappa_13_ds)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = -Math.Tan(SINSstate_OdoMod.Latitude) * SINSstate.Ds2_CumulativeByOdoTrack[0, 2] / SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 4] = -SINSstate.Ds2_CumulativeByOdoTrack[0, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 5] = -SINSstate.Ds2_CumulativeByOdoTrack[0, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 6] = -SINSstate.Ds2_CumulativeByOdoTrack[0, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 0] = -Math.Tan(SINSstate_OdoMod.Latitude) * SINSstate.Ds2_CumulativeByOdoTrack[1, 2] / SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 4] = -SINSstate.Ds2_CumulativeByOdoTrack[1, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 5] = -SINSstate.Ds2_CumulativeByOdoTrack[1, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 6] = -SINSstate.Ds2_CumulativeByOdoTrack[1, 2];

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = -SINSstate.Ds_CumulativeByOdoTrack[0, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = SINSstate.Ds_CumulativeByOdoTrack[0, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = SINSstate.Ds_CumulativeByOdoTrack[0, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 0] = -SINSstate.Ds_CumulativeByOdoTrack[1, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 1] = SINSstate.Ds_CumulativeByOdoTrack[1, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 2] = SINSstate.Ds_CumulativeByOdoTrack[1, 1];

                KalmanVars.Measure[KalmanVars.cnt_measures + 0] = (SINSstate_OdoMod.Longitude - Longitude_CP) * SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude) * Math.Cos(SINSstate_OdoMod.Latitude);
                KalmanVars.Measure[KalmanVars.cnt_measures + 1] = (SINSstate_OdoMod.Latitude - Latitude_CP) * SimpleOperations.RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);

                KalmanVars.Noize_Z[KalmanVars.cnt_measures + 0] = SINSstate.Imitator_GPS_PositionError;
                KalmanVars.Noize_Z[KalmanVars.cnt_measures + 1] = SINSstate.Imitator_GPS_PositionError;

                KalmanVars.cnt_measures += 2;

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = -Math.Tan(SINSstate.Latitude) * SINSstate.Ds2_CumulativeByOdoTrack[2, 2] / SINSstate.R_e;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 4] = -SINSstate.Ds2_CumulativeByOdoTrack[2, 0];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 5] = -SINSstate.Ds2_CumulativeByOdoTrack[2, 1];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 6] = -SINSstate.Ds2_CumulativeByOdoTrack[2, 2];

                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = -SINSstate.Ds_CumulativeByOdoTrack[2, 2];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = SINSstate.Ds_CumulativeByOdoTrack[2, 0];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = SINSstate.Ds_CumulativeByOdoTrack[2, 1];

                    KalmanVars.Measure[KalmanVars.cnt_measures + 0] = SINSstate_OdoMod.Altitude - Altitude_CP;
                    KalmanVars.Noize_Z[KalmanVars.cnt_measures + 0] = SINSstate.Imitator_GPS_PositionError;

                    KalmanVars.cnt_measures += 1;
                }

                //PrintMatrixToFile(KalmanVars.Matrix_H, KalmanVars.cnt_measures, iMx);
            }
            //PrintMatrixToFile(KalmanVars.Matrix_H, KalmanVars.cnt_measures, iMx);


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
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.OdometerData.odometer_left.Value / (1 + SINSstate.Cumulative_KappaEst[1]) - l_true) / l_true;
                else
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.OdometerData.odometer_left.Value - l_true) / l_true;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 0.002;
                KalmanVars.cnt_measures += 1;

                double tst = Math.Atan2(long_dif_calc, lat_dif_calc);
                double tst2 = Math.Atan2(long_dif_true, lat_dif_true);

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = Math.Tan(SINSstate.Latitude) / SINSstate.R_e;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 4] = -Math.Sin(SINSstate.Heading) * Math.Tan(SINSstate.Pitch);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 5] = -Math.Cos(SINSstate.Heading) * Math.Tan(SINSstate.Pitch);
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 6] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Heading - Math.Atan2(long_dif_true, lat_dif_true);
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 5.0 * SimpleData.ToRadian_min;
                KalmanVars.cnt_measures += 1;

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


            if (SINSstate.flag_iMx_kappa_13_ds && SINSstate.flag_OdoModelOnlyCP == false)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 4] = SINSstate.Ds2_CumulativeByOdoTrack[0, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 5] = SINSstate.Ds2_CumulativeByOdoTrack[0, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 6] = SINSstate.Ds2_CumulativeByOdoTrack[0, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] += Math.Tan(SINSstate.Latitude) * SINSstate.Ds2_CumulativeByOdoTrack[0, 2] / SINSstate.R_e;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 4] = SINSstate.Ds2_CumulativeByOdoTrack[1, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 5] = SINSstate.Ds2_CumulativeByOdoTrack[1, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 6] = SINSstate.Ds2_CumulativeByOdoTrack[1, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 0] += Math.Tan(SINSstate.Latitude) * SINSstate.Ds2_CumulativeByOdoTrack[1, 2] / SINSstate.R_e;

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = SINSstate.Ds_CumulativeByOdoTrack[0, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = -SINSstate.Ds_CumulativeByOdoTrack[0, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = -SINSstate.Ds_CumulativeByOdoTrack[0, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 0] = SINSstate.Ds_CumulativeByOdoTrack[1, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 1] = -SINSstate.Ds_CumulativeByOdoTrack[1, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 2] = -SINSstate.Ds_CumulativeByOdoTrack[1, 1];
            }


            //Формирование измерений по географическим координатам
            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - SINSstate_OdoMod.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
            KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[0, 1] * KalmanVars.OdoNoise_Dist;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.A_x0s[1, 1] * KalmanVars.OdoNoise_Dist;

            KalmanVars.cnt_measures += 2;


            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3] = 1.0;
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Altitude - SINSstate_OdoMod.Altitude;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_Dist;


                if (SINSstate.flag_iMx_kappa_13_ds && SINSstate.flag_OdoModelOnlyCP == false)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = Math.Tan(SINSstate.Latitude) * SINSstate.Ds2_CumulativeByOdoTrack[2, 2] / SINSstate.R_e;
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 4] = SINSstate.Ds2_CumulativeByOdoTrack[2, 0];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 5] = SINSstate.Ds2_CumulativeByOdoTrack[2, 1];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 6] = SINSstate.Ds2_CumulativeByOdoTrack[2, 2];

                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = SINSstate.Ds_CumulativeByOdoTrack[2, 2];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = -SINSstate.Ds_CumulativeByOdoTrack[2, 0];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = -SINSstate.Ds_CumulativeByOdoTrack[2, 1];
                }

                KalmanVars.cnt_measures += 1;
            }


            //---ДОПОЛНИТЕЛЬНОЕ измерение по вертикальной скорости---
            if (SINSstate.flag_iMx_r3_dV3 && SINSstate.add_velocity_to_position)
            {
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Vx_0[2] - SINSstate.OdoSpeed_x0[2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3 + 1] = 1.0;

                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_V;

                KalmanVars.cnt_measures += 1;
            }


            //if (SINSstate.add_velocity_to_position)
            //    Make_H_VELOCITY(KalmanVars, SINSstate, SINSstate_OdoMod);


            //PrintMatrixToFile(KalmanVars.Matrix_H, KalmanVars.cnt_measures, iMx);
        }












        //--------------------------------------------------------------------------
        //--------------------------СКОРОСТНАЯ КОРЕКЦИЯ-----------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_VELOCITY(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;


            //---КОРРЕКТИРОВАНИЕ ПО СКОРОСТИ В ПРОЕКЦИИ НА ГЕОГРАФИЮ---
            for (int i = 0; i < 3; i++)
                KalmanVars.Measure[(KalmanVars.cnt_measures + i)] = SINSstate.Vx_0[i] - SINSstate.OdoSpeed_x0[i];

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 2] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 3] = 1.0;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[0, 1] * KalmanVars.OdoNoise_V;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = SINSstate.A_x0s[1, 1] * KalmanVars.OdoNoise_V;


            if (SINSstate.flag_iMx_kappa_13_ds && SINSstate.flag_OdoModelOnlyCP == false)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[0, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[0, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[0, 1];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 0] = SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[1, 2];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 1] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[1, 0];
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_odo_model + 2] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[1, 1];
            }
            KalmanVars.cnt_measures += 2;


            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3 + 1] = 1.0;

                if (SINSstate.flag_iMx_kappa_13_ds && SINSstate.flag_OdoModelOnlyCP == false)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 0] = SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[2, 2];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 1] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[2, 0];
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_odo_model + 2] = -SINSstate.OdoSpeed_s[1] * SINSstate.A_x0s[2, 1];
                }
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = SINSstate.A_x0s[2, 1] * KalmanVars.OdoNoise_V;

                KalmanVars.cnt_measures += 1;
            }
        }

        public static void Make_H_VELOCITY_Scalar(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;


            double[] tempVect = new double[3];

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 2] = SINSstate.A_sx0[1, 0];
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 3] = SINSstate.A_sx0[1, 1];

            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.A_sx0[1, 0] * SINSstate.Vx_0[0] + SINSstate.A_sx0[1, 1] * SINSstate.Vx_0[1] + SINSstate.A_sx0[1, 2] * SINSstate.Vx_0[2] - SINSstate.OdoSpeed_s[1];
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_V;

            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3 + 1] = SINSstate.A_sx0[1, 2];
            }

            KalmanVars.cnt_measures += 1;
        }







        //--------------------------------------------------------------------------
        //-------------------------------КНС---------------------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_KNS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 2] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 3] = 1.0;

            for (int i = 0; i < 3; i++)
                KalmanVars.Measure[(KalmanVars.cnt_measures + i)] = SINSstate.Vx_0[i];

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_STOP;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = KalmanVars.OdoNoise_STOP;

            KalmanVars.cnt_measures += 2;


            if (SINSstate.flag_iMx_r3_dV3)
            {
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3 + 1] = 1.0;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_STOP;
                KalmanVars.cnt_measures += 1;
            }


        }



        //--------------------------------------------------------------------------
        //-------------------------------СПУТНИК---------------------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_GPS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r3_dV3 = SINSstate.iMx_r3_dV3, iMx_odo_model = SINSstate.iMx_odo_model,
                iMx_r12_odo = SINSstate.iMx_r12_odo;


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


                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 2] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 6] = SINSstate.Vx_0[1];
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Vx_0[0] - SINSstate.GPS_Data.gps_Ve.Value;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 1.0;
                KalmanVars.cnt_measures += 1;

                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 3] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 6] = -SINSstate.Vx_0[0];
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Vx_0[1] - SINSstate.GPS_Data.gps_Vn.Value;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 1.0;
                KalmanVars.cnt_measures += 1;

                if (SINSstate.flag_iMx_r3_dV3)
                {
                    KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r3_dV3] = 1.0;
                    KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = SINSstate.Altitude - SINSstate.GPS_Data.gps_Altitude.Value;
                    KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = 5.0;
                    KalmanVars.cnt_measures += 1;
                }

            }
        }



    }
}
