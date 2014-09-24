using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace Common_Namespace
{
    public class ParamsForModel
    {
        public double V_odo, F_odo;
        public double Can;
        public double less;
        public double V_increment_SINS, V_increment_odo;
        public double Count_More_than_20;
    }


    public static class ModelsOfOdoCorrection
    {




        public static void Model_With_Odo_Equations(SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, ParamsForModel OdoModel, StreamWriter ForHelp)
        {
            double[] dS_x = new double[3];
            double[] Vx_0_odo = new double[3];

            SINSstate.flag_UsingOdoVelocity = false;
            SINSstate.flag_UsingOdoPosition = true;

            if (SINSstate.OdometerData.odometer_left.isReady != 1)
            {
                SINSstate.flag_UsingCorrection = false;
            }
            else if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                SimpleOperations.CopyArray(dS_x, SINSstate.A_x0s * SINSstate.OdometerVector);

                if (SINSstate.Do_Smoothing == false)
                {
                    SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude + dS_x[1] / SimpleOperations.RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);
                    SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude + dS_x[0] / SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude) / Math.Cos(SINSstate_OdoMod.Latitude);
                    SINSstate_OdoMod.Altitude = SINSstate_OdoMod.Altitude + dS_x[2];

                    SimpleOperations.CopyArray(SINSstate_OdoMod.OdoSpeed_x0, SINSstate.A_x0s * SINSstate.OdoSpeed_s);
                }

                SINSstate.flag_UsingCorrection = true;
            }
        }










        public static void ModelWith_Odo_Limit_Measures(SINS_State SINSstate, SINS_State SINSstate_OdoMod, ParamsForModel OdoModel)
        {
            SINSstate.flag_UsingOdoVelocity = true;

            if (SINSstate.OdometerData.odometer_left.isReady != 1)
            {
                SINSstate.flag_UsingCorrection = false;
            }
            else if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                double[] Vx_0_odo = new double[3];

                OdoModel.V_odo = SINSstate.OdometerVector[1] / SINSstate.OdoTimeStepCount / SINSstate.timeStep;

                if (OdoModel.Can < SINSstate.Odo_Limit_Measures)
                {
                    SINSstate.flag_UsingCorrection = false;
                    OdoModel.Can++;
                }
                else
                {
                    SINSstate.flag_UsingCorrection = true;
                    OdoModel.Can = 0;
                    SimpleOperations.CopyArray(SINSstate.OdoSpeed_x0, Vx_0_odo);
                }
            }
        }




        public static void Model_Each_Odo_Measure(SINS_State SINSstate, SINS_State SINSstate_OdoMod, ParamsForModel OdoModel, StreamWriter ForHelp)
        {
            double[] dS_x = new double[3], gamma_ = new double[3], Vx_0_odo = new double[3];

            SINSstate.flag_UsingOdoVelocity = true;

            if (SINSstate.OdometerData.odometer_left.isReady != 1)
            {
                SINSstate.flag_UsingCorrection = false;
            }
            else if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                SimpleOperations.CopyArray(dS_x, SINSstate.A_x0s * SINSstate.OdometerVector);

                //---------Вычисляю скорость по матрицам ориентации B_i и B_i+1---------
                SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude + dS_x[1] / SINSstate.R_n;
                SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude + dS_x[0] / SINSstate.R_e / Math.Cos(SINSstate_OdoMod.Latitude);
                SINSstate_OdoMod.Altitude = SINSstate_OdoMod.Altitude + dS_x[2];

                SINSstate.flag_UsingCorrection = true;
            }
        }










        //НИНЫ БОРИСОВНОЙ
        public static void Model_Integral_of_Fx_2(SINS_State SINSstate, SINS_State SINSstate_OdoMod, ParamsForModel OdoModel, StreamWriter ForHelp)
        {
            double[] dS_x = new double[3], gamma_ = new double[3], Vx_0_odo = new double[3];

            SINSstate.flag_UsingOdoVelocity = true;

            if (SINSstate.OdometerData.odometer_left.isReady != 1)
            {
                SINSstate.flag_UsingCorrection = false;
            }
            else if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                if (Math.Abs(OdoModel.V_odo - SINSstate.OdoSpeedPrev) <= Math.Abs(SINSstate.F_z[1] * (SINSstate.OdoTimeStepCount * SINSstate.timeStep)) + 1.0)
                    SINSstate.flag_UsingCorrection = true;
                else
                    SINSstate.flag_UsingCorrection = false;
            }
        }


    }
}
