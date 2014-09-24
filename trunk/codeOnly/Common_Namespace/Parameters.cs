using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class Parameters : SimpleOperations
    {
        public static void StartSINS_Parameters(SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, Parameters Params, Proc_Help ProcHelp)
        {

            if (SINSstate.Global_file == "Imitator_Data")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.000000002;
            }





            if (SINSstate.Global_file == "Azimut_14.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise_V = 1.0;
                KalmanVars.OdoNoise_Dist = 0.2;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                //KalmanVars.Noise_Pos = 0.075 * Math.Sqrt(SINSstate.Freq);
                ////KalmanVars.Noise_Vel = 0.021 * 9.78049;
                ////KalmanVars.Noise_Angl = 61.0 * 3.141592 / 180.0 / 3600.0;
                //KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Vel[0] = 0.021; //Part of Ge
                KalmanVars.Noise_Vel[1] = 0.021;
                KalmanVars.Noise_Vel[2] = 0.021;
                KalmanVars.Noise_Angl[0] = 61.0 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[1] = 61.0 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[2] = 61.0 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.264 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 58.0 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 92.37074;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -115.791349 * SimpleData.ToRadian;
                SINSstate.Roll = 0.6767 * SimpleData.ToRadian;
                SINSstate.Pitch = -0.3837195 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }
            if (SINSstate.Global_file == "Azimut_15.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise_V = 1.0;
                KalmanVars.OdoNoise_Dist = 0.2;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);

                //KalmanVars.Noise_Vel = 0.035 * 9.78049;
                //KalmanVars.Noise_Angl = 206 * 3.141592 / 180.0 / 3600.0;
                //KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Vel[0] = 0.0003; //Part of Ge
                KalmanVars.Noise_Vel[1] = 0.0003;
                KalmanVars.Noise_Vel[2] = 0.0003;
                KalmanVars.Noise_Angl[0] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[1] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[2] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.2681502 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9990499 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 175.076;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }
            if (SINSstate.Global_file == "Azimut_24.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise_V = 1.0;
                KalmanVars.OdoNoise_Dist = 0.2;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);

                //KalmanVars.Noise_Vel = 0.041 * 9.78049;
                //KalmanVars.Noise_Angl = 149.0 * 3.141592 / 180.0 / 3600.0;
                //KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Vel[0] = 0.0003; //Part of Ge
                KalmanVars.Noise_Vel[1] = 0.0003;
                KalmanVars.Noise_Vel[2] = 0.0003;
                KalmanVars.Noise_Angl[0] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[1] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[2] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.268466 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9993716 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 177.7876;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }
            if (SINSstate.Global_file == "Azimut_29.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.odo_min_increment = 0.2;
                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment * 5;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                //KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Vel[0] = 0.0003; //Part of Ge
                KalmanVars.Noise_Vel[1] = 0.0003;
                KalmanVars.Noise_Vel[2] = 0.0003;
                KalmanVars.Noise_Angl[0] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[1] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[2] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.268466 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9987987 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 173.8157;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -26.26266 * SimpleData.ToRadian;
                SINSstate.Roll = 1.753250 * SimpleData.ToRadian;
                SINSstate.Pitch = -1.510889 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }
            if (SINSstate.Global_file == "Kama_04.09.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.01024;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise_V = 1.0;
                KalmanVars.OdoNoise_Dist = 0.1;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                //KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Vel[0] = 0.0003; //Part of Ge
                KalmanVars.Noise_Vel[1] = 0.0003;
                KalmanVars.Noise_Vel[2] = 0.0003;
                KalmanVars.Noise_Angl[0] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[1] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[2] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.268466 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9987987 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 176.6856;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }

            if (SINSstate.Global_file == "ktn004_15.03.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.01024;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise_V = 2.0;
                KalmanVars.OdoNoise_Dist = 1.1;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa = 0.01 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 1.1;
                //KalmanVars.Noise_Vel[0] = 0.007; //Part of Ge
                //KalmanVars.Noise_Vel[1] = 0.007;
                //KalmanVars.Noise_Vel[2] = 0.007;
                //KalmanVars.Noise_Angl[0] = 12.5 * 3.141592 / 180.0 / 3600.0;
                //KalmanVars.Noise_Angl[1] = 12.5 * 3.141592 / 180.0 / 3600.0;
                //KalmanVars.Noise_Angl[2] = 12.5 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 43.086884 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 56.2891327 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 91.48914;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.alpha_x = 0.1 * SimpleData.ToRadian;
                SINSstate.alpha_y = 0.08 * SimpleData.ToRadian;
                SINSstate.alpha_z = 0.46 * SimpleData.ToRadian;

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }
            if (SINSstate.Global_file == "ktn004_21.03.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.01024;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                KalmanVars.OdoNoise_V = 1.0;
                KalmanVars.OdoNoise_Dist = 0.2;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                //Параметром KalmanVars.Noise_Pos я могу задать наперед заданные значения СКО (ну не только этим паратром)
                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Vel[0] = 0.0003; //Part of Ge
                KalmanVars.Noise_Vel[1] = 0.0003;
                KalmanVars.Noise_Vel[2] = 0.0003;
                KalmanVars.Noise_Angl[0] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[1] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[2] = 20 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00002;

                //KalmanVars.Noise_Pos = 0.0075;
                //KalmanVars.Noise_Drift = 0.00002 * 3.141592 / 180.0 / 3600.0;
                //KalmanVars.Noise_Accel = 0.0000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 43.086999 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 56.2890926 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 91.48914;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.alpha_x = 0.1 * SimpleData.ToRadian;
                SINSstate.alpha_y = 0.08 * SimpleData.ToRadian;
                SINSstate.alpha_z = 0.46 * SimpleData.ToRadian;

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }







            //МИНСКИЕ ЗАЕЗДЫ

            if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.2;
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 3;

                SINSstate.dV_q = 2.5;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;
                SINSstate.ControlPointCount[0] = 29297;
                SINSstate.ControlPointCount[1] = 48829;
                SINSstate.ControlPointCount[2] = 73243;

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment * 5;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                KalmanVars.Noise_Pos = 1.1;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.485964934299;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 0.9414566620339;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 217.084;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }










            if (SINSstate.Global_file == "Azimut-T_18-Oct-2013_11-05-11")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.05;
                SINSstate.timeStep = SINSstate.Freq = 0.01;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;

                KalmanVars.OdoNoise_V = 0.5;
                KalmanVars.OdoNoise_Dist = 0.05;
                KalmanVars.OdoNoise_STOP = 0.125;

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_Vel[0] = 0.02; //Part of Ge
                KalmanVars.Noise_Vel[1] = 0.02;
                KalmanVars.Noise_Vel[2] = 0.02;
                KalmanVars.Noise_Angl[0] = 5.0 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[1] = 5.0 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Angl[2] = 5.0 * 3.141592 / 180.0 / 3600.0;


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 46.87201806 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 49.99452656 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 0.0;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }
            if (SINSstate.Global_file == "AZIMUT_T_2013_10_18_12_55")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.1268;
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 10.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 0.000075;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.982366681098938;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 1.00708794593811;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 272.181;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }








            if (SINSstate.Global_file == "Azimut_514_08Nov2013_11_15")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.1268;
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 0.000075;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.982068359851837;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 1.01227509975433;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 172.36;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }





            if (SINSstate.Global_file == "Saratov_run_2014_07_23")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.05;
                SINSstate.timeStep = SINSstate.Freq = 0.01048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa = 0.01 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 1.1;
                //KalmanVars.Noise_Vel[0] = 0.007; //Part of Ge
                //KalmanVars.Noise_Vel[1] = 0.007;
                //KalmanVars.Noise_Vel[2] = 0.007;
                //KalmanVars.Noise_Angl[0] = 12.5 * 3.141592 / 180.0 / 3600.0;
                //KalmanVars.Noise_Angl[1] = 12.5 * 3.141592 / 180.0 / 3600.0;
                //KalmanVars.Noise_Angl[2] = 12.5 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 46.87215103 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 49.99453181 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 29.314;

                if (SINSstate.Saratov_run_Final)
                {
                    ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 45.3817334 * SimpleData.ToRadian;
                    ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 49.80892188 * SimpleData.ToRadian;
                    ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 29.314;
                }

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.alpha_x = 0.0 * SimpleData.ToRadian;
                //SINSstate.alpha_y = 1.0 * SimpleData.ToRadian;
                SINSstate.alpha_z = 0.0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }
        }
    }
}
