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
            if (SINSstate.Global_file == "Saratov_01.11.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.LastCountForRead = 183000;
                //SINSstate.LastCountForRead = 936900;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 0.1;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Vel = 1.21 * 9.78049;
                KalmanVars.Noise_Angl = 121.0 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 45.91832179 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 51.65744354 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 107.224;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 45.91832179 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 51.65744354 * SimpleData.ToRadian;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -108.86 * SimpleData.ToRadian;
                SINSstate.Roll = -0.02769268;
                SINSstate.Pitch = -0.01095503;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.A_nxi = Matrix.DoA_eta_xi(SINSstate.Time);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }

            if (SINSstate.Global_file == "Azimut_14.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.LastCountForRead = 270000;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 1.0;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                //KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Pos = 0.075 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Vel = 0.021 * 9.78049;
                //KalmanVars.Noise_Angl = 61.0 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

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
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
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
            if (SINSstate.Global_file == "Azimut_15.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.LastCountForRead = 280000;
    
                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 1.0;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);

                KalmanVars.Noise_Vel = 0.035 * 9.78049;
                KalmanVars.Noise_Angl = 206 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

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
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
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

                SINSstate.LastCountForRead = 250000;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 1.0;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                //KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);

                KalmanVars.Noise_Vel = 0.041 * 9.78049;
                KalmanVars.Noise_Angl = 149.0 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

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
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
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

                SINSstate.LastCountForRead = 260000;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 1.0;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

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
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
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

                SINSstate.LastCountForRead = 450000;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 1.0;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

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
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
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

                SINSstate.LastCountForRead = 450000;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 1.0;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_Pos = 0.075 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Vel = 0.007 * 9.78049;
                KalmanVars.Noise_Angl = 12.5 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

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
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }
            if (SINSstate.Global_file == "ktn004_21.03.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.01024;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.LastCountForRead = 430000;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 1.0;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Vel = 0.21 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Angl = 70.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

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
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

                SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Altitude);
            }

            if (SINSstate.Global_file == "povorot_12.09.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.01024;
                SINSstate.Odo_Limit_Measures = 5;

                SINSstate.dV_q = 2.5;

                SINSstate.LastCountForRead = 60800;

                //KalmanVars.OdoNoise = 1.0 / SINSstate.Odo_Limit_Measures;
                KalmanVars.OdoNoise = 1.0;
                KalmanVars.OdoNoise_STOP = 0.005;       //!!!

                KalmanVars.Noise_Pos = 0.75 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Vel = 0.31 * 9.78049 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Angl = 30.0 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Drift = 0.2 * 3.141592 / 180.0 / 3600.0 * Math.Sqrt(SINSstate.Freq);
                KalmanVars.Noise_Accel = 0.0000002 * Math.Sqrt(SINSstate.Freq);

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.79958;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 0.85144;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 22.58301;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
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
