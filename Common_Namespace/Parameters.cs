using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace Common_Namespace
{
    public class Parameters : SimpleOperations
    {
        public static void ApplyMatrixStartCondition(SINS_State SINSstate)
        {
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


        public static void StartSINS_Parameters(SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, ParamToStart ParamStart, Proc_Help ProcHelp)
        {

            SINSstate.decrementVerticalNoise = 1.0;
            SINSstate.MyOwnKalman_Korrection = false;
            SINSstate.MyOwnKalman_Forecast = false;

            if (SINSstate.Global_file == "Imitator_Data")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.OdoLimitMeasuresNum = 1;

                SINSstate.decrementVerticalNoise = 1.0;
                SINSstate.existRelationHoriz_VS_Vertical = false;

                SINSstate.MyOwnKalman_Korrection = true;
                SINSstate.MyOwnKalman_Forecast = true;

                // === best configurations === //
                //VertRel=0	NoisModl=1	MyCorr=1	MyFut=0	CoordNois=1	Class=0.02
                //VertRel=0	NoisModl=1	MyCorr=1	MyFut=0	CoordNois=0	Class=0.02                                                                                                                                                                                      

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.000000002;
            }




            if (SINSstate.Global_file == "Azimut_15.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.OdoLimitMeasuresNum = 10;
                SINSstate.odo_min_increment = 0.2;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment ;
                KalmanVars.OdoNoise_STOP = 0.1;

                SINSstate.existRelationHoriz_VS_Vertical = false;


                //=== С параметрами ниже решение OdoSINS лучше SINSOdo (акцент на 3E-5 и 3E-7)
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-5; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-7; //3E-6- optim
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = 0.05;
                ParamStart.Experiment_stdKappa1 = 20.0; //минут
                ParamStart.Experiment_stdKappa3 = 20.0; //минут
                ParamStart.Experiment_GPS_PositionError = 5.0; // в метрах
                //===

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = (56.2681502 - 0.00151686666666666666666666666667) * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = (57.9990499 + 3.7787777777777777777777777777778e-4) * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 175.076;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -0 * SimpleData.ToRadian;
                SINSstate.Roll = 0 * SimpleData.ToRadian;
                SINSstate.Pitch = -0 * SimpleData.ToRadian;

                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }

            if (SINSstate.Global_file == "Azimut_24.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.OdoLimitMeasuresNum = 5;
                SINSstate.odo_min_increment = 0.2;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                SINSstate.existRelationHoriz_VS_Vertical = false;

                //=== 
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-3; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-5; //3E-6- optim
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = 0.05;
                ParamStart.Experiment_stdKappa1 = 20.0; //минут
                ParamStart.Experiment_stdKappa3 = 20.0; //минут
                ParamStart.Experiment_GPS_PositionError = 5.0; // в метрах
                //===

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.268466 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9993716 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 177.7876;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }
            if (SINSstate.Global_file == "Azimut_29.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.OdoLimitMeasuresNum = 10;

                SINSstate.odo_min_increment = 0.2;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                SINSstate.existRelationHoriz_VS_Vertical = false;

                //=== 
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-3; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-5; //3E-6- optim
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = 0.05;
                ParamStart.Experiment_stdKappa1 = 20.0; //минут
                ParamStart.Experiment_stdKappa3 = 20.0; //минут
                ParamStart.Experiment_GPS_PositionError = 5.0; // в метрах
                //===

                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.268466 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9987987 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 173.8157;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                //Углы найденные подбором минимизацией максимальной ошибки по позиции.
                SINSstate.Heading = -26.26266 * SimpleData.ToRadian;
                SINSstate.Roll = 1.753250 * SimpleData.ToRadian;
                SINSstate.Pitch = -1.510889 * SimpleData.ToRadian;

                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }

            if (SINSstate.Global_file == "ktn004_15.03.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.01024;
                SINSstate.OdoLimitMeasuresNum = 5;

                SINSstate.odo_min_increment = 0.035;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.decrementVerticalNoise = 1.0;
                SINSstate.existRelationHoriz_VS_Vertical = false;

                // -- С MyOwnKalman_Korrection=true при чекнутых шумах dR только в горизонте получается конечная ошибка  метра!!
                SINSstate.MyOwnKalman_Korrection = true;
                SINSstate.MyOwnKalman_Forecast = false;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = true; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-003; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-005; //3E-6- optim При этом ошибка - максимум 50 метров!!!
                //===

                // === best configurations === //
                //VertRel=0	NoisModl=1	MyCorr=1	MyFut=0	CoordNois=0	Class=0.02	Noise=1E-05

                KalmanVars.Noise_Pos = 0.5;
                // -------------------------------------------//



                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa = 0.01 * 3.141592 / 180.0 / 3600.0;

                ParamStart.Experiment_stdR = 0.05;
                ParamStart.Experiment_stdOdoR = 0.05; // метров
                ParamStart.Experiment_stdV = 0.01;
                ParamStart.Experiment_stdScale = 0.005;
                ParamStart.Experiment_stdKappa1 = 5.0; //минут
                ParamStart.Experiment_stdKappa3 = 5.0; //минут
                ParamStart.Experiment_GPS_PositionError = 10.0; // в метрах


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.7520087 - 3.1372635679012345679012345679012e-5;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 0.9824307 + 2.8596974074074074074074074074074e-6;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 91.48914;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);

                SINSstate.alpha_x = 0.1 * SimpleData.ToRadian;
                SINSstate.alpha_y = 0.08 * SimpleData.ToRadian;
                SINSstate.alpha_z = 0.46 * SimpleData.ToRadian;
            }


            if (SINSstate.Global_file == "GRTVout_GCEF_format_030715выезд")
            {

                SINSstate.timeStep = SINSstate.Freq = 0.020480;
                SINSstate.OdoLimitMeasuresNum = 5;

                SINSstate.odo_min_increment = 0.1;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.decrementVerticalNoise = 1.0;
                SINSstate.existRelationHoriz_VS_Vertical = false;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = true; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 5.00E-001; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-003; //3E-6- optim При этом ошибка - максимум 50 метров!!!
                //===

                KalmanVars.Noise_Pos = 0.5;
                // -------------------------------------------//



                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa = 0.01 * 3.141592 / 180.0 / 3600.0;

                ParamStart.Experiment_stdR = 0.05;
                ParamStart.Experiment_stdOdoR = 0.05; // метров
                ParamStart.Experiment_stdV = 0.01;
                ParamStart.Experiment_stdScale = 0.005;
                ParamStart.Experiment_stdKappa1 = 5.0; //минут
                ParamStart.Experiment_stdKappa3 = 5.0; //минут
                ParamStart.Experiment_GPS_PositionError = 10.0; // в метрах


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.264756 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.999337 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 10.0;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);

                //flElevation=-1.126
                SINSstate.alpha_x = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_y = 1.126 * SimpleData.ToRadian;
                SINSstate.alpha_z = 0.0 * SimpleData.ToRadian;
            }


            if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд завод)")
            {

                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.OdoLimitMeasuresNum = 5;

                SINSstate.odo_min_increment = 0.1;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.decrementVerticalNoise = 1.0;
                SINSstate.existRelationHoriz_VS_Vertical = false;

                SINSstate.MyOwnKalman_Korrection = true;
                SINSstate.MyOwnKalman_Forecast = false;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-004; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-006; //3E-6- optim При этом ошибка - максимум 50 метров!!!
                //===

                KalmanVars.Noise_Pos = 1.0;
                // -------------------------------------------//



                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                ParamStart.Experiment_stdR = 0.05;
                ParamStart.Experiment_stdOdoR = 0.05; // метров
                ParamStart.Experiment_stdV = 0.01;
                ParamStart.Experiment_stdScale = 0.005;
                ParamStart.Experiment_stdKappa1 = 1.0; //минут
                ParamStart.Experiment_stdKappa3 = 5.0; //минут
                ParamStart.Experiment_GPS_PositionError = 10.0; // в метрах


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.26555 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.998705555 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 159.8;

                //--- Координаты Кроссовского ---//
                //ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.26703 * SimpleData.ToRadian;
                //ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9983 * SimpleData.ToRadian;
                //ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 159.8;


                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);

                //flElevation=-1.126
                SINSstate.alpha_x = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_y = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_z = 0.0 * SimpleData.ToRadian;
            }


            if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд куликовка)")
            {

                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.OdoLimitMeasuresNum = 5;

                SINSstate.odo_min_increment = 0.1;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.decrementVerticalNoise = 1.0;
                SINSstate.existRelationHoriz_VS_Vertical = false;

                SINSstate.MyOwnKalman_Korrection = true;
                SINSstate.MyOwnKalman_Forecast = false;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-004; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-006; //3E-6- optim При этом ошибка - максимум 50 метров!!!
                //===

                // === best configurations === //
                //VertRel=1	NoisModl=0	MyCorr=0	MyFut=1	CoordNois=0	Class=0.02
                //VertRel=0	NoisModl=0	MyCorr=1	MyFut=0	CoordNois=1	Class=0.02

                KalmanVars.Noise_Pos = 1.0;
                // -------------------------------------------//



                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                ParamStart.Experiment_stdR = 0.05;
                ParamStart.Experiment_stdOdoR = 0.05; // метров
                ParamStart.Experiment_stdV = 0.01;
                ParamStart.Experiment_stdScale = 0.005;
                ParamStart.Experiment_stdKappa1 = 1.0; //минут
                ParamStart.Experiment_stdKappa3 = 5.0; //минут
                ParamStart.Experiment_GPS_PositionError = 10.0; // в метрах


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.75999166666 * SimpleData.ToRadian; //56.76146
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 58.02436111111 * SimpleData.ToRadian; //58.02398
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 187;

                //--- Координаты Кроссовского ---//
                //ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.76146 * SimpleData.ToRadian;
                //ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 58.02398 * SimpleData.ToRadian;
                //ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 187;


                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);

                //flElevation=-1.126
                SINSstate.alpha_x = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_y = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_z = 0.0 * SimpleData.ToRadian;
            }







            //МИНСКИЕ ЗАЕЗДЫ

            if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.2;
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.OdoLimitMeasuresNum = 1;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;
                SINSstate.ControlPointCount[0] = 29297;
                SINSstate.ControlPointCount[1] = 48829;
                SINSstate.ControlPointCount[2] = 73243;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 10.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                SINSstate.existRelationHoriz_VS_Vertical = false;

                //=== 
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-4; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-6; //3E-6- optim
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = -0.001;
                ParamStart.Experiment_stdKappa1 = 5.0; //минут
                ParamStart.Experiment_stdKappa3 = 5.0; //минут
                ParamStart.Experiment_GPS_PositionError = 5.0; // в метрах
                //===

                KalmanVars.Noise_Pos = 1.1;
                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.485964934299;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 0.9414566620339;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 217.084;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }










            if (SINSstate.Global_file == "AZIMUT_T_2013_10_18_12_55")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.1268;
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.OdoLimitMeasuresNum = 2;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 10.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                SINSstate.existRelationHoriz_VS_Vertical = false;

                //=== 
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-4; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-6; //3E-6- optim
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = -0.001;
                ParamStart.Experiment_stdKappa1 = 5.0; //минут
                ParamStart.Experiment_stdKappa3 = 5.0; //минут
                ParamStart.Experiment_GPS_PositionError = 5.0; // в метрах
                //===

                KalmanVars.Noise_Pos = 0.000075;
                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.982366681098938;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 1.00708794593811;
                ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = 272.181;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }








            if (SINSstate.Global_file == "Azimut_514_08Nov2013_11_15")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.1268;
                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                SINSstate.OdoLimitMeasuresNum = 1;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                SINSstate.existRelationHoriz_VS_Vertical = false;

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


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }





            if (SINSstate.Global_file == "Saratov_run_2014_07_23")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.odo_min_increment = 0.05;
                SINSstate.timeStep = SINSstate.Freq = 0.01048;
                SINSstate.OdoLimitMeasuresNum = 1;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                SINSstate.existRelationHoriz_VS_Vertical = false;

                SINSstate.MyOwnKalman_Korrection = false;

                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-2; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-4; //3E-6- optim
                ParamStart.Experiment_stdR = 0.10;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 0.01;
                ParamStart.Experiment_stdScale = 0.005;
                ParamStart.Experiment_stdKappa1 = 0.01; //минут
                ParamStart.Experiment_stdKappa3 = 0.01; //минут
                ParamStart.Experiment_GPS_PositionError = 5.0; // в метрах


                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa = 0.01 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Pos = 1.1;
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

                SINSstate.alpha_x = 0.0 * SimpleData.ToRadian;
                //SINSstate.alpha_y = 1.0 * SimpleData.ToRadian;
                SINSstate.alpha_z = 0.0 * SimpleData.ToRadian;

                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);


                string str_markers = "";
                StreamReader Markers = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23_Markers.csv");
                str_markers = Markers.ReadLine();
                str_markers = Markers.ReadLine();

                for (int i = 0; ; i++)
                {
                    if (Markers.EndOfStream == true) break;

                    str_markers = Markers.ReadLine();
                    string[] str_markers_array = str_markers.Split(' ');

                    for (int j = 0; j < str_markers_array.Count(); j++)
                        SINSstate.MarkersInputData[i, j] = Convert.ToDouble(str_markers_array[j]);

                    SINSstate.MarkersInputCount++;
                }
            }




        }
    }
}
