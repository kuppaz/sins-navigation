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

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);
        }


        public static void StartSINS_Parameters(SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, ParamToStart ParamStart, Proc_Help ProcHelp)
        {
            SINSstate.MyOwnKalman_Korrection = false;
            SINSstate.flag_equalizeVertNoise = false;
            SINSstate.first100m_StartHeightCorrection_value = 100.0;
            KalmanVars.Noise_Pos_Odo = 0.0;

            SINSstate.OdoVerticalNoiseMultiplicator = 5;

            SINSstate.AlgebraicCalibration_F_Zero = false;

            // --- Параметры намеренного дополнительного введения ошибок одометра --- //
            SINSstate.WRONG_scaleError = 0.0;
            SINSstate.WRONG_alpha_kappa_1 = 0.0;
            SINSstate.WRONG_alpha_kappa_3 = 0.0;

            ParamStart.Experiment_Marker_PositionError = 0.1; // в метрах
            ParamStart.Experiment_GPS_PositionError = 2.0; // в метрах


            if (SINSstate.Global_file == "Imitator_Data")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 1;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = false;
                SINSstate.Alignment_HeadingValue = 0.0 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 9000;

                SINSstate.existRelationHoriz_VS_Vertical = false;

                SINSstate.MyOwnKalman_Korrection = false;

                //---для имитатора---
                ParamStart.Imitator_NoiseModelFlag = true; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Imitator_Noise_Vel = 3E-4;
                ParamStart.Imitator_Noise_Angl = 3E-6;

                // === best configurations === //
                // Для 0.02 Heading_tmpDevide должен быть равен 1, а шумы - 3Е-6
                // Для 0.2 Heading_tmpDevide должен быть равен 10, а шумы - 3Е-6


                ParamStart.Imitator_Noise_OdoScale = 0.000000001;
                ParamStart.Imitator_Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;
                ParamStart.Imitator_Noise_Pos = 0.1;
                ParamStart.Imitator_Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                ParamStart.Imitator_Noise_Accel = 0.000000002;

                ParamStart.Imitator_stdR = 0.5;
                ParamStart.Imitator_stdOdoR = 0.5; // метров
                ParamStart.Imitator_stdV = 0.1;
                ParamStart.Imitator_stdScale = 0.01;
                ParamStart.Imitator_stdKappa1 = 20.0; //минут
                ParamStart.Imitator_stdKappa3 = 20.0; //минут


                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.000000002;
            }




            if (SINSstate.Global_file == "Azimut_15.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 10;
                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.2;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = false;
                SINSstate.Alignment_HeadingValue = 0.0 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 40000;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = false;

                //=== С параметрами ниже решение OdoSINS лучше SINSOdo (акцент на 3E-5 и 3E-7)
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-5; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-7; //3E-6- optim

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = 0.05;
                ParamStart.Experiment_stdKappa1 = 20.0; //минут
                ParamStart.Experiment_stdKappa3 = 20.0; //минут
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Pos_Odo = 0.0;

                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = (56.2681502 - 0.00151686666666666666666666666667) * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = (57.9990499 + 3.7787777777777777777777777777778e-4) * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 175.076;

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

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;
                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.2;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = false;
                SINSstate.Alignment_HeadingValue = 0.0 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 45000;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = false;

                //=== 
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-3; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-5; //3E-6- optim
                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = 0.05;
                ParamStart.Experiment_stdKappa1 = 20.0; //минут
                ParamStart.Experiment_stdKappa3 = 20.0; //минут
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Pos_Odo = 0.0;

                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.268466 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9993716 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 177.7876;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }
            if (SINSstate.Global_file == "Azimut_29.08.2012")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 10;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.2;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = false;
                SINSstate.Alignment_HeadingValue = 0.0 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 35000;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = false;

                //=== 
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-3; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-5; //3E-6- optim
                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = 0.05;
                ParamStart.Experiment_stdKappa1 = 20.0; //минут
                ParamStart.Experiment_stdKappa3 = 20.0; //минут
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 0.75;
                KalmanVars.Noise_Pos_Odo = 0.0;

                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.268466 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.9987987 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 173.8157;

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

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.035;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = 15.28 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 48000;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = true;

                // -- С MyOwnKalman_Korrection=true при чекнутых шумах dR только в горизонте получается конечная ошибка  метра!!
                SINSstate.MyOwnKalman_Korrection = false;

                SINSstate.first100m_StartHeightCorrection_value = 100.0;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-003; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-005; //3E-6- optim При этом ошибка - максимум 50 метров!!!
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.0;
                KalmanVars.Noise_Pos_Odo = 0.0;
                // -------------------------------------------//

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.1;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 1.0;

                SINSstate.SINS_is_accurateMounted_by_kappa_1 = false;
                SINSstate.SINS_is_accurateMounted_by_kappa_3 = true;
                SINSstate.SINS_is_accurateMounted_by_scaleError = false;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_1 == true)
                    ParamStart.Experiment_stdKappa1 = 2.0;
                else
                    ParamStart.Experiment_stdKappa1 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_3 == true)
                    ParamStart.Experiment_stdKappa3 = 2.0;
                else
                    ParamStart.Experiment_stdKappa3 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_scaleError == true)
                    ParamStart.Experiment_stdScale = 0.001;
                else
                    ParamStart.Experiment_stdScale = 0.01;


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.7520087 - 3.1372635679012345679012345679012e-5;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 0.9824307 + 2.8596974074074074074074074074074e-6;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 91.48914;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);

                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_scaleError = 0.0;

                if (Math.Abs(SINSstate.alpha_kappa_3) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa3 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_kappa_1) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa1 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                    ParamStart.Experiment_stdScale = 0.001;
            }



            if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд завод)")
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.1;
                SINSstate.OdoVerticalNoiseMultiplicator = 5;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = 153.4796 * SimpleData.ToRadian;
                SINSstate.Alignment_RollDetermined = true;
                SINSstate.Alignment_RollValue = 0.74554 * SimpleData.ToRadian;
                SINSstate.Alignment_PitchDetermined = true;
                SINSstate.Alignment_PitchValue = 3.27207 * SimpleData.ToRadian;


                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = Convert.ToInt32(500.0 / SINSstate.timeStep);

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = true;
                SINSstate.MyOwnKalman_Korrection = false;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-004; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-006; //3E-6- optim При этом ошибка - максимум 50 метров!!!
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.0;
                KalmanVars.Noise_Pos_Odo = 0.0;
                // -------------------------------------------//

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.1;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 1.0;

                SINSstate.SINS_is_accurateMounted_by_kappa_1 = true;
                SINSstate.SINS_is_accurateMounted_by_kappa_3 = true;
                SINSstate.SINS_is_accurateMounted_by_scaleError = true;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_1 == true)
                    ParamStart.Experiment_stdKappa1 = 2.0;
                else
                    ParamStart.Experiment_stdKappa1 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_3 == true)
                    ParamStart.Experiment_stdKappa3 = 2.0;
                else
                    ParamStart.Experiment_stdKappa3 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_scaleError == true)
                    ParamStart.Experiment_stdScale = 0.005;
                else
                    ParamStart.Experiment_stdScale = 0.01;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.267028 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.998296 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 159.8;


                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);

                //flElevation=-1.126
                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_scaleError = 0.0;

                if (Math.Abs(SINSstate.alpha_kappa_3) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa3 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_kappa_1) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa1 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                    ParamStart.Experiment_stdScale = 0.001;
            }


            if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд куликовка)")
            {
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1.5	OdoQzV=0.5	Class=0.2	Noise=NO	25.441	62.868	45.27	29374.959	9.705	15.406	15.403
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1.5	OdoQzV=4.5	Class=0.2	Noise=NO	25.442	62.874	45.274	29374.969	10.114	16.528	16.341
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1.5	OdoQzV=3.5	Class=0.2	Noise=NO	25.443	62.875	45.276	29374.968	10.109	16.568	16.267
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1.5	OdoQzV=2.5	Class=0.2	Noise=NO	25.443	62.875	45.277	29374.967	10.06	16.508	16.099
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1.5	OdoQzV=1.5	Class=0.2	Noise=NO	25.443	62.875	45.277	29374.965	9.979	16.285	15.922
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1	    OdoQzV=0.5	Class=0.2	Noise=NO	25.393	56.833	50.507	29350.243	9.763	15.552	15.552
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1	    OdoQzV=4.5	Class=0.2	Noise=NO	25.393	56.834	50.507	29350.253	10.17	16.646	16.482
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1	    OdoQzV=3.5	Class=0.2	Noise=NO	25.393	56.836	50.509	29350.253	10.166	16.685	16.408
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1	    OdoQzV=2.5	Class=0.2	Noise=NO	25.394	56.837	50.51	29350.251	10.116	16.624	16.242
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1	    OdoQzV=1.5	Class=0.2	Noise=NO	25.394	56.838	50.511	29350.249	10.036	16.402	16.067
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=0.5	OdoQzV=4.5	Class=0.2	Noise=NO	26.857	63.59	55.024	29366.467	10.094	16.448	16.346
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=0.5	OdoQzV=0.5	Class=0.2	Noise=NO	26.858	63.589	55.025	29366.459	9.863	15.759	15.725
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=0.5	OdoQzV=3.5	Class=0.2	Noise=NO	26.858	63.593	55.027	29366.467	10.139	16.576	16.369
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=0.5	OdoQzV=1.5	Class=0.2	Noise=NO	26.859	63.593	55.028	29366.464	10.037	16.429	16.01
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=0.5	OdoQzV=2.5	Class=0.2	Noise=NO	26.859	63.594	55.029	29366.466	10.125	16.601	16.255

                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.1;
                SINSstate.OdoVerticalNoiseMultiplicator = 5;

                /*!!!!!!!!!*/
                //SINSstate.global_odo_measure_noise = 1.5;
                //SINSstate.global_odo_measure_noise_Vertical = 0.5;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = -78.61045 * SimpleData.ToRadian;
                SINSstate.Alignment_RollDetermined = true;
                SINSstate.Alignment_RollValue = -1.81451 * SimpleData.ToRadian;
                SINSstate.Alignment_PitchDetermined = true;
                SINSstate.Alignment_PitchValue = -0.78953 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = Convert.ToInt32(450.0 / SINSstate.timeStep);

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.001;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = true;
                SINSstate.MyOwnKalman_Korrection = false;

                SINSstate.first100m_StartHeightCorrection_value = 100.0;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-003; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-005; //3E-6- optim При этом ошибка - максимум 50 метров!!!
                //===

                // === best configurations === //
                //VertRel=0	NoisModl=0	eqlzVert=1	MyCorr=1	CoordNois=1	Class=0.2	Noise=NO

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.0;
                KalmanVars.Noise_Pos_Odo = 0.01;
                // -------------------------------------------//

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.1 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.1;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 1.0;

                SINSstate.SINS_is_accurateMounted_by_kappa_1 = true;
                SINSstate.SINS_is_accurateMounted_by_kappa_3 = true;
                SINSstate.SINS_is_accurateMounted_by_scaleError = true;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_1 == true)
                    ParamStart.Experiment_stdKappa1 = 2.0;
                else
                    ParamStart.Experiment_stdKappa1 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_3 == true)
                    ParamStart.Experiment_stdKappa3 = 2.0;
                else
                    ParamStart.Experiment_stdKappa3 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_scaleError == true)
                    ParamStart.Experiment_stdScale = 0.005;
                else
                    ParamStart.Experiment_stdScale = 0.01;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.76146 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 58.02398 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 187;


                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);

                //flElevation=-1.126
                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_scaleError = 0.0;


                if (Math.Abs(SINSstate.alpha_kappa_3) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa3 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_kappa_1) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa1 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                    ParamStart.Experiment_stdScale = 0.001;
            }


            if (SINSstate.Global_file == "GRTV_Ekat_151029_1_zaezd")
            {
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=1	    OdoQzV=4.5	Class=0.2	Noise=NO	38.617	89.678	69.807	68.857	22.188	56.225	-16.51
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=7	OdoQz=1	    OdoQzV=4.5	Class=0.2	Noise=NO	38.619	88.299	70.821	69.848	20.221	60.34	5.482
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=9	OdoQz=1	    OdoQzV=3.5	Class=0.2	Noise=NO	39.018	87.836	72.343	71.354	21.518	57.056	-11.785
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=9	OdoQz=2.5	OdoQzV=3.5	Class=0.2	Noise=NO	38.994	87.933	72.397	71.383	21.423	57.061	-11.421
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1	    OdoQzV=1.5	Class=0.02	Noise=NO	39.499	91.923	69.936	68.986	28.862	61.061	21.981
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=3	    OdoQzV=1.5	Class=0.02	Noise=NO	38.601	88.628	70.295	69.281	28.833	61.091	22.465

                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.1;
                SINSstate.OdoVerticalNoiseMultiplicator = 5;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = 33.91437 * SimpleData.ToRadian;
                SINSstate.Alignment_RollDetermined = true;
                SINSstate.Alignment_RollValue = 0.06796 * SimpleData.ToRadian;
                SINSstate.Alignment_PitchDetermined = true;
                SINSstate.Alignment_PitchValue = 1.23866 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = Convert.ToInt32(500.0 / SINSstate.timeStep);

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = true;
                SINSstate.MyOwnKalman_Korrection = false;

                SINSstate.first100m_StartHeightCorrection_value = 130.0;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-003; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-005; //3E-6- optim 
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.0;
                KalmanVars.Noise_Pos_Odo = 0.01;
                // -------------------------------------------//

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.03 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.1 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.1;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 1.0;

                SINSstate.SINS_is_accurateMounted_by_kappa_1 = false;
                SINSstate.SINS_is_accurateMounted_by_kappa_3 = true;
                SINSstate.SINS_is_accurateMounted_by_scaleError = false;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_1 == true)
                    ParamStart.Experiment_stdKappa1 = 2.0;
                else
                    ParamStart.Experiment_stdKappa1 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_3 == true)
                    ParamStart.Experiment_stdKappa3 = 2.0;
                else
                    ParamStart.Experiment_stdKappa3 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_scaleError == true)
                    ParamStart.Experiment_stdScale = 0.001;
                else
                    ParamStart.Experiment_stdScale = 0.01;


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 60.71691011111111 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.06235011111111 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 306.0;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_scaleError = 0.0;


                if (Math.Abs(SINSstate.alpha_kappa_3) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa3 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_kappa_1) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa1 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                    ParamStart.Experiment_stdScale = 0.001;

                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }
            if (SINSstate.Global_file == "GRTV_Ekat_151029_2_zaezd")
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.1;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = 40.31628 * SimpleData.ToRadian;
                SINSstate.Alignment_RollDetermined = true;
                SINSstate.Alignment_RollValue = -0.45866 * SimpleData.ToRadian;
                SINSstate.Alignment_PitchDetermined = true;
                SINSstate.Alignment_PitchValue = -0.8464 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = Convert.ToInt32(380.0 / SINSstate.timeStep);

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = true;
                SINSstate.MyOwnKalman_Korrection = false;

                SINSstate.first100m_StartHeightCorrection_value = 130.0;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-003; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-005; //3E-6- optim
                //===

                // === best configurations === //
                //VertRel=0	NoisModl=0	eqlzVert=0	MyCorr=0	CoordNois=1	Class=0.2	Noise=NO -- не, ну не плохо. Повторили решение ПНППК + высота получше
                //VertRel=0	NoisModl=1	eqlzVert=1	MyCorr=1	CoordNois=1	Class=0.2	Noise=1E-05 -- выглядит красиво. Нет уверенности, что старт.тчку выбрал правильно

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.0;
                KalmanVars.Noise_Pos_Odo = 0.01;
                // -------------------------------------------//

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.001 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.1 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.1;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 1.0;

                SINSstate.SINS_is_accurateMounted_by_kappa_1 = false;
                SINSstate.SINS_is_accurateMounted_by_kappa_3 = true;
                SINSstate.SINS_is_accurateMounted_by_scaleError = false;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_1 == true)
                    ParamStart.Experiment_stdKappa1 = 2.0;
                else
                    ParamStart.Experiment_stdKappa1 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_3 == true)
                    ParamStart.Experiment_stdKappa3 = 2.0;
                else
                    ParamStart.Experiment_stdKappa3 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_scaleError == true)
                    ParamStart.Experiment_stdScale = 0.001;
                else
                    ParamStart.Experiment_stdScale = 0.01;


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 60.71691 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.06235 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 306.0;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian; //0.5
                SINSstate.alpha_scaleError = 0.0;

                if (Math.Abs(SINSstate.alpha_kappa_3) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa3 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_kappa_1) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa1 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                    ParamStart.Experiment_stdScale = 0.001;

                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }



            if (SINSstate.Global_file == "GRTV_ktn004_marsh16_afterbdnwin_20032012")
            {
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1	    OdoQzV=1.5	Class=0.2	Noise=NO	11.778	24.992	14.616	14.976	11.986	29.099	9.653
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1.5	OdoQzV=1.5	Class=0.2	Noise=NO	11.692	23.93	14.591	14.95	11.993	29.115	9.679
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=2	    OdoQzV=1.5	Class=0.2	Noise=NO	11.683	23.438	14.576	14.933	11.997	29.124	9.696
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=1	OdoQz=2	    OdoQzV=2.5	Class=0.2	Noise=NO	12.076	26.191	14.861	15.232	12.185	28.634	9.485
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=1	OdoQz=2.5	OdoQzV=2.5	Class=0.2	Noise=NO	11.862	25.151	14.799	15.165	12.192	28.654	9.512
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=1	OdoQz=3	    OdoQzV=2.5	Class=0.2	Noise=NO	11.786	24.427	14.757	15.119	12.197	28.667	9.53
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=0.5	OdoQzV=1.5	Class=0.2	Noise=NO	11.886	25.242	14.855	15.227	11.607	32.337	3.898
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=1	    OdoQzV=1.5	Class=0.2	Noise=NO	11.8	24.226	14.836	15.206	11.612	32.337	3.919
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=1.5	OdoQzV=1.5	Class=0.2	Noise=NO	11.785	23.775	14.822	15.19	11.616	32.336	3.937

                SINSstate.timeStep = SINSstate.Freq = 0.01024;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.03;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = 15.05 * SimpleData.ToRadian;
                SINSstate.Alignment_RollDetermined = true;
                SINSstate.Alignment_RollValue = -0.4925 * SimpleData.ToRadian;
                SINSstate.Alignment_PitchDetermined = true;
                SINSstate.Alignment_PitchValue = 0.3423 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = Convert.ToInt32(593.0 / SINSstate.timeStep);

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = true;
                SINSstate.MyOwnKalman_Korrection = false;

                SINSstate.first100m_StartHeightCorrection_value = 130.0;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-003; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-005; //3E-6- optim
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.0;
                KalmanVars.Noise_Pos_Odo = 0.01;
                // -------------------------------------------//

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.001 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.1 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.1;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 1.0;

                SINSstate.SINS_is_accurateMounted_by_kappa_1 = false;
                SINSstate.SINS_is_accurateMounted_by_kappa_3 = true;
                SINSstate.SINS_is_accurateMounted_by_scaleError = false;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_1 == true)
                    ParamStart.Experiment_stdKappa1 = 2.0;
                else
                    ParamStart.Experiment_stdKappa1 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_3 == true)
                    ParamStart.Experiment_stdKappa3 = 2.0;
                else
                    ParamStart.Experiment_stdKappa3 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_scaleError == true)
                    ParamStart.Experiment_stdScale = 0.001;
                else
                    ParamStart.Experiment_stdScale = 0.01;


                SINSstate.OdoVerticalNoiseMultiplicator = 5;


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 43.08689 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 56.28916 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 96.0;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian; //0.5
                SINSstate.alpha_scaleError = 0.0;

                if (Math.Abs(SINSstate.alpha_kappa_3) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa3 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_kappa_1) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa1 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                    ParamStart.Experiment_stdScale = 0.001;

                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }

            if (SINSstate.Global_file == "GRTV_ktn004_marsh16_repeat_21032012")
            {
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=1	OdoQz=2	    OdoQzV=4.5	Class=0.2	Noise=NO	12.732	28.508	16.732	17.699	12.954	56.376	5.619
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=1	OdoQz=2	    OdoQzV=3.5	Class=0.2	Noise=NO	12.754	28.514	16.721	17.686	12.184	53.486	5.85
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=1	OdoQz=2.5	OdoQzV=2.5	Class=0.2	Noise=NO	12.016	26.597	16.623	17.588	12.275	50.6	7.91
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=2	    OdoQzV=3.5	Class=0.2	Noise=NO	11.038	24.208	16.484	17.455	15.763	62.463	9.224
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1.5	OdoQzV=3.5	Class=0.2	Noise=NO	11.331	25.247	16.5	17.471	15.765	62.46	9.227
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=3	OdoQz=1	    OdoQzV=3.5	Class=0.2	Noise=NO	11.981	26.903	16.527	17.497	15.768	62.455	9.233
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=2.5	OdoQzV=2.5	Class=0.2	Noise=NO	10.88	23.751	16.67	17.637	15.669	61.948	9.767
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=1.5	OdoQzV=2.5	Class=0.2	Noise=NO	11.088	24.367	16.684	17.65	15.671	61.944	9.768
                //NoisModl=0	eqlzVert=1	MyCorr=0	CoordNois=1	OdoCntZ=5	OdoQz=1	    OdoQzV=2.5	Class=0.2	Noise=NO	11.436	25.607	16.7	17.666	15.674	61.94	9.771

                SINSstate.timeStep = SINSstate.Freq = 0.01024;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.03;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = 14.93 * SimpleData.ToRadian;
                SINSstate.Alignment_RollDetermined = true;
                SINSstate.Alignment_RollValue = -0.74279 * SimpleData.ToRadian;
                SINSstate.Alignment_PitchDetermined = true;
                SINSstate.Alignment_PitchValue = 0.34722 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = Convert.ToInt32(456.0 / SINSstate.timeStep);

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.5;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = true;
                SINSstate.MyOwnKalman_Korrection = false;

                SINSstate.first100m_StartHeightCorrection_value = 130.0;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-003; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-005; //3E-6- optim
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.0;
                KalmanVars.Noise_Pos_Odo = 0.01;
                // -------------------------------------------//

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.001 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.1 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.1;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 1.0;

                SINSstate.SINS_is_accurateMounted_by_kappa_1 = false;
                SINSstate.SINS_is_accurateMounted_by_kappa_3 = true;
                SINSstate.SINS_is_accurateMounted_by_scaleError = false;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_1 == true)
                    ParamStart.Experiment_stdKappa1 = 2.0;
                else
                    ParamStart.Experiment_stdKappa1 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_3 == true)
                    ParamStart.Experiment_stdKappa3 = 2.0;
                else
                    ParamStart.Experiment_stdKappa3 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_scaleError == true)
                    ParamStart.Experiment_stdScale = 0.001;
                else
                    ParamStart.Experiment_stdScale = 0.01;


                SINSstate.OdoVerticalNoiseMultiplicator = 5;


                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 43.08689 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 56.28916 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 96.0;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian; //0.5
                SINSstate.alpha_scaleError = 0.0;


                if (Math.Abs(SINSstate.alpha_kappa_3) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa3 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_kappa_1) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa1 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                    ParamStart.Experiment_stdScale = 0.001;

                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }










            //МИНСКИЕ ЗАЕЗДЫ

            if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 1;
                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.2;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;
                SINSstate.ControlPointCount[0] = 29297;
                SINSstate.ControlPointCount[1] = 48829;
                SINSstate.ControlPointCount[2] = 73243;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = -3.0504734;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 10300;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 10.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                SINSstate.existRelationHoriz_VS_Vertical = false;

                //=== 
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-4; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-6; //3E-6- optim
                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = -0.001;
                ParamStart.Experiment_stdKappa1 = 5.0; //минут
                ParamStart.Experiment_stdKappa3 = 5.0; //минут
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.1;
                KalmanVars.Noise_Pos_Odo = 0.0;

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.485964934299;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 0.9414566620339;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 217.084;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }










            if (SINSstate.Global_file == "AZIMUT_T_2013_10_18_12_55")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 2;
                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.1268;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 10.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = false;
                SINSstate.Alignment_HeadingValue = 0.0 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 22000;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = false;

                //=== 
                ParamStart.Experiment_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-4; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-6; //3E-6- optim
                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 1.0;
                ParamStart.Experiment_stdOdoR = 1.0; // метров
                ParamStart.Experiment_stdV = 0.1;
                ParamStart.Experiment_stdScale = -0.001;
                ParamStart.Experiment_stdKappa1 = 5.0; //минут
                ParamStart.Experiment_stdKappa3 = 5.0; //минут
                //===

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 0.000075;
                KalmanVars.Noise_Pos_Odo = 0.0;

                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;
                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.982366681098938;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 1.00708794593811;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 272.181;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }








            if (SINSstate.Global_file == "Azimut_514_08Nov2013_11_15")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.02048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 1;
                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.1268;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = false;
                SINSstate.Alignment_HeadingValue = 0.0 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 95000;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.01;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = false;

                KalmanVars.Noise_OdoScale = 0.000000001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.0000001 * 3.141592 / 180.0 / 3600.0;

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 0.000075;
                KalmanVars.Noise_Pos_Odo = 0.0;

                KalmanVars.Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.00000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 0.982068359851837;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 1.01227509975433;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 172.36;

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);
            }





            if (SINSstate.Global_file == "Saratov_run_2014_07_23")                 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            {
                SINSstate.timeStep = SINSstate.Freq = 0.01048;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 1;
                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.05;

                SINSstate.DoHaveControlPoints = true;
                SINSstate.NumberOfControlPoints = 3;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = false;
                SINSstate.Alignment_HeadingValue = 0.0 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = 27320;

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / 5.0;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.1;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = false;

                SINSstate.MyOwnKalman_Korrection = false;

                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 3E-2; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 3E-4; //3E-6- optim
                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.10;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 0.01;
                ParamStart.Experiment_stdScale = 0.005;
                ParamStart.Experiment_stdKappa1 = 0.01; //минут
                ParamStart.Experiment_stdKappa3 = 0.01; //минут


                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.1;
                KalmanVars.Noise_Pos_Odo = 0.0;

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 46.87215103 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 49.99453181 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 29.314;

                if (SINSstate.Saratov_run_Final)
                {
                    ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 45.3817334 * SimpleData.ToRadian;
                    ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 49.80892188 * SimpleData.ToRadian;
                    ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 29.314;
                }

                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_scaleError = 0.0;

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


            //--- В случае выставления поправки на угол kappa_1 именшаем нач.ковариацию ---//
            if (Math.Abs(SINSstate.alpha_kappa_3) > 0.01 * SimpleData.ToRadian)
                ParamStart.Experiment_stdKappa1 = 2.0; //минут
            //--- В случае выставления поправки на угол kappa_3 именшаем нач.ковариацию ---//
            if (Math.Abs(SINSstate.alpha_kappa_1) > 0.01 * SimpleData.ToRadian)
                ParamStart.Experiment_stdKappa3 = 2.0; //минут







            if (SINSstate.Global_file == "someOtherInput")
            {
                SINSstate.timeStep = SINSstate.Freq = 0.01024;

                // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
                SINSstate.OdoLimitMeasuresNum = 5;

                // --- Минимальное приращение показания одометра --- //
                SINSstate.odo_min_increment = 0.1;
                SINSstate.OdoVerticalNoiseMultiplicator = 5;

                /*!!!!!!!!!*/
                //SINSstate.global_odo_measure_noise = 0.5;

                // --- Заданный курс: флаг и значение --- //
                SINSstate.Alignment_HeadingDetermined = true;
                SINSstate.Alignment_HeadingValue = 155.5936 * SimpleData.ToRadian;
                SINSstate.Alignment_RollDetermined = true;
                SINSstate.Alignment_RollValue = 2.421428 * SimpleData.ToRadian;
                SINSstate.Alignment_PitchDetermined = true;
                SINSstate.Alignment_PitchValue = 0.4594205 * SimpleData.ToRadian;

                // --- Количество тактов БИНС для начальной выставки от начала  --- //
                ProcHelp.AlignmentCounts = Convert.ToInt32(450.0 / SINSstate.timeStep);

                KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / SINSstate.OdoLimitMeasuresNum;
                KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
                KalmanVars.OdoNoise_STOP = 0.001;

                SINSstate.existRelationHoriz_VS_Vertical = false;
                SINSstate.flag_equalizeVertNoise = true;
                SINSstate.MyOwnKalman_Korrection = false;

                SINSstate.first100m_StartHeightCorrection_value = 100.0;

                //=== 
                //---Здесь нужно брать класс точности 2.0
                ParamStart.Experiment_NoiseModelFlag = false; // false - Брать значения шума с выставки, true - задаваемые ниже
                ParamStart.Experiment_Noise_Vel = 1.00E-003; //3E-4- optim
                ParamStart.Experiment_Noise_Angl = 1.00E-005; //3E-6- optim При этом ошибка - максимум 50 метров!!!
                //===


                // --- Шум по горизонтальным координатам --- //
                KalmanVars.Noise_Pos = 1.0;
                KalmanVars.Noise_Pos_Odo = 0.01;
                // -------------------------------------------//

                KalmanVars.Noise_Drift = 0.002 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_Accel = 0.0000002;
                KalmanVars.Noise_OdoScale = 0.0001;
                KalmanVars.Noise_OdoKappa_1 = 0.2 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;
                KalmanVars.Noise_OdoKappa_3 = 0.1 * SimpleData.ToRadian_min;// 0.01 * 3.141592 / 180.0 / 3600.0;

                // --- Начальные ковариации --- //
                ParamStart.Experiment_stdR = 0.1;
                ParamStart.Experiment_stdOdoR = 0.1; // метров
                ParamStart.Experiment_stdV = 1.0;

                SINSstate.SINS_is_accurateMounted_by_kappa_1 = true;
                SINSstate.SINS_is_accurateMounted_by_kappa_3 = true;
                SINSstate.SINS_is_accurateMounted_by_scaleError = true;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_1 == true)
                    ParamStart.Experiment_stdKappa1 = 2.0;
                else
                    ParamStart.Experiment_stdKappa1 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_kappa_3 == true)
                    ParamStart.Experiment_stdKappa3 = 2.0;
                else
                    ParamStart.Experiment_stdKappa3 = 20.0;

                if (SINSstate.SINS_is_accurateMounted_by_scaleError == true)
                    ParamStart.Experiment_stdScale = 0.005;
                else
                    ParamStart.Experiment_stdScale = 0.01;

                ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.267033 * SimpleData.ToRadian;
                ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 57.998299 * SimpleData.ToRadian;
                ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = 172.6004;


                ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
                ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;


                ApplyMatrixStartCondition(SINSstate);
                ApplyMatrixStartCondition(SINSstate_OdoMod);

                //flElevation=-1.126
                SINSstate.alpha_kappa_1 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_kappa_3 = 0.0 * SimpleData.ToRadian;
                SINSstate.alpha_scaleError = 0.0;


                if (Math.Abs(SINSstate.alpha_kappa_3) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa3 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_kappa_1) > 0.001 * SimpleData.ToRadian)
                    ParamStart.Experiment_stdKappa1 = 2.0; //минут
                if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                    ParamStart.Experiment_stdScale = 0.001;
            }
        }
    }
}
