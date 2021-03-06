﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using Common_Namespace;
using SINSAlignment;
using SINSProcessingModes;
using SINS_motion_processing;
using System.Text.RegularExpressions;



namespace SINS_motion_processing_new_data
{
    public partial class SINS_Processing : Form
    {
        int iMx = SimpleData.iMx = 25;
        int iMq = SimpleData.iMq = SimpleData.iMx;
        int iMz = SimpleData.iMz = 15;
        int iMxSmthd = SimpleData.iMxSmthd = 9;

        int iMx_Vertical = SimpleData.iMx_Vertical = 25;
        int iMq_Vertical = SimpleData.iMq_Vertical = SimpleData.iMx_Vertical;

        StreamReader myFile;

        ParamToStart ParamStart = new ParamToStart();
        SINS_State SINSstate, SINSstate_OdoMod;
        Kalman_Vars KalmanVars;
        Proc_Help ProcHelp;

        int value_iMx_dV_12, value_iMx_alphaBeta, value_iMx_Nu0, value_iMx_f0_12, value_iMx_f0_3, value_iMx_dr3, value_iMx_dV3, value_iMx_r_odo_3;
        int value_iMx_r3_dV3 = 0, value_iMx_r_odo_12 = 0, value_iMx_kappa_3_ds = 0, value_iMx_kappa_1 = 0, Vertical_kappa1, Vertical_kappa3Scale, Vertical_alphaBeta, Vertical_nu0, Vertical_f0_12, Vertical_f0_3, Vertical_rOdo3;
        int noiseParam_LastCountForRead = 0, noiseParam_StartCountForRead = 0;
        bool iMx_r3_dV3, iMx_kappa_13_ds;

        string GlobalPrefixTelemetric = "";

        public static StreamWriter GRTV_output = new StreamWriter(SimpleData.PathOutputString + "S_GRTV_output.txt");

        private bool StartParamScanning = false;
        private double Cicle_Noise_Velocity = 0, Cicle_Noise_Angular = 0
            , Cicle_Noise_Velocity_Vert = 0, Cicle_Noise_Angular_Vert = 0;

        private double global_flag_AccuracyClass = 0, global_flag_AccuracyClass_vert = 0
                , global_odo_measure_noise = 0
                , global_odo_measure_noise_Vertical = 0;

        private int global_NoiseModelFlag = 0, global_NoiseModelFlag_vert = 0
                , global_flag_equalizeVertNoise = 0
                , global_MyOwnKalman_Korrection = 0
                , global_CoordinateNoiseExist = 0
                , global_OdoLimitMeasuresNum = 0;



        public SINS_Processing()
        {
            InitializeComponent();

            LockParamsOfStart();
            LockTypesOfCorrection();
            LockTheWayOfStart();
            LockDimOfVector();
            this.usingSNS.Enabled = false;
            this.Main_Block_Click_new.Enabled = false;
        }


        public void Main_Block_Click_new_Click(object sender, EventArgs e)
        {
            this.Single_Navigation_Processing();

            GRTV_output.Close();
            this.Close();
        }



        private int global_indx = 0;
        private double[] global_kappa1_grad = new double[50000]
                       , global_kappa3_grad = new double[50000]
                       , global_scale = new double[50000]
                       , global_HorizontalError = new double[50000]
                       , global_HorizontalErrorFromStart = new double[50000]
                       , global_VerticalError = new double[50000]
                       , global_V_Up = new double[50000]
                       ;

        private void CycleStartParamChoosing_Click(object sender, EventArgs e)
        {
            double NoiseVel_start = 1E-3,
                   NoiseVel_end = 1E-2,
                   NoiseVel_multpl = 10.0;

            StreamWriter Cycle_Start_Configurations = new StreamWriter(SimpleData.PathOutputString + "CycleParamScanning//[] Cycle_Start_Configurations.txt");

            //string str = "Count NoisModl eqlzVert MyCorr CoordNois OdoCntZ OdoIncr OdoIncrV Class Noise ";
            string str = "Count OdoCntZ OdoIncr OdoIncrV Class ClassV Noise NoiseV ";
            str += "HorErr_AVG HorErr_MAX HorErr_END HorStartErr_END ";
            str += "VertErr_AVG VertErr_MAX VertErr_END ";
            str += "V_Up_AVG V_Up_SPRD V_Up_END ";
            str += "kap1_AVG kap1_SPRD kap1_END ";
            str += "kap3_AVG kap3_SPRD kap3_END ";
            str += "scale_AVG scale_SPRD scale_END ";
            Cycle_Start_Configurations.WriteLine(str);

            int i = 0;
            for (this.global_NoiseModelFlag = 0; this.global_NoiseModelFlag <= 1; this.global_NoiseModelFlag++)
            {
                for (this.global_NoiseModelFlag_vert = 0; this.global_NoiseModelFlag_vert <= 1; this.global_NoiseModelFlag_vert++)
                {
                    for (this.Cicle_Noise_Velocity = NoiseVel_start; this.Cicle_Noise_Velocity <= NoiseVel_end; this.Cicle_Noise_Velocity *= NoiseVel_multpl)
                    {
                        this.Cicle_Noise_Angular = this.Cicle_Noise_Velocity / 100.0;
                        if (this.global_NoiseModelFlag == 0)
                            this.Cicle_Noise_Velocity = NoiseVel_end;
                        //====================== 

                        for (this.Cicle_Noise_Velocity_Vert = NoiseVel_start; this.Cicle_Noise_Velocity_Vert <= NoiseVel_end; this.Cicle_Noise_Velocity_Vert *= NoiseVel_multpl)
                        {
                            this.Cicle_Noise_Angular_Vert = this.Cicle_Noise_Velocity_Vert / 100.0;
                            if (this.global_NoiseModelFlag_vert == 0)
                                this.Cicle_Noise_Velocity_Vert = NoiseVel_end;
                            //====================== 

                            for (this.global_flag_equalizeVertNoise = 1; this.global_flag_equalizeVertNoise <= 1; this.global_flag_equalizeVertNoise++)
                            {
                                if (this.global_NoiseModelFlag == 1)
                                    this.global_flag_equalizeVertNoise = 1;
                                //====================== 

                                // -- Параметр использования собственного алгоритма коррекции ФК --//
                                for (this.global_MyOwnKalman_Korrection = 0; this.global_MyOwnKalman_Korrection <= 0; this.global_MyOwnKalman_Korrection++)
                                {
                                    // -- Параметр использования шума по горизонтальным координатам --//
                                    for (this.global_CoordinateNoiseExist = 1; this.global_CoordinateNoiseExist <= 1; this.global_CoordinateNoiseExist++)
                                    {
                                        // -- Параметры частоты фиксации показаний одометра (в шт. её обновления) --// Если 0, то берется из настроечных параметров
                                        for (this.global_OdoLimitMeasuresNum = 3; this.global_OdoLimitMeasuresNum <= 8; this.global_OdoLimitMeasuresNum += 2)
                                        {
                                            // -- Параметры матрицы начальной ковариации --// Если 0, то берется из настроечных параметров
                                            for (this.global_odo_measure_noise = 0.5; this.global_odo_measure_noise <= 2.6; this.global_odo_measure_noise += 1.0)
                                            {
                                                for (this.global_odo_measure_noise_Vertical = 0.5; this.global_odo_measure_noise_Vertical <= 5.1; this.global_odo_measure_noise_Vertical += 1.5)
                                                {
                                                    // -- Параметры матрицы начальной ковариации --//
                                                    for (this.global_flag_AccuracyClass = 0.02; this.global_flag_AccuracyClass <= 0.21; this.global_flag_AccuracyClass *= 10.0)
                                                    {
                                                        for (this.global_flag_AccuracyClass_vert = 0.02; this.global_flag_AccuracyClass_vert <= 0.21; this.global_flag_AccuracyClass_vert *= 10.0)
                                                        {
                                                            // === === === === === === === === ===//
                                                            this.StartParamScanning = true;
                                                            this.Single_Navigation_Processing();
                                                            // === === === === === === === === ===//

                                                            i++;

                                                            double[] array_kappa1_grad = new double[this.global_indx - 1]
                                                                , array_kappa3_grad = new double[this.global_indx - 1]
                                                                , array_scale = new double[this.global_indx - 1]
                                                                , array_HorizontalError = new double[this.global_indx - 1]
                                                                , array_HorizontalErrorFromStart = new double[this.global_indx - 1]
                                                                , array_VerticalError = new double[this.global_indx - 1]
                                                                , array_V_Up = new double[this.global_indx - 1]
                                                                ;

                                                            SimpleOperations.CopyArray(array_kappa1_grad, this.global_kappa1_grad);
                                                            SimpleOperations.CopyArray(array_kappa3_grad, this.global_kappa3_grad);
                                                            SimpleOperations.CopyArray(array_scale, this.global_scale);
                                                            SimpleOperations.CopyArray(array_HorizontalError, this.global_HorizontalError);
                                                            SimpleOperations.CopyArray(array_HorizontalErrorFromStart, this.global_HorizontalErrorFromStart);
                                                            SimpleOperations.CopyArray(array_VerticalError, this.global_VerticalError);
                                                            for (int r = 0; r < array_VerticalError.Length; r++)
                                                                array_VerticalError[r] = Math.Abs(array_VerticalError[r]);
                                                            SimpleOperations.CopyArray(array_V_Up, this.global_V_Up);

                                                            string str_out = "";
                                                            str_out += i
                                                                //+ " NoisModl=" + global_NoiseModelFlag
                                                                //+ " eqlzVert=" + global_flag_equalizeVertNoise
                                                                //+ " MyCorr=" + global_MyOwnKalman_Korrection
                                                                //+ " CoordNois=" + global_CoordinateNoiseExist
                                                                + " OdoCntZ=" + global_OdoLimitMeasuresNum
                                                                + " OdoQz=" + global_odo_measure_noise
                                                                + " OdoQzV=" + global_odo_measure_noise_Vertical
                                                                + " Class=" + global_flag_AccuracyClass
                                                                + " ClassVert=" + global_flag_AccuracyClass_vert
                                                                ;
                                                            if (global_NoiseModelFlag == 0)
                                                                str_out += " Noise=NO";
                                                            else
                                                                str_out += " Noise=" + this.Cicle_Noise_Angular;

                                                            if (global_NoiseModelFlag_vert == 0)
                                                                str_out += " NoiseV=NO";
                                                            else
                                                                str_out += " NoiseV=" + this.Cicle_Noise_Angular_Vert;

                                                            if (this.global_indx > 1)
                                                                str_out += " " + Math.Round(array_HorizontalError.Average(), 3) + " " + Math.Round(array_HorizontalError.Max(), 3) + " " + Math.Round(array_HorizontalError[this.global_indx - 2], 3)
                                                                    + " " + Math.Round(array_HorizontalErrorFromStart[this.global_indx - 2], 3)
                                                                    + " " + Math.Round(array_VerticalError.Average(), 3) + " " + Math.Round(array_VerticalError.Max(), 3) + " " + Math.Round(this.global_VerticalError[this.global_indx - 2], 3)
                                                                    + " " + Math.Round(array_V_Up.Average(), 3) + " " + Math.Round(array_V_Up.Max() - array_V_Up.Min(), 3) + " " + Math.Round(array_V_Up[this.global_indx - 2], 3)
                                                                    + " " + Math.Round(array_kappa1_grad.Average(), 5) + " " + Math.Round(array_kappa1_grad.Max() - array_kappa1_grad.Min(), 5) + " " + Math.Round(array_kappa1_grad[this.global_indx - 2], 5)
                                                                    + " " + Math.Round(array_kappa3_grad.Average(), 5) + " " + Math.Round(array_kappa3_grad.Max() - array_kappa3_grad.Min(), 5) + " " + Math.Round(array_kappa3_grad[this.global_indx - 2], 5)
                                                                    + " " + Math.Round(array_scale.Average(), 5) + " " + Math.Round(array_scale.Max() - array_scale.Min(), 5) + " " + Math.Round(array_scale[this.global_indx - 2], 5)
                                                                    ;

                                                            Cycle_Start_Configurations.WriteLine(str_out);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            Cycle_Start_Configurations.Close();
            this.Close();
        }






        public void Single_Navigation_Processing()
        {
            int l = 0;

            this.DefineDimentionOfErrorVector();                                                            //---формирование размерности вектора ошибок---//
            this.DefineClassElementAndFlags();
            this.SelectDataIn();                                                                            //---выбор входного набора данных---//

            //StreamWriter ForHelp_2 = new StreamWriter(SimpleData.PathOutputString + "Debaging//ForHelp_2.txt");

            //=== Вычисляем LastCountForRead ===
            SINSstate.LastCountForRead = -20;
            int maxStrLength = 0;
            for (; ; )
            {
                string tmpstr = myFile.ReadLine();
                if (myFile.EndOfStream) break;
                SINSstate.LastCountForRead++;
                maxStrLength = Math.Max(maxStrLength, tmpstr.Length);

                //рамки, чтобы можно было сверяться
                if (SINSstate.Global_file == "Saratov_run_2014_07_23")
                {
                    string[] dataArray = tmpstr.Split(' ');
                    double time = Convert.ToDouble(dataArray[0]);

                    if (time > 5145)
                        break;
                }

                //ForHelp_2.WriteLine(tmpstr);
                //if (SINSstate.LastCountForRead > 300000)
                //        break;
            }
            int LastCountForRead = SINSstate.LastCountForRead;
            myFile.Close();
            //ForHelp_2.Close();

            this.SelectDataIn();
            //=== ===


            //---Инициализация начальных условий при отсутствии выставки---//
            Parameters.StartSINS_Parameters(SINSstate, SINSstate_OdoMod, KalmanVars, ParamStart, ProcHelp);


            // === Перебор вариантов стартовых настроек (без подбора шумов) === //
            if (this.StartParamScanning == true)
            {
                SINSstate.global_paramsCycleScanning_Path = "CycleParamScanning//";
                SINSstate.global_paramsCycleScanning =
                    //+ ";dQMd=" 
                    this.global_NoiseModelFlag.ToString()
                    //+ ";eqlzUp=" 
                    + this.global_flag_equalizeVertNoise.ToString()
                    //+ ";Cor=" 
                    + this.global_MyOwnKalman_Korrection.ToString()
                    //+ ";RdQ=" 
                    + this.global_CoordinateNoiseExist.ToString()
                    + this.global_OdoLimitMeasuresNum.ToString()
                    + ";odoQz=" + this.global_odo_measure_noise.ToString()
                    + ";odoQzV=" + this.global_odo_measure_noise_Vertical.ToString()
                    + ";cls=" + this.global_flag_AccuracyClass.ToString()
                    + ";clsV=" + this.global_flag_AccuracyClass_vert.ToString()
                    ;
                if (global_NoiseModelFlag == 0)
                    SINSstate.global_paramsCycleScanning += ";Noise=NO";
                else
                    SINSstate.global_paramsCycleScanning += ";Noise=" + this.Cicle_Noise_Angular;

                if (global_NoiseModelFlag_vert == 0)
                    SINSstate.global_paramsCycleScanning += ";NoiseV=NO";
                else
                    SINSstate.global_paramsCycleScanning += ";NoiseV=" + this.Cicle_Noise_Angular_Vert;

                SINSstate.global_paramsCycleScanning += "_";

                if (global_MyOwnKalman_Korrection == 0) SINSstate.MyOwnKalman_Korrection = false;
                else SINSstate.MyOwnKalman_Korrection = true;

                if (global_NoiseModelFlag == 0)
                    ParamStart.Experiment_NoiseModelFlag = ParamStart.Imitator_NoiseModelFlag = false;
                else
                {
                    ParamStart.Experiment_NoiseModelFlag = ParamStart.Imitator_NoiseModelFlag = true;

                    ParamStart.Experiment_Noise_Vel = ParamStart.Imitator_Noise_Vel = this.Cicle_Noise_Velocity;
                    ParamStart.Experiment_Noise_Angl = ParamStart.Imitator_Noise_Angl = this.Cicle_Noise_Angular;
                }

                if (global_NoiseModelFlag_vert == 0)
                    ParamStart.Experiment_NoiseModelFlag_Vert = false;
                else
                {
                    ParamStart.Experiment_NoiseModelFlag_Vert = true;
                    ParamStart.Experiment_Noise_Vel_vert = this.Cicle_Noise_Velocity_Vert;
                    ParamStart.Experiment_Noise_Angl_vert = this.Cicle_Noise_Angular_Vert;
                }

                if (global_CoordinateNoiseExist == 0)
                    this.iMqDeltaR.Checked = this.iMqDeltaRodo.Checked = SINSstate.flag_iMqDeltaR = SINSstate.flag_iMqDeltaRodo = false;
                else
                    this.iMqDeltaR.Checked = this.iMqDeltaRodo.Checked = SINSstate.flag_iMqDeltaR = SINSstate.flag_iMqDeltaRodo = true;

                if (this.global_OdoLimitMeasuresNum != 0)
                    SINSstate.OdoLimitMeasuresNum = this.global_OdoLimitMeasuresNum;

                if (this.global_odo_measure_noise != 0)
                    SINSstate.global_odo_measure_noise = this.global_odo_measure_noise;
                if (this.global_odo_measure_noise_Vertical != 0)
                    SINSstate.global_odo_measure_noise_Vertical = this.global_odo_measure_noise_Vertical;

                // --- Уровень начальных ковариаций ---//
                if (this.global_flag_AccuracyClass == 0.02) { SINSstate.flag_AccuracyClass_0_02grph = true; SINSstate.flag_AccuracyClass_0_2_grph = false; SINSstate.flag_AccuracyClass_2_0_grph = false; }
                if (this.global_flag_AccuracyClass == 0.2) { SINSstate.flag_AccuracyClass_0_02grph = false; SINSstate.flag_AccuracyClass_0_2_grph = true; SINSstate.flag_AccuracyClass_2_0_grph = false; }
                if (this.global_flag_AccuracyClass == 2.0) { SINSstate.flag_AccuracyClass_0_02grph = false; SINSstate.flag_AccuracyClass_0_2_grph = false; SINSstate.flag_AccuracyClass_2_0_grph = true; }

                SINSstate.global_flag_AccuracyClass_vert = this.global_flag_AccuracyClass_vert;

                SINSstate.flag_equalizeVertNoise = false;
                if (this.global_flag_equalizeVertNoise == 1)
                    SINSstate.flag_equalizeVertNoise = true;
            }
            // === === === === === === === === === === === === === === === === //



            SINS_State SINSstate2;
            //------Выставка------
            if (SINSstate.Global_file.ToLower().Contains("imitator"))
            {
                ImitatorHeaderReadAndApply();
                SINSstate2 = SINS_State.DeepCopy(SINSstate);
            }
            else
            {
                SINSstate2 = SINS_State.DeepCopy(SINSstate);

                if (SINSstate.flag_UsingNavAlignment == true && ProcHelp.AlignmentCounts != 0)
                    l = SINSAlignment_Navigantion.SINS_Alignment_Navigation(ProcHelp, SINSstate, SINSstate2, SINSstate_OdoMod, myFile, KalmanVars, GRTV_output);
                else if (SINSstate.flag_UsingClasAlignment == true && ProcHelp.AlignmentCounts != 0)
                    l = SINSAlignment_Classical.SINS_Alignment_Classical(ProcHelp, SINSstate, SINSstate2, SINSstate_OdoMod, myFile, KalmanVars, GRTV_output);


                //--- stdF и stdNu значения, определяющие классы точности датчиков. На основе них формируется стартовые ковариации инструментальных погрешностей инерц.датчиков, а также угловых ошибок ---
                if (SINSstate.flag_AccuracyClass_0_0grph)
                    for (int j = 0; j < 3; j++)
                    {
                        SINSstate.stdF[j] = 0.0 * 9.81; //далее умножается G
                        SINSstate.stdNu = 0.0; //град/час
                    }
                if (SINSstate.flag_AccuracyClass_0_02grph)
                    for (int j = 0; j < 3; j++)
                    {
                        SINSstate.stdF[j] = 1E-5 * 9.81; //далее умножается G
                        SINSstate.stdNu = 0.01; //град/час
                    }
                if (SINSstate.flag_AccuracyClass_0_2_grph)
                    for (int j = 0; j < 3; j++)
                    {
                        SINSstate.stdF[j] = 1E-4 * 9.81; //далее умножается G
                        SINSstate.stdNu = 0.1; //град/час
                    }
                if (SINSstate.flag_AccuracyClass_2_0_grph)
                    for (int j = 0; j < 3; j++)
                    {
                        SINSstate.stdF[j] = 1E-3 * 9.81; //далее умножается G
                        SINSstate.stdNu = 1.0; //град/час
                    }

                if (ParamStart.Experiment_NoiseModelFlag == true)
                {
                    KalmanVars.Noise_Vel[0] = ParamStart.Experiment_Noise_Vel;
                    KalmanVars.Noise_Angl[0] = ParamStart.Experiment_Noise_Angl;
                    KalmanVars.Noise_Vel[1] = ParamStart.Experiment_Noise_Vel;
                    KalmanVars.Noise_Angl[1] = ParamStart.Experiment_Noise_Angl;
                }
                if (ParamStart.Experiment_NoiseModelFlag_Vert)
                {
                    KalmanVars.Noise_Vel[2] = ParamStart.Experiment_Noise_Vel_vert;
                    KalmanVars.Noise_Angl[2] = ParamStart.Experiment_Noise_Angl_vert;
                }

                //---Для экспериментальных данных SINSstate.stdF_Oz будут равны SINSstate.stdF, т.к. далее используются в нач.ков. матрице
                for (int j = 0; j < 3; j++)
                {
                    SINSstate.stdF_Oz[j] = SINSstate.stdF[j];
                    SINSstate.stdNu_Oz[j] = SINSstate.stdNu;
                }

                SINSstate.stdR = ParamStart.Experiment_stdR;
                SINSstate.stdOdoR = ParamStart.Experiment_stdOdoR; // = 1.0; // метров
                SINSstate.stdV = ParamStart.Experiment_stdV;
                SINSstate.stdAlpha1 = -SINSstate.stdF[1] / 9.81; //радиан
                SINSstate.stdAlpha2 = SINSstate.stdF[0] / 9.81; //радиан
                SINSstate.stdBeta3 = SINSstate.stdNu * SimpleData.ToRadian / 3600.0 / (SimpleData.U * Math.Cos(SINSstate.Latitude)); //радиан
                SINSstate.stdScale = ParamStart.Experiment_stdScale;
                SINSstate.stdKappa1 = ParamStart.Experiment_stdKappa1; //минут
                SINSstate.stdKappa3 = ParamStart.Experiment_stdKappa3; //минут
            }



            if (SINSstate.Global_file.ToLower().Contains("imitator"))
                SINSstate.Noise_GPS_PositionError = ParamStart.Imitator_GPS_PositionError;
            else
                SINSstate.Noise_GPS_PositionError = ParamStart.Experiment_GPS_PositionError;

            SINSstate.Noise_Marker_PositionError = ParamStart.Experiment_Marker_PositionError;


            //---Инициализация начальной матрицы ковариации---
            if (SINSstate.flag_Odometr_SINS_case == true)
                Odometr_SINS.InitOfCovarianceMatrixes(SINSstate, KalmanVars);
            else
                SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);



            //---Переопределяем размерности векторов и матриц после выставки---
            this.DefineDimentionOfErrorVector();

            SINSstate2 = SINS_State.DeepCopy(SINSstate);

            DateTime start = DateTime.Now;
            if (SINSstate.flag_OnlyAlignment == false)
            {
                SINSstate2 = SINS_State.DeepCopy(SINSstate);
                ////------БИНС + ОДОМЕТР------
                if (OnlyIntegrating.Checked == false && SINSstate.flag_OnlyAlignment == false || SINSstate.flag_Odometr_SINS_case == true)
                    SINS_Corrected.SINS_Corrected_Processing(l, false, myFile, SINSstate, SINSstate2, KalmanVars, ProcHelp, SINSstate_OdoMod, GRTV_output);
                //------Автономное решение-----
                else if (SINSstate.flag_OnlyAlignment == false)
                    SINS_Autonomous.SINS_Autonomous_Processing(l, myFile, SINSstate, SINSstate2, KalmanVars, ProcHelp, SINSstate_OdoMod, GRTV_output);

                if (SINSstate.flag_Smoothing)
                    SINS_Corrected.SINS_Corrected_Processing(l, true, myFile, SINSstate, SINSstate2, KalmanVars, ProcHelp, SINSstate_OdoMod, GRTV_output);
            }


            DateTime end = DateTime.Now;
            Console.Write(" Processing time: " + (end - start).ToString());
            Console.Write(" "); Console.Write(" ");

            if (this.StartParamScanning)
            {
                this.global_indx = SINSstate.global_indx;
                SimpleOperations.CopyArray(this.global_kappa1_grad, SINSstate.global_kappa1_grad);
                SimpleOperations.CopyArray(this.global_kappa3_grad, SINSstate.global_kappa3_grad);
                SimpleOperations.CopyArray(this.global_scale, SINSstate.global_scale);
                SimpleOperations.CopyArray(this.global_HorizontalError, SINSstate.global_HorizontalError);
                SimpleOperations.CopyArray(this.global_HorizontalErrorFromStart, SINSstate.global_HorizontalErrorFromStart);
                SimpleOperations.CopyArray(this.global_VerticalError, SINSstate.global_VerticalError);
                SimpleOperations.CopyArray(this.global_V_Up, SINSstate.global_V_Up);
            }

            myFile.Close();
        }

        //---------------------------------------------------------------------------------------
        //---------------------------------------------------------------------------------------
        //---------------------------------------------------------------------------------------











        public void ImitatorHeaderReadAndApply()
        {
            string[] dataArray;
            ProcHelp.datastring = myFile.ReadLine();
            dataArray = ProcHelp.datastring.Split(' ');

            SINSstate.timeStep = SINSstate.Freq = 1.0 / Convert.ToDouble(dataArray[7]);
            SINSstate.odo_min_increment = Convert.ToDouble(dataArray[25]) / 100.0;               /*Поставил *10.0, чтобы шум был заведомо больше - так лучше оценивается*/
            if (SINSstate.odo_min_increment < 0.0001)
                SINSstate.odo_min_increment = 0.01;

            //---Здесь SINSstate.stdF и SINSstate.stNu считываются в проекции на географию и используются только для определения начальной ошибки по углам
            SINSstate.stdF[0] = Convert.ToDouble(dataArray[9]) * 9.81;
            SINSstate.stdF[1] = Convert.ToDouble(dataArray[11]) * 9.81;
            SINSstate.stdNu = Convert.ToDouble(dataArray[15]);

            SINSstate.stdF_Oz[0] = Convert.ToDouble(dataArray[39]) * 9.81;
            SINSstate.stdF_Oz[1] = Convert.ToDouble(dataArray[41]) * 9.81;
            SINSstate.stdF_Oz[2] = Convert.ToDouble(dataArray[43]) * 9.81;
            SINSstate.stdNu_Oz[0] = Convert.ToDouble(dataArray[45]);
            SINSstate.stdNu_Oz[1] = Convert.ToDouble(dataArray[47]);
            SINSstate.stdNu_Oz[2] = Convert.ToDouble(dataArray[49]);

            for (int j = 0; j < 3; j++)
            {
                if (ParamStart.Imitator_NoiseModelFlag == true)
                {
                    KalmanVars.Noise_Vel[j] = ParamStart.Imitator_Noise_Vel;
                    KalmanVars.Noise_Angl[j] = ParamStart.Imitator_Noise_Angl;
                }
                else
                {
                    KalmanVars.Noise_Vel[j] = 1.0 / 3.0 / Convert.ToDouble(dataArray[13]);
                    KalmanVars.Noise_Angl[j] = 1.0 / 3.0 / Convert.ToDouble(dataArray[17]);
                }
            }

            double tmpFaktor = 2.0;
            KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / Convert.ToDouble(dataArray[27]) / tmpFaktor;
            KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
            KalmanVars.OdoNoise_STOP = 0.01;

            ParamStart.Imitator_GPS_PositionError = Convert.ToDouble(dataArray[37]);


            KalmanVars.Noise_OdoScale = ParamStart.Imitator_Noise_OdoScale;
            KalmanVars.Noise_OdoKappa_1 = ParamStart.Imitator_Noise_OdoKappa;
            KalmanVars.Noise_OdoKappa_3 = ParamStart.Imitator_Noise_OdoKappa;
            KalmanVars.Noise_Pos = ParamStart.Imitator_Noise_Pos;
            KalmanVars.Noise_Drift = ParamStart.Imitator_Noise_Drift;
            KalmanVars.Noise_Accel = ParamStart.Imitator_Noise_Accel;

            ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = Convert.ToDouble(dataArray[3]);
            ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = Convert.ToDouble(dataArray[1]);
            ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Height_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Height_prev = Convert.ToDouble(dataArray[5]);
            ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
            ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

            //Углы найденные подбором минимизацией максимальной ошибки по позиции.
            double Heading_tmpDevide = 1.0;

            double Heading_addError = SINSstate.stdNu * SimpleData.ToRadian / 3600.0 / (SimpleData.U * Math.Cos(SINSstate.Latitude)),
                Pitch_addError = (SINSstate.stdF[1] / 9.81 * Math.Cos(SINSstate.Heading) + SINSstate.stdF[0] / 9.81 * Math.Sin(SINSstate.Heading)),
                Roll_addError = -(-SINSstate.stdF[1] / 9.81 * Math.Sin(SINSstate.Heading) + SINSstate.stdF[0] / 9.81 * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch)
                ;
            SINSstate.Heading = Convert.ToDouble(dataArray[29]) + Heading_addError ;
            SINSstate.Pitch = Convert.ToDouble(dataArray[33]) + Pitch_addError ;
            SINSstate.Roll = Convert.ToDouble(dataArray[31]) + Roll_addError ;

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

            SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Height);

            SINSstate_OdoMod.A_sx0 = SimpleOperations.A_sx0(SINSstate_OdoMod);
            SINSstate_OdoMod.A_x0s = SINSstate_OdoMod.A_sx0.Transpose();
            SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Longitude);
            SINSstate_OdoMod.A_nx0 = SINSstate_OdoMod.A_x0n.Transpose();
            SINSstate_OdoMod.A_nxi = SimpleOperations.A_ne(SINSstate_OdoMod.Time, SINSstate_OdoMod.Longitude_Start);
            SINSstate_OdoMod.AT = Matrix.Multiply(SINSstate_OdoMod.A_sx0, SINSstate_OdoMod.A_x0n);
            SINSstate_OdoMod.AT = Matrix.Multiply(SINSstate_OdoMod.AT, SINSstate_OdoMod.A_nxi);

            SINSstate_OdoMod.R_e = SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height);
            SINSstate_OdoMod.R_n = SimpleOperations.RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height);


            //------------------------------------------------------
            SINSstate.stdR = ParamStart.Imitator_stdR; // метров
            SINSstate.stdOdoR = ParamStart.Imitator_stdOdoR; // метров
            SINSstate.stdV = ParamStart.Imitator_stdV; // м/с
            SINSstate.stdAlpha1 = -SINSstate.stdF[1] / 9.81 / Heading_tmpDevide; //радиан
            SINSstate.stdAlpha2 = SINSstate.stdF[0] / 9.81 / Heading_tmpDevide; //радиан
            SINSstate.stdBeta3 = SINSstate.stdNu * SimpleData.ToRadian / 3600.0 / (SimpleData.U * Math.Cos(SINSstate.Latitude)) / Heading_tmpDevide; //радиан
            SINSstate.stdScale = ParamStart.Imitator_stdScale; //коэффициент в долях
            SINSstate.stdKappa1 = ParamStart.Imitator_stdKappa1; //минут
            SINSstate.stdKappa3 = ParamStart.Imitator_stdKappa3; //минут
        }







        public void DefineDimentionOfErrorVector()
        {
            iMx = SimpleData.iMx = 13;
            iMq = SimpleData.iMq = SimpleData.iMx;

            iMx_r3_dV3 = iMx_r_3_dV_3.Checked;
            iMx_kappa_13_ds = iMx_kappa_1_3_ds.Checked;

            iMx = SimpleData.iMx = 0;

            // ---------- dR ----------//
            iMx = SimpleData.iMx += 2;


            // ---------- dR_ODO_12 ----------//
            if (this.Odometr_SINS_case.Checked)
            {
                value_iMx_r_odo_12 = SimpleData.iMx;
                iMx = SimpleData.iMx += 2;
            }

            // ---------- dV_12 ----------//
            value_iMx_dV_12 = SimpleData.iMx;
            iMx = SimpleData.iMx += 2;


            // ---------- alphaBeta ----------//
            value_iMx_alphaBeta = SimpleData.iMx;
            iMx = SimpleData.iMx += 3;

            // ---------- Nu0 ----------//
            value_iMx_Nu0 = SimpleData.iMx;
            iMx = SimpleData.iMx += 3;

            // ---------- f0_12 ----------//
            value_iMx_f0_12 = SimpleData.iMx;
            iMx = SimpleData.iMx += 2;

            // ---------- kappa_13_ds ----------//
            if (iMx_kappa_13_ds)
            {
                if (this.SeparateHorizVSVertical.Checked == false)
                {
                    value_iMx_kappa_1 = SimpleData.iMx;
                    iMx = SimpleData.iMx += 1;
                }

                value_iMx_kappa_3_ds = SimpleData.iMx;
                iMx = SimpleData.iMx += 2;
            }


            // ---------- dR_3 ----------//
            if (iMx_r3_dV3)
            {
                value_iMx_dr3 = SimpleData.iMx;
                iMx = SimpleData.iMx += 1;
            }

            // ---------- dV_3 ----------//
            if (iMx_r3_dV3)
            {
                value_iMx_dV3 = SimpleData.iMx;
                iMx = SimpleData.iMx += 1;
            }

            // ---------- f0_3 ----------//
            //if (this.SeparateHorizVSVertical.Checked == false)
            {
                value_iMx_f0_3 = SimpleData.iMx;
                iMx = SimpleData.iMx += 1;
            }

            // ---------- dR_ODO_3 ----------//
            if (this.Odometr_SINS_case.Checked)
            {
                if (this.iMx_r_3_dV_3.Checked)
                {
                    value_iMx_r_odo_3 = SimpleData.iMx;
                    iMx = SimpleData.iMx += 1;
                }
            }



            //----------------------------Размерность Вектора шумов---------------------------
            iMq = SimpleData.iMq = SimpleData.iMx;


            //----------------------------Индексы для сглаживания---------------------------
            if (this.iMSmthd_Is_2.Checked)
                iMxSmthd = SimpleData.iMxSmthd = 2;
            if (this.iMSmthd_Is_4.Checked)
                iMxSmthd = SimpleData.iMxSmthd = 4;
            if (this.iMSmthd_Is_7.Checked)
                iMxSmthd = SimpleData.iMxSmthd = 7;







            // ------------------------------------------//
            if (this.SeparateHorizVSVertical.Checked)
            {
                SimpleData.iMx_Vertical = 2;

                // --- Вертикальный одометрический канал
                if (this.Odometr_SINS_case.Checked)
                {
                    Vertical_rOdo3 = SimpleData.iMx_Vertical;
                    SimpleData.iMx_Vertical++;
                }

                // --- Добавляем ньютонометры ---
                if (true)
                {
                    if (false)
                    {
                        Vertical_f0_12 = SimpleData.iMx_Vertical;
                        SimpleData.iMx_Vertical += 2;
                    }
                    // --- Добавляем вертикальный 0 ньютонометра ---
                    Vertical_f0_3 = SimpleData.iMx_Vertical;
                    SimpleData.iMx_Vertical++;
                }

                // --- Если включаем ошибки одометра в вектор
                if (true)
                {
                    Vertical_kappa1 = SimpleData.iMx_Vertical;
                    SimpleData.iMx_Vertical += 1;

                    if (false)
                    {
                        Vertical_kappa3Scale = SimpleData.iMx_Vertical;
                        SimpleData.iMx_Vertical += 2;
                    }
                }

                // ---------------------------//
                SimpleData.iMq_Vertical = SimpleData.iMx_Vertical;
                // ---------------------------//
            }

        }

        public void DefineClassElementAndFlags()
        {
            SINSstate = new SINS_State(); SINSstate_OdoMod = new SINS_State();
            KalmanVars = new Kalman_Vars();
            ProcHelp = new Proc_Help();

            SINSstate.FreqOutput = Convert.ToInt32(this.Output_Freq.Text);

            SINSstate.flag_iMx_r3_dV3 = this.iMx_r_3_dV_3.Checked;
            SINSstate.flag_iMx_kappa_13_ds = this.iMx_kappa_1_3_ds.Checked;

            SINSstate.flag_iMqDeltaR = iMqDeltaR.Checked;
            SINSstate.flag_iMqDeltaF = iMqDeltaF.Checked;
            SINSstate.flag_iMqDeltaNu = iMqDeltaNu.Checked;
            SINSstate.flag_iMqVarkappa1 = iMqVarkappa1.Checked;
            SINSstate.flag_iMqVarkappa3 = iMqVarkappa3.Checked;
            SINSstate.flag_iMqKappa = iMqKappa.Checked;
            SINSstate.flag_iMqDeltaRodo = iMqDeltaRodo.Checked;
            SINSstate.flag_Imitator_Telemetric = Imitator_Telemetric.Checked;


            SINSstate.value_iMx_dV_12 = value_iMx_dV_12;
            SINSstate.value_iMx_alphaBeta = value_iMx_alphaBeta;
            SINSstate.value_iMx_Nu0 = value_iMx_Nu0;
            SINSstate.value_iMx_f0_12 = value_iMx_f0_12;
            SINSstate.value_iMx_f0_3 = value_iMx_f0_3;
            SINSstate.value_iMx_dr3 = value_iMx_dr3;
            SINSstate.value_iMx_dV3 = value_iMx_dV3;
            SINSstate.value_iMx_r_odo_3 = value_iMx_r_odo_3;
            SINSstate.value_iMx_r_odo_12 = value_iMx_r_odo_12;

            SINSstate.value_iMx_kappa_1 = value_iMx_kappa_1;
            SINSstate.value_iMx_kappa_3_ds = value_iMx_kappa_3_ds;


            //---флаги---
            SINSstate.flag_Autonomous_Solution = this.OnlyIntegrating.Checked;
            SINSstate.flag_UsingAvegering = this.UsingAveraging.Checked;
            SINSstate.flag_Using_SNS = this.usingSNS.Checked;
            SINSstate.flag_FeedbackExist = this.feedbackExist.Checked;
            SINSstate.flag_EstimateExist = this.EstimateExist.Checked;
            SINSstate.flag_UsingClasAlignment = this.UsingClasAlignment.Checked;
            SINSstate.flag_UsingNavAlignment = this.UsingNavAlignment.Checked;
            SINSstate.flag_Odometr_SINS_case = this.Odometr_SINS_case.Checked;
            SINSstate.flag_OdoSINSWeakConnect = this.WeakConnect.Checked;
            SINSstate.flag_OdoSINSWeakConnect_MODIF = this.ModifWeakConnect.Checked;
            SINSstate.flag_UseOnlyStops = this.Use_Only_Stops.Checked;
            SINSstate.flag_OnlyAlignment = this.OnlyAlignment.Checked;
            SINSstate.flag_first100m_StartHeightCorrection = this.flag_first100m_StartHeightCorrection.Checked;

            SINSstate.flag_Smoothing = this.flag_Smoothing.Checked;
            SINSstate.flag_iMSmthd_Is_2_plus_Odo = this.iMSmthd_Is_2_plus_Odo.Checked;

            SINSstate.flag_NotUse_ZUPT = this.flag_not_use_zupt.Checked;
            SINSstate.flag_using_Checkpotints = this.flag_using_Checkpotints.Checked;
            SINSstate.add_velocity_to_position = this.add_velocity_to_position.Checked;
            SINSstate.flag_UseAlgebraDrift = this.flag_UseAlgebraDrift.Checked;
            SINSstate.flag_VupOdo_till_VupSINS = this.flag_VupOdo_till_VupSINS.Checked;

            SINSstate.flag_AccuracyClass_NoErr = this.AccuracyClass_NoErr.Checked;
            SINSstate.flag_AccuracyClass_0_0grph = this.AccuracyClass_0_0grph.Checked;
            SINSstate.flag_AccuracyClass_0_02grph = this.AccuracyClass_0_02grph.Checked;
            SINSstate.flag_AccuracyClass_0_2_grph = this.AccuracyClass_0_2_grph.Checked;
            SINSstate.flag_AccuracyClass_2_0_grph = this.AccuracyClass_2_0_grph.Checked;
            SINSstate.flag_AccuracyClass_Custom = this.AccuracyClass_Custom.Checked;

            //---флаги коррекции---
            SINSstate.flag_UsingOdoVelocity = this.flag_UsingOdoVelocity.Checked;
            SINSstate.flag_onlyZeroSideVelocity = this.flag_onlyZeroSideVelocity.Checked;
            SINSstate.flag_UsingOdoPosition = this.flag_UsingOdoPosition.Checked;

            SINSstate.flag_GRTV_output = this.flag_GRTV_output.Checked;


            SINSstate.flag_SeparateHorizVSVertical = this.SeparateHorizVSVertical.Checked;

            // ------------------------------------------//
            if (this.SeparateHorizVSVertical.Checked)
            {
                SINSstate.Vertical_kappa1 = Vertical_kappa1;
                SINSstate.Vertical_kappa3Scale = Vertical_kappa3Scale;
                SINSstate.Vertical_f0_12 = Vertical_f0_12;
                SINSstate.Vertical_f0_3 = Vertical_f0_3;
                SINSstate.Vertical_rOdo3 = Vertical_rOdo3;
            }
        }

        public void SelectDataIn()
        {
            string tt9 = Regex.Replace(Application.StartupPath.ToString(), "(\\\\bin|\\\\Debug)", String.Empty);

            if (Azimut_15_08_2012.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "AzimutB_210530_Other_120815_Autolab_DPC_100Hz_14-40-04.dat");
                SINSstate.Global_file = "Azimut_15.08.2012";
            }
            else if (Azimut_24_08_2012.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "AzimutB_210530_Other_120824_Autolab_Circle_AzimutKama_12-07-25.dat");
                SINSstate.Global_file = "Azimut_24.08.2012";
            }
            else if (Azimut_29_08_2012.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "AzimutB_210530_Other_120829_Autolab_Circle_09-21-35.dat");
                SINSstate.Global_file = "Azimut_29.08.2012";
            }
            else if (ktn004_15_03_2012.Checked == true)
            {
                SINSstate.Global_file = "ktn004_15.03.2012";
                if (this.Imitator_Telemetric.Checked == true)
                {
                    this.GlobalPrefixTelemetric = SimpleData.PathInputString + "TelemetricData//Telemetric_Imitator_" + SINSstate.Global_file;
                    myFile = new StreamReader(this.GlobalPrefixTelemetric + "_Errors.dat");
                    SINSstate.Global_file = "Imitator_Telemetric";
                }
                else
                {
                    myFile = new StreamReader(SimpleData.PathInputString + "ktn004_static4hour_marsh_sns_15-Mar-2012,16-29-45_dat.dat");
                }
            }
            else if (Imitator_Data.Checked == true)
            {
                SINSstate.Global_file = "Imitator_Data";
                if (this.Imitator_Telemetric.Checked == true)
                {
                    this.GlobalPrefixTelemetric = SimpleData.PathInputString + "TelemetricData//Telemetric_Imitator_" + SINSstate.Global_file;
                    myFile = new StreamReader(this.GlobalPrefixTelemetric + "_Errors.dat");
                    SINSstate.Global_file = "Imitator_Telemetric";
                }
                else
                {
                    myFile = new StreamReader(SimpleData.PathInputString + "Imitator_Analytic_Errors.dat");
                }
            }



            //МИНСКИЕ ЗАЕЗДЫ
            if (Azimuth_minsk_race_4_3to6to2.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "120222_AzimutB_210530_Race_4_Control_3-6-2_11-49-20_dat.dat");
                SINSstate.Global_file = "Azimuth_minsk_race_4_3to6to2";
            }

            if (AZIMUT_T_12_32_16_09_13_TLM_2z.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "Azimut-T//Azimut-T_18-Oct-2013_12-56-43.txt");
                SINSstate.Global_file = "AZIMUT_T_2013_10_18_12_55";
            }
            if (Azimut_514_08Nov2013_11_15.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "Azimut-10B//Azimut_514_08Nov2013_11_15.txt");
                SINSstate.Global_file = "Azimut_514_08Nov2013_11_15";
            }
            if (GRTVout_GCEF_format_070715_zavod.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "GRTVout_GCEF_format (070715выезд завод).txt");
                SINSstate.Global_file = "GRTVout_GCEF_format (070715выезд завод)";
            }
            if (GRTVout_GCEF_format_070715_kulikova.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "GRTVout_GCEF_format (070715выезд куликовка).txt");
                SINSstate.Global_file = "GRTVout_GCEF_format (070715выезд куликовка)";
            }

            if (GRTV_Ekat_151029_1_zaezd.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "PNPPK_Ekat//GRTV_Ekat_151029_1_zaezd.txt");
                SINSstate.Global_file = "GRTV_Ekat_151029_1_zaezd";
            }
            if (GRTV_Ekat_151029_2_zaezd.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "PNPPK_Ekat//GRTV_Ekat_151029_2_zaezd.txt");
                SINSstate.Global_file = "GRTV_Ekat_151029_2_zaezd";
            }
            if (GRTV_ktn004_marsh16_afterbdnwin_20032012.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "PNPPK_Ekat//GRTV_ktn004_marsh16_afterbdnwin_20032012.txt");
                SINSstate.Global_file = "GRTV_ktn004_marsh16_afterbdnwin_20032012";
            }
            if (GRTV_ktn004_marsh16_repeat_21032012.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "PNPPK_Ekat//GRTV_ktn004_marsh16_repeat_21032012.txt");
                //myFile = new StreamReader("C:\\Users\\kuppaz\\Dropbox\\5#_AspaWork\\2015.09.02 - Введение, GRTV\\!от Колеватова АП\\Example_GRTV\\GRTV\\GRTV\\Output in TXT\\GRTVout_GCEF_format.txt");
                SINSstate.Global_file = "GRTV_ktn004_marsh16_repeat_21032012";
            }


            if (someOtherInput.Checked == true)
            {
                myFile = new StreamReader("D:\\NavLab\\GRTVout_GCEF_format_Azimuth10B_450612_48H_17-Feb-2016,17-31-23.dat.txt");
                SINSstate.Global_file = "someOtherInput";
            }


            if (topo_saratov.Checked == true)
            {
                if (SINSstate.FreqOutput == 10)
                    SINSstate.FreqOutput = 100;

                if (this.SaratovFullRun.Checked == false)
                    myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23.dat");
                else
                {
                    if (this.saratovWithVirtualMarksInside.Checked)
                        myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23_Virtual_full.dat");
                    else
                        myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23_full.dat");
                }

                SINSstate.Global_file = "Saratov_run_2014_07_23";
            }
        }












        //---ИНТЕРФЕЙС---//

        public void LockTypesOfCorrection()
        {
            this.flag_UsingOdoPosition.Enabled = false; this.flag_UsingOdoVelocity.Enabled = false; this.Use_Only_Stops.Enabled = false;
        }
        public void FreeTypesOfCorrection()
        {
            this.flag_UsingOdoPosition.Enabled = true; this.flag_UsingOdoVelocity.Enabled = true; this.Use_Only_Stops.Enabled = true;
        }

        public void LockInData()
        {
            this.Azimut_15_08_2012.Enabled = false; this.Azimut_24_08_2012.Enabled = false; this.Azimut_29_08_2012.Enabled = false;
            this.ktn004_15_03_2012.Enabled = false; this.Imitator_Data.Enabled = false; this.Azimuth_minsk_race_4_3to6to2.Enabled = false;
            this.topo_saratov.Enabled = false; this.AZIMUT_T_12_32_16_09_13_TLM_2z.Enabled = false; this.Azimut_514_08Nov2013_11_15.Enabled = false;
            this.GRTVout_GCEF_format_070715_zavod.Enabled = false;
            this.GRTVout_GCEF_format_070715_kulikova.Enabled = false;
            this.GRTV_Ekat_151029_1_zaezd.Enabled = false;
            this.GRTV_Ekat_151029_2_zaezd.Enabled = false;
            this.GRTV_ktn004_marsh16_afterbdnwin_20032012.Enabled = false;
            this.GRTV_ktn004_marsh16_repeat_21032012.Enabled = false;
        }
        public void FreeInData()
        {
            this.Azimut_15_08_2012.Enabled = true; this.Azimut_24_08_2012.Enabled = true; this.Azimut_29_08_2012.Enabled = true;
            this.ktn004_15_03_2012.Enabled = true; this.Imitator_Data.Enabled = true; this.Azimuth_minsk_race_4_3to6to2.Enabled = true;
            this.topo_saratov.Enabled = true; this.AZIMUT_T_12_32_16_09_13_TLM_2z.Enabled = true; this.Azimut_514_08Nov2013_11_15.Enabled = true;
            this.GRTVout_GCEF_format_070715_zavod.Enabled = true;
            this.GRTVout_GCEF_format_070715_kulikova.Enabled = true;
            this.GRTV_Ekat_151029_1_zaezd.Enabled = true;
            this.GRTV_Ekat_151029_2_zaezd.Enabled = true;
            this.GRTV_ktn004_marsh16_afterbdnwin_20032012.Enabled = true;
            this.GRTV_ktn004_marsh16_repeat_21032012.Enabled = true;
        }

        public void LockParamsOfStart()
        {
            this.OnlyAlignment.Enabled = false; this.UsingClasAlignment.Enabled = false; this.UsingAveraging.Enabled = false; this.UsingNavAlignment.Enabled = false;
        }
        public void FreeParamsOfStart()
        {
            this.OnlyAlignment.Enabled = true; this.UsingClasAlignment.Enabled = true; this.UsingAveraging.Enabled = true; this.UsingNavAlignment.Enabled = true;
        }

        public void LockTheWayOfStart()
        {
            this.OnlyIntegrating.Enabled = false; this.feedbackExist.Enabled = false; this.Odometr_SINS_case.Enabled = false; this.EstimateExist.Enabled = false;
        }
        public void FreeTheWayOfStart()
        {
            this.OnlyIntegrating.Enabled = true; this.feedbackExist.Enabled = true; this.Odometr_SINS_case.Enabled = true; this.EstimateExist.Enabled = true;
        }


        public void LockDimOfVector()
        {
            this.iMx_r_3_dV_3.Enabled = false; this.iMx_kappa_1_3_ds.Enabled = false;
        }
        public void FreeDimOfVector()
        {
            this.iMx_r_3_dV_3.Enabled = true; this.iMx_kappa_1_3_ds.Enabled = true;
        }


        public void UnCheckTheWayOfStart()
        {
            this.OnlyIntegrating.Checked = false; this.feedbackExist.Checked = false; this.Odometr_SINS_case.Checked = false; this.EstimateExist.Checked = false;
        }
        public void UnCheckTypesOfCorrection()
        {
            this.flag_UsingOdoPosition.Checked = false; this.flag_UsingOdoVelocity.Checked = false;
            this.Use_Only_Stops.Checked = false;
            this.usingSNS.Checked = false;
        }


        //---Входные наборы данных---//
        public void CheckedTrueDataIn()
        {
            FreeParamsOfStart();
            FreeTheWayOfStart();
            FreeDimOfVector();
            LockInData();
        }
        public void CheckedFalseDataIn()
        {
            UnCheckTheWayOfStart();
            UnCheckTypesOfCorrection();
            LockTypesOfCorrection();
            LockParamsOfStart();
            LockTheWayOfStart();
            LockDimOfVector();
            FreeInData();
            this.usingSNS.Enabled = false;
        }

        private void Imitator_Data_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Imitator_Data.Checked == true)
            {
                CheckedTrueDataIn();
                this.Imitator_Data.Enabled = true;

                this.AccuracyClass_NoErr.Enabled = false;
                this.AccuracyClass_0_0grph.Enabled = false;
                this.AccuracyClass_Custom.Enabled = false;
                this.AccuracyClass_0_02grph.Enabled = false;
                this.AccuracyClass_0_2_grph.Enabled = false;
                this.AccuracyClass_2_0_grph.Enabled = false;
            }
            else
            {
                CheckedFalseDataIn();

                this.AccuracyClass_0_0grph.Enabled = true;
                this.AccuracyClass_0_02grph.Enabled = true;
                this.AccuracyClass_0_2_grph.Enabled = true;
                this.AccuracyClass_2_0_grph.Enabled = true;
            }

            this.Imitator_Telemetric.Enabled = true;
        }
        private void Imitator_Telemetric_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Imitator_Telemetric.Checked == true)
            {
                this.AccuracyClass_NoErr.Enabled = true;
                this.AccuracyClass_0_0grph.Enabled = true;
                this.AccuracyClass_0_02grph.Enabled = true;
                this.AccuracyClass_0_2_grph.Enabled = true;
                this.AccuracyClass_2_0_grph.Enabled = true;
            }
            else
            {
                this.AccuracyClass_NoErr.Enabled = false;
                this.AccuracyClass_0_0grph.Enabled = false;
                this.AccuracyClass_0_02grph.Enabled = false;
                this.AccuracyClass_0_2_grph.Enabled = false;
                this.AccuracyClass_2_0_grph.Enabled = false;
            }
        }

        private void Azimut_15_08_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Azimut_15_08_2012.Checked == true)
            {
                CheckedTrueDataIn();
                this.Azimut_15_08_2012.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void Azimut_24_08_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Azimut_24_08_2012.Checked == true)
            {
                CheckedTrueDataIn();
                this.Azimut_24_08_2012.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void Azimut_29_08_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Azimut_29_08_2012.Checked == true)
            {
                CheckedTrueDataIn();
                this.Azimut_29_08_2012.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void ktn004_15_03_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.ktn004_15_03_2012.Checked == true)
            {
                CheckedTrueDataIn();
                this.ktn004_15_03_2012.Enabled = true;
            }
            else CheckedFalseDataIn();

            this.Imitator_Telemetric.Enabled = true;
        }

        private void Azimuth_minsk_race_4_3to6to2_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Azimuth_minsk_race_4_3to6to2.Checked == true)
            {
                CheckedTrueDataIn();
                this.Azimuth_minsk_race_4_3to6to2.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void _16_09_13_TLM_1zaezd_CheckedChanged(object sender, EventArgs e)
        {
            if (this.topo_saratov.Checked == true)
            {
                CheckedTrueDataIn();
                this.topo_saratov.Enabled = true;
            }
            else CheckedFalseDataIn();
        }
        private void AZIMUT_T_12_32_16_09_13_TLM_2z_CheckedChanged(object sender, EventArgs e)
        {
            if (this.AZIMUT_T_12_32_16_09_13_TLM_2z.Checked == true)
            {
                CheckedTrueDataIn();
                this.AZIMUT_T_12_32_16_09_13_TLM_2z.Enabled = true;
            }
            else CheckedFalseDataIn();
        }
        private void Azimut_514_08Nov2013_11_15_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Azimut_514_08Nov2013_11_15.Checked == true)
            {
                CheckedTrueDataIn();
                this.Azimut_514_08Nov2013_11_15.Enabled = true;
            }
            else CheckedFalseDataIn();
        }
        private void GRTVout_GCEF_format_070715_zavod_CheckedChanged(object sender, EventArgs e)
        {
            if (this.GRTVout_GCEF_format_070715_zavod.Checked == true)
            {
                CheckedTrueDataIn();
                this.GRTVout_GCEF_format_070715_zavod.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void GRTVout_GCEF_format_070715_kulikova_CheckedChanged(object sender, EventArgs e)
        {
            if (this.GRTVout_GCEF_format_070715_kulikova.Checked == true)
            {
                CheckedTrueDataIn();
                this.GRTVout_GCEF_format_070715_kulikova.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void GRTV_Ekat_151029_1_zaezd_CheckedChanged(object sender, EventArgs e)
        {
            if (this.GRTV_Ekat_151029_1_zaezd.Checked == true)
            {
                CheckedTrueDataIn();
                this.GRTV_Ekat_151029_1_zaezd.Enabled = true;
            }
            else CheckedFalseDataIn();
        }
        private void GRTV_Ekat_151029_2_zaezd_CheckedChanged(object sender, EventArgs e)
        {
            if (this.GRTV_Ekat_151029_2_zaezd.Checked == true)
            {
                CheckedTrueDataIn();
                this.GRTV_Ekat_151029_2_zaezd.Enabled = true;
            }
            else CheckedFalseDataIn();
        }
        private void GRTV_ktn004_marsh16_afterbdnwin_20032012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.GRTV_ktn004_marsh16_afterbdnwin_20032012.Checked == true)
            {
                CheckedTrueDataIn();
                this.GRTV_ktn004_marsh16_afterbdnwin_20032012.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void GRTV_ktn004_marsh16_repeat_21032012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.GRTV_ktn004_marsh16_repeat_21032012.Checked == true)
            {
                CheckedTrueDataIn();
                this.GRTV_ktn004_marsh16_repeat_21032012.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void someOtherInput_CheckedChanged(object sender, EventArgs e)
        {
            if (this.someOtherInput.Checked == true)
            {
                CheckedTrueDataIn();
                this.someOtherInput.Enabled = true;
            }
            else CheckedFalseDataIn();
        }






        //---Режимы старта---//
        private void OnlyIntegrating_CheckedChanged(object sender, EventArgs e)
        {
            if (this.OnlyIntegrating.Checked == true)
            {
                LockTheWayOfStart(); this.OnlyIntegrating.Enabled = true;
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = true;
            }
            else
            {
                FreeTheWayOfStart();
                this.Main_Block_Click_new.Enabled = false;
            }
        }

        private void feedbackExist_CheckedChanged(object sender, EventArgs e)
        {
            if (this.feedbackExist.Checked == true)
            {
                LockTheWayOfStart(); this.feedbackExist.Enabled = true;
                FreeTypesOfCorrection();
                this.usingSNS.Enabled = true;

                this.WeakConnect.Enabled = false;
                this.ModifWeakConnect.Enabled = false;
            }
            else
            {
                UnCheckTypesOfCorrection();
                FreeTheWayOfStart();
                LockTypesOfCorrection();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;

                this.WeakConnect.Enabled = true;
                this.ModifWeakConnect.Enabled = true;
            }

            if (this.feedbackExist.Checked && !this.Odometr_SINS_case.Checked)
            {
                this.flag_UsingOdoPosition.Checked = false;
                this.flag_UsingOdoPosition.Enabled = false;
            }
            if (this.feedbackExist.Checked && this.Odometr_SINS_case.Checked)
                this.flag_UsingOdoPosition.Enabled = true;
        }

        private void EstimateExist_CheckedChanged(object sender, EventArgs e)
        {
            if (this.EstimateExist.Checked == true)
            {
                LockTheWayOfStart(); this.EstimateExist.Enabled = true;
                FreeTypesOfCorrection();
                this.usingSNS.Enabled = true;

                this.WeakConnect.Enabled = false;
                this.ModifWeakConnect.Enabled = false;
            }
            else
            {
                UnCheckTypesOfCorrection();
                FreeTheWayOfStart();
                LockTypesOfCorrection();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;

                this.WeakConnect.Enabled = true;
                this.ModifWeakConnect.Enabled = true;
            }

            if (this.EstimateExist.Checked && !this.Odometr_SINS_case.Checked)
            {
                this.flag_UsingOdoPosition.Checked = false;
                this.flag_UsingOdoPosition.Enabled = false;
            }
            if (this.EstimateExist.Checked && this.Odometr_SINS_case.Checked)
                this.flag_UsingOdoPosition.Enabled = true;
        }

        private void Odometr_SINS_case_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Odometr_SINS_case.Checked == true)
            {
                //LockTheWayOfStart(); this.Odometr_SINS_case.Enabled = true;
                //LockNumbOfMeasures();
                this.iMx_kappa_1_3_ds.Checked = true;
                this.WeakConnect.Enabled = true;
                this.ModifWeakConnect.Enabled = true;
                this.Use_Only_Stops.Enabled = false;

                this.iMqDeltaR.Checked = true;
                this.iMqDeltaRodo.Checked = true;
            }
            else
            {
                FreeTheWayOfStart();
                this.iMx_kappa_1_3_ds.Checked = false;
                this.WeakConnect.Enabled = false;
                this.ModifWeakConnect.Enabled = false;
                this.Use_Only_Stops.Enabled = true;

                this.iMqDeltaR.Checked = false;
                this.iMqDeltaRodo.Checked = false;
            }
        }
        private void WeakConnect_CheckedChanged(object sender, EventArgs e)
        {
            //if (this.WeakConnect.Checked == true)
            //{
            //    FreeTypesOfCorrection();
            //    this.ModifWeakConnect.Enabled = false;
            //    this.usingSNS.Enabled = true;
            //    this.Main_Block_Click_new.Enabled = true;
            //}
            //else
            //{
            //    LockTypesOfCorrection();
            //    this.usingSNS.Enabled = false;
            //    this.Main_Block_Click_new.Enabled = false;
            //    this.ModifWeakConnect.Enabled = true;
            //}
        }
        private void ModifWeakConnect_CheckedChanged(object sender, EventArgs e)
        {
            //if (this.ModifWeakConnect.Checked == true)
            //{
            //    FreeTypesOfCorrection();
            //    this.WeakConnect.Enabled = false;
            //    this.usingSNS.Enabled = true;
            //    this.Main_Block_Click_new.Enabled = true;
            //}
            //else
            //{
            //    LockTypesOfCorrection();
            //    this.usingSNS.Enabled = false;
            //    this.Main_Block_Click_new.Enabled = false;
            //    this.WeakConnect.Enabled = true;
            //}
        }




        //---Режимы коррекции---//
        private void CheckedTrueTypesOfCorrection()
        {
            LockTypesOfCorrection();
            this.Main_Block_Click_new.Enabled = true;
        }
        private void CheckedFlaseTypesOfCorrection()
        {
            FreeTypesOfCorrection();
            this.Main_Block_Click_new.Enabled = false;
        }


        private void Use_Only_Stops_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_Only_Stops.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Use_Only_Stops.Enabled = true;
                this.flag_not_use_zupt.Checked = false;
                this.flag_not_use_zupt.Enabled = false;
            }
            else
            {
                CheckedFlaseTypesOfCorrection();
                this.flag_not_use_zupt.Enabled = true;
            }
        }

        private void flag_UsingOdoVelocity_CheckedChanged(object sender, EventArgs e)
        {
            if (this.flag_UsingOdoVelocity.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.flag_UsingOdoVelocity.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }


        private void flag_UsingOdoPosition_CheckedChanged(object sender, EventArgs e)
        {
            if (this.flag_UsingOdoPosition.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.flag_not_use_zupt.Checked = true;
                this.flag_UsingOdoPosition.Enabled = true;
            }
            else
            {
                this.flag_not_use_zupt.Checked = false;
                CheckedFlaseTypesOfCorrection();
            }
        }

        private void usingSNS_CheckedChanged(object sender, EventArgs e)
        {
            if (this.usingSNS.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.usingSNS.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }


        private void OnlyAlignment_CheckedChanged(object sender, EventArgs e)
        {
            if (this.OnlyAlignment.Checked == true)
            {
                this.Main_Block_Click_new.Enabled = true;
                LockTheWayOfStart();
            }
            if (this.OnlyAlignment.Checked == false)
            {
                this.Main_Block_Click_new.Enabled = false;
                FreeTheWayOfStart();
            }
        }

        private void flag_autonomous_dinamic_mode_CheckedChanged(object sender, EventArgs e)
        {
        }

        private void flag_UseAlgebraDrift_CheckedChanged(object sender, EventArgs e)
        {
        }

        private void iMx_r_3_dV_3_CheckedChanged(object sender, EventArgs e)
        {
        }

        private void iMq_eq_iMx_CheckedChanged(object sender, EventArgs e)
        {
            if (this.iMq_eq_iMx.Checked == true)
            {
                this.iMqKappa.Checked = true;
                this.iMqDeltaR.Checked = true;
                this.iMqDeltaF.Checked = true;
                this.iMqDeltaNu.Checked = true;
                this.iMqDeltaRodo.Checked = true;
                this.iMqVarkappa3.Checked = true;
            }
            else
            {
                this.iMqKappa.Checked = false;
                this.iMqDeltaF.Checked = false;
                this.iMqDeltaNu.Checked = false;
                this.iMqVarkappa3.Checked = false;

                if (this.Odometr_SINS_case.Checked == false)
                {
                    this.iMqDeltaR.Checked = false;
                    this.iMqDeltaRodo.Checked = false;
                }
            }
        }

        private void SINS_Processing_Load(object sender, EventArgs e)
        {

        }

        private void DoFeedBackKappa_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void iMx_kappa_1_3_ds_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void OdoModelOnlyCP_CheckedChanged(object sender, EventArgs e)
        {

        }

        

        









    }
}
