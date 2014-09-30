using System;
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

 

namespace SINS_motion_processing_new_data
{
    public partial class SINS_Processing : Form
    {
        int iMx = SimpleData.iMx = 25;
        int iMq = SimpleData.iMq = SimpleData.iMx;
        int iMz = SimpleData.iMz = 15;
        int iMxSmthd = SimpleData.iMxSmthd = 9;

        StreamReader myFile;

        SINS_State SINSstate, SINSstate_OdoMod;
        Kalman_Vars KalmanVars;
        Parameters Params;
        ParamsForModel OdoModel;
        Proc_Help ProcHelp;

        int value_iMx_r3_dV3 = 0, value_iMx_r_odo_12 = 0, value_iMx_kappa_13_ds = 0;
        bool iMx_r3_dV3;
        bool iMx_kappa_13_ds;

        string GlobalPrefixTelemetric = "";
        

        public SINS_Processing()
        {
            InitializeComponent();

            LockParamsOfStart();
            LockTypesOfCorrection();
            LockTheWayOfStart();
            LockNumbOfMeasures();
            LockDimOfVector();
            this.usingSNS.Enabled = false;
            this.Main_Block_Click_new.Enabled = false;
        }


        ParamToStart ParamStart = new ParamToStart();


        public struct Entry
        {
            public string str;
        }


        public void Main_Block_Click_new_Click(object sender, EventArgs e)
        {
            int l = 0;


            //------------------------------------------------------------------------
            //------------------------------------------------------------------------
            //---для экспериментов---
            ParamStart.Experiment_NoiseModelFlag = false; // Брать модельные значения, а не задаваемые ниже
            ParamStart.Experiment_Noise_Vel = 3E-2; //3E-4- optim --Не играют роли для Экспериментов
            ParamStart.Experiment_Noise_Angl = 3E-4; //3E-6- optim --Не играют роли для Экспериментов

            ParamStart.Experiment_stdR = 1.0;
            ParamStart.Experiment_stdOdoR = 1.0; // метров
            ParamStart.Experiment_stdV = 0.1;
            ParamStart.Experiment_stdScale = 0.005;
            ParamStart.Experiment_stdKappa1 = 20.0; //минут
            ParamStart.Experiment_stdKappa3 = 20.0; //минут


            //---для имитатора---
            ParamStart.Imitator_NoiseModelFlag = true; // Брать модельные значения, а не задаваемые ниже
            ParamStart.Imitator_Noise_Vel = 3E-4; 
            ParamStart.Imitator_Noise_Angl = 3E-6; 

            ParamStart.Imitator_Noise_OdoScale = 0.000000001;
            ParamStart.Imitator_Noise_OdoKappa = 0.0000001 * 3.141592 / 180.0 / 3600.0;
            ParamStart.Imitator_Noise_Pos = 0.75;
            ParamStart.Imitator_Noise_Drift = 0.0000002 * 3.141592 / 180.0 / 3600.0;
            ParamStart.Imitator_Noise_Accel = 0.000000002;

            ParamStart.Imitator_stdR = 1.1;
            ParamStart.Imitator_stdOdoR = 1.1; // метров
            ParamStart.Imitator_stdV = 0.1;
            ParamStart.Imitator_stdScale = 0.01;
            ParamStart.Imitator_stdKappa1 = 20.0; //минут
            ParamStart.Imitator_stdKappa3 = 20.0; //минут

            ParamStart.Modeling_Params_OdoKappa1 = 1 * SimpleData.ToRadian;
            ParamStart.Modeling_Params_OdoKappa3 = -2 * SimpleData.ToRadian;
            ParamStart.Modeling_Params_OdoIncrement = 10.0; // в сантиметрах
            ParamStart.Modeling_Params_OdoScaleErr = 1.02;
            ParamStart.Modeling_Params_OdoFrequency = 5;
            ParamStart.Modeling_Params_df_s = 100.0; //(rnd_1.NextDouble() - 0.5) / Params_df_s //100.0 - норма
            ParamStart.Modeling_Params_dnu_s = 10000.0; //(rnd_5.NextDouble() - 0.5) / Params_dnu_s //10000.0 - норма
            //------------------------------------------------------------------------
            //------------------------------------------------------------------------






            this.DefineDimentionOfErrorVector();                                                            //---формирование размерности вектора ошибок---//
            this.DefineClassElementAndFlags();
            this.SelectDataIn();                                                                            //---выбор входного набора данных---//

            StreamWriter InputForSmoothFile = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//" + SINSstate.Global_file + "_Back.dat");

            //Вычисляем LastCountForRead
            SINSstate.LastCountForRead = -20;
            int maxStrLength = 0;
            for (; ; )
            {
                string tmpstr = myFile.ReadLine();
                if (myFile.EndOfStream) break;
                SINSstate.LastCountForRead++;
                maxStrLength = Math.Max(maxStrLength, tmpstr.Length);
            }
            int LastCountForRead = SINSstate.LastCountForRead;
            myFile.BaseStream.Seek(0, SeekOrigin.Begin);



            if (SINSstate.Global_file.ToLower().Contains("imitator"))
            {
                DateTime fileCreatedDate1 = File.GetLastWriteTime(this.GlobalPrefixTelemetric + "_Clear.dat");
                DateTime fileCreatedDate2 = File.GetLastWriteTime(this.GlobalPrefixTelemetric + "_Errors.dat");

                //сюда заходим, если уже выбран myFile= Errors
                if (fileCreatedDate1 < fileCreatedDate2)
                {
                    ImitatorHeaderReadAndApply();
                    if ((SINSstate.stdNu == 0.2 && SINSstate.flag_AccuracyClass_0_2_grph == false)
                        || (SINSstate.stdNu == 0.02 && SINSstate.flag_AccuracyClass_0_02grph == false)
                        || (SINSstate.stdNu == 2.0 && SINSstate.flag_AccuracyClass_2_0_grph == false)
                        || (SINSstate.stdNu < 0.001 && SINSstate.flag_AccuracyClass_NoErr == false))
                    {
                        myFile.Close();
                        myFile = new StreamReader(this.GlobalPrefixTelemetric + "_Clear.dat");
                        fileCreatedDate2 = File.GetLastWriteTime(this.GlobalPrefixTelemetric + "_Clear.dat");
                    }
                    else
                        myFile = new StreamReader(this.GlobalPrefixTelemetric + "_Errors.dat");
                }

                if (fileCreatedDate1 >= fileCreatedDate2)
                {
                    ImitatorHeaderReadAndApply();
                    ImitatorFirstProcessing.mainProcessing(SINSstate, SINSstate_OdoMod, KalmanVars, ProcHelp, myFile, this.GlobalPrefixTelemetric, ParamStart);

                    myFile = new StreamReader(this.GlobalPrefixTelemetric + "_Errors.dat");

                    SINSstate = new SINS_State();
                    SINSstate_OdoMod = new SINS_State();
                    KalmanVars = new Kalman_Vars();
                    Params = new Parameters();
                    OdoModel = new ParamsForModel();
                    ProcHelp = new Proc_Help();

                    this.DefineClassElementAndFlags();
                    this.SelectDataIn();

                    SINSstate.LastCountForRead = LastCountForRead - 10;
                }
            }


            Parameters.StartSINS_Parameters(SINSstate, SINSstate_OdoMod, KalmanVars, Params, ProcHelp);     //---Инициализация начальных условий при отсутствии выставки---//


            ProcHelp.AlgnCnt = 0;
            if (SINSstate.Global_file == "Azimut_14.08.2012") ProcHelp.AlgnCnt = 21486;
            if (SINSstate.Global_file == "ktn004_15.03.2012") ProcHelp.AlgnCnt = 48000;
            if (SINSstate.Global_file == "ktn004_21.03.2012") ProcHelp.AlgnCnt = 45000;
            if (SINSstate.Global_file == "Azimut_29.08.2012") ProcHelp.AlgnCnt = 35000;
            if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2") ProcHelp.AlgnCnt = 10300;
            if (SINSstate.Global_file == "Imitator_Data") ProcHelp.AlgnCnt = 9000;

            if (SINSstate.Global_file == "Azimut-T_18-Oct-2013_11-05-11") ProcHelp.AlgnCnt = 38000;
            if (SINSstate.Global_file == "Saratov_run_2014_07_23")
            {
                ProcHelp.AlgnCnt = 27000;
                if (this.SaratAlignStart.Checked == true || this.SaratENDStart.Checked == true) 
                    ProcHelp.AlgnCnt = SINSstate.LastCountForRead;
            }
            //if (SINSstate.Global_file == "Saratov_run_2014_07_23") ProcHelp.AlgnCnt = 180000;

            if (SINSstate.Global_file == "AZIMUT_T_2013_10_18_12_55") ProcHelp.AlgnCnt = 22000;
            if (SINSstate.Global_file == "Azimut_514_08Nov2013_11_15") ProcHelp.AlgnCnt = 95000;

            if (SINSstate.Global_file == "Imitator_Telemetric") ProcHelp.AlgnCnt = 100;


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

                if (SINSstate.flag_UsingNavAlignment == true)
                    l = SINSAlignment_Navigantion.SINS_Alignment_Navigation(ProcHelp, SINSstate, SINSstate2, SINSstate_OdoMod, myFile, KalmanVars);
                else if (SINSstate.flag_UsingClasAlignment == true)
                    l = SINSAlignment_Classical.SINS_Alignment_Classical(ProcHelp, SINSstate, SINSstate2, SINSstate_OdoMod, myFile, KalmanVars);

                if (SINSstate.flag_AccuracyClass_0_02grph)
                {
                    SINSstate.stdF = 1E-5 * 9.81; //далее умножается G
                    SINSstate.stdNu = 0.02; //град/час
                }
                if (SINSstate.flag_AccuracyClass_0_2_grph)
                {
                    SINSstate.stdF = 1E-4 * 9.81; //далее умножается G
                    SINSstate.stdNu = 0.2; //град/час
                }
                if (SINSstate.flag_AccuracyClass_2_0_grph)
                {
                    SINSstate.stdF = 1E-3 * 9.81; //далее умножается G
                    SINSstate.stdNu = 2.0; //град/час
                }

                if (ParamStart.Experiment_NoiseModelFlag == true)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        KalmanVars.Noise_Vel[j] = ParamStart.Experiment_Noise_Vel;
                        KalmanVars.Noise_Angl[j] = ParamStart.Experiment_Noise_Angl;

                    }
                }

                SINSstate.stdR = ParamStart.Experiment_stdR;
                SINSstate.stdOdoR = ParamStart.Experiment_stdOdoR = 1.0; // метров
                SINSstate.stdV = ParamStart.Experiment_stdV;
                SINSstate.stdAlpha12 = SINSstate.stdF / 9.81; //радиан
                SINSstate.stdBeta3 = SINSstate.stdNu * SimpleData.ToRadian / 3600.0 / (SimpleData.U * Math.Cos(SINSstate.Latitude)); //радиан
                SINSstate.stdScale = ParamStart.Experiment_stdScale;
                SINSstate.stdKappa1 = ParamStart.Experiment_stdKappa1; //минут
                SINSstate.stdKappa3 = ParamStart.Experiment_stdKappa3; //минут
            }


            //---Инициализация начальной матрицы ковариации---
            if (SINSstate.flag_Odometr_SINS_case)
                Odometr_SINS.InitOfCovarianceMatrixes(SINSstate, KalmanVars);
            else
                SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);



            //---Переопределяем размерности векторов и матриц после выставки---
            this.DefineDimentionOfErrorVector();

            SINSprocessing.Gauss_Kruger(SINSstate);

            SINSstate2 = SINS_State.DeepCopy(SINSstate);

            DateTime start = DateTime.Now;
            if (SINSstate.flag_OnlyAlignment == false)
            {
                ////------БИНС + ОДОМЕТР------
                if (OnlyIntegrating.Checked == false && SINSstate.flag_OnlyAlignment == false || SINSstate.flag_Odometr_SINS_case == true)
                    SINS_Corrected.SINS_Corrected_Processing(l, false, myFile, SINSstate, SINSstate2, KalmanVars, ProcHelp, SINSstate_OdoMod, OdoModel);
                //------Автономное решение-----
                else if (SINSstate.flag_OnlyAlignment == false)
                    SINS_Autonomous.SINS_Autonomous_Processing(l, myFile, SINSstate, SINSstate2, KalmanVars, ProcHelp, SINSstate_OdoMod, OdoModel);

                if (SINSstate.flag_Smoothing)
                    SINS_Corrected.SINS_Corrected_Processing(l, true, myFile, SINSstate, SINSstate2, KalmanVars, ProcHelp, SINSstate_OdoMod, OdoModel);
            }


            DateTime end = DateTime.Now;
            Console.Write(" Processing time: " + (end - start).ToString());
            Console.Write(" "); Console.Write(" ");

            myFile.Close(); this.Close();
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
            SINSstate.odo_min_increment = Convert.ToDouble(dataArray[23]) / 100.0;
            if (SINSstate.odo_min_increment < 0.0001)
                SINSstate.odo_min_increment = 0.01;

            SINSstate.stdF = Convert.ToDouble(dataArray[9]) * 9.81;
            SINSstate.stdNu = Convert.ToDouble(dataArray[13]);
            for (int j = 0; j < 3; j++)
            {
                if (ParamStart.Imitator_NoiseModelFlag == true)
                {
                    KalmanVars.Noise_Vel[j] = ParamStart.Imitator_Noise_Vel;
                    KalmanVars.Noise_Angl[j] = ParamStart.Imitator_Noise_Angl;
                }
                else
                {
                    KalmanVars.Noise_Vel[j] = 1.0 / 3.0 / Convert.ToDouble(dataArray[11]);
                    KalmanVars.Noise_Angl[j] = 1.0 / 3.0 / Convert.ToDouble(dataArray[15]);
                }

            }

            KalmanVars.OdoNoise_V = SINSstate.odo_min_increment / SINSstate.Freq / Convert.ToDouble(dataArray[25]);
            KalmanVars.OdoNoise_Dist = SINSstate.odo_min_increment;
            KalmanVars.OdoNoise_STOP = 0.01;

            KalmanVars.Noise_OdoScale = ParamStart.Imitator_Noise_OdoScale;
            KalmanVars.Noise_OdoKappa = ParamStart.Imitator_Noise_OdoKappa;
            KalmanVars.Noise_Pos = ParamStart.Imitator_Noise_Pos;
            KalmanVars.Noise_Drift = ParamStart.Imitator_Noise_Drift;
            KalmanVars.Noise_Accel = ParamStart.Imitator_Noise_Accel;

            ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = Convert.ToDouble(dataArray[3]);
            ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = Convert.ToDouble(dataArray[1]);
            ProcHelp.AltSNS = SINSstate_OdoMod.Altitude = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = SINSstate.Altitude_prev = Convert.ToDouble(dataArray[5]);
            ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
            ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

            //Углы найденные подбором минимизацией максимальной ошибки по позиции.
            SINSstate.Heading = Convert.ToDouble(dataArray[27]) + SINSstate.stdNu * SimpleData.ToRadian / 3600.0 / (SimpleData.U * Math.Cos(SINSstate.Latitude));
            SINSstate.Roll = Convert.ToDouble(dataArray[29]) + SINSstate.stdF / 9.81;
            SINSstate.Pitch = Convert.ToDouble(dataArray[31]) + SINSstate.stdF / 9.81;

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

            SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);


            //------------------------------------------------------
            SINSstate.stdR = ParamStart.Imitator_stdR; // метров
            SINSstate.stdOdoR = ParamStart.Imitator_stdOdoR; // метров
            SINSstate.stdV = ParamStart.Imitator_stdV; // м/с
            SINSstate.stdAlpha12 = SINSstate.stdF / 9.81; //радиан
            SINSstate.stdBeta3 = SINSstate.stdNu * SimpleData.ToRadian / 3600.0 / (SimpleData.U * Math.Cos(SINSstate.Latitude)); //радиан
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

            if (iMx_r3_dV3)
            {
                value_iMx_r3_dV3 = iMx;
                iMx = SimpleData.iMx += 2;
            }

            if (iMx_kappa_13_ds)
            {
                value_iMx_kappa_13_ds = iMx;
                iMx = SimpleData.iMx += 3;
            }

            if (this.Odometr_SINS_case.Checked)
            {
                value_iMx_r_odo_12 = iMx;
                iMx = SimpleData.iMx += 2;

                if (this.iMx_r_odo_3.Checked)
                    iMx = SimpleData.iMx += 1;
            }

            //----------------------------Вектор шумов---------------------------
            iMq = SimpleData.iMq = 0;

            if (iMqDeltaR.Checked)
                iMq = SimpleData.iMq += 2;

            iMq = SimpleData.iMq += 5;

            if (iMx_r3_dV3)
            {
                if (iMqDeltaR.Checked)
                    iMq = SimpleData.iMq += 1;
                iMq = SimpleData.iMq += 1;
            }

            if (iMqDeltaF.Checked)
                iMq = SimpleData.iMq += 3;
            if (iMqDeltaNu.Checked)
                iMq = SimpleData.iMq += 3;

            if (iMqVarkappa13.Checked)
                iMq = SimpleData.iMq += 2;
            if (iMqKappa.Checked)
                iMq = SimpleData.iMq += 1;
            
            if (this.Odometr_SINS_case.Checked)
            {
                if (iMqDeltaRodo.Checked)
                    iMq = SimpleData.iMq += 2;

                if (this.iMx_r_odo_3.Checked)
                    iMq = SimpleData.iMq += 1;
            }

            if (this.iMSmthd_Is_2.Checked)
            {
                iMxSmthd = SimpleData.iMxSmthd = 2;
                if (this.iMx_r_3_dV_3.Checked)
                    iMxSmthd = SimpleData.iMxSmthd = 3;
            }
            if (this.iMSmthd_Is_7.Checked)
                iMxSmthd = SimpleData.iMxSmthd = 7;

            //if (iMq_eq_iMx.Checked == true)
            //    iMq = SimpleData.iMq = iMx;
        }

        public void DefineClassElementAndFlags()
        {
            SINSstate = new SINS_State(); SINSstate_OdoMod = new SINS_State();
            KalmanVars = new Kalman_Vars();
            Params = new Parameters();
            OdoModel = new ParamsForModel();
            ProcHelp = new Proc_Help();

            SINSstate.FreqOutput = Convert.ToInt32(this.Output_Freq.Text);

            SINSstate.flag_iMx_r3_dV3 = this.iMx_r_3_dV_3.Checked;
            SINSstate.flag_iMx_kappa_13_ds = this.iMx_kappa_1_3_ds.Checked;

            SINSstate.flag_iMqDeltaR = iMqDeltaR.Checked;
            SINSstate.flag_iMqDeltaF = iMqDeltaF.Checked;
            SINSstate.flag_iMqDeltaNu = iMqDeltaNu.Checked;
            SINSstate.flag_iMqVarkappa13 = iMqVarkappa13.Checked;
            SINSstate.flag_iMqKappa = iMqKappa.Checked;
            SINSstate.flag_iMqDeltaRodo = iMqDeltaRodo.Checked;
            SINSstate.flag_Imitator_Telemetric = Imitator_Telemetric.Checked;

            if (SINSstate.flag_iMx_r3_dV3)
                SINSstate.iMx_r3_dV3 = value_iMx_r3_dV3;

            if (this.Odometr_SINS_case.Checked)
                SINSstate.iMx_r12_odo = value_iMx_r_odo_12;

            if (SINSstate.flag_iMx_kappa_13_ds)
                SINSstate.iMx_odo_model = value_iMx_kappa_13_ds;

            //---флаги---
            SINSstate.flag_Autonomous_Solution = this.OnlyIntegrating.Checked;
            SINSstate.flag_UsingAvegering = this.UsingAveraging.Checked;
            SINSstate.flag_UsingAltitudeCorrection = this.UsingAltitudeCorrection.Checked;
            SINSstate.flag_Using_SNS = this.usingSNS.Checked;
            SINSstate.flag_FeedbackExist = this.feedbackExist.Checked;
            if (this.Odometr_SINS_case.Checked)
                SINSstate.flag_DoFeedBackKappa = this.DoFeedBackKappa.Checked;
            SINSstate.flag_DoFeedBackDeltaFW = this.DoFeedBackDeltaFW.Checked;
            SINSstate.flag_EstimateExist = this.EstimateExist.Checked;
            SINSstate.flag_UsingClasAlignment = this.UsingClasAlignment.Checked;
            SINSstate.flag_UsingNavAlignment = this.UsingNavAlignment.Checked;
            SINSstate.flag_Odometr_SINS_case = this.Odometr_SINS_case.Checked;
            SINSstate.flag_OdoSINSWeakConnect = this.WeakConnect.Checked;
            SINSstate.flag_OdoSINSWeakConnect_MODIF = this.ModifWeakConnect.Checked;
            SINSstate.flag_Using_iMx_r_odo_3 = this.iMx_r_odo_3.Checked;
            SINSstate.flag_UseOdoVelocity_In_Oz = this.UseOdo_In_Oz.Checked;
            SINSstate.flag_UseOnlyStops = this.Use_Only_Stops.Checked;
            SINSstate.flag_OnlyAlignment = this.OnlyAlignment.Checked;
            SINSstate.flag_Smoothing = this.flag_Smoothing.Checked;
            SINSstate.flag_not_use_kns = this.flag_not_use_kns.Checked;
            SINSstate.flag_using_slippage = this.flag_using_slippage.Checked;
            SINSstate.flag_using_Checkpotints = this.flag_using_Checkpotints.Checked;
            SINSstate.flag_using_GoCalibrInCP = this.flag_using_GoCalibrInCP.Checked;
            SINSstate.flag_autonomous_dinamic_mode = this.flag_autonomous_dinamic_mode.Checked;
            SINSstate.add_velocity_to_position = this.add_velocity_to_position.Checked;
            SINSstate.flag_UseAlgebraDrift = this.flag_UseAlgebraDrift.Checked;
            SINSstate.flag_UsingScalarOdoMeasure = this.flag_UsingScalarOdoMeasure.Checked;
            SINSstate.flag_OdoModelOnlyCP = this.OdoModelOnlyCP.Checked;

            SINSstate.flag_AccuracyClass_NoErr = this.AccuracyClass_NoErr.Checked;
            SINSstate.flag_AccuracyClass_0_02grph = this.AccuracyClass_0_02grph.Checked;
            SINSstate.flag_AccuracyClass_0_2_grph = this.AccuracyClass_0_2_grph.Checked;
            SINSstate.flag_AccuracyClass_2_0_grph = this.AccuracyClass_2_0_grph.Checked;

            //---флаги коррекции---
            SINSstate.Use_Each_Odo_Measure = this.Use_Each_Odo_Measure.Checked;
            SINSstate.Ungolonom_Velocity_model = this.Ungolonom_model.Checked;
            SINSstate.Use_dV_by_F_Constraint = this.Use_dV_by_F_Constraint.Checked;
            SINSstate.Use_Const_dV_Constraint = this.Use_Const_dV_Constraint.Checked;
            SINSstate.Use_Constant_Constraint = this.Use_Constant_Constraint.Checked;
            SINSstate.Use_Const_Freq = this.Use_Const_Freq.Checked;
            //SINSstate.Use_dV_Constraints = this.Use_dV_Constraints.Checked;
            SINSstate.Use_Odo_Distance = this.Use_Odo_Distance.Checked;
            SINSstate.Use_Integral_of_Fx_2 = this.Use_Integral_of_Fx_2.Checked;
        }

        public void DataInCheck(string FileLink)
        {
            DateTime fileCreatedDate1 = File.GetLastWriteTime(FileLink + "_Clear.dat");
            DateTime fileCreatedDate2 = File.GetLastWriteTime(FileLink + "_Errors.dat");

            if (fileCreatedDate1 >= fileCreatedDate2)
                myFile = new StreamReader(FileLink + "_Clear.dat");
            else
                myFile = new StreamReader(FileLink + "_Errors.dat");
        }

        public void SelectDataIn()
        {
            if (Azimut_14_08_2012.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//AzimutB_210530_Other_120814_Autolab_10-31-26_2.dat");
                SINSstate.Global_file = "Azimut_14.08.2012";
            }
            else if (Azimut_15_08_2012.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//AzimutB_210530_Other_120815_Autolab_DPC_100Hz_14-40-04.dat");
                SINSstate.Global_file = "Azimut_15.08.2012";
            }
            else if (Azimut_24_08_2012.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//AzimutB_210530_Other_120824_Autolab_Circle_AzimutKama_12-07-25.dat");
                SINSstate.Global_file = "Azimut_24.08.2012";
            }
            else if (Azimut_29_08_2012.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//AzimutB_210530_Other_120829_Autolab_Circle_09-21-35.dat");
                SINSstate.Global_file = "Azimut_29.08.2012";
            }
            else if (Kama_04_09_2012.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Kama_120904_04-Sep-2012,14-44-11.dat");
                SINSstate.Global_file = "Kama_04.09.2012";
                for (int i = 0; i < 14649; i++) myFile.ReadLine();
            }

            else if (ktn004_21_03_2012.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//ktn004_marsh16_repeat_21-Mar-2012,17-21-07_dat.dat");
                SINSstate.Global_file = "ktn004_21.03.2012";
            }
            else if (ktn004_15_03_2012.Checked == true)
            {
                SINSstate.Global_file = "ktn004_15.03.2012";
                if (this.Imitator_Telemetric.Checked == true)
                {
                    this.GlobalPrefixTelemetric = "D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//TelemetricData//Telemetric_Imitator_" + SINSstate.Global_file;
                    this.DataInCheck(this.GlobalPrefixTelemetric);
                    SINSstate.Global_file = "Imitator_Telemetric";
                }
                else
                {
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//ktn004_static4hour_marsh_sns_15-Mar-2012,16-29-45_dat.dat");
                }
            }
            else if (Imitator_Data.Checked == true)
            {
                SINSstate.Global_file = "Imitator_Data";
                if (this.Imitator_Telemetric.Checked == true)
                {
                    this.GlobalPrefixTelemetric = "D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//TelemetricData//Telemetric_Imitator_" + SINSstate.Global_file;
                    this.DataInCheck(this.GlobalPrefixTelemetric);
                    SINSstate.Global_file = "Imitator_Telemetric";
                }
                else
                {
                    this.GlobalPrefixTelemetric = "D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Imitator_Analytic";
                    this.DataInCheck(this.GlobalPrefixTelemetric);
                }
            }



            //МИНСКИЕ ЗАЕЗДЫ
            if (Azimuth_minsk_race_4_3to6to2.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//120222_AzimutB_210530_Race_4_Control_3-6-2_11-49-20_dat.dat");
                SINSstate.Global_file = "Azimuth_minsk_race_4_3to6to2";
            }


            //---AZIMUT-T----
            //if (_16_09_13_TLM_1zaezd.Checked == true)
            //{
            //    SINSstate.FreqOutput = 100;
            //    //myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Topo_Saratov//topo_short.dat");
            //    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Topo_Saratov//topo.xch");
            //    SINSstate.Global_file = "Azimut-T_18-Oct-2013_11-05-11";
            //}



            if (AZIMUT_T_12_32_16_09_13_TLM_2z.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Azimut-T//Azimut-T_18-Oct-2013_12-56-43.txt");
                SINSstate.Global_file = "AZIMUT_T_2013_10_18_12_55";
            }
            if (Azimut_514_08Nov2013_11_15.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Azimut-10B//Azimut_514_08Nov2013_11_15.txt");
                SINSstate.Global_file = "Azimut_514_08Nov2013_11_15";
            }



            if (_16_09_13_TLM_1zaezd.Checked == true)
            {
                if (SINSstate.FreqOutput == 10)
                    SINSstate.FreqOutput = 100;

                if (this.SaratovFullRun.Checked == false && !this.SaratAlignStart.Checked && !this.SaratENDStart.Checked && !this.SaratWithZalipan.Checked)
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Saratov_run_2014_07_23.dat");
                else if (this.SaratovFullRun.Checked == false && !this.SaratAlignStart.Checked && !this.SaratENDStart.Checked && this.SaratWithZalipan.Checked)
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Saratov_run_2014_07_23_noFilter.dat");

                else if (this.SaratAlignStart.Checked && !this.SaratWithZalipan.Checked)
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Saratov_run_2014_07_23_Align.dat");
                else if (this.SaratENDStart.Checked && !this.SaratWithZalipan.Checked)
                {
                    SINSstate.Saratov_run_Final = true;
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Saratov_run_2014_07_23_finalAlign.dat");
                }

                else if (this.SaratAlignStart.Checked && this.SaratWithZalipan.Checked)
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Saratov_run_2014_07_23_Align_noFilter.dat");
                else if (this.SaratENDStart.Checked && this.SaratWithZalipan.Checked)
                {
                    SINSstate.Saratov_run_Final = true;
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Saratov_run_2014_07_23_finalAlign_noFilter.dat");
                }

                else if (!this.SaratWithZalipan.Checked)
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Saratov_run_2014_07_23_full.dat");
                else if (this.SaratWithZalipan.Checked)
                    myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//Saratov_run_2014_07_23_full_noFilter.dat");

                SINSstate.Global_file = "Saratov_run_2014_07_23";
            }
        }












        //---ИНТЕРФЕЙС---//

        public void LockTypesOfCorrection()
        {
            this.Use_Odo_Distance.Enabled = false; this.Use_Each_Odo_Measure.Enabled = false; this.Use_Constant_Constraint.Enabled = false;
            this.Use_Const_dV_Constraint.Enabled = false; this.Use_Const_dV_Constraint.Enabled = false; this.Use_dV_by_F_Constraint.Enabled = false;
            this.Use_Const_Freq.Enabled = false; this.Use_Only_Stops.Enabled = false; this.Use_Integral_of_Fx_2.Enabled = false; this.Ungolonom_model.Enabled = false;
        }
        public void FreeTypesOfCorrection()
        {
            this.Use_Odo_Distance.Enabled = true; this.Use_Each_Odo_Measure.Enabled = true; this.Use_Constant_Constraint.Enabled = true;
            this.Use_Const_dV_Constraint.Enabled = true; this.Use_Const_dV_Constraint.Enabled = true; this.Use_dV_by_F_Constraint.Enabled = true;
            this.Use_Const_Freq.Enabled = true; this.Use_Only_Stops.Enabled = true; this.Use_Integral_of_Fx_2.Enabled = true; this.Ungolonom_model.Enabled = true;
        }

        public void LockInData()
        {
            this.Azimut_14_08_2012.Enabled = false; this.Azimut_15_08_2012.Enabled = false; this.Azimut_24_08_2012.Enabled = false; this.Azimut_29_08_2012.Enabled = false; this.Kama_04_09_2012.Enabled = false;
            this.ktn004_21_03_2012.Enabled = false; this.ktn004_15_03_2012.Enabled = false; this.Imitator_Data.Enabled = false;
            this.Azimuth_minsk_race_4_3to6to2.Enabled = false;
            this._16_09_13_TLM_1zaezd.Enabled = false; this.AZIMUT_T_12_32_16_09_13_TLM_2z.Enabled = false; this.Azimut_514_08Nov2013_11_15.Enabled = false;
        }
        public void FreeInData()
        {
            this.Azimut_14_08_2012.Enabled = true; this.Azimut_15_08_2012.Enabled = true; this.Azimut_24_08_2012.Enabled = true; this.Azimut_29_08_2012.Enabled = true; this.Kama_04_09_2012.Enabled = true;
            this.ktn004_21_03_2012.Enabled = true; this.ktn004_15_03_2012.Enabled = true; this.Imitator_Data.Enabled = true;
            this.Azimuth_minsk_race_4_3to6to2.Enabled = true;
            this._16_09_13_TLM_1zaezd.Enabled = true; this.AZIMUT_T_12_32_16_09_13_TLM_2z.Enabled = true; this.Azimut_514_08Nov2013_11_15.Enabled = true;
        }

        public void LockParamsOfStart()
        {
            this.OnlyAlignment.Enabled = false; this.UsingClasAlignment.Enabled = false; this.UsingAveraging.Enabled = false; this.UsingAltitudeCorrection.Enabled = false; this.UsingNavAlignment.Enabled = false;
        }
        public void FreeParamsOfStart()
        {
            this.OnlyAlignment.Enabled = true; this.UsingClasAlignment.Enabled = true; this.UsingAveraging.Enabled = true; this.UsingAltitudeCorrection.Enabled = true; this.UsingNavAlignment.Enabled = true;
        }

        public void LockTheWayOfStart()
        {
            this.OnlyIntegrating.Enabled = false; this.feedbackExist.Enabled = false; this.Odometr_SINS_case.Enabled = false; this.EstimateExist.Enabled = false;
        }
        public void FreeTheWayOfStart()
        {
            this.OnlyIntegrating.Enabled = true; this.feedbackExist.Enabled = true; this.Odometr_SINS_case.Enabled = true; this.EstimateExist.Enabled = true;
        }

        public void LockNumbOfMeasures()
        {
            this.UseOdo_In_Oz.Enabled = false;
        }
        public void FreeNumbOfMeasures()
        {
            this.UseOdo_In_Oz.Enabled = true;
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
            this.Use_Odo_Distance.Checked = false; this.Use_Each_Odo_Measure.Checked = false; this.Use_Constant_Constraint.Checked = false;
            this.Use_Const_dV_Constraint.Checked = false; this.Use_Const_dV_Constraint.Checked = false; this.Use_dV_by_F_Constraint.Checked = false;
            this.Use_Const_Freq.Checked = false; this.Use_Only_Stops.Checked = false;
            this.usingSNS.Checked = false; this.Use_Integral_of_Fx_2.Checked = false;
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

                this.AccuracyClass_NoErr.Enabled = true;
                this.AccuracyClass_0_02grph.Enabled = true;
                this.AccuracyClass_0_2_grph.Enabled = true;
                this.AccuracyClass_2_0_grph.Enabled = true;
            }
            else
            {
                CheckedFalseDataIn();

                this.AccuracyClass_NoErr.Enabled = false;
                this.AccuracyClass_0_02grph.Enabled = false;
                this.AccuracyClass_0_2_grph.Enabled = false;
                this.AccuracyClass_2_0_grph.Enabled = false;
            }

            this.Imitator_Telemetric.Enabled = true;
        }
        private void Imitator_Telemetric_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Imitator_Telemetric.Checked == true)
            {
                this.AccuracyClass_NoErr.Enabled = true;
                this.AccuracyClass_0_02grph.Enabled = true;
                this.AccuracyClass_0_2_grph.Enabled = true;
                this.AccuracyClass_2_0_grph.Enabled = true;
            }
            else
            {
                this.AccuracyClass_NoErr.Enabled = false;
                this.AccuracyClass_0_02grph.Enabled = false;
                this.AccuracyClass_0_2_grph.Enabled = false;
                this.AccuracyClass_2_0_grph.Enabled = false;
            }
        }

        private void Azimut_14_08_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Azimut_14_08_2012.Checked == true)
            {
                CheckedTrueDataIn(); 
                this.Azimut_14_08_2012.Enabled = true;
            }
            else CheckedFalseDataIn();
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

        private void Kama_04_09_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Kama_04_09_2012.Checked == true)
            {
                CheckedTrueDataIn(); 
                this.Kama_04_09_2012.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void ktn004_21_03_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.ktn004_21_03_2012.Checked == true)
            {
                CheckedTrueDataIn(); 
                this.ktn004_21_03_2012.Enabled = true;
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
            if (this._16_09_13_TLM_1zaezd.Checked == true)
            {
                CheckedTrueDataIn();
                this._16_09_13_TLM_1zaezd.Enabled = true;
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
            if (this.Ungolonom_model.Checked == true)
                return;

            if (this.feedbackExist.Checked == true)
            {
                LockTheWayOfStart(); this.feedbackExist.Enabled = true;
                FreeTypesOfCorrection();
                FreeNumbOfMeasures();
                this.usingSNS.Enabled = true;

                this.WeakConnect.Enabled = false;
                this.ModifWeakConnect.Enabled = false;

                //this.DoFeedBackKappa.Checked = true;
                //this.DoFeedBackDeltaFW.Checked = true;
            }
            else
            {
                UnCheckTypesOfCorrection();
                FreeTheWayOfStart();
                LockTypesOfCorrection();
                LockNumbOfMeasures();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;

                this.WeakConnect.Enabled = true;
                this.ModifWeakConnect.Enabled = true;

                //this.DoFeedBackKappa.Checked = false;
                //this.DoFeedBackDeltaFW.Checked = false;
            }
        }

        private void EstimateExist_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Ungolonom_model.Checked == true)
                return;
            if (this.EstimateExist.Checked == true)
            {
                LockTheWayOfStart(); this.EstimateExist.Enabled = true;
                FreeTypesOfCorrection();
                FreeNumbOfMeasures();
                this.usingSNS.Enabled = true;

                this.WeakConnect.Enabled = false;
                this.ModifWeakConnect.Enabled = false;
            }
            else
            {
                UnCheckTypesOfCorrection();
                FreeTheWayOfStart();
                LockTypesOfCorrection();
                LockNumbOfMeasures();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;

                this.WeakConnect.Enabled = true;
                this.ModifWeakConnect.Enabled = true;
            }
        }

        private void Odometr_SINS_case_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Odometr_SINS_case.Checked == true)
            {
                //LockTheWayOfStart(); this.Odometr_SINS_case.Enabled = true;
                //LockNumbOfMeasures();
                this.iMx_kappa_1_3_ds.Checked = true;
                //this.flag_autonomous_dinamic_mode.Checked = true;
                this.WeakConnect.Enabled = true;
                this.ModifWeakConnect.Enabled = true;
            }
            else
            {
                FreeTheWayOfStart();
                this.flag_autonomous_dinamic_mode.Checked = false;
                this.iMx_kappa_1_3_ds.Checked = false;
                this.WeakConnect.Enabled = false;
                this.ModifWeakConnect.Enabled = false;
            }
        }
        private void WeakConnect_CheckedChanged(object sender, EventArgs e)
        {
            if (this.WeakConnect.Checked == true)
            {
                FreeTypesOfCorrection();
                FreeNumbOfMeasures();
                this.ModifWeakConnect.Enabled = false;
                this.usingSNS.Enabled = true;
                this.Main_Block_Click_new.Enabled = true;
            }
            else
            {
                LockTypesOfCorrection();
                LockNumbOfMeasures();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;
                this.ModifWeakConnect.Enabled = true;
            }
        }
        private void ModifWeakConnect_CheckedChanged(object sender, EventArgs e)
        {
            if (this.ModifWeakConnect.Checked == true)
            {
                FreeTypesOfCorrection();
                FreeNumbOfMeasures();
                this.WeakConnect.Enabled = false;
                this.usingSNS.Enabled = true;
                this.Main_Block_Click_new.Enabled = true;
            }
            else
            {
                LockTypesOfCorrection();
                LockNumbOfMeasures();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;
                this.WeakConnect.Enabled = true;
            }
        }
        



        //---Режимы коррекции---//
        private void CheckedTrueTypesOfCorrection()
        {
            LockTypesOfCorrection();
            FreeNumbOfMeasures();
            this.Main_Block_Click_new.Enabled = true;
        }
        private void CheckedFlaseTypesOfCorrection()
        {
            FreeTypesOfCorrection();
            LockNumbOfMeasures();
            this.Main_Block_Click_new.Enabled = false;
        }

        private void Use_Const_Freq_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_Const_Freq.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Use_Const_Freq.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }

        private void Use_Only_Stops_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_Only_Stops.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Use_Only_Stops.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }

        private void Use_Each_Odo_Measure_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_Each_Odo_Measure.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Use_Each_Odo_Measure.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }

        private void Ungolonom_model_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Ungolonom_model.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Ungolonom_model.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();

            if (this.Ungolonom_model.Checked == true)
            {
                this.feedbackExist.Checked = true; this.feedbackExist.Enabled = true;
                this.EstimateExist.Checked = false; this.EstimateExist.Enabled = false;
            }
        }


        private void Use_Constant_Constraint_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_Constant_Constraint.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Use_Constant_Constraint.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }

        private void Use_Const_dV_Constraint_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_Const_dV_Constraint.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Use_Const_dV_Constraint.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }

        private void Use_dV_by_F_Constraint_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_dV_by_F_Constraint.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Use_dV_by_F_Constraint.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }

        private void Use_Odo_Distance_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_Odo_Distance.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.UseOdo_In_Oz.Enabled = false;
                this.flag_not_use_kns.Checked = true;
                this.Use_Odo_Distance.Enabled = true;
            }
            else
            {
                this.flag_not_use_kns.Checked = false;
                CheckedFlaseTypesOfCorrection();
            }
        }
        private void Use_Integral_of_Fx_2_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_Integral_of_Fx_2.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.UseOdo_In_Oz.Enabled = false;
                this.Use_Integral_of_Fx_2.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
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
            if (this.iMx_r_3_dV_3.Checked == true)
            {
                if (this.Odometr_SINS_case.Checked == true)
                {
                    this.iMx_r_odo_3.Checked = true;
                    this.iMx_r_odo_3.Enabled = false;
                }
            }
            if (this.iMx_r_3_dV_3.Checked == false)
            {
                if (this.Odometr_SINS_case.Checked == true)
                {
                    this.iMx_r_odo_3.Checked = false;
                    this.iMx_r_odo_3.Enabled = true;
                }
            }
        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {

        }

        

        

        

        

        

        

        

        

        

        

        

        


    }
}
