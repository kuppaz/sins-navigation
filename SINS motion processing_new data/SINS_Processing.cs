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

        int iMx_Vertical = SimpleData.iMx_Vertical = 25;
        int iMq_Vertical = SimpleData.iMq_Vertical = SimpleData.iMx_Vertical;

        StreamReader myFile;

        ParamToStart ParamStart = new ParamToStart();
        SINS_State SINSstate, SINSstate_OdoMod;
        Kalman_Vars KalmanVars;
        Proc_Help ProcHelp;

        int value_iMx_dV_12, value_iMx_alphaBeta, value_iMx_Nu0, value_iMx_f0_12, value_iMx_f0_3, value_iMx_dr3, value_iMx_dV3, value_iMx_r_odo_3;
        int value_iMx_r3_dV3 = 0, value_iMx_r_odo_12 = 0, value_iMx_kappa_13_ds = 0, Vertical_kappa1, Vertical_kappa3Scale, Vertical_alphaBeta, Vertical_nu0, Vertical_f0_12, Vertical_f0_3, Vertical_rOdo3;
        int noiseParam_LastCountForRead = 0, noiseParam_StartCountForRead = 0;
        bool iMx_r3_dV3, iMx_kappa_13_ds;

        string GlobalPrefixTelemetric = "";

        public static StreamWriter GRTV_output = new StreamWriter(SimpleData.PathOutputString + "S_GRTV_output.txt");


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
            this.Single_Navigation_Processing(false, 0.0, 0.0);

            GRTV_output.Close();
            this.Close();
        }




        private void NoiseParamsScanning_Click(object sender, EventArgs e)
        {
            double NoiseVel_start = 1E-4, 
                   NoiseVel_end = 1E-1, 
                   NoiseVel_multpl = 5.0;

            double NoiseAngl_start = 1E-4, 
                   NoiseAngl_end = 1E-1,
                   NoiseAngl_multpl = 5.0;

            for (double Cicle_Noise_Velocity = NoiseVel_start; Cicle_Noise_Velocity <= NoiseVel_end; Cicle_Noise_Velocity = Cicle_Noise_Velocity * NoiseVel_multpl)
            {
                for (double Cicle_Noise_Angular = NoiseAngl_start; Cicle_Noise_Angular <= NoiseAngl_end; Cicle_Noise_Angular = Cicle_Noise_Angular * NoiseAngl_multpl)
                {
                    this.Single_Navigation_Processing(true, Cicle_Noise_Velocity, Cicle_Noise_Angular);
                }
            }


            int j = 0;
            string datastring = "";
            string[] dataArray;

            for (double Cicle_Noise_Velocity = NoiseVel_start; Cicle_Noise_Velocity <= NoiseVel_end; Cicle_Noise_Velocity = Cicle_Noise_Velocity * NoiseVel_multpl)
            {
                for (double Cicle_Noise_Angular = NoiseAngl_start; Cicle_Noise_Angular <= NoiseAngl_end; Cicle_Noise_Angular = Cicle_Noise_Angular * NoiseAngl_multpl)
                {
                    j++;
                    StreamReader Cicle_Debag_Solution = new StreamReader(SimpleData.PathOutputString + "Debaging//Solution_"
                        + Cicle_Noise_Angular.ToString("E2") + "_" + Cicle_Noise_Velocity.ToString("E2") + ".txt");

                    if (j == 1)
                    {
                        for (; ; )
                        {
                            string tmpstr = Cicle_Debag_Solution.ReadLine();
                            if (Cicle_Debag_Solution.EndOfStream) break;
                            noiseParam_LastCountForRead++;
                        }
                        Cicle_Debag_Solution.Close();
                        Cicle_Debag_Solution = new StreamReader(SimpleData.PathOutputString + "Debaging//Solution_"
                            + Cicle_Noise_Angular.ToString("E2") + "_" + Cicle_Noise_Velocity.ToString("E2") + ".txt");

                        FileStream fs = File.Create(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_HorizError.csv");
                        fs.Close();
                        FileStream fs2 = File.Create(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_Height.csv");
                        fs2.Close();
                    }

                    StreamWriter NoiseParam_CicleScanning_HorizError = new StreamWriter(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_HorizError_.csv");
                    StreamWriter NoiseParam_CicleScanning_Height = new StreamWriter(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_Height_.csv");

                    StreamReader NoiseParam_CicleScanning_HorizError_Read = new StreamReader(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_HorizError.csv");
                    StreamReader NoiseParam_CicleScanning_Height_Read = new StreamReader(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_Height.csv");


                    double[] minDataArray = new double[10];
                    double[] maxDataArray = new double[10];
                    for (int y = 0; y < minDataArray.Length; y++)
                        minDataArray[y] = 1000000.0;

                    for (int i = 0; i <= this.noiseParam_LastCountForRead; i++)
                    {
                        datastring = Cicle_Debag_Solution.ReadLine();
                        dataArray = datastring.Split(' ');


                        string str = "";
                        if (j == 1)
                        {
                            if (i == 0)
                                NoiseParam_CicleScanning_HorizError.WriteLine("Time;" + Cicle_Noise_Velocity.ToString("E2") + "_" + Cicle_Noise_Angular.ToString("E2"));
                            str = dataArray[0] + ";" + dataArray[2];
                        }
                        else
                        {
                            if (i == 0)
                                NoiseParam_CicleScanning_HorizError.WriteLine(NoiseParam_CicleScanning_HorizError_Read.ReadLine()
                                    + ";" + Cicle_Noise_Velocity.ToString("E2") + "_" + Cicle_Noise_Angular.ToString("E2"));
                            str = NoiseParam_CicleScanning_HorizError_Read.ReadLine() + ";" + dataArray[2];
                        }
                        if (Convert.ToDouble(dataArray[2]) < minDataArray[2])
                            minDataArray[2] = Convert.ToDouble(dataArray[2]);
                        if (Convert.ToDouble(dataArray[2]) > maxDataArray[2])
                            maxDataArray[2] = Convert.ToDouble(dataArray[2]);
                        NoiseParam_CicleScanning_HorizError.WriteLine(str);


                        str = "";
                        if (j == 1)
                        {
                            if (i == 0)
                                NoiseParam_CicleScanning_Height.WriteLine("Time;" + Cicle_Noise_Velocity.ToString("E2") + "_" + Cicle_Noise_Angular.ToString("E2"));
                            str = dataArray[0] + ";" + dataArray[1];
                        }
                        else
                        {
                            if (i == 0)
                                NoiseParam_CicleScanning_Height.WriteLine(NoiseParam_CicleScanning_Height_Read.ReadLine() + ";" 
                                    + Cicle_Noise_Velocity.ToString("E2") + "_" + Cicle_Noise_Angular.ToString("E2"));
                            str = NoiseParam_CicleScanning_Height_Read.ReadLine() + ";" + dataArray[1];
                        }
                        if (Convert.ToDouble(dataArray[1]) < minDataArray[1])
                            minDataArray[1] = Convert.ToDouble(dataArray[1]);
                        if (Convert.ToDouble(dataArray[1]) > maxDataArray[1])
                            maxDataArray[1] = Convert.ToDouble(dataArray[1]);
                        NoiseParam_CicleScanning_Height.WriteLine(str);
                    }

                    if (j == 1)
                    {
                        NoiseParam_CicleScanning_Height.WriteLine("Min;" + minDataArray[1]);
                        NoiseParam_CicleScanning_Height.WriteLine("Max;" + maxDataArray[1]);
                        NoiseParam_CicleScanning_Height.WriteLine("Diff;" + (maxDataArray[1] - minDataArray[1]));

                        NoiseParam_CicleScanning_HorizError.WriteLine("Min;" + minDataArray[2]);
                        NoiseParam_CicleScanning_HorizError.WriteLine("Max;" + maxDataArray[2]);
                        NoiseParam_CicleScanning_HorizError.WriteLine("Diff;" + (maxDataArray[2] - minDataArray[2]));
                    }
                    else
                    {
                        NoiseParam_CicleScanning_Height.WriteLine(NoiseParam_CicleScanning_Height_Read.ReadLine() + ";" + minDataArray[1]);
                        NoiseParam_CicleScanning_Height.WriteLine(NoiseParam_CicleScanning_Height_Read.ReadLine() + ";" + maxDataArray[1]);
                        NoiseParam_CicleScanning_Height.WriteLine(NoiseParam_CicleScanning_Height_Read.ReadLine() + ";" + (maxDataArray[1] - minDataArray[1]));

                        NoiseParam_CicleScanning_HorizError.WriteLine(NoiseParam_CicleScanning_HorizError_Read.ReadLine() + ";" + minDataArray[2]);
                        NoiseParam_CicleScanning_HorizError.WriteLine(NoiseParam_CicleScanning_HorizError_Read.ReadLine() + ";" + maxDataArray[2]);
                        NoiseParam_CicleScanning_HorizError.WriteLine(NoiseParam_CicleScanning_HorizError_Read.ReadLine() + ";" + (maxDataArray[2] - minDataArray[2]));
                    }

                    NoiseParam_CicleScanning_HorizError.Close();
                    NoiseParam_CicleScanning_HorizError_Read.Close();
                    NoiseParam_CicleScanning_Height.Close();
                    NoiseParam_CicleScanning_Height_Read.Close();

                    File.Delete(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_HorizError.csv");
                    File.Delete(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_Height.csv");
                    File.Move(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_HorizError_.csv", SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_HorizError.csv");
                    File.Move(SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_Height_.csv", SimpleData.PathOutputString + "Debaging//NoiseParam_CicleScanning_Height.csv");


                    Cicle_Debag_Solution.Close();
                    File.Delete(SimpleData.PathOutputString + "Debaging//Solution_"
                        + Cicle_Noise_Angular.ToString("E2") + "_" + Cicle_Noise_Velocity.ToString("E2") + ".txt");
                }
            }


            GRTV_output.Close();
            this.Close();
        }









        public void Single_Navigation_Processing(bool NoiseParamScanning, double Cicle_Noise_Velocity, double Cicle_Noise_Angular)
        {
            int l = 0;

            //------------------------------------------------------------------------
            //------------------------------------------------------------------------

            //---для имитатора---
            ParamStart.Imitator_NoiseModelFlag = true; // false - Брать значения шума с выставки, true - задаваемые ниже
            ParamStart.Imitator_Noise_Vel = 3E-4;
            ParamStart.Imitator_Noise_Angl = 3E-6;

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

            //------------------------------------------------------------------------
            //------------------------------------------------------------------------



            this.DefineDimentionOfErrorVector();                                                            //---формирование размерности вектора ошибок---//
            this.DefineClassElementAndFlags();
            this.SelectDataIn();                                                                            //---выбор входного набора данных---//


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

                    if (time > 51450)
                        break;
                }
            }
            int LastCountForRead = SINSstate.LastCountForRead;
            myFile.Close();

            this.SelectDataIn();
            //=== ===


            //---Инициализация начальных условий при отсутствии выставки---//
            Parameters.StartSINS_Parameters(SINSstate, SINSstate_OdoMod, KalmanVars, ParamStart, ProcHelp);



            ProcHelp.AlgnCnt = 0;
            if (SINSstate.Global_file == "Imitator_Data") ProcHelp.AlgnCnt = 9000;
            if (SINSstate.Global_file == "Imitator_Telemetric") ProcHelp.AlgnCnt = 100;

            if (SINSstate.Global_file == "Azimut_15.08.2012") ProcHelp.AlgnCnt = 40000;
            //--- 1. Хорошее решение при коррекции по V как в режиме Odo+SINS так и в режиме SINS+Odo

            if (SINSstate.Global_file == "Azimut_24.08.2012") ProcHelp.AlgnCnt = 45000;
            //--- 1. Хорошее решение Odo+SINS, но плохое SINS+Odo; Также плохие измерения GPS, поэтому их использовать не стоит

            if (SINSstate.Global_file == "Azimut_29.08.2012") ProcHelp.AlgnCnt = 35000;
            //--- 1. Хорошее решение Odo+SINS, но плохое SINS+Odo; Также плохие измерения GPS, поэтому их использовать не стоит

            if (SINSstate.Global_file == "ktn004_15.03.2012") ProcHelp.AlgnCnt = 48000;
            //--- 1. Решение по Odo+SINS немного лучше. Хороший эксперимент для сравнения моделей.
            //--- 2. Добавил деление на SINSstate.OdoLimitMeasuresNum при вычислении KalmanVars.OdoNoise_V

            if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2") ProcHelp.AlgnCnt = 10300;
            //--- 1. Решение по Odo+SINS немного лучше.

            if (SINSstate.Global_file == "AZIMUT_T_2013_10_18_12_55") ProcHelp.AlgnCnt = 22000;
            //--- 1. Поездка по прямой от одной до друго точки туда-сюда. Как то плохо пока, не настроено.

            if (SINSstate.Global_file == "Azimut_514_08Nov2013_11_15") ProcHelp.AlgnCnt = 95000;

            if (SINSstate.Global_file == "GRTVout_GCEF_format_030715выезд") ProcHelp.AlgnCnt = 4300;
            if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд завод)") ProcHelp.AlgnCnt = 5000;
            if (SINSstate.Global_file == "GRTVout_GCEF_format (070715выезд куликовка)") ProcHelp.AlgnCnt = 7500;

            if (SINSstate.Global_file == "Saratov_run_2014_07_23")
            {
                ProcHelp.AlgnCnt = 27320;
                if (this.SaratAlignStart.Checked == true || this.SaratENDStart.Checked == true)
                    ProcHelp.AlgnCnt = SINSstate.LastCountForRead;
            }



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

                if (SINSstate.flag_UsingNavAlignment == true && ProcHelp.AlgnCnt != 0)
                    l = SINSAlignment_Navigantion.SINS_Alignment_Navigation(ProcHelp, SINSstate, SINSstate2, SINSstate_OdoMod, myFile, KalmanVars, GRTV_output);
                else if (SINSstate.flag_UsingClasAlignment == true && ProcHelp.AlgnCnt != 0)
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
                        SINSstate.stdNu = 0.02; //град/час
                    }
                if (SINSstate.flag_AccuracyClass_0_2_grph)
                    for (int j = 0; j < 3; j++)
                    {
                        SINSstate.stdF[j] = 1E-4 * 9.81; //далее умножается G
                        SINSstate.stdNu = 0.2; //град/час
                    }
                if (SINSstate.flag_AccuracyClass_2_0_grph)
                    for (int j = 0; j < 3; j++)
                    {
                        SINSstate.stdF[j] = 1E-3 * 9.81; //далее умножается G
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


            //---Инициализация начальной матрицы ковариации---
            if (SINSstate.flag_Odometr_SINS_case == true)
                Odometr_SINS.InitOfCovarianceMatrixes(SINSstate, KalmanVars);
            else
                SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);

            
            //--- Если запустили циклический подбор параметров шумов ---//
            if (NoiseParamScanning)
            {
                for (int j = 0; j < 3; j++)
                {
                    KalmanVars.Noise_Vel[j] = Cicle_Noise_Velocity;
                    KalmanVars.Noise_Angl[j] = Cicle_Noise_Angular;
                }
            }



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
            SINSstate.stdF[0] = Convert.ToDouble(dataArray[9]) *9.81;
            SINSstate.stdF[1] = Convert.ToDouble(dataArray[11]) *9.81;
            SINSstate.stdNu = Convert.ToDouble(dataArray[15]);

            SINSstate.stdF_Oz[0] = Convert.ToDouble(dataArray[39]) *9.81;
            SINSstate.stdF_Oz[1] = Convert.ToDouble(dataArray[41]) *9.81;
            SINSstate.stdF_Oz[2] = Convert.ToDouble(dataArray[43]) *9.81;
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
            double Heading_tmpDevide = 1.0;

            double Heading_addError = SINSstate.stdNu * SimpleData.ToRadian / 3600.0 / (SimpleData.U * Math.Cos(SINSstate.Latitude)),
                Pitch_addError = (SINSstate.stdF[1] / 9.81 * Math.Cos(SINSstate.Heading) + SINSstate.stdF[0] / 9.81 * Math.Sin(SINSstate.Heading)),
                Roll_addError = -(-SINSstate.stdF[1] / 9.81 * Math.Sin(SINSstate.Heading) + SINSstate.stdF[0] / 9.81 * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch)
                ;
            SINSstate.Heading = Convert.ToDouble(dataArray[29]) + Heading_addError / Heading_tmpDevide;
            SINSstate.Pitch = Convert.ToDouble(dataArray[33]) + Pitch_addError / Heading_tmpDevide;
            SINSstate.Roll = Convert.ToDouble(dataArray[31]) + Roll_addError / Heading_tmpDevide;

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

            SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude);
            SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);

            SINSstate_OdoMod.A_sx0 = SimpleOperations.A_sx0(SINSstate_OdoMod);
            SINSstate_OdoMod.A_x0s = SINSstate_OdoMod.A_sx0.Transpose();
            SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Longitude);
            SINSstate_OdoMod.A_nx0 = SINSstate_OdoMod.A_x0n.Transpose();
            SINSstate_OdoMod.A_nxi = SimpleOperations.A_ne(SINSstate_OdoMod.Time, SINSstate_OdoMod.Longitude_Start);
            SINSstate_OdoMod.AT = Matrix.Multiply(SINSstate_OdoMod.A_sx0, SINSstate_OdoMod.A_x0n);
            SINSstate_OdoMod.AT = Matrix.Multiply(SINSstate_OdoMod.AT, SINSstate_OdoMod.A_nxi);

            SINSstate_OdoMod.R_e = SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);
            SINSstate_OdoMod.R_n = SimpleOperations.RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Altitude);


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
                value_iMx_kappa_13_ds = SimpleData.iMx;
                iMx = SimpleData.iMx += 3;
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
            value_iMx_f0_3 = SimpleData.iMx;
            iMx = SimpleData.iMx += 1;

            // ---------- dR_ODO_3 ----------//
            if (this.Odometr_SINS_case.Checked)
            {
                if (this.iMx_r_odo_3.Checked)
                {
                    value_iMx_r_odo_3 = SimpleData.iMx;
                    iMx = SimpleData.iMx += 1;
                }
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
                {
                    iMq = SimpleData.iMq += 2;

                    if (this.iMx_r_odo_3.Checked)
                        iMq = SimpleData.iMq += 1;
                }
            }

            // ---------------------------//
            iMq = SimpleData.iMq = SimpleData.iMx;
            // ---------------------------//


            //----------------------------Индексы для сглаживания---------------------------
            if (this.iMSmthd_Is_2.Checked)
                iMxSmthd = SimpleData.iMxSmthd = 2;
            if (this.iMSmthd_Is_4.Checked)
                iMxSmthd = SimpleData.iMxSmthd = 4;
            if (this.iMSmthd_Is_7.Checked)
                iMxSmthd = SimpleData.iMxSmthd = 7;

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
            SINSstate.flag_iMqVarkappa13 = iMqVarkappa13.Checked;
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
            SINSstate.value_iMx_kappa_13_ds = value_iMx_kappa_13_ds;


            //---флаги---
            SINSstate.flag_Autonomous_Solution = this.OnlyIntegrating.Checked;
            SINSstate.flag_UsingAvegering = this.UsingAveraging.Checked;
            SINSstate.flag_UsingAltitudeCorrection = this.UsingAltitudeCorrection.Checked;
            SINSstate.flag_Using_SNS = this.usingSNS.Checked;
            SINSstate.flag_FeedbackExist = this.feedbackExist.Checked;
            SINSstate.flag_EstimateExist = this.EstimateExist.Checked;
            SINSstate.flag_UsingClasAlignment = this.UsingClasAlignment.Checked;
            SINSstate.flag_UsingNavAlignment = this.UsingNavAlignment.Checked;
            SINSstate.flag_Odometr_SINS_case = this.Odometr_SINS_case.Checked;
            SINSstate.flag_OdoSINSWeakConnect = this.WeakConnect.Checked;
            SINSstate.flag_OdoSINSWeakConnect_MODIF = this.ModifWeakConnect.Checked;
            SINSstate.flag_Using_iMx_r_odo_3 = this.iMx_r_odo_3.Checked;
            SINSstate.flag_UseOnlyStops = this.Use_Only_Stops.Checked;
            SINSstate.flag_OnlyAlignment = this.OnlyAlignment.Checked;

            SINSstate.flag_Smoothing = this.flag_Smoothing.Checked;
            SINSstate.flag_iMSmthd_Is_2_plus_Odo = this.iMSmthd_Is_2_plus_Odo.Checked;

            SINSstate.flag_NotUse_ZUPT = this.flag_not_use_zupt.Checked;
            SINSstate.flag_using_slippage = this.flag_using_slippage.Checked;
            SINSstate.flag_using_Checkpotints = this.flag_using_Checkpotints.Checked;
            SINSstate.flag_using_GoCalibrInCP = this.flag_using_GoCalibrInCP.Checked;
            SINSstate.add_velocity_to_position = this.add_velocity_to_position.Checked;
            SINSstate.flag_UseAlgebraDrift = this.flag_UseAlgebraDrift.Checked;
            SINSstate.flag_OdoModelOnlyCP = this.OdoModelOnlyCP.Checked;
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

        }

        public void SelectDataIn()
        {
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

            if (GRTVout_GCEF_format_030715.Checked == true)
            {
                myFile = new StreamReader(SimpleData.PathInputString + "GRTVout_GCEF_format (030715выезд).txt");
                SINSstate.Global_file = "GRTVout_GCEF_format_030715выезд";
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



            if (topo_saratov.Checked == true)
            {
                if (SINSstate.FreqOutput == 10)
                    SINSstate.FreqOutput = 100;

                if (this.SaratovFullRun.Checked == false && !this.SaratAlignStart.Checked && !this.SaratENDStart.Checked)
                    myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23.dat");

                else if (this.SaratAlignStart.Checked)
                    myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23_Align.dat");
                //myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23_Align_FULL.dat");
                else if (this.SaratENDStart.Checked)
                {
                    SINSstate.Saratov_run_Final = true;
                    myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23_finalAlign.dat");
                }
                else
                {
                    if (this.saratovOdoVirtual_1.Checked)
                        myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23_full_odo_1.dat");
                    else if (this.saratovOdoVirtual_2.Checked)
                        myFile = new StreamReader(SimpleData.PathInputString + "Saratov_run_2014_07_23_full_odo_2.dat");
                    else if (this.saratovWithVirtualMarksInside.Checked)
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
            this.GRTVout_GCEF_format_030715.Enabled = false;
            this.GRTVout_GCEF_format_070715_zavod.Enabled = false;
            this.GRTVout_GCEF_format_070715_kulikova.Enabled = false;
        }
        public void FreeInData()
        {
            this.Azimut_15_08_2012.Enabled = true; this.Azimut_24_08_2012.Enabled = true; this.Azimut_29_08_2012.Enabled = true;
            this.ktn004_15_03_2012.Enabled = true; this.Imitator_Data.Enabled = true; this.Azimuth_minsk_race_4_3to6to2.Enabled = true;
            this.topo_saratov.Enabled = true; this.AZIMUT_T_12_32_16_09_13_TLM_2z.Enabled = true; this.Azimut_514_08Nov2013_11_15.Enabled = true;
            this.GRTVout_GCEF_format_030715.Enabled = true;
            this.GRTVout_GCEF_format_070715_zavod.Enabled = true;
            this.GRTVout_GCEF_format_070715_kulikova.Enabled = true;
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
        private void GRTVout_GCEF_format_030715_CheckedChanged(object sender, EventArgs e)
        {
            if (this.GRTVout_GCEF_format_030715.Checked == true)
            {
                CheckedTrueDataIn();
                this.GRTVout_GCEF_format_030715.Enabled = true;
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

        private void iMq_eq_iMx_CheckedChanged(object sender, EventArgs e)
        {
            if (this.iMq_eq_iMx.Checked == true)
            {
                this.iMqKappa.Checked = true;
                this.iMqDeltaR.Checked = true;
                this.iMqDeltaF.Checked = true;
                this.iMqDeltaNu.Checked = true;
                this.iMqDeltaRodo.Checked = true;
                this.iMqVarkappa13.Checked = true;
            }
            else
            {
                this.iMqKappa.Checked = false;
                this.iMqDeltaF.Checked = false;
                this.iMqDeltaNu.Checked = false;
                this.iMqVarkappa13.Checked = false;

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
