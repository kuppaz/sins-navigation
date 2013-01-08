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
using SINS_motion_processing;

 

namespace SINS_motion_processing_new_data
{
    public partial class SINS_Processing : Form
    {
        int iMx = SimpleData.iMx = 25;
        int iMq = SimpleData.iMq = SimpleData.iMx;
        int iMz = SimpleData.iMz = 9;


        string datastringCovariance;
        double V_odo;
        int tt = 0, GK_lap = 0;
        double[,] distance_GK_Sarat = new double[5,46];

        double for_min_pos_error = 1000.0, tempCount;
        string for_min_pos_error_string;

        SINS_Alignment SINSAlignment = new SINS_Alignment();

        StreamReader myFile;

        StreamWriter Nav_FeedbackSolution = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_FeedbackSolution.txt");
        StreamWriter Nav_Errors = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_Errors.txt");
        StreamWriter Nav_Autonomous = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_Autonomous.txt");
        StreamWriter Nav_EstimateSolution = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_EstimateSolution.txt");
        StreamWriter Nav_StateErrorsVector = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_StateErrorsVector.txt");
        StreamWriter ForHelp = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output//ForHelp.txt");
        StreamWriter Nav_vert_chan_test = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Nav_vert_chan_test.txt");

        StreamWriter Dif_GK = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output//Dif_GK.txt");

        SINS_State SINSstate;
        SINS_State SINSstate2;
        SINS_State SINSstate_OdoMod;
        Kalman_Vars KalmanVars;
        Parameters Params;
        ParamsForModel OdoModel;
        Proc_Help ProcHelp;
        

        //[DllImport("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//Motion Imitator//SINS_Aligment//SINS_Aligment.dll")]
        //unsafe private static extern int main();

        public SINS_Processing()
        {
            InitializeComponent();
        }

        // ТРАТАТА - свн выдался на ура!!!

        private void Main_Block_Click_new_Click(object sender, EventArgs e)
        {
            DateTime datetime_start = DateTime.Now;

            iMx = SimpleData.iMx = 13;
            iMq = SimpleData.iMq = SimpleData.iMx;

            if (this.Odometr_SINS.Checked)
                iMx_r_odo_1_2.Checked = true;

            int value_iMx_r3_dV3 = 0, value_iMx_r_odo_12 = 0, value_iMx_kappa_13_ds = 0;

            bool iMx_r3_dV3 = iMx_r_3_dV_3.Checked;
            if (iMx_r3_dV3)
            {
                value_iMx_r3_dV3 = iMx;
                iMx = SimpleData.iMx += 2;
            }

            bool iMx_r_odo_12 = iMx_r_odo_1_2.Checked;
            if (iMx_r_odo_12)
            {
                value_iMx_r_odo_12 = iMx;
                iMx = SimpleData.iMx += 2;
            }

            bool iMx_kappa_13_ds = iMx_kappa_1_3_ds.Checked;
            if (iMx_kappa_13_ds)
            {
                value_iMx_kappa_13_ds = iMx;
                iMx = SimpleData.iMx += 3;
            }

            iMq = SimpleData.iMq = SimpleData.iMx;

             SINSstate = new SINS_State();
             SINSstate2 = new SINS_State();
             SINSstate_OdoMod = new SINS_State();
             KalmanVars = new Kalman_Vars();
             Params = new Parameters();
             OdoModel = new ParamsForModel();
             ProcHelp = new Proc_Help();

            SINSstate.iMx_r3_dV3 = iMx_r3_dV3;
            SINSstate.iMx_r_odo_12 = iMx_r_odo_12;
            SINSstate.iMx_kappa_13_ds = iMx_kappa_13_ds;
            if (iMx_r3_dV3)
                SINSstate.value_iMx_r3_dV3 = value_iMx_r3_dV3;
            if (iMx_r_odo_12)
                SINSstate.value_iMx_r_odo_12 = value_iMx_r_odo_12;
            if (iMx_kappa_13_ds)
                SINSstate.value_iMx_kappa_13_ds = value_iMx_kappa_13_ds;


            

            if (Saratov_01_11_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//SarOEG_20121213_refactor.dat");
                SINSstate.Global_file = "Saratov_01.11.2012";
            }
            else if (Azimut_14_08_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//AzimutB_210530_Other_120814_Autolab_10-31-26_2.dat");
                SINSstate.Global_file = "Azimut_14.08.2012";
            }
            else if (Azimut_15_08_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//AzimutB_210530_Other_120815_Autolab_DPC_100Hz_14-40-04.dat");
                SINSstate.Global_file = "Azimut_15.08.2012";
            }
            else if (Azimut_24_08_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//AzimutB_210530_Other_120824_Autolab_Circle_AzimutKama_12-07-25.dat");
                SINSstate.Global_file = "Azimut_24.08.2012";
            }
            else if (Azimut_29_08_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//AzimutB_210530_Other_120829_Autolab_Circle_09-21-35.dat");
                SINSstate.Global_file = "Azimut_29.08.2012";
            }
            else if (Kama_04_09_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//Kama_120904_04-Sep-2012,14-44-11.dat");
                SINSstate.Global_file = "Kama_04.09.2012";
                for (int i = 0; i < 14649; i++ ) myFile.ReadLine();
            }

            else if (ktn004_21_03_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//ktn004_marsh16_repeat_21-Mar-2012,17-21-07_dat.dat");
                SINSstate.Global_file = "ktn004_21.03.2012";
            }
            else if (ktn004_15_03_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//ktn004_static4hour_marsh_sns_15-Mar-2012,16-29-45_dat.dat");
                SINSstate.Global_file = "ktn004_15.03.2012";
            }
            else if (povorot_12_09_2012.Checked == true)
            {
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//povorot_12-Sep-2012,13-26-38_dat.dat");
                SINSstate.Global_file = "povorot_12.09.2012";
            }


            Parameters.StartSINS_Parameters(SINSstate, SINSstate_OdoMod, KalmanVars, Params, ProcHelp); //---Инициализация начальных условий при отсутствии выставки---//
            SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);     //---Инициализация ковариационных матриц матриц вектора ошибок---//

            //KalmanVars.OdoNoise = 1.0;
            //SINSstate.Azimth = Math.PI / 4.0;

            //Для определения углов методом подбора минимального максимального расстояния от начальной позиции
            //if (false)
            //{
            //    myFile.Close(); Nav_FeedbackSolution.Close(); Nav_EstimateSolution.Close(); Nav_StateErrorsVector.Close();
            //    Alignment_Errors.Close(); Alignment_SINSstate.Close(); Alignment_Corrected_State.Close(); Alignment_StateErrorsVector.Close(); Alignment_avg_rougth.Close();

            //    AngleHandle anglehandle = new AngleHandle();
            //    anglehandle.Main_Block_Click_new_Click();
            //    this.Close();
            //}
            //------------------------------------------------

            int t = 0, l = 0;

            Nav_FeedbackSolution.WriteLine("time  count  OdoStepCount  OdoSpeed  Latitude  Longitude  Altitude LatitudeSNS LongitudeSNS  AltitudeSNS  SpeedSNS  V_x1  V_x2  V_x3  Yaw  Roll  Pitch Correct PositionError PositionError_start");
            Nav_EstimateSolution.WriteLine("count   Latitude Longitude LatitudeSNS LongitudeSNS V_x1  V_x2  V_x3  Correct  Yaw YawCor  Roll RollCor  Pitch PitchCor PositionError V_abs");
            Nav_Errors.WriteLine("dLat  dLong  dV_x1  dV_x2  dV_x3  dHeading  dRoll  dPitch");
            Nav_Autonomous.WriteLine("Time OdoStepCount OdoSpeed Latitude Latitude_SNS  Longitude Longitude_SNS SpeedSNS V_x1  V_x2  V_x3 Yaw  Roll  Pitch PosError PosError_Start Azimth");


            SINSstate.usinganglecorrection = true;     //Не кооректировать углы ориентации
            SINSstate.Autonomous = this.OnlyIntegrating.Checked;
            SINSstate.UsingAvegering = this.UsingAveraging.Checked;
            SINSstate.UsingAltitudeCorrection = this.UsingAltitudeCorrection.Checked;
            SINSstate.usingSNS = this.usingSNS.Checked;
            SINSstate.feedbackExist = this.feedbackExist.Checked;

            SINSstate.UsingClasAlignment = this.UsingClasAlignment.Checked;
            SINSstate.UsingNavAlignment = this.UsingNavAlignment.Checked;

            SINSstate.Odometr_SINS = this.Odometr_SINS.Checked;

            SINSstate.FreqOutput = Convert.ToInt32(this.Output_Freq.Text);


            ProcHelp.AlgnCnt = 0;

            if (SINSstate.Global_file == "Azimut_14.08.2012") ProcHelp.AlgnCnt = 21486;
            if (SINSstate.Global_file == "ktn004_15.03.2012") ProcHelp.AlgnCnt = 48000;
            if (SINSstate.Global_file == "ktn004_21.03.2012") ProcHelp.AlgnCnt = 45000;
            if (SINSstate.Global_file == "Azimut_29.08.2012") ProcHelp.AlgnCnt = 35000;
            if (SINSstate.Global_file == "povorot_12.09.2012") ProcHelp.AlgnCnt = 1500;

            if (SINSstate.Global_file == "Saratov_01.11.2012") ProcHelp.AlgnCnt = 19200;

            

            //---Выставка---
            l = SINS_Alignment.SINSAlignment(ProcHelp, SINSstate, SINSstate2, myFile, KalmanVars);
            ProcHelp.initCount = false;

            if (SINSstate.Global_file == "Saratov_01.11.2012")
            {
                //SINSstate.UsingClasAlignment = false;
                //UsingClasAlignment.Checked = false;
                //ProcHelp.AlgnCnt = 10;

                for (int i = 0; i < 46; i++)
                    ProcHelp.distance_GK_Sarat[i] = 1000.0;

                SINSstate.GK_Latitude[0] = 51.65744354 * SimpleData.ToRadian;				SINSstate.GK_Longitude[0] = 45.91832179 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[1] = 51.65736976 * SimpleData.ToRadian;				SINSstate.GK_Longitude[1] = 45.91801222 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[2] = 51.65727971 * SimpleData.ToRadian;				SINSstate.GK_Longitude[2] = 45.91762382 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[3] = 51.65726450 * SimpleData.ToRadian;				SINSstate.GK_Longitude[3] = 45.91752491 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[4] = 51.65727589 * SimpleData.ToRadian;				SINSstate.GK_Longitude[4] = 45.91745401 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[5] = 51.65730910 * SimpleData.ToRadian;				SINSstate.GK_Longitude[5] = 45.91739258 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[6] = 51.65734559 * SimpleData.ToRadian;				SINSstate.GK_Longitude[6] = 45.91735828 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[7] = 51.65737465 * SimpleData.ToRadian;				SINSstate.GK_Longitude[7] = 45.91727285 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[8] = 51.65737334 * SimpleData.ToRadian;				SINSstate.GK_Longitude[8] = 45.91716680 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[9] = 51.65735419 * SimpleData.ToRadian;				SINSstate.GK_Longitude[9] = 45.91707277 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[10] = 51.65731701 * SimpleData.ToRadian;				SINSstate.GK_Longitude[10] = 45.91687866 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[11] = 51.65727808 * SimpleData.ToRadian;				SINSstate.GK_Longitude[11] = 45.91672552 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[12] = 51.65722199 * SimpleData.ToRadian;				SINSstate.GK_Longitude[12] = 45.91662125 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[13] = 51.65716790 * SimpleData.ToRadian;				SINSstate.GK_Longitude[13] = 45.91655937 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[14] = 51.65711749 * SimpleData.ToRadian;				SINSstate.GK_Longitude[14] = 45.91651203 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[15] = 51.65709286 * SimpleData.ToRadian;				SINSstate.GK_Longitude[15] = 45.91641882 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[16] = 51.65708167 * SimpleData.ToRadian;				SINSstate.GK_Longitude[16] = 45.91635739 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[17] = 51.65703936 * SimpleData.ToRadian;				SINSstate.GK_Longitude[17] = 45.91614723 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[18] = 51.65698476 * SimpleData.ToRadian;				SINSstate.GK_Longitude[18] = 45.91589917 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[19] = 51.65689765 * SimpleData.ToRadian;				SINSstate.GK_Longitude[19] = 45.91550328 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[20] = 51.65689492 * SimpleData.ToRadian;				SINSstate.GK_Longitude[20] = 45.91543676 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[21] = 51.65691551 * SimpleData.ToRadian;				SINSstate.GK_Longitude[21] = 45.91540262 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[22] = 51.65704944 * SimpleData.ToRadian;				SINSstate.GK_Longitude[22] = 45.91532339 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[23] = 51.65707977 * SimpleData.ToRadian;				SINSstate.GK_Longitude[23] = 45.91531771 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[24] = 51.65710306 * SimpleData.ToRadian;				SINSstate.GK_Longitude[24] = 45.91537133 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[25] = 51.65717924 * SimpleData.ToRadian;				SINSstate.GK_Longitude[25] = 45.91572499 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[26] = 51.65727245 * SimpleData.ToRadian;				SINSstate.GK_Longitude[26] = 45.91614317 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[27] = 51.65726635 * SimpleData.ToRadian;				SINSstate.GK_Longitude[27] = 45.91623031 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[28] = 51.65724287 * SimpleData.ToRadian;				SINSstate.GK_Longitude[28] = 45.91625902 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[29] = 51.65712406 * SimpleData.ToRadian;				SINSstate.GK_Longitude[29] = 45.91633022 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[30] = 51.65709286 * SimpleData.ToRadian;				SINSstate.GK_Longitude[30] = 45.91641882 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[31] = 51.65706786 * SimpleData.ToRadian;				SINSstate.GK_Longitude[31] = 45.91655874 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[32] = 51.65670888 * SimpleData.ToRadian;				SINSstate.GK_Longitude[32] = 45.91677764 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[33] = 51.65627597 * SimpleData.ToRadian;				SINSstate.GK_Longitude[33] = 45.91704150 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[34] = 51.65622206 * SimpleData.ToRadian;				SINSstate.GK_Longitude[34] = 45.91715524 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[35] = 51.65621375 * SimpleData.ToRadian;				SINSstate.GK_Longitude[35] = 45.91737896 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[36] = 51.65622808 * SimpleData.ToRadian;				SINSstate.GK_Longitude[36] = 45.91751543 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[37] = 51.65631557 * SimpleData.ToRadian;				SINSstate.GK_Longitude[37] = 45.91793501 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[38] = 51.65658905 * SimpleData.ToRadian;				SINSstate.GK_Longitude[38] = 45.91912273 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[39] = 51.65662906 * SimpleData.ToRadian;				SINSstate.GK_Longitude[39] = 45.91918654 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[40] = 51.65668143 * SimpleData.ToRadian;				SINSstate.GK_Longitude[40] = 45.91923808 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[41] = 51.65674261 * SimpleData.ToRadian;				SINSstate.GK_Longitude[41] = 45.91923338 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[42] = 51.65747169 * SimpleData.ToRadian;				SINSstate.GK_Longitude[42] = 45.91882389 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[43] = 51.65750359 * SimpleData.ToRadian;				SINSstate.GK_Longitude[43] = 45.91878763 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[44] = 51.65752586 * SimpleData.ToRadian;				SINSstate.GK_Longitude[44] = 45.91873369 * SimpleData.ToRadian;
                SINSstate.GK_Latitude[45] = 51.65752379 * SimpleData.ToRadian;				SINSstate.GK_Longitude[45] = 45.91867080 * SimpleData.ToRadian;



                //SINSstate.Heading = -1.93;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);
            }

            SINSstate_OdoMod.A_x0n_prev = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);


            if (OnlyIntegrating.Checked == false && OnlyAlignment.Checked == false)
            {
                for (int i = l; i < SINSstate.LastCountForRead; i++)
                {
                    if (SINSstate.UsingClasAlignment == false) { if (i < ProcHelp.AlgnCnt) { myFile.ReadLine(); continue; } }

                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate);
                    ProcessingHelp.DefSNSData(ProcHelp, SINSstate);

                    if (t == 0) { SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z); SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z); t = 1; }

                    //---------------Флаг коррекции по одометру---------------

                    if (this.Use_Only_Stops.Checked == false)
                    {
                        if (SINSstate.Odometr_SINS == false)
                        {
                            SINSstate.UsingOdoVelocity = true;

                            //Флаг использования измерений в приборных осях
                            if (this.UseOdo_In_Oz.Checked == true)
                                SINSstate.UseOdoVelocity_In_Oz = true;
                            //---


                            if (Use_Each_Odo_Measure.Checked == true)
                                ModelsOfOdoCorrection.Model_Each_Odo_Measure(SINSstate, SINSstate_OdoMod, OdoModel, ForHelp);

                            else if (this.Use_dV_by_F_Constraint.Checked == true)
                                ModelsOfOdoCorrection.ModelWith_dV_by_F_Constraint(SINSstate, SINSstate_OdoMod, OdoModel, ForHelp);

                            else if (this.Use_Const_dV_Constraint.Checked == true)
                                ModelsOfOdoCorrection.ModelWith_dV_Const_Constraints(SINSstate, SINSstate_OdoMod, OdoModel, ForHelp);

                            else if (this.UseStatisticCoeff.Checked == true)
                                ModelsOfOdoCorrection.ModelWith_StatisticCoeff(SINSstate, SINSstate_OdoMod, OdoModel, ForHelp);

                            else if (this.Use_Constant_Constraint.Checked == true)
                                ModelsOfOdoCorrection.ModelWith_Constant_Constraint(SINSstate, SINSstate_OdoMod, OdoModel);

                            else if (this.Use_Const_Freq.Checked == true)
                                ModelsOfOdoCorrection.ModelWith_Odo_Limit_Measures(SINSstate, SINSstate_OdoMod, OdoModel);

                            else if (this.Use_dV_Constraints.Checked == true)
                                ModelsOfOdoCorrection.ModelWith_dV_Constraints(SINSstate, SINSstate_OdoMod, OdoModel);

                            else if (this.Use_First_Odo_Data.Checked == true || this.Use_First_Odo_Data_One_Measure.Checked == true)
                            {
                                SINSstate.UsingOdoVelocity = false;

                                if (this.Use_First_Odo_Data.Checked == true)
                                    SINSstate.Use_First_Odo_Data_3_Measure = true;
                                else
                                    SINSstate.Use_First_Odo_Data_1_Measure = true;

                                ModelsOfOdoCorrection.Model_First_Odo_Data(SINSstate, SINSstate_OdoMod, OdoModel, ForHelp);
                            }
                        }
                        else if (SINSstate.Odometr_SINS == true) //if (SINSstate.iMx_r_odo_12 == true)
                        {
                            ModelsOfOdoCorrection.Model_With_Odo_Equations(SINSstate, SINSstate_OdoMod, OdoModel, ForHelp);
                            SINSstate.UsingCorrection = false;

                            SINSstate.feedbackExist = true;
                            SINSstate.iMx_r_odo_12 = true;

                            SINSstate2.Latitude = SINSstate.Latitude;
                            SINSstate2.Longitude = SINSstate.Longitude;
                            SINSstate2.Altitude = SINSstate.Altitude;
                        }
                    }
                    else 
                        SINSstate.UsingCorrection = false;

                    //---------------Формирование флага остановки
                    if (SINSstate.FLG_Stop == 1 && SINSstate.Odometr_SINS == false)
                    {
                        //string distance_coll = "";
                        //double distance_GK;
                        //for (int ii = 0; ii < 5; ii++)
                        //{
                        //    distance_GK = Math.Sqrt(Math.Pow((SINSstate.Latitude - SINSstate.GK_Latitude[ii]) * SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude), 2) +
                        //                         Math.Pow((SINSstate.Longitude - SINSstate.GK_Longitude[ii]) * SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude), 2));
                        //    distance_coll = distance_coll + " " + distance_GK;
                        //}
                        //Dif_GK.WriteLine(distance_coll);
                        //if (SimpleData.iMx == 15)
                        //{
                        SINSstate.KNS_flg = true;
                        SINSstate.UsingCorrection = true;
                        //}
                    }
                    else
                    {
                        SINSstate.KNS_flg = false;
                    }



                    //---------------------------------------MAIN STEPS----------------------------------------------------
                    SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate_OdoMod);
                    SINSprocessing.Make_A(SINSstate, KalmanVars);

                    if (SINSstate.Odometr_SINS == false) 
                        KalmanProcs.KalmanForecast(KalmanVars);
                    else if (SINSstate.Odometr_SINS == true && SINSstate.OdometerData.odometer_left.isReady == 1)
                        KalmanProcs.KalmanForecast(KalmanVars);

                    if (SINSstate.UsingCorrection == true || (SINSstate.GPS_Data.gps_Altitude.isReady == 1 && SINSstate.usingSNS == true))
                    {
                        SINSprocessing.MAKE_H_AND_CORRECTION(KalmanVars, SINSstate, SINSstate_OdoMod);

                        //Может надо будет переместить обратно в условие
                        SINSprocessing.CalcStateErrors(KalmanVars.ErrorConditionVector_p, SINSstate);
                        if (feedbackExist.Checked == false)
                            SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2);
                        else
                            SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate);

                        ProcHelp.corrected = 1;
                    }
                    else
                    {
                        ProcHelp.corrected = 0;
                    }



                    if (feedbackExist.Checked == false)
                    {
                        SimpleOperations.CopyArray(SINSstate_OdoMod.Vx_0_prev, SINSstate_OdoMod.Vx_0);
                    }
                    /*----------------------------------------END---------------------------------------------*/


                    //if (SINSstate.Odometr_SINS == true)
                        ForHelp.WriteLine(KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0].ToString() + " " + KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1]);


                    /*------------------------------------OUTPUT-------------------------------------------------*/
                    ProcessingHelp.OutPutInfo(i, ProcHelp, OdoModel, SINSstate, SINSstate2, KalmanVars, Nav_EstimateSolution, Nav_Autonomous, Nav_FeedbackSolution, Nav_vert_chan_test, Nav_StateErrorsVector, Nav_Errors);

                    if (i > 10000 && i % 2000 == 0)
                        Console.WriteLine(SINSstate.Count.ToString() + ",  " + (SINSstate.Latitude * SimpleData.ToDegree - ProcHelp.LatSNS).ToString() + ",  " + ProcHelp.distance_from_start.ToString() + ",  " + SINSstate.F_x[2].ToString().ToString());

                    if ((SINSstate.UsingCorrection == true || (SINSstate.GPS_Data.gps_Altitude.isReady == 1 && SINSstate.usingSNS == true)) && feedbackExist.Checked == true)
                    {
                        SINSprocessing.NullingOfCorrectedErrors(KalmanVars);
                    }



                    if (SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        SINSstate.OdometerLeftPrev_2 = SINSstate.OdometerData.odometer_left.Value;
                        SINSstate.OdometerRightPrev_2 = SINSstate.OdometerData.odometer_right.Value;
                        SINSstate.OdoSpeedPrev_2 = OdoModel.V_odo;
                        SINSstate.OdoTimeStepCount_2 = 0;

                        if (SINSstate.UsingCorrection == true || SINSstate.Odometr_SINS == true)
                        {
                            SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                            SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                            SINSstate.OdoSpeedPrev = OdoModel.V_odo;
                            SINSstate.OdoTimeStepCount = 0;
                        }
                    }
                    //---------------------------------------END----------------------------------------------------


                    //---Вычисление минимального отклонения от стартовой точки, если такое надо вообще---
                    if (ProcHelp.distance_from_start > 35.0 && for_min_pos_error != 1000.0)
                    {
                        for_min_pos_error_string = for_min_pos_error_string + tempCount.ToString() + "   " + for_min_pos_error.ToString() + "   ";
                        for_min_pos_error = 1000.0;
                    }
                    else if (ProcHelp.distance_from_start < 20.0)
                    {
                        if (for_min_pos_error > ProcHelp.distance_from_start)
                        {
                            for_min_pos_error = ProcHelp.distance_from_start;
                            tempCount = SINSstate.Count;
                        }
                    }

                }
            }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
            else if (OnlyAlignment.Checked == false)
            {
                int Count_More_than_20 = 0;
                double[] dS_x = new double[3];

                SINSstate2.Latitude = SINSstate.Latitude;
                SINSstate2.Longitude = SINSstate.Longitude;

                ForHelp.WriteLine("Time, sec \t Heading, deg \t Roll, deg \t Pitch, deg \t Odometr, meters");

                //XmlPars.ReadDataForAAStudent();

                for (int i = l; i < SINSstate.LastCountForRead; i++)
                {
                    if (UsingClasAlignment.Checked == false) { if (i < ProcHelp.AlgnCnt) { myFile.ReadLine(); continue; } }

                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate);
                    ProcessingHelp.DefSNSData(ProcHelp, SINSstate);

                    if (t == 0) { SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z); SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z); t = 1; }

                    if (SINSstate.OdometerData.odometer_left.isReady != 1)
                    {
                        SINSstate.OdoTimeStepCount++;
                        SINSstate.UsingCorrection = false;

                        //V_increment_SINS = V_increment_SINS + Math.Sqrt(Math.Pow(SINSstate.Vx_0[0] - SINSstate.Vx_0_prev[0], 2) + Math.Pow(SINSstate.Vx_0[1] - SINSstate.Vx_0_prev[1], 2) + Math.Pow(SINSstate.Vx_0[2] - SINSstate.Vx_0_prev[2], 2));
                    }
                    else if (SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        SINSstate.OdometerVector[1] = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;
                        SimpleOperations.CopyArray(dS_x, SINSstate.A_x0s * SINSstate.OdometerVector);

                        SINSstate2.Latitude = SINSstate2.Latitude + dS_x[1] / SINSstate.R_n;
                        SINSstate2.Longitude = SINSstate2.Longitude + dS_x[0] / SINSstate.R_e / Math.Cos(SINSstate2.Latitude);

                        //SINSstate.Latitude = SINSstate2.Latitude;
                        //SINSstate.Longitude = SINSstate2.Longitude;

                        SINSstate.UsingCorrection = true;
                    }


                    SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate_OdoMod);

                    if (SINSstate.Global_file == "povorot_12.09.2012")
                    {
                        SINSprocessing.Make_A(SINSstate, KalmanVars);
                        KalmanProcs.KalmanForecast(KalmanVars);

                        SINSprocessing.MAKE_H_AND_CORRECTION_alignment(KalmanVars, SINSstate, SINSstate_OdoMod);

                        SINSstate.DeltaLatitude = KalmanVars.ErrorConditionVector_p[1] / SINSstate.R_n;
                        SINSstate.DeltaLongitude = KalmanVars.ErrorConditionVector_p[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

                        SINSstate.DeltaV_1 = KalmanVars.ErrorConditionVector_p[2] + (SINSstate.Vx_0[1] * Math.Sin(SINSstate.Latitude) * SINSstate.DeltaLongitude + SINSstate.Vx_0[1] * KalmanVars.ErrorConditionVector_p[6]);
                        SINSstate.DeltaV_2 = KalmanVars.ErrorConditionVector_p[3] - (SINSstate.Vx_0[0] * Math.Sin(SINSstate.Latitude) * SINSstate.DeltaLongitude + SINSstate.Vx_0[0] * KalmanVars.ErrorConditionVector_p[6]);

                        SINSstate.DeltaRoll = -(KalmanVars.ErrorConditionVector_p[4] * Math.Sin(SINSstate.Heading) + KalmanVars.ErrorConditionVector_p[5] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
                        SINSstate.DeltaPitch = -KalmanVars.ErrorConditionVector_p[4] * Math.Cos(SINSstate.Heading) + KalmanVars.ErrorConditionVector_p[5] * Math.Sin(SINSstate.Heading);
                        SINSstate.DeltaHeading = KalmanVars.ErrorConditionVector_p[6] + SINSstate.DeltaLongitude * Math.Sin(SINSstate.Latitude) + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);


                        if (feedbackExist.Checked == false)
                        {
                            SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2);
                            SINSstate2.Vx_0[0] = SINSstate.Vx_0[0] - SINSstate.DeltaV_1;
                            SINSstate2.Vx_0[1] = SINSstate.Vx_0[1] - SINSstate.DeltaV_2;
                        }
                        else
                        {
                            SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate);
                            SINSstate.Vx_0[0] = SINSstate.Vx_0[0] - SINSstate.DeltaV_1;
                            SINSstate.Vx_0[1] = SINSstate.Vx_0[1] - SINSstate.DeltaV_2;
                        }

                        ProcHelp.corrected = 1;

                        if (feedbackExist.Checked == true)
                            SINSprocessing.NullingOfCorrectedErrors(KalmanVars);

                    }
                    if (SINSstate.Global_file == "Saratov_01.11.2012" && true)
                    {
                        SINSprocessing.Make_A(SINSstate, KalmanVars);
                        KalmanProcs.KalmanForecast(KalmanVars);

                        for (int j = 0; j < SimpleData.iMx * SimpleData.iMz; j++) KalmanVars.Matrix_H[j] = 0.0;

                        double noize = 0.1;

                        if (SINSstate.Count > 24170 && SINSstate.Count < 25540)
                        {
                            SINSstate.OdoSpeed[1] = 0.0;
                            noize = 0.01;
                        }

                        KalmanVars.Measure[0] = SINSstate.Vx_0[0] - SINSstate.A_x0s[0, 1] * SINSstate.OdoSpeed[1];
                        KalmanVars.Measure[1] = SINSstate.Vx_0[1] - SINSstate.A_x0s[1, 1] * SINSstate.OdoSpeed[1];
                        KalmanVars.Measure[2] = SINSstate.Vx_0[2] - SINSstate.A_x0s[2, 1] * SINSstate.OdoSpeed[1];

                        KalmanVars.Noize_Z[0] = noize;
                        KalmanVars.Noize_Z[1] = noize;
                        KalmanVars.Noize_Z[2] = noize;

                        KalmanVars.Matrix_H[0 * SimpleData.iMx + 2] = 1.0;
                        KalmanVars.Matrix_H[0 * SimpleData.iMx + 6] = SINSstate.Vx_0[1];
                        KalmanVars.Matrix_H[1 * SimpleData.iMx + 3] = 1.0;
                        KalmanVars.Matrix_H[1 * SimpleData.iMx + 6] = -SINSstate.Vx_0[0];

                        KalmanVars.cnt_measures = 2;

                        if (SINSstate.iMx_r3_dV3)
                        {
                            KalmanVars.Matrix_H[2 * SimpleData.iMx + SINSstate.value_iMx_r3_dV3 + 1] = 1.0;
                            KalmanVars.Matrix_H[2 * SimpleData.iMx + 5] = SINSstate.Vx_0[0];
                            KalmanVars.Matrix_H[2 * SimpleData.iMx + 0] = -SINSstate.Vx_0[0] / SINSstate.R_e;
                            KalmanVars.Matrix_H[2 * SimpleData.iMx + 4] = -SINSstate.Vx_0[1];
                            KalmanVars.Matrix_H[2 * SimpleData.iMx + 1] = -SINSstate.Vx_0[1] / SINSstate.R_n;

                            KalmanVars.Matrix_H[0 * SimpleData.iMx + 5] = -SINSstate.Vx_0[2];
                            KalmanVars.Matrix_H[0 * SimpleData.iMx + 0] = SINSstate.Vx_0[2] / SINSstate.R_e;
                            KalmanVars.Matrix_H[1 * SimpleData.iMx + 4] = SINSstate.Vx_0[2];
                            KalmanVars.Matrix_H[1 * SimpleData.iMx + 1] = SINSstate.Vx_0[2] / SINSstate.R_n;

                            KalmanVars.cnt_measures = 3;
                        }

                        KalmanProcs.KalmanCorrection(KalmanVars);

                        SINSstate.UsingOdoVelocity = true;

                        SINSprocessing.CalcStateErrors(KalmanVars.ErrorConditionVector_p, SINSstate);
                        if (feedbackExist.Checked == false)
                            SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2);
                        else
                            SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate);

                        if (feedbackExist.Checked == true)
                            SINSprocessing.NullingOfCorrectedErrors(KalmanVars);





                        string distance_coll = "";
                        double distance_GK;

                        if (SINSstate.Count == 58000 || SINSstate.Count == 90035 || SINSstate.Count == 121690 || SINSstate.Count == 152230)
                        {
                            GK_lap++;
                            for (int ii = 0; ii < 46; ii++)
                                ProcHelp.distance_GK_Sarat[ii] = 1000.0;
                        }

                        for (int ii = 0; ii < 46; ii++)
                        {
                            distance_GK = Math.Sqrt(Math.Pow((SINSstate.Latitude - SINSstate.GK_Latitude[ii]) * SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude), 2) +
                                                 Math.Pow((SINSstate.Longitude - SINSstate.GK_Longitude[ii]) * SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude), 2));

                            distance_coll = distance_coll + " " + distance_GK;

                            if (distance_GK < 15.0)
                            {
                                if (distance_GK < ProcHelp.distance_GK_Sarat[ii])
                                {
                                    ProcHelp.distance_GK_Sarat[ii] = distance_GK;
                                    distance_GK_Sarat[GK_lap, ii] = distance_GK;
                                }
                            }
                        }
                        

                    }
                    /*----------------------------------------END---------------------------------------------*/



                    /*------------------------------------OUTPUT-------------------------------------------------*/

                    //if (SINSstate.OdometerData.odometer_left.isReady == 1 && SINSstate.UsingCorrection == true)// && OdoModel.V_odo != 0
                    if (SINSstate.Count % 2 == 0)
                        ForHelp.WriteLine((SINSstate.Count*SINSstate.timeStep).ToString() + " \t " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " \t " + (SINSstate.Roll * SimpleData.ToDegree).ToString()
                                         + " \t " + (SINSstate.Pitch * SimpleData.ToDegree).ToString() + " \t " + SINSstate.OdometerData.odometer_left.Value.ToString());

                    /*-------------------------------------------------------------------------------------*/
                    if (i > 5000 && i % 1000 == 0)
                        Console.WriteLine(SINSstate.Count.ToString() + ",  " + (SINSstate.Latitude * SimpleData.ToDegree - ProcHelp.LatSNS).ToString() + ",  " + ProcHelp.distance.ToString() + ",  "
                                        + SINSstate.F_x[2].ToString().ToString() + ",  " + SINSstate.Altitude.ToString() + ",  " + ProcHelp.AltSNS.ToString());

                    ProcessingHelp.OutPutInfo(i, ProcHelp, OdoModel, SINSstate, SINSstate2, KalmanVars, Nav_EstimateSolution, Nav_Autonomous, Nav_FeedbackSolution, Nav_vert_chan_test, Nav_StateErrorsVector, Nav_Errors);

                    if (SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        SINSstate.OdometerLeftPrev_2 = SINSstate.OdometerData.odometer_left.Value;
                        SINSstate.OdometerRightPrev_2 = SINSstate.OdometerData.odometer_right.Value;
                        SINSstate.OdoSpeedPrev_2 = OdoModel.V_odo;
                        SINSstate.OdoTimeStepCount_2 = 0;

                        if (SINSstate.UsingCorrection == true)
                        {
                            SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                            SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                            SINSstate.OdoSpeedPrev = OdoModel.V_odo;
                            SINSstate.OdoTimeStepCount = 0;
                        }
                    }
                }
            }

            DateTime datetime_end = DateTime.Now;

            Console.WriteLine(datetime_end - datetime_start);

            //ForHelp.WriteLine(for_min_pos_error_string + "   " + SINSstate.Count.ToString() + " " + distance.ToString());
            //Console.WriteLine(SINSstate.StartErrorString);

            string distance_str = " ";
            for (int ii = 0; ii < 46; ii++)
            {
                distance_str = " ";
                for (int iii = 0; iii <= GK_lap; iii++)
                {
                    distance_str = distance_str + "\t" + distance_GK_Sarat[iii, ii].ToString();
                }
                Dif_GK.WriteLine(distance_str);
            }

            Dif_GK.Close();

            distance_str = " ";

            

            myFile.Close(); ForHelp.Close(); Nav_FeedbackSolution.Close(); Nav_EstimateSolution.Close(); Nav_StateErrorsVector.Close(); this.Close();
        }

    }
}
