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

        StreamReader myFile;

        StreamWriter Nav_FeedbackSolution = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_FeedbackSolution.txt");
        StreamWriter Nav_Errors = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_Errors.txt");
        StreamWriter Nav_Autonomous = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_Autonomous.txt");
        StreamWriter Nav_EstimateSolution = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_EstimateSolution.txt");
        StreamWriter Nav_StateErrorsVector = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Nav_StateErrorsVector.txt");
        StreamWriter ForHelp = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//ForHelp.txt");
        StreamWriter Nav_vert_chan_test = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Nav_vert_chan_test.txt");

        //StreamWriter Dif_GK = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Output//Dif_GK.txt");

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
            LockParamsOfStart();
            LockTypesOfCorrection();
            LockTheWayOfStart();
            LockNumbOfMeasures();
            LockDimOfVector();
            this.usingSNS.Enabled = false;
            this.Main_Block_Click_new.Enabled = false;
        }


        public void Main_Block_Click_new_Click(object sender, EventArgs e)
        {
            DateTime datetime_start = DateTime.Now;

            this.DefineDimentionOfErrorVector();                                                            //---формирование размерности вектора ошибок---//
            this.SelectDataIn();                                                                            //---выбор входного набора данных---//

            Parameters.StartSINS_Parameters(SINSstate, SINSstate_OdoMod, KalmanVars, Params, ProcHelp);     //---Инициализация начальных условий при отсутствии выставки---//
            SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);                                 //---Инициализация ковариационных матриц матриц вектора ошибок---//

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
            SINSstate.Odometr_SINS_case = this.Odometr_SINS_case.Checked;
            SINSstate.Use_3_Measure = this.Use_3_Measure.Checked;
            SINSstate.Use_1_Measure = this.Use_1_Measure.Checked;
            SINSstate.FreqOutput = Convert.ToInt32(this.Output_Freq.Text);


            ProcHelp.AlgnCnt = 0;

            if (SINSstate.Global_file == "Azimut_14.08.2012") ProcHelp.AlgnCnt = 21486;
            if (SINSstate.Global_file == "ktn004_15.03.2012") ProcHelp.AlgnCnt = 48000;
            if (SINSstate.Global_file == "ktn004_21.03.2012") ProcHelp.AlgnCnt = 45000;
            if (SINSstate.Global_file == "Azimut_29.08.2012") ProcHelp.AlgnCnt = 35000;
            if (SINSstate.Global_file == "povorot_12.09.2012") ProcHelp.AlgnCnt = 1500;

            if (SINSstate.Global_file == "Saratov_01.11.2012") ProcHelp.AlgnCnt = 19200;
            if (SINSstate.Global_file == "Saratov_01.11.2012") ProcessingHelp.SaratovGK_Positions(SINSstate, ProcHelp);

            

            //---Выставка---//
            l = SINS_Alignment.SINSAlignment(ProcHelp, SINSstate, SINSstate2, myFile, KalmanVars);

            ProcHelp.initCount = false;
            SINSstate_OdoMod.A_x0n_prev = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);



            //------Случай ОДОМЕТР + БИНС------
            if (SINSstate.Odometr_SINS_case == true)
            {
                Odometr_SINS.Odometr_SINSProcessing(l, ProcHelp, SINSstate, SINSstate2, SINSstate_OdoMod, OdoModel, myFile, KalmanVars, ForHelp, Nav_EstimateSolution, Nav_Autonomous, Nav_FeedbackSolution, Nav_vert_chan_test, Nav_StateErrorsVector, Nav_Errors);
                this.Close();
                return;
            }

            //------Случай БИНС + ОДОМЕТР------
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

                        else if (this.Use_Odo_Distance.Checked == true)
                        {
                            SINSstate.UsingOdoVelocity = false;
                            ModelsOfOdoCorrection.Model_First_Odo_Data(SINSstate, SINSstate_OdoMod, OdoModel, ForHelp);
                        }
                    }
                    else
                        SINSstate.UsingCorrection = false;

                    //---------------Формирование флага остановки------------
                    if (SINSstate.FLG_Stop == 1)
                    {
                        SINSstate.KNS_flg = true;
                        SINSstate.UsingCorrection = true;
                    }
                    else
                    {
                        SINSstate.KNS_flg = false;
                    }



                    //---------------------------------------MAIN STEPS----------------------------------------------------
                    SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate_OdoMod);
                    SINSprocessing.Make_A(SINSstate, KalmanVars);

                    KalmanProcs.KalmanForecast(KalmanVars);

                    ForHelp.WriteLine(KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0].ToString() + " " + KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1]);

                    if (SINSstate.UsingCorrection == true || (SINSstate.GPS_Data.gps_Altitude.isReady == 1 && SINSstate.usingSNS == true))
                    {
                        SINSprocessing.MAKE_H_AND_CORRECTION(KalmanVars, SINSstate, SINSstate_OdoMod);

                        //Может надо будет переместить обратно в условие
                        SINSprocessing.CalcStateErrors(KalmanVars.ErrorConditionVector_p, SINSstate);
                        if (SINSstate.feedbackExist == false)
                            SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2);
                        else
                            SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate);

                        ProcHelp.corrected = 1;
                    }
                    else
                    {
                        ProcHelp.corrected = 0;
                    }


                    if (SINSstate.feedbackExist == false)
                    {
                        SimpleOperations.CopyArray(SINSstate_OdoMod.Vx_0_prev, SINSstate_OdoMod.Vx_0);
                    }
                    /*----------------------------------------END---------------------------------------------*/


                    //if (SINSstate.Odometr_SINS == true)
                        //ForHelp.WriteLine(KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0].ToString() + " " + KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1]);


                    /*------------------------------------OUTPUT-------------------------------------------------*/
                    ProcessingHelp.OutPutInfo(i, ProcHelp, OdoModel, SINSstate, SINSstate2, KalmanVars, Nav_EstimateSolution, Nav_Autonomous, Nav_FeedbackSolution, Nav_vert_chan_test, Nav_StateErrorsVector, Nav_Errors);

                    if (i > 10000 && i % 2000 == 0)
                        Console.WriteLine(SINSstate.Count.ToString() + ",  " + (SINSstate.Latitude * SimpleData.ToDegree - ProcHelp.LatSNS).ToString() + ",  " + ProcHelp.distance_from_start.ToString() + ",  " + SINSstate.F_x[2].ToString().ToString());

                    if ((SINSstate.UsingCorrection == true || (SINSstate.GPS_Data.gps_Altitude.isReady == 1 && SINSstate.usingSNS == true)) && SINSstate.feedbackExist == true)
                    {
                        SINSprocessing.NullingOfCorrectedErrors(KalmanVars);
                    }



                    if (SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        SINSstate.OdometerLeftPrev_2 = SINSstate.OdometerData.odometer_left.Value;
                        SINSstate.OdometerRightPrev_2 = SINSstate.OdometerData.odometer_right.Value;
                        SINSstate.OdoSpeedPrev_2 = OdoModel.V_odo;
                        SINSstate.OdoTimeStepCount_2 = 0;

                        if (SINSstate.UsingCorrection == true || SINSstate.Odometr_SINS_case == true)
                        {
                            SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                            SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                            SINSstate.OdoSpeedPrev = OdoModel.V_odo;
                            SINSstate.OdoTimeStepCount = 0;

                            SINSstate.Latitude_prev = SINSstate.Latitude;   SINSstate2.Latitude_prev = SINSstate2.Latitude;
                            SINSstate.Longitude_prev = SINSstate.Longitude; SINSstate2.Longitude_prev = SINSstate2.Longitude;
                            SINSstate.Altitude_prev = SINSstate.Altitude;   SINSstate2.Altitude_prev = SINSstate2.Altitude;
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
//------------------------------------------------------------------АВТОНОМНЫЙ РЕЖИМ--------------------------------------------------------------------------------------------------------//
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

                    SINSprocessing.Make_A(SINSstate, KalmanVars);
                    if (SINSstate.OdometerData.odometer_left.isReady == 1)
                        KalmanProcs.KalmanForecast(KalmanVars);
                    ForHelp.WriteLine(KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0].ToString() + " " + KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1]);


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
                    //if (SINSstate.Count % 2 == 0)
                    //    ForHelp.WriteLine((SINSstate.Count*SINSstate.timeStep).ToString() + " \t " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " \t " + (SINSstate.Roll * SimpleData.ToDegree).ToString()
                    //                     + " \t " + (SINSstate.Pitch * SimpleData.ToDegree).ToString() + " \t " + SINSstate.OdometerData.odometer_left.Value.ToString());

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

            //string distance_str = " ";
            //for (int ii = 0; ii < 46; ii++)
            //{
            //    distance_str = " ";
            //    for (int iii = 0; iii <= GK_lap; iii++)
            //    {
            //        distance_str = distance_str + "\t" + distance_GK_Sarat[iii, ii].ToString();
            //    }
            //    Dif_GK.WriteLine(distance_str);
            //}

            //Dif_GK.Close();

            //distance_str = " ";

            

            myFile.Close(); ForHelp.Close(); Nav_FeedbackSolution.Close(); Nav_EstimateSolution.Close(); Nav_StateErrorsVector.Close(); this.Close();
        }







        public void DefineDimentionOfErrorVector()
        {
            iMx = SimpleData.iMx = 13;
            iMq = SimpleData.iMq = SimpleData.iMx;

            int value_iMx_r3_dV3 = 0, value_iMx_r_odo_12 = 0, value_iMx_kappa_13_ds = 0;

            bool iMx_r3_dV3 = iMx_r_3_dV_3.Checked;
            if (iMx_r3_dV3)
            {
                value_iMx_r3_dV3 = iMx;
                iMx = SimpleData.iMx += 2;
            }

            if (this.Odometr_SINS_case.Checked)
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
            SINSstate.iMx_kappa_13_ds = iMx_kappa_13_ds;
            if (iMx_r3_dV3)
                SINSstate.value_iMx_r3_dV3 = value_iMx_r3_dV3;
            if (this.Odometr_SINS_case.Checked)
                SINSstate.value_iMx_r_odo_12 = value_iMx_r_odo_12;
            if (iMx_kappa_13_ds)
                SINSstate.value_iMx_kappa_13_ds = value_iMx_kappa_13_ds;
        }

        public void SelectDataIn()
        {
            if (Saratov_01_11_2012.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//SarOEG_20121213_refactor.dat");
                SINSstate.Global_file = "Saratov_01.11.2012";
            }
            else if (Azimut_14_08_2012.Checked == true)
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
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//ktn004_static4hour_marsh_sns_15-Mar-2012,16-29-45_dat.dat");
                SINSstate.Global_file = "ktn004_15.03.2012";
            }
            else if (povorot_12_09_2012.Checked == true)
            {
                myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//povorot_12-Sep-2012,13-26-38_dat.dat");
                SINSstate.Global_file = "povorot_12.09.2012";
            }
        }

        //---ИНТЕРФЕЙС---//

        public void LockTypesOfCorrection()
        {
            this.Use_Odo_Distance.Enabled = false; this.Use_Each_Odo_Measure.Enabled = false; this.Use_dV_Constraints.Enabled = false; this.Use_Constant_Constraint.Enabled = false;
            this.UseStatisticCoeff.Enabled = false; this.Use_Const_dV_Constraint.Enabled = false; this.Use_Const_dV_Constraint.Enabled = false; this.Use_dV_by_F_Constraint.Enabled = false;
            this.Use_Const_Freq.Enabled = false; this.Use_Only_Stops.Enabled = false;
        }
        public void FreeTypesOfCorrection()
        {
            this.Use_Odo_Distance.Enabled = true; this.Use_Each_Odo_Measure.Enabled = true; this.Use_dV_Constraints.Enabled = true; this.Use_Constant_Constraint.Enabled = true;
            this.UseStatisticCoeff.Enabled = true; this.Use_Const_dV_Constraint.Enabled = true; this.Use_Const_dV_Constraint.Enabled = true; this.Use_dV_by_F_Constraint.Enabled = true;
            this.Use_Const_Freq.Enabled = true; this.Use_Only_Stops.Enabled = true;
        }

        public void LockInData()
        {
            this.Azimut_14_08_2012.Enabled = false; this.Azimut_15_08_2012.Enabled = false; this.Azimut_24_08_2012.Enabled = false; this.Azimut_29_08_2012.Enabled = false; this.Kama_04_09_2012.Enabled = false;
            this.ktn004_21_03_2012.Enabled = false; this.ktn004_15_03_2012.Enabled = false; this.povorot_12_09_2012.Enabled = false; this.Saratov_01_11_2012.Enabled = false;
        }
        public void FreeInData()
        {
            this.Azimut_14_08_2012.Enabled = true; this.Azimut_15_08_2012.Enabled = true; this.Azimut_24_08_2012.Enabled = true; this.Azimut_29_08_2012.Enabled = true; this.Kama_04_09_2012.Enabled = true;
            this.ktn004_21_03_2012.Enabled = true; this.ktn004_15_03_2012.Enabled = true; this.povorot_12_09_2012.Enabled = true; this.Saratov_01_11_2012.Enabled = true;
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
            this.Use_3_Measure.Enabled = false; this.Use_1_Measure.Enabled = false; this.UseOdo_In_Oz.Enabled = false;
        }
        public void FreeNumbOfMeasures()
        {
            this.Use_3_Measure.Enabled = true; this.Use_1_Measure.Enabled = true; this.UseOdo_In_Oz.Enabled = true;
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
            this.Use_Odo_Distance.Checked = false; this.Use_Each_Odo_Measure.Checked = false; this.Use_dV_Constraints.Checked = false; this.Use_Constant_Constraint.Checked = false;
            this.UseStatisticCoeff.Checked = false; this.Use_Const_dV_Constraint.Checked = false; this.Use_Const_dV_Constraint.Checked = false; this.Use_dV_by_F_Constraint.Checked = false;
            this.Use_Const_Freq.Checked = false; this.Use_Only_Stops.Checked = false;
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
        }

        private void povorot_12_09_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.povorot_12_09_2012.Checked == true)
            {
                CheckedTrueDataIn(); 
                this.povorot_12_09_2012.Enabled = true;
            }
            else CheckedFalseDataIn();
        }

        private void Saratov_01_11_2012_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Saratov_01_11_2012.Checked == true)
            {
                CheckedTrueDataIn(); 
                this.Saratov_01_11_2012.Enabled = true;
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
                FreeNumbOfMeasures();
                this.usingSNS.Enabled = true;
            }
            else
            {
                UnCheckTypesOfCorrection();
                FreeTheWayOfStart();
                LockTypesOfCorrection();
                LockNumbOfMeasures();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;
            }
        }

        private void EstimateExist_CheckedChanged(object sender, EventArgs e)
        {
            if (this.EstimateExist.Checked == true)
            {
                LockTheWayOfStart(); this.EstimateExist.Enabled = true;
                FreeTypesOfCorrection();
                FreeNumbOfMeasures();
                this.usingSNS.Enabled = true;
            }
            else
            {
                UnCheckTypesOfCorrection();
                FreeTheWayOfStart();
                LockTypesOfCorrection();
                LockNumbOfMeasures();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;
            }
        }

        private void Odometr_SINS_case_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Odometr_SINS_case.Checked == true)
            {
                LockTheWayOfStart(); this.Odometr_SINS_case.Enabled = true;
                LockNumbOfMeasures();
                this.usingSNS.Enabled = true;
                this.Main_Block_Click_new.Enabled = true;
            }
            else
            {
                FreeTheWayOfStart();
                this.usingSNS.Enabled = false;
                this.Main_Block_Click_new.Enabled = false;
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

        private void Use_dV_Constraints_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Use_dV_Constraints.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.Use_dV_Constraints.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
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

        private void UseStatisticCoeff_CheckedChanged(object sender, EventArgs e)
        {
            if (this.UseStatisticCoeff.Checked == true)
            {
                CheckedTrueTypesOfCorrection();
                this.UseStatisticCoeff.Enabled = true;
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
                this.Use_Odo_Distance.Enabled = true;
            }
            else CheckedFlaseTypesOfCorrection();
        }


    }
}
