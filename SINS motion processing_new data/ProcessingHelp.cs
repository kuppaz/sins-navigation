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

namespace SINS_motion_processing
{
    public partial class ProcessingHelp
    {

        /*-------------------------------Вспомогательные функции---------------------------------------------------------*/

        public static void DefSNSData(Proc_Help ProcHelp, SINS_State SINSstate)
        {
            if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
            {
                ProcHelp.LatSNS = SINSstate.GPS_Data.gps_Latitude.Value * 180 / Math.PI;
                ProcHelp.LongSNS = SINSstate.GPS_Data.gps_Longitude.Value * 180 / Math.PI;
                ProcHelp.AltSNS = SINSstate.GPS_Data.gps_Altitude.Value;
                ProcHelp.SpeedSNS = Math.Sqrt(SINSstate.GPS_Data.gps_Ve.Value * SINSstate.GPS_Data.gps_Ve.Value + SINSstate.GPS_Data.gps_Vn.Value * SINSstate.GPS_Data.gps_Vn.Value);
                ProcHelp.Ve_SNS = SINSstate.GPS_Data.gps_Ve.Value;
                ProcHelp.Vn_SNS = SINSstate.GPS_Data.gps_Vn.Value;
            }
        }

        public static void ReadSINSStateFromString(Proc_Help ProcHelp, StreamReader myFile, SINS_State SINSstate)
        {
            string[] dataArray;

            ProcHelp.datastring = myFile.ReadLine();
            dataArray = ProcHelp.datastring.Split(' ');
            int t = 0;

            for (int y = 0; y < dataArray.Length; y++)
            {
                if (dataArray[y] != "")
                    t++;
            }
            string[] dataArray2 = new string[t];
            t = 0;

            for (int y = 0; y < dataArray.Length; y++)
            {
                if (dataArray[y] != "")
                {
                    dataArray2[t] = dataArray[y];
                    t++;
                }
            }

            if (SINSstate.Global_file == "Saratov_01.11.2012")
            {
                SINSstate.Count = Convert.ToDouble(dataArray2[0]);

                if (ProcHelp.initCount == false) { ProcHelp.initCount = true; SINSstate.initCount = SINSstate.Count - 1; }

                SINSstate.Time = Convert.ToDouble(dataArray2[1]);

                SINSstate.timeStep = Convert.ToDouble(dataArray2[2]);

                SINSstate.F_z[0] = Convert.ToDouble(dataArray2[3]); SINSstate.W_z[0] = Convert.ToDouble(dataArray2[6]) ;
                SINSstate.F_z[1] = Convert.ToDouble(dataArray2[4]); SINSstate.W_z[1] = Convert.ToDouble(dataArray2[7]) ;
                SINSstate.F_z[2] = Convert.ToDouble(dataArray2[5]); SINSstate.W_z[2] = Convert.ToDouble(dataArray2[8]) ;

                SINSstate.OdoSpeed[1] = Convert.ToDouble(dataArray2[9]);

                SINSstate.Count = Convert.ToDouble(dataArray2[0]);
            }
            else
            {

                //SINSstate.Time = Convert.ToDouble(dataArray2[0]);
                SINSstate.Count = Convert.ToDouble(dataArray2[0]);

                if (ProcHelp.initCount == false) { ProcHelp.initCount = true; SINSstate.initCount = SINSstate.Count - 1; }

                SINSstate.Time = (SINSstate.Count - SINSstate.initCount) * SINSstate.timeStep;

                SINSstate.F_z[1] = Convert.ToDouble(dataArray2[1]); SINSstate.W_z[1] = Convert.ToDouble(dataArray2[4]);
                SINSstate.F_z[2] = Convert.ToDouble(dataArray2[2]); SINSstate.W_z[2] = Convert.ToDouble(dataArray2[5]);
                SINSstate.F_z[0] = Convert.ToDouble(dataArray2[3]); SINSstate.W_z[0] = Convert.ToDouble(dataArray2[6]);

                SINSstate.GPS_Data.gps_Latitude.Value = Convert.ToDouble(dataArray2[7]);
                SINSstate.GPS_Data.gps_Latitude.isReady = Convert.ToInt32(dataArray2[8]);
                SINSstate.GPS_Data.gps_Longitude.Value = Convert.ToDouble(dataArray2[9]);
                SINSstate.GPS_Data.gps_Longitude.isReady = Convert.ToInt32(dataArray2[10]);
                SINSstate.GPS_Data.gps_Altitude.Value = Convert.ToDouble(dataArray2[11]);
                SINSstate.GPS_Data.gps_Altitude.isReady = Convert.ToInt32(dataArray2[12]);

                SINSstate.GPS_Data.gps_Vn.Value = Convert.ToDouble(dataArray2[13]);
                SINSstate.GPS_Data.gps_Vn.isReady = Convert.ToInt32(dataArray2[14]);
                SINSstate.GPS_Data.gps_Ve.Value = Convert.ToDouble(dataArray2[15]);
                SINSstate.GPS_Data.gps_Ve.isReady = Convert.ToInt32(dataArray2[16]);

                SINSstate.FLG_Stop = Convert.ToInt32(dataArray2[17]);

                SINSstate.OdometerData.odometer_left.Value = Convert.ToDouble(dataArray2[18]);
                SINSstate.OdometerData.odometer_left.isReady = Convert.ToInt32(dataArray2[19]);
                SINSstate.OdometerData.odometer_right.Value = Convert.ToDouble(dataArray2[20]);
                SINSstate.OdometerData.odometer_right.isReady = Convert.ToInt32(dataArray2[21]);
            }
        }


        public static void OutPutInfo(int i, Proc_Help ProcHelp, ParamsForModel OdoModel, SINS_State SINSstate, SINS_State SINSstate2, Kalman_Vars KalmanVars, StreamWriter Nav_EstimateSolution, StreamWriter Nav_Autonomous,
                StreamWriter Nav_FeedbackSolution, StreamWriter Nav_vert_chan_test, StreamWriter Nav_StateErrorsVector, StreamWriter Nav_Errors)
        {
            ProcHelp.distance = Math.Sqrt(Math.Pow((SINSstate2.Latitude - ProcHelp.LatSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(SINSstate2.Latitude, SINSstate.Altitude), 2) +
                                 Math.Pow((SINSstate2.Longitude - ProcHelp.LongSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(SINSstate2.Latitude, SINSstate.Altitude), 2));
            ProcHelp.distance_from_start = Math.Sqrt(Math.Pow((SINSstate2.Latitude - SINSstate.Latitude_Start) * SimpleOperations.RadiusE(SINSstate2.Latitude, SINSstate.Altitude), 2) +
                                 Math.Pow((SINSstate2.Longitude - SINSstate.Longitude_Start) * SimpleOperations.RadiusE(SINSstate2.Latitude, SINSstate.Altitude), 2));

            /*----------------------------------OUTPUT ESTIMATE------------------------------------------------------*/
            if ((SINSstate.feedbackExist == false && SINSstate.Autonomous == false && i % SINSstate.FreqOutput == 0) || (SINSstate.Global_file == "povorot_12.09.2012" && SINSstate.feedbackExist == false))
            {
                ProcHelp.datastring = SINSstate.Time.ToString()
                                 + " " + ((SINSstate2.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n).ToString()
                                 + " " + ((SINSstate2.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e).ToString()
                                 + " " + ProcHelp.LatSNS.ToString() + " " + ProcHelp.LongSNS.ToString()
                                 + " " + SINSstate2.Vx_0[0].ToString() + " " + SINSstate2.Vx_0[1].ToString() + " " + SINSstate2.Vx_0[2].ToString()
                                 + " " + ProcHelp.corrected.ToString()
                                 + " " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " " + (SINSstate2.Heading * SimpleData.ToDegree).ToString()
                                 + " " + (SINSstate.Roll * SimpleData.ToDegree).ToString() + " " + (SINSstate2.Roll * SimpleData.ToDegree).ToString()
                                 + " " + (SINSstate.Pitch * SimpleData.ToDegree).ToString() + " " + (SINSstate2.Pitch * SimpleData.ToDegree).ToString() + " " + ProcHelp.distance.ToString() + " " + ProcHelp.distance_from_start.ToString() + " " + SINSstate.V_norm.ToString()
                                 + " " + OdoModel.V_increment_odo.ToString() + " " + (OdoModel.V_increment_SINS + SINSstate.dV_q).ToString() + " " + OdoModel.Can.ToString()
                                 + " " + SINSstate.OdometerData.odometer_left.isReady.ToString() + " " + SINSstate.OdometerData.odometer_left.Value.ToString() + " " + SINSstate.OdometerVector[1].ToString()
                                 ;
                Nav_EstimateSolution.WriteLine(ProcHelp.datastring);
            }
            else
            {
                ProcHelp.distance = Math.Sqrt(Math.Pow((SINSstate.Latitude - ProcHelp.LatSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(ProcHelp.LatSNS, SINSstate.Altitude), 2) +
                                     Math.Pow((SINSstate.Longitude - ProcHelp.LongSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(ProcHelp.LatSNS, SINSstate.Altitude), 2));
                ProcHelp.distance_from_start = Math.Sqrt(Math.Pow((SINSstate.Latitude - SINSstate.Latitude_Start) * SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude), 2) +
                                 Math.Pow((SINSstate.Longitude - SINSstate.Longitude_Start) * SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude), 2));

                /*----------------------------------OUTPUT AUTONOMOUS------------------------------------------------------*/
                if (SINSstate.Autonomous == true && i % SINSstate.FreqOutput == 0)
                {
                    ProcHelp.datastring = (SINSstate.Time).ToString() + " " + SINSstate.OdoTimeStepCount.ToString() + " " + SINSstate.OdometerVector[1].ToString()
                         + " " + ((SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n).ToString()
                         + " " + ((SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e).ToString() + " " + (ProcHelp.LatSNS).ToString() + " " + (ProcHelp.LongSNS).ToString()
                                        + " " + ProcHelp.SpeedSNS.ToString()
                                        + " " + SINSstate.Vx_0[0].ToString() + " " + SINSstate.Vx_0[1].ToString() + " " + SINSstate.Vx_0[2].ToString()
                                        + " " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " " + (SINSstate.Roll * SimpleData.ToDegree).ToString() + " " + (SINSstate.Pitch * SimpleData.ToDegree).ToString()
                                        + " " + ProcHelp.distance.ToString() + " " + ProcHelp.distance_from_start.ToString() + " " + SINSstate.Azimth.ToString() + " " + SINSstate.OdometerData.odometer_left.isReady.ToString()
                                        + " " + SINSstate.OdometerData.odometer_left.Value.ToString()
                                        ;
                    Nav_Autonomous.WriteLine(ProcHelp.datastring);
                }
                /*----------------------------------OUTPUT FEEDBACK------------------------------------------------------*/
                else if (i % SINSstate.FreqOutput == 0)
                {
                    ProcHelp.datastring = (SINSstate.Time).ToString() + " " + SINSstate.Count.ToString() + " " + SINSstate.OdoTimeStepCount.ToString()
                                        + " " + SINSstate.OdoSpeed[1].ToString() 
                                        + " " + ((SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n).ToString()
                                        + " " + ((SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e).ToString() + " " + SINSstate.Altitude.ToString()
                                        + " " + ProcHelp.LatSNS.ToString() + " " + ProcHelp.LongSNS.ToString() + " " + ProcHelp.AltSNS.ToString() + " " + ProcHelp.SpeedSNS.ToString()
                                        + " " + SINSstate.Vx_0[0].ToString() + " " + SINSstate.Vx_0[1].ToString() + " " + SINSstate.Vx_0[2].ToString()
                                        + " " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " " + (SINSstate.Roll * SimpleData.ToDegree).ToString() + " " + (SINSstate.Pitch * SimpleData.ToDegree).ToString()
                                        + " " + ProcHelp.corrected.ToString() + " " + SINSstate.OdometerData.odometer_left.isReady.ToString()
                                        + " " + ProcHelp.distance.ToString() + " " + ProcHelp.distance_from_start.ToString() + " " + SINSstate.FLG_Stop
                                        // + " " + OdoModel.V_increment_odo.ToString() + " " + (OdoModel.V_increment_SINS + SINSstate.dV_q).ToString() + " " + OdoModel.Can.ToString()
                                        ;
                    Nav_FeedbackSolution.WriteLine(ProcHelp.datastring);
                }
            }


            if (i % SINSstate.FreqOutput == 0)
            {
                /*----------------------------------OUTPUT ERRORS------------------------------------------------------*/
                ProcHelp.datastring = (SINSstate.DeltaLatitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.DeltaLongitude * SimpleData.ToDegree).ToString() + " " + SINSstate.DeltaV_1.ToString() + " " + SINSstate.DeltaV_2.ToString() + " "
                                + SINSstate.DeltaV_3.ToString() + " " + SINSstate.DeltaHeading.ToString() + " " + SINSstate.DeltaRoll.ToString() + " " + SINSstate.DeltaPitch.ToString();
                Nav_Errors.WriteLine(ProcHelp.datastring);


                /*----------------------------------OUTPUT StateErrors------------------------------------------------------*/
                ProcHelp.datastring = SINSstate.Count.ToString() + " " + KalmanVars.ErrorConditionVector_p[0].ToString() + " " + KalmanVars.ErrorConditionVector_p[1].ToString() + " " + KalmanVars.ErrorConditionVector_p[2].ToString()
                                 + " " + KalmanVars.ErrorConditionVector_p[3].ToString() + " " + KalmanVars.ErrorConditionVector_p[4].ToString() + " " + KalmanVars.ErrorConditionVector_p[5].ToString()
                                  + " " + KalmanVars.ErrorConditionVector_p[6].ToString() + " " + KalmanVars.ErrorConditionVector_p[7].ToString() + " " + KalmanVars.ErrorConditionVector_p[8].ToString()
                                   + " " + KalmanVars.ErrorConditionVector_p[9].ToString() + " " + KalmanVars.ErrorConditionVector_p[10].ToString() + " " + KalmanVars.ErrorConditionVector_p[11].ToString()
                                    + " " + KalmanVars.ErrorConditionVector_p[12].ToString();
                if (SINSstate.iMx_r3_dV3)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_r3_dV3].ToString() + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_r3_dV3 + 1].ToString();
                if (SINSstate.Odometr_SINS_case)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_r_odo_12].ToString() + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_r_odo_12 + 1].ToString();
                if (SINSstate.iMx_kappa_13_ds)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_kappa_13_ds].ToString() + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_kappa_13_ds + 1].ToString() + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_kappa_13_ds + 2].ToString();

                Nav_StateErrorsVector.WriteLine(ProcHelp.datastring);
            }

        }



        public static void SaratovGK_Positions(SINS_State SINSstate, Proc_Help ProcHelp)
        {
            //SINSstate.UsingClasAlignment = false;
            //UsingClasAlignment.Checked = false;
            //ProcHelp.AlgnCnt = 10;

            for (int i = 0; i < 46; i++)
                ProcHelp.distance_GK_Sarat[i] = 1000.0;

            SINSstate.GK_Latitude[0] = 51.65744354 * SimpleData.ToRadian; SINSstate.GK_Longitude[0] = 45.91832179 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[1] = 51.65736976 * SimpleData.ToRadian; SINSstate.GK_Longitude[1] = 45.91801222 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[2] = 51.65727971 * SimpleData.ToRadian; SINSstate.GK_Longitude[2] = 45.91762382 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[3] = 51.65726450 * SimpleData.ToRadian; SINSstate.GK_Longitude[3] = 45.91752491 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[4] = 51.65727589 * SimpleData.ToRadian; SINSstate.GK_Longitude[4] = 45.91745401 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[5] = 51.65730910 * SimpleData.ToRadian; SINSstate.GK_Longitude[5] = 45.91739258 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[6] = 51.65734559 * SimpleData.ToRadian; SINSstate.GK_Longitude[6] = 45.91735828 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[7] = 51.65737465 * SimpleData.ToRadian; SINSstate.GK_Longitude[7] = 45.91727285 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[8] = 51.65737334 * SimpleData.ToRadian; SINSstate.GK_Longitude[8] = 45.91716680 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[9] = 51.65735419 * SimpleData.ToRadian; SINSstate.GK_Longitude[9] = 45.91707277 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[10] = 51.65731701 * SimpleData.ToRadian; SINSstate.GK_Longitude[10] = 45.91687866 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[11] = 51.65727808 * SimpleData.ToRadian; SINSstate.GK_Longitude[11] = 45.91672552 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[12] = 51.65722199 * SimpleData.ToRadian; SINSstate.GK_Longitude[12] = 45.91662125 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[13] = 51.65716790 * SimpleData.ToRadian; SINSstate.GK_Longitude[13] = 45.91655937 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[14] = 51.65711749 * SimpleData.ToRadian; SINSstate.GK_Longitude[14] = 45.91651203 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[15] = 51.65709286 * SimpleData.ToRadian; SINSstate.GK_Longitude[15] = 45.91641882 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[16] = 51.65708167 * SimpleData.ToRadian; SINSstate.GK_Longitude[16] = 45.91635739 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[17] = 51.65703936 * SimpleData.ToRadian; SINSstate.GK_Longitude[17] = 45.91614723 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[18] = 51.65698476 * SimpleData.ToRadian; SINSstate.GK_Longitude[18] = 45.91589917 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[19] = 51.65689765 * SimpleData.ToRadian; SINSstate.GK_Longitude[19] = 45.91550328 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[20] = 51.65689492 * SimpleData.ToRadian; SINSstate.GK_Longitude[20] = 45.91543676 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[21] = 51.65691551 * SimpleData.ToRadian; SINSstate.GK_Longitude[21] = 45.91540262 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[22] = 51.65704944 * SimpleData.ToRadian; SINSstate.GK_Longitude[22] = 45.91532339 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[23] = 51.65707977 * SimpleData.ToRadian; SINSstate.GK_Longitude[23] = 45.91531771 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[24] = 51.65710306 * SimpleData.ToRadian; SINSstate.GK_Longitude[24] = 45.91537133 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[25] = 51.65717924 * SimpleData.ToRadian; SINSstate.GK_Longitude[25] = 45.91572499 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[26] = 51.65727245 * SimpleData.ToRadian; SINSstate.GK_Longitude[26] = 45.91614317 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[27] = 51.65726635 * SimpleData.ToRadian; SINSstate.GK_Longitude[27] = 45.91623031 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[28] = 51.65724287 * SimpleData.ToRadian; SINSstate.GK_Longitude[28] = 45.91625902 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[29] = 51.65712406 * SimpleData.ToRadian; SINSstate.GK_Longitude[29] = 45.91633022 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[30] = 51.65709286 * SimpleData.ToRadian; SINSstate.GK_Longitude[30] = 45.91641882 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[31] = 51.65706786 * SimpleData.ToRadian; SINSstate.GK_Longitude[31] = 45.91655874 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[32] = 51.65670888 * SimpleData.ToRadian; SINSstate.GK_Longitude[32] = 45.91677764 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[33] = 51.65627597 * SimpleData.ToRadian; SINSstate.GK_Longitude[33] = 45.91704150 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[34] = 51.65622206 * SimpleData.ToRadian; SINSstate.GK_Longitude[34] = 45.91715524 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[35] = 51.65621375 * SimpleData.ToRadian; SINSstate.GK_Longitude[35] = 45.91737896 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[36] = 51.65622808 * SimpleData.ToRadian; SINSstate.GK_Longitude[36] = 45.91751543 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[37] = 51.65631557 * SimpleData.ToRadian; SINSstate.GK_Longitude[37] = 45.91793501 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[38] = 51.65658905 * SimpleData.ToRadian; SINSstate.GK_Longitude[38] = 45.91912273 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[39] = 51.65662906 * SimpleData.ToRadian; SINSstate.GK_Longitude[39] = 45.91918654 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[40] = 51.65668143 * SimpleData.ToRadian; SINSstate.GK_Longitude[40] = 45.91923808 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[41] = 51.65674261 * SimpleData.ToRadian; SINSstate.GK_Longitude[41] = 45.91923338 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[42] = 51.65747169 * SimpleData.ToRadian; SINSstate.GK_Longitude[42] = 45.91882389 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[43] = 51.65750359 * SimpleData.ToRadian; SINSstate.GK_Longitude[43] = 45.91878763 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[44] = 51.65752586 * SimpleData.ToRadian; SINSstate.GK_Longitude[44] = 45.91873369 * SimpleData.ToRadian;
            SINSstate.GK_Latitude[45] = 51.65752379 * SimpleData.ToRadian; SINSstate.GK_Longitude[45] = 45.91867080 * SimpleData.ToRadian;

            //SINSstate.Heading = -1.93;

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);
        }

    }
}
