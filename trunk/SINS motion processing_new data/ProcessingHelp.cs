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
                                        + " " + SINSstate.OdometerVector[1].ToString() 
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
                if (SINSstate.iMx_r_odo_12)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_r_odo_12].ToString() + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_r_odo_12 + 1].ToString();
                if (SINSstate.iMx_kappa_13_ds)
                    ProcHelp.datastring = ProcHelp.datastring + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_kappa_13_ds].ToString() + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_kappa_13_ds + 1].ToString() + " " + KalmanVars.ErrorConditionVector_p[SINSstate.value_iMx_kappa_13_ds + 2].ToString();

                Nav_StateErrorsVector.WriteLine(ProcHelp.datastring);
            }

        }

    }
}
