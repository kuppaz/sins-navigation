using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace SINSProcessingModes
{
    public class SINS_Autonomous
    {

        public static void SINS_Autonomous_Processing(int l, StreamReader myFile, SINS_State SINSstate, SINS_State SINSstate2, Kalman_Vars KalmanVars, Proc_Help ProcHelp, SINS_State SINSstate_OdoMod, StreamWriter GRTV_output)
        {
            int t = 0;

            double[,] distance_GK_Sarat = new double[5, 46];

            StreamWriter STD_data = new StreamWriter(SimpleData.PathOutputString + "Debaging//S_STD.txt");
            StreamWriter ForHelp_2 = new StreamWriter(SimpleData.PathOutputString + "Debaging//ForHelp_2.txt");

            StreamWriter Nav_FeedbackSolution = new StreamWriter(SimpleData.PathOutputString + "S_SlnFeedBack.txt");
            StreamWriter Nav_EstimateSolution = new StreamWriter(SimpleData.PathOutputString + "S_SlnEstimate.txt");
            StreamWriter Nav_Errors = new StreamWriter(SimpleData.PathOutputString + "S_Errors.txt");
            StreamWriter Nav_Autonomous = new StreamWriter(SimpleData.PathOutputString + "S_Autonomous.txt");
            StreamWriter Nav_StateErrorsVector = new StreamWriter(SimpleData.PathOutputString + "S_ErrVect.txt");
            StreamWriter Nav_Smoothed = new StreamWriter(SimpleData.PathOutputString + "S_smoothed_SlnFeedBack.txt");
            StreamWriter ForHelp = new StreamWriter(SimpleData.PathOutputString + "Debaging//ForHelp.txt");
            StreamWriter KMLFileOut = new StreamWriter(SimpleData.PathOutputString + "KMLFiles//KMLFileOut_Forward.kml");
            StreamWriter KMLFileOutSmthd = new StreamWriter(SimpleData.PathOutputString + "KMLFiles//KMLFileOut_Smoothed.kml");

            StreamWriter Speed_Angles = new StreamWriter(SimpleData.PathOutputString + "Debaging//Speed_Angles.txt");
            StreamWriter DinamicOdometer = new StreamWriter(SimpleData.PathOutputString + "DinamicOdometer.txt");
            StreamWriter Cicle_Debag_Solution = new StreamWriter(SimpleData.PathOutputString + "Debaging//Solution_.txt");

            Nav_Errors.WriteLine("dLat  dLong  dV_x1  dV_x2  dV_x3  dHeading  dRoll  dPitch");
            Nav_Autonomous.WriteLine("Time OdoCnt OdoV Latitude Longitude Altitude LatSNS-Lat LngSNS-Lng LatSNS LongSNS LatSNSrad LongSNSrad SpeedSNS V_x1  V_x2  V_x3 Yaw  Roll  Pitch PosError PosError_Start Azimth");

            double[] dS_x = new double[3];

            SINSstate2.Latitude = SINSstate.Latitude;
            SINSstate2.Longitude = SINSstate.Longitude;



            //---Инициализация начальной матрицы ковариации---
            SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);

            //SINSstate.LastCountForRead = 100000;

            for (int i = l; i < SINSstate.LastCountForRead; i++)
            {
                if (SINSstate.flag_UsingClasAlignment == false) { if (i < ProcHelp.AlgnCnt) { myFile.ReadLine(); continue; } }

                ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);
                ProcessingHelp.DefSNSData(ProcHelp, SINSstate);

                if (t == 0) { SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z); SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z); t = 1; }

                if (SINSstate.OdometerData.odometer_left.isReady != 1)
                {
                    SINSstate.OdoTimeStepCount++;
                    SINSstate.flag_UsingCorrection = false;

                    //V_increment_SINS = V_increment_SINS + Math.Sqrt(Math.Pow(SINSstate.Vx_0[0] - SINSstate.Vx_0_prev[0], 2) + Math.Pow(SINSstate.Vx_0[1] - SINSstate.Vx_0_prev[1], 2) + Math.Pow(SINSstate.Vx_0[2] - SINSstate.Vx_0_prev[2], 2));
                }
                else if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    SINSstate.OdometerVector[0] = 0.0;
                    SINSstate.OdometerVector[2] = 0.0;
                    SINSstate.OdoTimeStepCount++;
                    SINSstate.OdometerVector[1] = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;
                    SimpleOperations.CopyArray(dS_x, SINSstate.A_x0s * SINSstate.OdometerVector);

                    SINSstate2.Latitude = SINSstate2.Latitude + dS_x[1] / SINSstate.R_n;
                    SINSstate2.Longitude = SINSstate2.Longitude + dS_x[0] / SINSstate.R_e / Math.Cos(SINSstate2.Latitude);

                    SINSstate.OdometerVector[1] = (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev) / SINSstate.OdoTimeStepCount / SINSstate.timeStep;
                    SimpleOperations.CopyArray(SINSstate.OdoSpeed_x0, SINSstate.A_x0s * SINSstate.OdometerVector);

                    SINSstate.flag_UsingCorrection = true;
                }

                SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate_OdoMod);
                //SINSprocessing.bins(SINSstate);

                SINSprocessing.Make_A(SINSstate, KalmanVars, SINSstate_OdoMod);
                //if (SINSstate.OdometerData.odometer_left.isReady == 1)
                //KalmanProcs.KalmanForecast(KalmanVars);

                double dT = SINSstate.timeStep;
                SINSstate.InertialOdometer = SINSstate.InertialOdometer + dT * (SINSstate.InertialOdometer_V + dT * (SINSstate.F_z[1] - SINSstate.g * Math.Sin(SINSstate.Pitch)));
                SINSstate.InertialOdometer_V = SINSstate.InertialOdometer_V + dT * (SINSstate.F_z[1] - SINSstate.g * Math.Sin(SINSstate.Pitch));
                if (i % 10 == 0)
                    ForHelp_2.WriteLine(SINSstate.Time + " " + SINSstate.InertialOdometer + " " + SINSstate.OdometerData.odometer_left.Value + " " + SINSstate.InertialOdometer_V + " " + SINSstate.OdoSpeed_s[1]);
                //SINSstate.OdometerData.odometer_left.Value = SINSstate.InertialOdometer;

                SimpleOperations.CopyArray(SINSstate.OdoSpeed_x0, SINSstate.A_x0s * SINSstate.OdometerVector);
                if (i % 10 == 0)
                    ForHelp.WriteLine(SINSstate.Time + " " + SINSstate.CourseHeading + " " + SINSstate.Heading + " " + SINSstate.CoursePitch + " " + SINSstate.beta_c + " " + SINSstate.alpha_c + " " + SINSstate.gamma_c
                        + " " + SINSstate.OdoSpeed_x0[0] + " " + SINSstate.OdoSpeed_x0[1] + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[1] + " " + SINSstate2.Vx_0[0] + " " + SINSstate2.Vx_0[1]
                        + " " + SINSstate.A_x0s[0, 1] + " " + SINSstate.A_x0s[1, 1] + " " + SINSstate.A_x0s[2, 1]);


                //ForHelp.WriteLine(((SINSstate2.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n).ToString() + " " + ((SINSstate2.Longitude - SINSstate.Longitude_Start) * SINSstate.R_n).ToString());
                //ForHelp.WriteLine(SINSstate.Count + " " + SINSstate.A_x0s[0, 0] + " " + SINSstate.A_x0s[1, 1] + " " + SINSstate.A_x0s[2, 2] + " " + SINSstate.A_x0s[0, 1] + " " + SINSstate.A_x0s[0, 2] + " " + SINSstate.A_x0s[1, 2]);

                /*----------------------------------------END---------------------------------------------*/



                /*------------------------------------OUTPUT-------------------------------------------------*/

                if (i > 10000 && i % 4000 == 0)
                    Console.WriteLine(SINSstate.Count.ToString()
                        + ",  FromSNS=" + Math.Round(ProcHelp.distance, 2) + " м" + ",  FromStart=" + Math.Round(ProcHelp.distance_from_start, 2) + " м"
                        + ",  Vx_1=" + Math.Round(SINSstate.Vx_0[0], 2) + ",  Vx_2=" + Math.Round(SINSstate.Vx_0[1], 3)
                        );

                ProcessingHelp.OutPutInfo(i, i, ProcHelp, SINSstate, SINSstate2, SINSstate2, SINSstate2, KalmanVars, Nav_EstimateSolution, Nav_Autonomous, Nav_FeedbackSolution, Nav_StateErrorsVector, Nav_Errors, STD_data, Speed_Angles, DinamicOdometer, Speed_Angles, KMLFileOut, KMLFileOut, GRTV_output, Cicle_Debag_Solution);

                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    if (SINSstate.flag_UsingCorrection == true)
                    {
                        SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                        SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                        SINSstate.OdoTimeStepCount = 0;
                    }
                }
            }

            ForHelp.Close(); Nav_FeedbackSolution.Close(); Nav_EstimateSolution.Close(); Nav_StateErrorsVector.Close();
        }
    }
}
