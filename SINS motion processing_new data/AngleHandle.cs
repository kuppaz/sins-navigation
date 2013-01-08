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


namespace SINS_motion_processing
{
    public class AngleHandle
    {
        int iMx = SimpleData.iMx = 15;
        int iMq = SimpleData.iMq = SimpleData.iMx;
        int iMz = SimpleData.iMz = 9;

        string datastring, datastringCovariance;
        string[] dataArray;
        double LatSNS, LongSNS, AltSNS, SpeedSNS, Ve_SNS, Vn_SNS, distance, V_odo;
        int AlgnCnt;
        bool initCount = false;
        int corrected = 0, tt =0;

        double Heading_start, Max_Position_Err_Abs;
        double[] Max_Position_Err = new double[10];

        StreamWriter Nav_SINSstate_UsingMeasures;
        StreamReader myFile;

        StreamWriter Alignment_Errors = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Alignment_Errors.dat");
        StreamWriter Alignment_SINSstate = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Alignment_SINSstate.dat");
        StreamWriter Alignment_Corrected_State = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Alignment_Corrected_State.dat");
        StreamWriter Alignment_StateErrorsVector = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Alignment_StateErrorsVector.dat");
        StreamWriter Alignment_avg_rougth = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Alignment_avg_rougth.dat");



        StreamWriter Heading_Search = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Heading_Search.dat");


        SINS_State SINSstate = new SINS_State();
        SINS_State SINSstate2 = new SINS_State();
        Kalman_Vars KalmanVars = new Kalman_Vars();

        //[DllImport("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//Motion Imitator//SINS_Aligment//SINS_Aligment.dll")]
        //unsafe private static extern int main();

        public void Main_Block_Click_new_Click()
        {
            for (int u = 0; u < 10; u++)
            {
                int t = 0, l = 0, Can = 0;

                if (u > 0)
                {
                    Nav_SINSstate_UsingMeasures.Dispose();
                    myFile.Dispose();
                }

                Nav_SINSstate_UsingMeasures = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//Nav_SINSstate_UsingMeasures.txt");
                myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//AzimutB_210530_Other_120814_Autolab_10-31-26_2.dat");
                //StreamReader myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//AzimutB_210530_Other_120815_Autolab_DPC_100Hz_14-40-04.dat");

                Nav_SINSstate_UsingMeasures.WriteLine("time  count  OdoStepCount  OdoSpeed  Latitude  Longitude  Altitude LatitudeSNS LongitudeSNS  AltitudeSNS  SpeedSNS  V_x1  V_x2  V_x3  Correct  Yaw  Roll  Pitch PositionError V_virtual_vert_chanel");

                SINSstate = new SINS_State();
                SINSstate2 = new SINS_State();
                KalmanVars = new Kalman_Vars();

                for (int o = 0; o < 10; o++)
                    Max_Position_Err[o] = 0.0;
                Max_Position_Err_Abs = 0.0;

                Heading_start = -(26.26266 + 1.0 * u);
                //SINSstate.Heading = -(115.751349 + 0.01 * u) * SimpleData.ToRadian;
                SINSstate.Heading = -(26.26266 + 1.0 * u) * SimpleData.ToRadian;
                //SINSstate.Roll = (0.5767 + 0.01 * u) * SimpleData.ToRadian;
                SINSstate.Roll = 1.753250 * SimpleData.ToRadian;
                //SINSstate.Pitch = -(0.2937195 + 0.01 * u) * SimpleData.ToRadian;
                SINSstate.Pitch = -1.510889 * SimpleData.ToRadian;



                SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = 56.264 * SimpleData.ToRadian;
                SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = 58.0 * SimpleData.ToRadian;
                SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Altitude = 92.37074;

                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);

                SINSstate.timeStep = SINSstate.Freq = 0.02048;
                //SINSprocessing.InitStartCondition(SINSstate, SINSstate.Global_file);       //---Инициализация начальных условий при отсутствии выставки---//
                SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);     //---Инициализация ковариационных матриц матриц вектора ошибок---//

                LatSNS = SINSstate.LatSNS * 180 / Math.PI;
                LongSNS = SINSstate.LongSNS * 180 / Math.PI;
                AltSNS = SINSstate.AltSNS;

                /*БЛОК ПАРАМЕТРОВ*/
                iMx = SimpleData.iMx = 15;
                iMq = SimpleData.iMq = SimpleData.iMx;

                AlgnCnt = 35000;

                //l = SINS_Alignment();

                string Angles = (SINSstate.Heading / SimpleData.ToRadian).ToString() + " " + (SINSstate.Roll / SimpleData.ToRadian).ToString() + " " + (SINSstate.Pitch / SimpleData.ToRadian).ToString();

                initCount = false;


                int o1 = 0, oo = 0;
                for (int i = l; i < 50009; i++)
                {

                    if (i < AlgnCnt) { myFile.ReadLine(); continue; }

                    o1++;
                    if (o1 == 26000)
                    {
                        o1 = 0;
                        oo++;
                    }

                    //if (i == 50000) break;

                    ReadSINSStateFromString();
                    DefSNSData();
                    if (t == 0) { SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z); SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z); t = 1; }


                    SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate2);

                    /*-------------------------------------------------------------------------------------*/
                    if (Math.Abs(distance) > Max_Position_Err[oo])
                        Max_Position_Err[oo] = Math.Abs(distance);

                    if (Math.Abs(distance) > Max_Position_Err_Abs)
                        Max_Position_Err_Abs = Math.Abs(distance);

                    if (i > 44000 && i % 2000 == 0)
                        Console.WriteLine(SINSstate.Count.ToString() + ",  " + (SINSstate.Latitude * SimpleData.ToDegree - LatSNS).ToString() + ",  " + distance.ToString() + ",  "
                                        + SINSstate.F_x[2].ToString().ToString() + ",  " + SINSstate.Altitude.ToString() + ",  " + AltSNS.ToString());
                    OutPutInfo();
                }

                Heading_Search.WriteLine(u.ToString() + " " + Heading_start.ToString() + " " + Max_Position_Err_Abs.ToString() + " " + Max_Position_Err[0].ToString() + " " + Max_Position_Err[1].ToString() + " " + Max_Position_Err[2].ToString()
                     + " " + Max_Position_Err[3].ToString() + " " + Max_Position_Err[4].ToString() + " " + Max_Position_Err[5].ToString() + " " + Max_Position_Err[6].ToString() + " " + Max_Position_Err[7].ToString() + " " + Max_Position_Err[8].ToString()
                      + " " + Max_Position_Err[9].ToString());
                //myFile.Close(); Nav_SINSstate_UsingMeasures.Close(); Nav_Corrected_State.Close(); Nav_StateErrorsVector.Close(); CovarianceMatrix.Close(); //this.Close();
            }
            Heading_Search.Close();
        }

        /*-------------------------------Вспомогательные функции---------------------------------------------------------*/
        private void ReadSINSStateFromString()
        {
            datastring = myFile.ReadLine();
            dataArray = datastring.Split(' ');
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

            //SINSstate.Time = Convert.ToDouble(dataArray2[0]);
            SINSstate.Count = Convert.ToDouble(dataArray2[0]);
            if (initCount == false) { initCount = true; SINSstate.initCount = SINSstate.Count - 1; }
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

        private void OutPutInfo()
        {
            //distance = Math.Sqrt(Math.Pow((SINSstate.Altitude - AltSNS),2) + Math.Pow((SINSstate.Latitude - LatSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(LatSNS, SINSstate.Altitude), 2) + Math.Pow((SINSstate.Longitude - LongSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(LatSNS, SINSstate.Altitude), 2));
            distance = Math.Sqrt(Math.Pow((SINSstate.Latitude - LatSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(LatSNS, SINSstate.Altitude), 2) + Math.Pow((SINSstate.Longitude - LongSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(LatSNS, SINSstate.Altitude), 2));



            datastring = (SINSstate.Count * SINSstate.timeStep).ToString() + " " + SINSstate.OdoTimeStepCount.ToString() + " " + SINSstate.OdometerVector[1].ToString() + " " + " " + (SINSstate.Latitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + " " +
                                SpeedSNS.ToString() + " " + SINSstate.Vx_0[0].ToString() + " " + SINSstate.Vx_0[1].ToString() + " " + SINSstate.Vx_0[2].ToString() + " " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + " "
                                  + (SINSstate.Roll * SimpleData.ToDegree).ToString() + " " + (SINSstate.Pitch * SimpleData.ToDegree).ToString() + " " + distance.ToString() + " " + SINSstate.Azimth.ToString();
            Nav_SINSstate_UsingMeasures.WriteLine(datastring);
        }

        private void DefSNSData()
        {
            if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
            {
                LatSNS = SINSstate.GPS_Data.gps_Latitude.Value * 180 / Math.PI;
                LongSNS = SINSstate.GPS_Data.gps_Longitude.Value * 180 / Math.PI;
                AltSNS = SINSstate.GPS_Data.gps_Altitude.Value;
                SpeedSNS = Math.Sqrt(SINSstate.GPS_Data.gps_Ve.Value * SINSstate.GPS_Data.gps_Ve.Value + SINSstate.GPS_Data.gps_Vn.Value * SINSstate.GPS_Data.gps_Vn.Value);
                Ve_SNS = SINSstate.GPS_Data.gps_Ve.Value;
                Vn_SNS = SINSstate.GPS_Data.gps_Vn.Value;
            }
        }

        public int RougthAlignment(int AlgnCnt)
        {
            int k = 0, i=0;
            double[] f_avg = new double[3]; double[] w_avg = new double[3]; double[] w_avg_x = new double[3]; double[] U_s = new double[3];
            Matrix A_xs = new Matrix(3,3);

            Alignment_Errors.WriteLine("dR1  dR2  dV1  dV2  Alpha1 Alpha2 Beta3  Nu1  Nu2  Nu3  dF1  dF2  dF3");
            Alignment_Corrected_State.WriteLine("Time  Count  LatCrtd Lat  LongCrtd    Long  AltitudeCrtd V1 V2 V3 HeadingCor Heading  Roll  Pitch");
            Alignment_avg_rougth.WriteLine("f_1 f_2 f_3 w_1 w_2 w_3 heading roll pitch Latitude");

            for (i = 0; i < AlgnCnt; i++)
            {
                if (i < 10) { myFile.ReadLine(); continue; }

                ReadSINSStateFromString();

                f_avg[0] += SINSstate.F_z[0];       w_avg[0] += SINSstate.W_z[0];
                f_avg[1] += SINSstate.F_z[1];       w_avg[1] += SINSstate.W_z[1];
                f_avg[2] += SINSstate.F_z[2];       w_avg[2] += SINSstate.W_z[2];
                k++;

                SINSstate.Pitch = Math.Atan(f_avg[1] / Math.Sqrt(f_avg[0] * f_avg[0] + f_avg[2] * f_avg[2]));
                SINSstate.Roll = -Math.Atan(f_avg[0] / f_avg[2]);
                A_xs = SimpleOperations.A_xs(SINSstate);
                w_avg_x = Matrix.Multiply(A_xs, w_avg);

                SINSstate.Heading = -Math.Atan(w_avg_x[0] / w_avg_x[1]);
                //SINSstate.Latitude = Math.Atan(w_avg_x[2] / Math.Sqrt(w_avg_x[1] * w_avg_x[1] + w_avg_x[0] * w_avg_x[0]));

                Alignment_avg_rougth.WriteLine((f_avg[0] / k).ToString() + " " + (f_avg[1] / k).ToString() + " " +(f_avg[2] / k).ToString() + " " + (w_avg[0] / k).ToString() + " " + (w_avg[1] / k).ToString() + " " + (w_avg[2] / k).ToString()
                        + " " + SINSstate.Heading.ToString() + " " + SINSstate.Roll.ToString() + " " + SINSstate.Pitch.ToString() + " " + SINSstate.Latitude.ToString());
            }

            f_avg[0] = f_avg[0] / k;    w_avg[0] = w_avg[0] / k;          
            f_avg[1] = f_avg[1] / k;    w_avg[1] = w_avg[1] / k;        
            f_avg[2] = f_avg[2] / k;    w_avg[2] = w_avg[2] / k;
                        
            SINSstate.Pitch = Math.Atan(f_avg[1] / Math.Sqrt(f_avg[0] * f_avg[0] + f_avg[2] * f_avg[2]));
            SINSstate.Roll = -Math.Atan(f_avg[0] / f_avg[2]);
            A_xs = SimpleOperations.A_xs(SINSstate);
            w_avg_x = Matrix.Multiply(A_xs, w_avg);

            SINSstate.Heading = -Math.Atan(w_avg_x[0] / w_avg_x[1]);
            //SINSstate.Latitude = Math.Atan(w_avg_x[2] / Math.Sqrt(w_avg_x[1] * w_avg_x[1] + w_avg_x[0] * w_avg_x[0]));

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);

            U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);
            if (Math.Abs(w_avg[0] - U_s[0]) < 0.00001) { }
            else
            {
                SINSstate.Heading = SINSstate.Heading - Math.PI;
                U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);
            }

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);

            return i;
        }


      
    }
}
