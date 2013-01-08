using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using Common_Namespace;

namespace OdoData
{
    public partial class data_exploration : Form
    {
        public data_exploration()
        {
            InitializeComponent();
        }

        string datastring = "";
        string[] dataArray;
        SINS_State SINSstate = new SINS_State();
        StreamReader myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//povorot_12-Sep-2012,13-26-38_dat.dat");
        //StreamReader myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//Motion Imitator//MovingImitator//SINS motion processing_new data//ktn004_marsh16_repeat_21-Mar-2012,17-21-07_dat.txt");
        StreamWriter Odo_Explaration = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//OdoData//Odo_Explaration.dat");
        

        private void button1_Click(object sender, EventArgs e)
        {
            int t = 0, moreThanOne = 0, tripling = 0, triplingAbs = 0, doublingAbs = 0, odoCount = 0, flag = 0, left_NULL = 0, right_NULL = 0, doubling_interval =0, GPS_cnt =0;
            double lastcount =0, dCount, lOdo = 0, rOdo = 0, dTime = 0.01024, dLeftOdo, dRightOdo, temp =0;
            double[] F_z_lastlast = new double[3];
            bool GPS_flg = false;
            myFile.ReadLine();

            int doubling_65_80 = 0, doubling_10_20 = 0, doubling_40_65 = 0, doubling_80_ = 0, doubling_20_40 = 0, doubling_0_5 = 0, doubling_5_10 = 0;

            //Odo_Explaration.WriteLine("count  LeftOdo  RightOdo  dLeft  dRight  V_l  V_r");


            myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//SarOEG_20121213.cnf");
            int ii = 0;
            SINSstate.Time = 11;
            datastring = myFile.ReadLine();

            for (int i = 0; i < 949545; i++)
            {

                string[] dataArray;

                datastring = myFile.ReadLine();
                dataArray = datastring.Split(' ');
                int tt = 0;

                for (int y = 0; y < dataArray.Length; y++)
                {
                    if (dataArray[y] != "")
                        tt++;
                }
                string[] dataArray2 = new string[tt];
                tt = 0;

                for (int y = 0; y < dataArray.Length; y++)
                {
                    if (dataArray[y] != "")
                    {
                        dataArray2[tt] = dataArray[y];
                        tt++;
                    }
                }

                if ((SINSstate.F_z[0] != Convert.ToDouble(dataArray2[16]) && SINSstate.F_z[1] != Convert.ToDouble(dataArray2[17]) && SINSstate.F_z[2] != Convert.ToDouble(dataArray2[18])) || ii == 0)
                //if (true)
                {
                    ii++;
                    SINSstate.Count = ii;

                    

                    dTime = Convert.ToDouble(dataArray2[0]) - SINSstate.Time; 

                    SINSstate.Time = Convert.ToDouble(dataArray2[0]);

                    SINSstate.F_z[0] = Convert.ToDouble(dataArray2[16]); SINSstate.W_z[0] = Convert.ToDouble(dataArray2[13]);
                    SINSstate.F_z[1] = Convert.ToDouble(dataArray2[17]); SINSstate.W_z[1] = Convert.ToDouble(dataArray2[14]);
                    SINSstate.F_z[2] = Convert.ToDouble(dataArray2[18]); SINSstate.W_z[2] = Convert.ToDouble(dataArray2[15]);

                    SINSstate.OdoSpeed[1] = Convert.ToDouble(dataArray2[24]);

                    Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + SINSstate.Time.ToString() + " " + dTime.ToString() + " " + SINSstate.F_z[0].ToString() + " " + SINSstate.F_z[1].ToString() + " " + SINSstate.F_z[2].ToString()
                             + " " + SINSstate.W_z[0].ToString() + " " + SINSstate.W_z[1].ToString() + " " + SINSstate.W_z[2].ToString() + " " + SINSstate.OdoSpeed[1].ToString());
                }

                if (i > 50000 && i % 10000 == 0)
                    Console.WriteLine(i.ToString());
            }

            myFile.Close();


            for (int i = 0; i < 60000; i++)
            {
                ReadSINSStateFromString();

                if (false)
                {
                    if (SINSstate.GPS_Data.gps_Ve.isReady == 1)
                        temp = Math.Sqrt(SINSstate.GPS_Data.gps_Ve.Value * SINSstate.GPS_Data.gps_Ve.Value + SINSstate.GPS_Data.gps_Vn.Value * SINSstate.GPS_Data.gps_Vn.Value);

                    if (SINSstate.OdometerData.odometer_right.isReady == 1)
                    {
                        odoCount++;
                        if (t == 0)
                        {
                            t++;
                            SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                            SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                            continue;
                        }

                        dLeftOdo = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;
                        dRightOdo = SINSstate.OdometerData.odometer_right.Value - SINSstate.OdometerRightPrev;
                        Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + odoCount.ToString() + " " + SINSstate.OdometerData.odometer_left.Value.ToString() + " " + SINSstate.OdometerData.odometer_right.Value.ToString() + " " + dLeftOdo.ToString()
                                         + " " + dRightOdo.ToString() + " " + (dLeftOdo / dTime / odoCount) + " " + (dRightOdo / dTime / odoCount) + " " + temp.ToString());

                        if (i > 30000 && i % 1000 == 0)
                            Console.WriteLine(SINSstate.Count.ToString());

                        SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                        SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                        odoCount = 0;
                    }
                    else
                        odoCount++;
                }
                else if (true)
                {
                    if (t == 0)
                    {
                        t++;
                        lastcount = SINSstate.Count - 1;
                    }

                    if (SINSstate.GPS_Data.gps_Latitude.Value != 0.0)
                        GPS_cnt++;
                    //else
                    //{
                    //    GPS_cnt++;
                    //    GPS_flg = true;
                    //}

                    dCount = SINSstate.Count - lastcount;
                    //if (Math.Abs(dCount) < 0.01)
                    //    moreThanOne++;
                    doubling_interval++;
                    if (Math.Abs(SINSstate.F_z[2] - SINSstate.F_z_prev[2]) < 0.00001)
                    {
                        int bucket = 0;

                        if (Math.Abs(SINSstate.F_z[2] - F_z_lastlast[2]) < 0.00001)
                        {
                            tripling++;
                            if (Math.Abs(SINSstate.F_z[1] - F_z_lastlast[1]) < 0.00001)
                                triplingAbs++;
                        }
                        moreThanOne++;
                        if (Math.Abs(SINSstate.F_z[1] - SINSstate.F_z_prev[1]) < 0.00001)
                        {
                            doublingAbs++;
                            if (doubling_interval > 65 && doubling_interval <= 75)
                            {
                                doubling_65_80++;
                                bucket = 6;
                            }
                            if (doubling_interval > 10 && doubling_interval <= 20)
                            {
                                doubling_10_20++;
                                bucket = 3;
                            }
                            if (doubling_interval > 5 && doubling_interval <= 10)
                            {
                                doubling_5_10++;
                                bucket = 2;
                            }
                            if (doubling_interval > 1 && doubling_interval <= 4)
                            {
                                doubling_0_5++;
                                bucket = 1;
                            }
                            if (doubling_interval > 20 && doubling_interval <= 40)
                            {
                                doubling_20_40++;
                                bucket = 4;
                            }
                            if (doubling_interval > 40 && doubling_interval <= 65)
                            {
                                doubling_40_65++;
                                bucket = 5;
                            }
                            if (doubling_interval > 80)
                            {
                                doubling_80_++;
                                bucket = 7;
                            }

                            //18.7152563310686 0.0617665225447807 0 0.0247066090179123 0.0617665225447807 81.0129709697344 0.123533045089561
                            
                            //Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + SINSstate.F_z[0].ToString() + " " + SINSstate.F_z[1].ToString() + " " + SINSstate.F_z[2].ToString() + " "
                            //             + " " + SINSstate.W_z[0].ToString() + " " + SINSstate.W_z[1].ToString() + " " + SINSstate.W_z[2].ToString() + " "
                            //            + moreThanOne.ToString() + " " + doublingAbs.ToString() + "  " + doubling_interval.ToString() + "  " + triplingAbs.ToString());
                            Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + SINSstate.F_z_prev[0].ToString() + " " + SINSstate.F_z_prev[1].ToString() + " " + SINSstate.F_z_prev[2].ToString()
                                        + " " + SINSstate.W_z[0].ToString() + " " + SINSstate.W_z[1].ToString() + " " + SINSstate.W_z[2].ToString()
                                        + " " + moreThanOne.ToString() + " " + doublingAbs.ToString() + "  " + doubling_interval.ToString() + "  " + triplingAbs.ToString() + " " + bucket.ToString());
                            //Console.WriteLine(SINSstate.Count.ToString());

                            doubling_interval = 0;
                        }
                        
                    }
                    SimpleOperations.CopyArray(F_z_lastlast, SINSstate.F_z_prev);

                    SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
                    SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
                }
                else if (false)
                {
                    if (SINSstate.GPS_Data.gps_Ve.isReady == 1)
                        temp = Math.Sqrt(SINSstate.GPS_Data.gps_Ve.Value * SINSstate.GPS_Data.gps_Ve.Value + SINSstate.GPS_Data.gps_Vn.Value * SINSstate.GPS_Data.gps_Vn.Value);

                    Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + SINSstate.GPS_Data.gps_Ve.Value.ToString() + " " + SINSstate.GPS_Data.gps_Vn.Value.ToString() +
                            " " + (temp).ToString());


                    if (i > 30000 && i % 1000 == 0)
                        Console.WriteLine(SINSstate.Count.ToString() + "  " + moreThanOne.ToString());
                    lastcount = SINSstate.Count;
                    SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z);

                }
                else
                {
                    if (SINSstate.OdometerData.odometer_left.isReady == 0)
                        left_NULL++;
                    if (SINSstate.OdometerData.odometer_right.isReady == 0)
                        right_NULL++;

                    if (true)
                    {
                        if ((SINSstate.OdometerData.odometer_left.isReady == 1 && SINSstate.OdometerData.odometer_right.isReady != 1) ||
                            (SINSstate.OdometerData.odometer_left.isReady != 1 && SINSstate.OdometerData.odometer_right.isReady == 1))
                            flag = 1;
                        else
                            flag = 0;
                    }

                    if (flag == 1)
                        Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + SINSstate.OdometerData.odometer_left.Value + " " + SINSstate.OdometerData.odometer_right.Value);

                    if (i > 30000 && i % 1000 == 0)
                        Console.WriteLine(left_NULL.ToString() + "  " + right_NULL.ToString());
                }

            }
            Console.WriteLine((100 * (double)doubling_0_5 / doublingAbs).ToString() + " " + (100 * (double)doubling_5_10 / doublingAbs).ToString() + " " + (100 * (double)doubling_10_20 / doublingAbs).ToString() + " " + (100 * (double)doubling_20_40 / doublingAbs).ToString() + " " + (100 * (double)doubling_40_65 / doublingAbs).ToString() + " " + (100 * (double)doubling_65_80 / doublingAbs).ToString() + " " + ((100 * (double)doubling_80_ / doublingAbs)).ToString());
            this.Close();
        }

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
}
