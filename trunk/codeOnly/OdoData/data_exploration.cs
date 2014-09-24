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

        int last_count;

        string datastring = "";
        string[] dataArray;
        SINS_State SINSstate = new SINS_State();

        StreamReader myFile;

        StreamWriter Odo_Explaration = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//Odo_Explaration.dat");
        StreamWriter ExplarationLog = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//ExplarationLog.dat");
        StreamWriter enjoy_parse = new StreamWriter("D://enjoy_parse.txt");


        private void Express_Analisis_Click(object sender, EventArgs e)
        {
            string file_str2 = "D://LavNavSolution_2//Азимут-Т//Заезд 18.10.2013//данные 18.10.2013//2013_10_18(11_06)_ARM.txt";
            myFile = new StreamReader(file_str2);
            myFile.ReadLine();

            int cnt = 0;
            for (; ; )
            {
                file_str2 = myFile.ReadLine();
                dataArray = file_str2.Split(';');
                if (myFile.EndOfStream) break;

                SINSstate.Latitude = Convert.ToDouble(dataArray[3]);
                SINSstate.Longitude = Convert.ToDouble(dataArray[4]);

                SINSprocessing.Phi_Lambda_GAUSS_KRUGER(SINSstate.Latitude, SINSstate.Longitude, SINSstate, 0);

                SINSstate.GK_Latitude[0] = SINSstate.GK_Latitude[0];
                SINSstate.GK_Longitude[0] = SINSstate.GK_Longitude[0];
                cnt++;
            }
            myFile.Close();
            myFile = new StreamReader(file_str2);






            int LastCountForRead = 0;
            string file_str = "D://LavNavSolution_2//Азимут-Т//Заезд 18.10.2013//данные 18.10.2013//2013_10_18(11_06)_TLM.txt";
            myFile = new StreamReader(file_str);
            for (; ; )
            {
                myFile.ReadLine();
                if (myFile.EndOfStream) break;
                LastCountForRead++;
            }
            myFile.Close();
            myFile = new StreamReader(file_str);


            int CountDoubledF = 0, CountTrippledF = 0, CountDoubledW = 0, CountTrippledW = 0, CountSNS = 0, CountSNSZero = 0, OdoImpuls = 0;
            int[] OdoImpulsIncrement = new int[LastCountForRead];
            double Time = 0.0, TimePrev = 0.0, TimeStart = 0.0, LatSNS = 0.0, LongSNS = 0.0, OdoCoeff = 0.0; ;
            double[] Fz = new double[3], Fz_Prev = new double[3], Fz_Prev2 = new double[3], Wz = new double[3], Wz_Prev = new double[3], Wz_Prev2 = new double[3], dT = new double[LastCountForRead];
            double[] AbsNDoubled_F = new double[100000], AbsNDoubled_W = new double[100000], AbsNTrippled_F = new double[100000], AbsNTrippled_W = new double[100000];
            double Abs_N_Doubled_F = 0, Abs_N_Doubled_W = 0, Abs_N_Trippled_F = 0, Abs_N_Trippled_W = 0;
            string strRead = "";
            string[] strReadArray;

            strRead = myFile.ReadLine();

            for (int i = 0; ; i++)
            {
                if (myFile.EndOfStream == true) break;

                strRead = myFile.ReadLine();
                strReadArray = strRead.Split(';');

                Time = Convert.ToDouble(strReadArray[0].Split(':')[0]) * 3600.0 + Convert.ToDouble(strReadArray[0].Split(':')[1]) * 60 + Convert.ToDouble(strReadArray[0].Split(':')[2]);

                Fz[0] = Convert.ToDouble(strReadArray[6]);
                Fz[1] = Convert.ToDouble(strReadArray[4]);
                Fz[2] = Convert.ToDouble(strReadArray[5]);
                Wz[0] = Convert.ToDouble(strReadArray[3]);
                Wz[1] = Convert.ToDouble(strReadArray[1]);
                Wz[2] = Convert.ToDouble(strReadArray[2]);

                //---SNS---//
                if (strReadArray.Length >= 10)
                {
                    CountSNS++;
                    LatSNS = Convert.ToDouble(strReadArray[14]);
                    LongSNS = Convert.ToDouble(strReadArray[15]);
                    OdoCoeff = Convert.ToDouble(strReadArray[31]);

                    if (LatSNS < 0.00001 && LongSNS < 0.00001)
                        CountSNSZero++;
                }

                if (Fz[0] < 0.00001 && Fz[1] < 0.00001 && Fz[2] < 0.00001)
                {
                    i = -1;
                    continue;
                }

                if (i == 0)
                {
                    TimePrev = Time - 0.02048;
                    TimeStart = Time;
                }

                //---ODOMETER---//
                OdoImpulsIncrement[i] = Convert.ToInt32(strReadArray[7]) - OdoImpuls;
                OdoImpuls = Convert.ToInt32(strReadArray[7]);

                //---ACCS & DUS---//
                if (Math.Abs(Fz[0] - Fz_Prev[0]) < 0.0000001 && Math.Abs(Fz[1] - Fz_Prev[1]) < 0.0000001 && Math.Abs(Fz[2] - Fz_Prev[2]) < 0.0000001)
                {
                    if (Math.Abs(Fz_Prev2[0] - Fz_Prev[0]) < 0.0000001 && Math.Abs(Fz_Prev2[1] - Fz_Prev[1]) < 0.0000001 && Math.Abs(Fz_Prev2[2] - Fz_Prev[2]) < 0.0000001)
                    {
                        AbsNTrippled_F[CountTrippledF] = i - Abs_N_Trippled_F;
                        Abs_N_Trippled_F = i;
                        CountTrippledF++;
                    }
                    else
                    {
                        AbsNDoubled_F[CountDoubledF] = i - Abs_N_Doubled_F;
                        Abs_N_Doubled_F = i;
                        CountDoubledF++;
                    }
                }

                if (Math.Abs(Wz[0] - Wz_Prev[0]) < 0.0000001 && Math.Abs(Wz[1] - Wz_Prev[1]) < 0.0000001 && Math.Abs(Wz[2] - Wz_Prev[2]) < 0.0000001)
                {
                    if (Math.Abs(Wz_Prev2[0] - Wz_Prev[0]) < 0.0000001 && Math.Abs(Wz_Prev2[1] - Wz_Prev[1]) < 0.0000001 && Math.Abs(Wz_Prev2[2] - Wz_Prev[2]) < 0.0000001)
                    {
                        AbsNTrippled_W[CountTrippledW] = i - Abs_N_Trippled_W;
                        Abs_N_Trippled_W = i;
                        CountTrippledW++;
                    }
                    else
                    {
                        AbsNDoubled_F[CountDoubledW] = i - Abs_N_Doubled_W;
                        Abs_N_Doubled_W = i;
                        CountDoubledW++;
                    }
                }

                dT[i] = Time - TimePrev;

                Odo_Explaration.WriteLine(i + ";" + OdoImpulsIncrement[i].ToString() + ";" + strRead);

                //-------
                TimePrev = Time;
                for (int j = 0; j < 3; j++)
                {
                    Fz_Prev2[j] = Fz_Prev[j];
                    Fz_Prev[j] = Fz[j];
                }
            }

            double mean_AbsNDoubled_F = 0.0, mean_AbsNDoubled_W = 0.0, mean_AbsNTrippled_F = 0.0, mean_AbsNTrippled_W = 0.0;

            for (int i = 0; i < CountDoubledF; i++)
                mean_AbsNDoubled_F += AbsNDoubled_F[i];
            for (int i = 0; i < CountDoubledW; i++)
                mean_AbsNDoubled_W += AbsNDoubled_W[i];
            for (int i = 0; i < CountTrippledF; i++)
                mean_AbsNTrippled_F += AbsNTrippled_F[i];
            for (int i = 0; i < CountTrippledW; i++)
                mean_AbsNTrippled_W += AbsNTrippled_W[i];

            mean_AbsNDoubled_F = mean_AbsNDoubled_F / CountDoubledF;
            mean_AbsNDoubled_W = mean_AbsNDoubled_W / CountDoubledW;
            mean_AbsNTrippled_F = mean_AbsNTrippled_F / CountTrippledF;
            mean_AbsNTrippled_W = mean_AbsNTrippled_W / CountTrippledW;

            ExplarationLog.WriteLine("Всего тактов в обработанном файле, зарегистрированных за интервал времени " + Math.Round(Time - TimeStart, 1) + " секунд: " + LastCountForRead.ToString());

            ExplarationLog.WriteLine(" ");
            if (CountDoubledF != 0) ExplarationLog.WriteLine("Колличество залипаний на 2 тактах по ACCS: " + CountDoubledF.ToString() + ". В среднем повторение раз в " + Math.Round(mean_AbsNDoubled_F, 0) + " тактов БИНС.");
            if (CountTrippledF != 0) ExplarationLog.WriteLine("Колличество залипаний на 3 тактах по ACCS: " + CountTrippledF.ToString() + ". В среднем повторение раз в " + Math.Round(mean_AbsNTrippled_F, 0) + " тактов БИНС.");
            if (CountDoubledW != 0) ExplarationLog.WriteLine("Колличество залипаний на 2 тактах по ДУС: " + CountDoubledW.ToString() + ". В среднем повторение раз в " + Math.Round(mean_AbsNDoubled_W, 0) + " тактов БИНС.");
            if (CountTrippledW != 0) ExplarationLog.WriteLine("Колличество залипаний на 3 тактах по ДУС: " + CountTrippledW.ToString() + ". В среднем повторение раз в " + Math.Round(mean_AbsNTrippled_W, 0) + " тактов БИНС.");

            ExplarationLog.WriteLine(" ");
            if (CountSNSZero != 0) ExplarationLog.WriteLine("Нулевые значения СНС координаты принимают в " + CountSNSZero.ToString() + " тактах, что составляет "
                                                        + Math.Round(Convert.ToDouble(CountSNSZero * 100.0 / Convert.ToDouble(CountSNS)), 1) + "% от общего числа тактов, содержащих расширенную информацию.");

            ExplarationLog.WriteLine(" ");
            ExplarationLog.WriteLine("Максимальное приращение импульсов одометра: " + Math.Abs(OdoImpulsIncrement.Min()) + ", что соответсвует пройденному пути в "
                                                        + Math.Round(OdoCoeff * OdoImpulsIncrement.Min(), 3) + " метр.");


            //------------------------------------------------------------//
            int ZeroTimeStep = 0;
            double avg_dT = 0.0, avg_std_dT = 0.0;
            for (int i = 0; i < LastCountForRead; i++)
            {
                if (dT[i] < 0.001)
                    ZeroTimeStep++;
                avg_dT += dT[i];
            }

            avg_dT = avg_dT / LastCountForRead;
            int[] odoImpCounts = new int[20];
            for (int i = 0; i < LastCountForRead; i++)
            {
                avg_std_dT += Math.Abs(avg_dT - dT[i]);
                for (int j = 1; j <= 20; j++)
                {
                    if (Math.Abs(OdoImpulsIncrement[i]) == j) 
                        odoImpCounts[j]++;
                }
            }
            avg_std_dT = avg_std_dT / LastCountForRead;

            ExplarationLog.WriteLine(" ");
            ExplarationLog.WriteLine("С учетом частоты регистрации данных одометра, распределение приращений импульсов следующее:");
            for (int i = 0; i < 20; i++)
            {
                if (odoImpCounts[i] == 0) continue;
                ExplarationLog.WriteLine("--Колличество приращений импульсов одометра величины " + i.ToString() + " - " + odoImpCounts[i].ToString() 
                                            + " (" + Math.Round(Convert.ToDouble(odoImpCounts[i]) * 100.0/LastCountForRead, 2) + "%), что соответсвует пройденному пути в "
                                                            + Math.Abs(Math.Round(OdoCoeff * i, 3)) + " метр, или скорости " + Math.Abs(Math.Round(OdoCoeff * i / 0.02048, 3)) + " м/с.");
            }


            ExplarationLog.WriteLine(" ");
            if (ZeroTimeStep != 0) ExplarationLog.WriteLine("Нулевых приращений метки времени: " + ZeroTimeStep.ToString() + ", что составляет "
                                                            + Math.Round(Convert.ToDouble(ZeroTimeStep * 100.0 / Convert.ToDouble(LastCountForRead)), 1) + "% от общего числа тактов.");
            else
                ExplarationLog.WriteLine("Нулевых приращений метки времени нет.");
            ExplarationLog.WriteLine("Максимальное приращение времени: " + Math.Round(dT.Max(), 3) + " секунд.");
            ExplarationLog.WriteLine("Среднее значение приращения времени: " + Math.Round(avg_dT, 3) + " секунд.");
            ExplarationLog.WriteLine("Среднее значение отклонения от среднего приращения времени: " + Math.Round(avg_std_dT, 5) + " секунд.");

            

            myFile.Close();
            ExplarationLog.Close();

            this.Close();
        }
        











        private void button1_Click(object sender, EventArgs e)
        {
            int t = 0, moreThanOne = 0, tripling = 0, triplingAbs = 0, doublingAbs = 0, odoCount = 0, flag = 0, left_NULL = 0, right_NULL = 0, doubling_interval =0, GPS_cnt =0;
            double lastcount =0, dCount, lOdo = 0, rOdo = 0, dTime = 0.01024, dLeftOdo, dRightOdo, temp =0;
            double[] F_z_lastlast = new double[3], W_z_lastlast = new double[3];
            bool GPS_flg = false;

            //myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//povorot_12-Sep-2012,13-26-38_dat.dat");
            //myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//Motion Imitator//MovingImitator//SINS motion processing_new data//ktn004_marsh16_repeat_21-Mar-2012,17-21-07_dat.txt");

            myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//120222_AzimutB_210530_Race_4_Control_3-6-2_11-49-20_dat.dat"); last_count = 77000;
            //myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//120222_AzimutB_210530_Race_2_Adjustment_6-3_DPCnotCal_10-27-49_dat.dat"); last_count = 55370;
            //myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//120222_AzimutB_210530_Race_3_Adjustment_6-3_11-19-46_dat.dat"); last_count = 51525;
            //myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//120222_AzimutB_210530_Race_5_Control_2-3_12-26-56_dat.dat"); last_count = 66100;
            //myFile = new StreamReader("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//All_data//120222_AzimutB_210530_Race_6_Control_3-2_12-55-50_dat.dat"); last_count = 47000;

            //StreamReader myFile = new StreamReader("D://enjoy2.txt");
            //myFile.ReadLine();

            int doubling_65_80 = 0, doubling_10_20 = 0, doubling_40_65 = 0, doubling_80_ = 0, doubling_20_40 = 0, doubling_0_5 = 0, doubling_5_10 = 0;

            //Odo_Explaration.WriteLine("count  LeftOdo  RightOdo  dLeft  dRight  V_l  V_r");


            //myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//All_data//SarOEG_20121213.cnf");
            int ii = 0;
            SINSstate.Time = 11;

            string datastring_prev = "";
            datastring_prev = myFile.ReadLine();

            //int j = 0, iii=0;
            //for (iii = 0; iii < 1411; iii++)
            //{
            //    datastring = myFile.ReadLine();

            //    if (datastring.Contains("Автор") == true)
            //    {
            //        enjoy_parse.WriteLine(datastring_prev);
            //    }

            //    datastring_prev = datastring;
            //}

            //enjoy_parse.Close();

            if (false)
            {
                for (int i = 0; i < last_count; i++)
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

                        SINSstate.OdoSpeed_x0[1] = Convert.ToDouble(dataArray2[24]);

                        Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + SINSstate.Time.ToString() + " " + dTime.ToString() + " " + SINSstate.F_z[0].ToString() + " " + SINSstate.F_z[1].ToString() + " " + SINSstate.F_z[2].ToString()
                                 + " " + SINSstate.W_z[0].ToString() + " " + SINSstate.W_z[1].ToString() + " " + SINSstate.W_z[2].ToString() + " " + SINSstate.OdoSpeed_x0[1].ToString());
                    }

                    if (i > 10000 && i % 10000 == 0)
                        Console.WriteLine(i.ToString());
                }

                myFile.Close();
            }
            else if (true)
            {
                for (int i = 0; i < last_count; i++)
                {
                    SINSstate.timeStep = dTime = 0.02048;
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

                            //if (SINSstate.Count * SINSstate.timeStep < 590.0) SINSstate.FLG_Stop = 1;
                            //else if (SINSstate.Count * SINSstate.timeStep > 885.0 && SINSstate.Count * SINSstate.timeStep < 1090.0) SINSstate.FLG_Stop = 1;
                            //else if (SINSstate.Count * SINSstate.timeStep > 1228.0 && SINSstate.Count * SINSstate.timeStep < 1260.0) SINSstate.FLG_Stop = 1;
                            //else if (SINSstate.Count * SINSstate.timeStep > 1300.0 && SINSstate.Count * SINSstate.timeStep < 1360.0) SINSstate.FLG_Stop = 1;
                            //else if (SINSstate.Count * SINSstate.timeStep > 1455.0) SINSstate.FLG_Stop = 1;
                            //else SINSstate.FLG_Stop = 0;

                            dLeftOdo = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;
                            dRightOdo = SINSstate.OdometerData.odometer_right.Value - SINSstate.OdometerRightPrev;
                            Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + odoCount.ToString() + " " + SINSstate.OdometerData.odometer_left.Value.ToString() + " " + SINSstate.OdometerData.odometer_right.Value.ToString() + " " + dLeftOdo.ToString()
                                             + " " + dRightOdo.ToString() + " " + (dLeftOdo / dTime / odoCount) + " " + (dRightOdo / dTime / odoCount) + " " + SINSstate.FLG_Stop.ToString());

                            if (i > 30000 && i % 1000 == 0)
                                Console.WriteLine(SINSstate.Count.ToString());

                            SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                            SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                            odoCount = 0;
                        }
                        else
                            odoCount++;
                    }
                    else if (false)
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
                        if (true)
                        {
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
                                    if (Math.Abs(SINSstate.F_z[0] - SINSstate.F_z_prev[0]) < 0.00001)
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

                            }
                        }
                        else
                        {
                            if (Math.Abs(SINSstate.W_z[2] - SINSstate.W_z_prev[2]) < 0.000001)
                            {
                                int bucket = 0;

                                if (Math.Abs(SINSstate.W_z[2] - W_z_lastlast[2]) < 0.000001)
                                {
                                    tripling++;
                                    if (Math.Abs(SINSstate.W_z[1] - W_z_lastlast[1]) < 0.000001)
                                        triplingAbs++;
                                }
                                moreThanOne++;
                                if (Math.Abs(SINSstate.W_z[1] - SINSstate.W_z_prev[1]) < 0.000001)
                                {
                                    if (Math.Abs(SINSstate.W_z[0] - SINSstate.W_z_prev[0]) < 0.000001)
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

                            }
                        }
                        SimpleOperations.CopyArray(F_z_lastlast, SINSstate.F_z_prev);
                        SimpleOperations.CopyArray(W_z_lastlast, SINSstate.W_z_prev);

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

                        SINSstate.OdoTimeStepCount++;

                        if (true)
                        {
                            if ((SINSstate.OdometerData.odometer_left.isReady == 1 && SINSstate.OdometerData.odometer_right.isReady != 1) ||
                                (SINSstate.OdometerData.odometer_left.isReady != 1 && SINSstate.OdometerData.odometer_right.isReady == 1))
                                flag = 1;
                            else
                                flag = 0;
                        }

                        if (SINSstate.OdometerData.odometer_left.isReady == 1)
                        {
                            Odo_Explaration.WriteLine(SINSstate.Count.ToString() + " " + SINSstate.OdoTimeStepCount.ToString() + " " + SINSstate.OdometerData.odometer_left.Value + " " + SINSstate.OdometerData.odometer_right.Value);
                            SINSstate.OdoTimeStepCount = 0;
                        }

                        if (i > 30000 && i % 1000 == 0)
                            Console.WriteLine(left_NULL.ToString() + "  " + right_NULL.ToString());
                    }

                }
                Console.WriteLine((100 * (double)doubling_0_5 / doublingAbs).ToString() + " " + (100 * (double)doubling_5_10 / doublingAbs).ToString() + " " + (100 * (double)doubling_10_20 / doublingAbs).ToString() + " " + (100 * (double)doubling_20_40 / doublingAbs).ToString() + " " + (100 * (double)doubling_40_65 / doublingAbs).ToString() + " " + (100 * (double)doubling_65_80 / doublingAbs).ToString() + " " + ((100 * (double)doubling_80_ / doublingAbs)).ToString());
            }
            Odo_Explaration.Close();
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
