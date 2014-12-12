using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using Common_Namespace;

namespace MovingImitator
{
    public partial class Imitation : Form
    {
        //Основные постоянные величины
        public double StartHeading = 45.0 * SimpleData.ToRadian,
                      StartRoll = 0.0 * SimpleData.ToRadian,
                      StartPitch = 0.0 * SimpleData.ToRadian;

        public double StartLatitude = 55.0 * SimpleData.ToRadian,
                      StartLongitude = 37.0 * SimpleData.ToRadian,
                      StartAltitude = 100.0;

        double dT = 0.005;

        public Imitation()
        {
            InitializeComponent();
        }


        public void button1_Click(object sender, EventArgs e)
        {

            //----------------------------------------------------------------------------------------------------------------------------//

            //Объявление необходимых переменных
            int flag_stop = 0;
            double CurrentTime = 0.0, OdometerMeasure = 0.0, OdometerMeasure_prev_flag = 0.0, Params_OdoKappa1, Params_OdoKappa3, Alignment_End = 0.0;

            double[] dOrientationAngles = new double[3];

            double[] temp = new double[3], Acceleration_s = new double[3], Acceleration_x0 = new double[3],
                    RelativeAngular_sx0 = new double[3], RelativeAngular_x0s = new double[3], RelativeAngular_x0 = new double[3], RelativeAngular_x0_in_s = new double[3];

            double[] OdometerCoordinate_s = new double[3],
                     OdometerCoordinate_x0 = new double[3],
                     OdometerPosition = new double[3],
                     OdometerVelocity_x0 = new double[3];


            double odometer_left_ValueTrue = 0.0, Params_OdoIncrement = 0.0, Params_OdoScaleErr = 0.0;
            double Params_df_0 = 0.0;
            double Params_dnu_0 = 0.0;
            double Params_df_s = 0.0;
            double Params_dnu_s = 0.0;

            int OdometerMeasure_flag = 2, Params_OdoFrequency = 5;

            SINS_State SINSstate = new SINS_State();

            Matrix A_odoZ = new Matrix(3, 3);

            //FileStream FileExitInfoClear = new FileStream("ExitInfoClear.dat", FileMode.OpenOrCreate, FileAccess.Write);
            StreamWriter ExitInfoClear = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//Imitator_data//ExitInfoClear.dat");
            ExitInfoClear.WriteLine("Time \t f_s1 \t f_s2 \t f_s3 \t W_s1 \t W_s2 \t W_s3 \t Velocity_s[0] \t Velocity_s[1] \t Velocity_s[2] \t Velocity_x0_1 \t Velocity_x0_2 \t Velocity_x0_3 CourseHeading  CoursePitch  beta_c  alpha_c  gamma_c  \t Heading \t Roll \t Pitch \t Latitude \t Longitude \t Altitude \t OdoLatitude \t OdoLongitude \t OdoAltitude");

            StreamWriter Imitator_Data_for_Process = new StreamWriter(SimpleData.PathInputString + "Imitator_Analytic_Clear.dat");
            //Imitator_Data_for_Process.WriteLine("Count Fx[m/s^2] Fy[m/s^2] Fz[m/s^2] omega_x[r/s] omega_y[r/s] omega_z[r/s] Lat[d] isN Lon[d] isN Hei[m] isN Vn[m/s] isN Ve[m/s] isN StopFLG odo_left odo_left_flag odo_right odo_right_flag");

            //Выставление начальных условий для вектора углов ориентации
            SINSstate.Heading = StartHeading;
            SINSstate.Latitude = SINSstate.Latitude_prev = SINSstate.GPS_Data.gps_Latitude.Value = StartLatitude; OdometerCoordinate_s[0] = 0.0;// В осях приборного трехгранника
            SINSstate.Roll = StartRoll;
            SINSstate.Longitude = SINSstate.Longitude_prev = SINSstate.GPS_Data.gps_Longitude.Value = StartLongitude; OdometerCoordinate_s[1] = 0.0;
            SINSstate.Pitch = StartPitch;
            SINSstate.Altitude = SINSstate.Altitude_prev = SINSstate.GPS_Data.gps_Altitude.Value = StartAltitude; OdometerCoordinate_s[2] = 0.0;
            SINSstate.Heading_prev = SINSstate.Heading;
            SINSstate.Roll_prev = SINSstate.Roll;
            SINSstate.Pitch_prev = SINSstate.Pitch;




            //----------------------------------------------------------------------------------------//
            Params_OdoKappa1 = 0 * SimpleData.ToRadian;
            Params_OdoKappa3 = -0 * SimpleData.ToRadian;
            Params_OdoIncrement = 0.0; // в сантиметрах
            Params_OdoScaleErr = 1.0;
            Params_OdoFrequency = 5;

            Params_df_0 = 0.0; //далее умножается G
            Params_dnu_0 = 0.0; //град/час
            Params_df_s = 0.0; //(rnd_1.NextDouble() - 0.5) / Params_df_s //100.0 - норма
            Params_dnu_s = 0.0; //(rnd_5.NextDouble() - 0.5) / Params_dnu_s //10000.0 - норма
            //----------------------------------------------------------------------------------------//






            Imitator_Data_for_Process.WriteLine("Latitude= " + StartLatitude + " Longitude= " + StartLongitude + " Height= " + StartAltitude + " SINS_Freq= " + 1.0 / dT + " df_0= " + Params_df_0 + " df_s= " + Params_df_s
                + " nu_0= " + Params_dnu_0 + " nu_s= " + Params_dnu_s + " OdoKappa1= " + Params_OdoKappa1 + " OdoKappa3= " + Math.Abs(Params_OdoKappa3) + " OdoScale= " + Params_OdoScaleErr + " OdoIncrement= " + Params_OdoIncrement
                + " OdoFreq= " + Params_OdoFrequency + " Heading= " + (StartHeading - Params_OdoKappa3).ToString() + " Roll= " + StartRoll + " Pitch= " + (StartPitch + Params_OdoKappa1).ToString());


            Random rnd_1 = new Random(), rnd_2 = new Random(), rnd_3 = new Random(), rnd_4 = new Random(), rnd_5 = new Random(), rnd_6 = new Random();

            int AlignmentCount = 10000;
            double CurTimeWithAlign = 0.0;
            ///////////////////////////////////////////// Рабочий цикл /////////////////////////////////////////////////
            while (CurrentTime < 1520.0)
            //while (CurTimeWithAlign < 26000.0)
            {
                SINSstate.Count++;
                CurrentTime += dT;

                if (SINSstate.Count <= AlignmentCount)
                {
                    Alignment_End = CurrentTime;
                    SINSstate.FLG_Stop = 1;
                }
                else
                    SINSstate.FLG_Stop = 0;

                CurTimeWithAlign = CurrentTime - Alignment_End;



                //Задание углов ориентации Курса, крена, тангажа и их производных
                //SINSstate.Heading = StartHeading + 45.0 * SimpleData.ToRadian * Math.Sin(Math.PI / 1000.0 * CurTimeWithAlign);
                if (CurTimeWithAlign <= 13000.0) SINSstate.Heading = StartHeading + 45.0 * SimpleData.ToRadian * Math.Sin(Math.PI / 100.0 * CurTimeWithAlign);
                else if (CurTimeWithAlign > 13000.0 && CurTimeWithAlign <= 13100.0) SINSstate.Heading = StartHeading + Math.PI / 3.0 * Math.Sin(Math.PI / 2.0 * (CurTimeWithAlign - 13000.0) / 100.0);
                else
                    SINSstate.Heading = StartHeading + Math.PI / 3.0 + 45.0 * SimpleData.ToRadian * Math.Sin(Math.PI / 100.0 * CurTimeWithAlign);

                //if (CurTimeWithAlign > 75.0 && CurTimeWithAlign <= 95.0) SINSstate.Heading = StartHeading + Math.PI / 2.0 * (CurTimeWithAlign - 75.0) / 20.0;
                //if (CurTimeWithAlign > 300.0 && CurTimeWithAlign <= 400.0) SINSstate.Heading = StartHeading + Math.PI / 2.0 - Math.PI * (CurTimeWithAlign - 300.0) / 100.0;
                //if (CurTimeWithAlign > 400.0 && CurTimeWithAlign <= 410.0) SINSstate.Heading = StartHeading - Math.PI / 2.0 + Math.PI / 2.0 * (CurTimeWithAlign - 400.0) / 20.0;

                //if (CurTimeWithAlign > 1300.0 && CurTimeWithAlign <= 1350.0) SINSstate.Heading = StartHeading + Math.PI / 2.0 + Math.PI / 2.0 * (CurTimeWithAlign - 1300.0) / 50.0;
                //SINSstate.Roll = StartRoll + 5.0 * SimpleData.ToRadian * Math.Sin(0.021 * CurTimeWithAlign);
                //SINSstate.Pitch = StartPitch + 1.1 * SimpleData.ToRadian * Math.Sin(0.025 * CurTimeWithAlign);
                //if (CurTimeWithAlign > 200.0 && CurTimeWithAlign <= 250.0) SINSstate.Pitch = StartPitch + 0.1 * Math.Sin(Math.PI * (CurTimeWithAlign - 200.0) / 50.0);



                if (SINSstate.Heading >= Math.PI)
                {
                    SINSstate.Heading = SINSstate.Heading - 2.0 * Math.PI;
                    if (Math.Sign(SINSstate.Heading) != Math.Sign(SINSstate.Heading_prev) && Math.Abs(SINSstate.Heading - SINSstate.Heading_prev) > 0.1)
                        SINSstate.Heading_prev = SINSstate.Heading_prev - 2.0 * Math.PI;
                }
                if (SINSstate.Heading < -Math.PI)
                {
                    SINSstate.Heading = SINSstate.Heading + 2.0 * Math.PI;
                    if (Math.Sign(SINSstate.Heading) != Math.Sign(SINSstate.Heading_prev) && Math.Abs(SINSstate.Heading - SINSstate.Heading_prev) > 0.1)
                        SINSstate.Heading_prev = SINSstate.Heading_prev + 2.0 * Math.PI;
                }
                //SimpleOperations.AnglesHRP(CurrentTime - Alignment_End, SINSstate, StartHeading, StartRoll, StartPitch);
                dOrientationAngles = SimpleOperations.DerivativeOfAnglesHRP(dT, SINSstate);
                //dOrientationAngles = SimpleOperations.DerivativeOfAnglesHRP_analitic(CurrentTime - Alignment_End);

                //Задание матриц ориентации между основными трехгранниками
                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                //A_odoZ = SimpleOperations.A_odoZ(kappa1, kappa3);
                //---Формирование скоростной системы координат---//
                if (SimpleOperations.AbsoluteVectorValue(SINSstate.Vx_0) > 0.00001)
                {
                    SINSstate.CoursePitch = Math.Atan2(SINSstate.Vx_0[2], Math.Sqrt(SINSstate.Vx_0[0] * SINSstate.Vx_0[0] + SINSstate.Vx_0[1] * SINSstate.Vx_0[1]));
                    SINSstate.CourseHeading = Math.Atan2(SINSstate.Vx_0[0], SINSstate.Vx_0[1]);
                }
                else
                {
                    SINSstate.CoursePitch = 0.0;
                    SINSstate.CourseHeading = SINSstate.Heading;
                }
                SINSstate.A_cx0 = SimpleOperations.A_cx0(SINSstate);
                SINSstate.A_x0c = SINSstate.A_cx0.Transpose();
                SimpleOperations.CopyMatrix(SINSstate.A_sc, SINSstate.A_sx0 * SINSstate.A_x0c);
                SINSstate.alpha_c = -Math.Atan2(SINSstate.A_sc[2, 1], SINSstate.A_sc[1, 1]);
                SINSstate.gamma_c = -Math.Atan2(SINSstate.A_sc[0, 2], SINSstate.A_sc[0, 0]);
                SINSstate.beta_c = Math.Atan2(SINSstate.A_sc[0, 1] * Math.Cos(SINSstate.gamma_c), SINSstate.A_sc[0, 0]);





                //Задание относительной линейной скорости в различных проекциях, а так же их производных
                SINSstate.Vz[0] = 0.0;
                SINSstate.Vz[1] = 0.0;
                if (CurTimeWithAlign > 0.0 && CurTimeWithAlign <= 50.0) SINSstate.Vz[1] = 40.0 * Math.Sin(Math.PI / 2.0 * (CurTimeWithAlign - 0) / 50.0);
                if (CurTimeWithAlign > 50.0 && CurTimeWithAlign <= 25500.0) SINSstate.Vz[1] = 40.0;
                if (CurTimeWithAlign > 25500.0 && CurTimeWithAlign <= 25550.0) SINSstate.Vz[1] = 40.0 * Math.Cos(Math.PI / 2.0 * (CurTimeWithAlign - 25500.0) / 50.0);

                //if (CurTimeWithAlign > 75.0 && CurTimeWithAlign <= 95.0) SINSstate.Vz[1] = 3.0 * Math.Sin(Math.PI * (CurTimeWithAlign - 75.0) / 20.0);

                //if (CurTimeWithAlign > 120.0 && CurTimeWithAlign <= 140.0) SINSstate.Vz[1] = 7.0 * Math.Sin(Math.PI / 2.0 * (CurTimeWithAlign - 120.0) / 20.0);
                //if (CurTimeWithAlign > 140.0 && CurTimeWithAlign <= 250.0) SINSstate.Vz[1] = 7.0;
                //if (CurTimeWithAlign > 250.0 && CurTimeWithAlign <= 270.0) SINSstate.Vz[1] = 7.0 * Math.Cos(Math.PI / 2.0 * (CurTimeWithAlign - 250.0) / 20.0);

                //if (CurTimeWithAlign > 300.0 && CurTimeWithAlign <= 400.0) SINSstate.Vz[1] = 4.0 * Math.Sin(Math.PI * (CurTimeWithAlign - 300.0) / 100.0);
                //if (CurTimeWithAlign > 400.0 && CurTimeWithAlign <= 420.0) SINSstate.Vz[1] = 4.0 * Math.Sin(Math.PI * (CurTimeWithAlign - 400.0) / 20.0);

                //if (CurTimeWithAlign > 420.0 && CurTimeWithAlign <= 440.0) SINSstate.Vz[1] = 6.0 * Math.Sin(Math.PI / 2.0 * (CurTimeWithAlign - 420.0) / 20.0);
                //if (CurTimeWithAlign > 440.0 && CurTimeWithAlign <= 650.0) SINSstate.Vz[1] = 6.0;
                //if (CurTimeWithAlign > 650.0 && CurTimeWithAlign <= 670.0) SINSstate.Vz[1] = 6.0 * Math.Cos(Math.PI / 2.0 * (CurTimeWithAlign - 650.0) / 20.0);



                //if (CurTimeWithAlign > 1300.0 && CurTimeWithAlign <= 1350.0) SINSstate.Vz[1] = 1.0 * Math.Sin(Math.PI * (CurTimeWithAlign - 1300.0) / 50.0);
                //if (CurTimeWithAlign > 1350.0 && CurTimeWithAlign <= 1370.0) SINSstate.Vz[1] = 4.0 * Math.Sin(Math.PI / 2.0 * (CurTimeWithAlign - 1350.0) / 20.0);
                //if (CurTimeWithAlign > 1370.0 && CurTimeWithAlign <= 2000.0) SINSstate.Vz[1] = 4.0;
                //if (CurTimeWithAlign > 2000.0 && CurTimeWithAlign <= 2020.0) SINSstate.Vz[1] = 4.0 * Math.Cos(Math.PI / 2.0 * (CurTimeWithAlign - 2000.0) / 20.0);
                SINSstate.Vz[2] = 0.0;
                //if (CurTimeWithAlign > 250.0 && CurTimeWithAlign < 300.0) SINSstate.Vz[2] = 2.0 * Math.Sin(Math.PI * (CurTimeWithAlign - 250.0) / 50.0);

                /*неправильная формула*/
                SINSstate.OdoAbsSpeed = Math.Sign(SINSstate.Vz[1]) * SimpleOperations.AbsoluteVectorValue(SINSstate.Vz);
                //SINSstate.OdoAbsSpeed = SINSstate.OdoAbsSpeed * Math.Cos(SINSstate.beta_c);
                //SINSstate.OdoAbsSpeed = SINSstate.OdoAbsSpeed * Params_OdoScaleErr;
                //SINSstate.OdoAbsSpeed = SINSstate.OdoAbsSpeed * Math.Cos(kappa[0]) * Math.Cos(kappa[2]);






                SINSstate.Vx_0 = Matrix.Multiply(SINSstate.A_x0s, SINSstate.Vz);
                //SINSstate.Vx_0[2] = 0.0;
                if (Math.Abs(SINSstate.Vz[1]) < 0.0001 && Math.Abs(SINSstate.Heading_prev - SINSstate.Heading) < 0.0000001)
                    SINSstate.FLG_Stop = 1;

                //---Формирование ПОКАЗАНИЙ ОДОМЕТРА---//
                odometer_left_ValueTrue = odometer_left_ValueTrue + SINSstate.OdoAbsSpeed * dT;

                //---расчет с учетом инкремента---//
                //if (Params_OdoIncrement > 0.001)
                //{
                //    double tmp1 = Math.Floor(odometer_left_ValueTrue);
                //    double tmp2 = Math.Floor(odometer_left_ValueTrue * 100) - Math.Floor(odometer_left_ValueTrue) * 100;
                //    double tmp3 = Math.Floor((tmp2) / Params_OdoIncrement);
                //    SINSstate.OdometerData.odometer_left.Value = tmp1 + tmp3 * Params_OdoIncrement / 100.0;
                //}
                //else
                SINSstate.OdometerData.odometer_left.Value = odometer_left_ValueTrue;

                if (SINSstate.Count % Params_OdoFrequency == 0)
                    SINSstate.OdometerData.odometer_left.isReady = 1;
                else
                    SINSstate.OdometerData.odometer_left.isReady = 2;



                //Формирование позиционной информации
                SimpleOperations.PositionIntegration_x0(dT, SINSstate);
                //SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                //SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);


                SINSstate.GPS_Data.gps_Latitude.Value = SINSstate.Latitude;
                SINSstate.GPS_Data.gps_Latitude.isReady = 2;
                SINSstate.GPS_Data.gps_Longitude.Value = SINSstate.Longitude;
                SINSstate.GPS_Data.gps_Longitude.isReady = 2;
                SINSstate.GPS_Data.gps_Altitude.Value = SINSstate.Altitude;
                SINSstate.GPS_Data.gps_Altitude.isReady = 2;

                if (odometer_left_ValueTrue % 60000.0 < SINSstate.Vz[1] * dT + 0.01 && odometer_left_ValueTrue > 1.0)
                {
                    SINSstate.GPS_Data.gps_Latitude.isReady = 1;
                    SINSstate.GPS_Data.gps_Longitude.isReady = 1;
                    SINSstate.GPS_Data.gps_Altitude.isReady = 1;
                }


                //Относительные угловые скорости
                RelativeAngular_sx0 = SimpleOperations.RelativeAngular_sx0(CurrentTime - Alignment_End, SINSstate, dOrientationAngles);
                RelativeAngular_x0s = SimpleOperations.RelativeAngular_x0s(CurrentTime - Alignment_End, SINSstate, dOrientationAngles);
                RelativeAngular_x0 = SimpleOperations.RelativeAngular_x0(SINSstate.Vx_0, SINSstate.Latitude, SINSstate.Altitude);
                RelativeAngular_x0_in_s = SINSstate.A_sx0 * RelativeAngular_x0;


                //Задание угловой скорости географического трехгранника
                SINSstate.u_x = SimpleOperations.U_x0(SINSstate.Latitude);
                SINSstate.u_s = SINSstate.A_sx0 * SINSstate.u_x;

                //Формирование вектора удельной силы тяжести
                SINSstate.g = SimpleOperations.GilmertGravityForce(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.F_x = SimpleOperations.Force_x0(dT, SINSstate.Vx_0, SINSstate.Vx_0_prev, RelativeAngular_x0, SINSstate.u_x, SINSstate.g);
                SINSstate.F_z = SINSstate.A_sx0 * SINSstate.F_x;


                //Формирование Абсолютной угловой скорости приборного трехгранника
                for (int i = 0; i < 3; i++)
                    SINSstate.W_z[i] = RelativeAngular_sx0[i] + RelativeAngular_x0_in_s[i] + SINSstate.u_s[i];

                //-----------------------------------------Информации о местоположении одометра---------------------------------------------
                //OdometerCoordinate_x0 = Matrix.Multiply(A_x0s, OdometerCoordinate_s);
                //OdometerPosition[0] = Position[0] + OdometerCoordinate_x0[1] / SimpleOperations.RadiusN(Position[0], Position[2]);
                //OdometerPosition[1] = Position[1] + OdometerCoordinate_x0[0] / SimpleOperations.RadiusE(Position[0], Position[2]) / Math.Cos(Position[0]);
                //OdometerPosition[2] = Position[2] + OdometerCoordinate_x0[2];

                //temp = Matrix.Multiply(Matrix.SkewSymmetricMatrix(RelativeAngular_x0s), OdometerCoordinate_x0);
                //for (int i = 0; i < 3; i++)
                //    OdometerVelocity_x0[i] = Velocity_x0[i] + temp[i];

                ////Добавление несоосности оси одометра и оси Oz1
                //temp = Matrix.Multiply(A_odoZ, Matrix.Multiply(A_sx0, OdometerVelocity_x0));

                ////---Формирование самого измерения пройденного пути---//
                //OdometerMeasure = SimpleOperations.AbsoluteVectorValue(temp);///////////!!!///////////


                //---Поворот приборных осей относительно динамической---//
                //double[] kappa = new double[3];
                //kappa[0] = Params_OdoKappa1;
                //kappa[2] = Params_OdoKappa3;
                //SimpleOperations.CopyArray(SINSstate.F_z, SimpleOperations.A_odoZ(kappa[0], kappa[2]) * SINSstate.F_z);
                //SimpleOperations.CopyArray(SINSstate.W_z, SimpleOperations.A_odoZ(kappa[0], kappa[2]) * SINSstate.W_z);

                //double df_0 = 9.81 * Params_df_0;
                //double dW_0 = Params_dnu_0 * SimpleData.ToRadian / 3600.0; //0.2 grad/hour

                //if (Math.Abs(Params_df_s) > 0.001 || Math.Abs(Params_dnu_s) > 0.001)
                //{
                //    SINSstate.F_z[0] += (rnd_1.NextDouble() - 0.5) / Params_df_s;
                //    SINSstate.F_z[1] += (rnd_2.NextDouble() - 0.5) / Params_df_s;
                //    SINSstate.F_z[2] += (rnd_3.NextDouble() - 0.5) / Params_df_s;
                //    SINSstate.W_z[0] -= (rnd_4.NextDouble() - 0.5) / Params_dnu_s;
                //    SINSstate.W_z[1] -= (rnd_5.NextDouble() - 0.5) / Params_dnu_s;
                //    SINSstate.W_z[2] -= (rnd_6.NextDouble() - 0.5) / Params_dnu_s;
                //}


                //SINSstate.F_z[0] += df_0;
                //SINSstate.F_z[1] += df_0;
                //SINSstate.F_z[2] += df_0;
                //SINSstate.W_z[0] -= dW_0;
                //SINSstate.W_z[1] -= dW_0;
                //SINSstate.W_z[2] -= dW_0;

                //Вывод сформированных данных в файл
                if (SINSstate.Count % 5 == 0)
                    ExitInfoClear.WriteLine(CurrentTime.ToString() + " " + SINSstate.F_z[0].ToString() + " " + SINSstate.F_z[1].ToString() + " " + SINSstate.F_z[2].ToString() + " " +
                                SINSstate.W_z[0].ToString() + " " + SINSstate.W_z[1].ToString() + " " + SINSstate.W_z[2].ToString() + " " + SINSstate.Vz[0].ToString() + " " + SINSstate.Vz[1].ToString() + " " + SINSstate.Vz[2].ToString() + " " +
                                SINSstate.Vx_0[0].ToString() + " " + SINSstate.Vx_0[1].ToString() + " " + SINSstate.Vx_0[2].ToString() + " " +
                                SINSstate.CourseHeading + " " + SINSstate.CoursePitch + " " + SINSstate.beta_c + " " + SINSstate.alpha_c + " " + SINSstate.gamma_c + " " +
                                SINSstate.Heading.ToString() + " " + SINSstate.Roll.ToString() + " " + SINSstate.Pitch.ToString() + " " +
                                SINSstate.Latitude.ToString() + " " + SINSstate.Longitude.ToString() + " " + SINSstate.Altitude.ToString() + " " +
                                OdometerPosition[0].ToString() + " " + OdometerPosition[1].ToString() + " " + OdometerPosition[2].ToString() + " " +
                                SimpleOperations.AbsoluteVectorValue(SINSstate.F_z).ToString() + " " + SINSstate.g.ToString());


                Imitator_Data_for_Process.WriteLine(SINSstate.Count + " " + SINSstate.F_z[1] + " " + SINSstate.F_z[2] + " " + SINSstate.F_z[0] + " " + SINSstate.W_z[1] + " " + SINSstate.W_z[2] + " " + SINSstate.W_z[0]
                                 + " " + SINSstate.GPS_Data.gps_Latitude.Value.ToString() + " " + SINSstate.GPS_Data.gps_Latitude.isReady.ToString() + " " + SINSstate.GPS_Data.gps_Longitude.Value.ToString()
                                 + " " + SINSstate.GPS_Data.gps_Longitude.isReady.ToString() + " " + SINSstate.GPS_Data.gps_Altitude.Value.ToString() + " " + SINSstate.GPS_Data.gps_Altitude.isReady.ToString()
                                 + " " + SINSstate.GPS_Data.gps_Vn.Value.ToString() + " " + SINSstate.GPS_Data.gps_Vn.isReady.ToString() + " " + SINSstate.GPS_Data.gps_Ve.Value.ToString() + " " + SINSstate.GPS_Data.gps_Ve.isReady.ToString()
                                 + " " + SINSstate.FLG_Stop.ToString() + " " + SINSstate.OdometerData.odometer_left.Value.ToString() + " " + SINSstate.OdometerData.odometer_left.isReady.ToString()
                                 + " " + SINSstate.OdometerData.odometer_right.Value.ToString() + " " + SINSstate.OdometerData.odometer_right.isReady.ToString()
                                 + " " + SINSstate.Heading + " " + SINSstate.Roll + " " + SINSstate.Pitch);


                SimpleOperations.CopyArray(SINSstate.Vz_prev, SINSstate.Vz);
                SimpleOperations.CopyArray(SINSstate.Vx_0_prev, SINSstate.Vx_0);
                SINSstate.Heading_prev = SINSstate.Heading;
                SINSstate.Roll_prev = SINSstate.Roll;
                SINSstate.Pitch_prev = SINSstate.Pitch;
                SINSstate.Latitude_prev = SINSstate.Latitude;
                SINSstate.Longitude_prev = SINSstate.Longitude;
                SINSstate.Altitude_prev = SINSstate.Altitude;
            }
            ///////////////////////////////////////////// Конец рабочего цикла ////////////////////////////////////////////
            ExitInfoClear.Close(); Imitator_Data_for_Process.Close();
            //MessageBox.Show("     Done                             ", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
            this.Close();
            //Авто копирование выходного файла в папку с обработкой
        }










        private void Filtering_Click(object sender, EventArgs e)
        {
            StreamReader Imitator_Telemetric = new StreamReader("D://SINS Solution//MovingImitator_Azimut//Imitator_data//TelemetricData.txt");
            StreamWriter OutPutFile = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//Imitator_data//TelemetricData_Median.txt");

            string readString = "";
            string[] readStringArray_1 = new string[10], readStringArray_2 = new string[10], readStringArray_3 = new string[10], readStringArray_Out = new string[10];

            for (int i = 0; ; i++)
            {
                readString = Imitator_Telemetric.ReadLine();
                if (Imitator_Telemetric.EndOfStream == true) break;

                if (i < 2)
                {
                    if (i == 0) readStringArray_3 = readString.Split(' ');
                    if (i == 1) readStringArray_2 = readString.Split(' ');
                    continue;
                }
                else
                {
                    for (int j = 0; j < 10; j++)
                    {
                        readStringArray_3[j] = readStringArray_2[j];
                        readStringArray_2[j] = readStringArray_1[j];
                    }
                    readStringArray_1 = readString.Split(' ');

                    readStringArray_Out[0] = readStringArray_1[0];
                    readStringArray_Out[3] = readStringArray_1[3];
                    readStringArray_Out[6] = readStringArray_1[6];

                    double[] temp = new double[3];

                    if (i == 19)
                        i = i;

                    temp[0] = Convert.ToDouble(readStringArray_3[1]); temp[1] = Convert.ToDouble(readStringArray_2[1]); temp[2] = Convert.ToDouble(readStringArray_1[1]);
                    readStringArray_Out[1] = DoubleSort_3(temp).ToString();
                    temp[0] = Convert.ToDouble(readStringArray_3[2]); temp[1] = Convert.ToDouble(readStringArray_2[2]); temp[2] = Convert.ToDouble(readStringArray_1[2]);
                    readStringArray_Out[2] = DoubleSort_3(temp).ToString();

                    temp[0] = Convert.ToDouble(readStringArray_3[4]); temp[1] = Convert.ToDouble(readStringArray_2[4]); temp[2] = Convert.ToDouble(readStringArray_1[4]);
                    readStringArray_Out[4] = DoubleSort_3(temp).ToString();
                    temp[0] = Convert.ToDouble(readStringArray_3[5]); temp[1] = Convert.ToDouble(readStringArray_2[5]); temp[2] = Convert.ToDouble(readStringArray_1[5]);
                    readStringArray_Out[5] = DoubleSort_3(temp).ToString();

                    temp[0] = Convert.ToDouble(readStringArray_3[7]); temp[1] = Convert.ToDouble(readStringArray_2[7]); temp[2] = Convert.ToDouble(readStringArray_1[7]);
                    readStringArray_Out[7] = DoubleSort_3(temp).ToString();
                    temp[0] = Convert.ToDouble(readStringArray_3[8]); temp[1] = Convert.ToDouble(readStringArray_2[8]); temp[2] = Convert.ToDouble(readStringArray_1[8]);
                    readStringArray_Out[8] = DoubleSort_3(temp).ToString();
                    temp[0] = Convert.ToDouble(readStringArray_3[9]); temp[1] = Convert.ToDouble(readStringArray_2[9]); temp[2] = Convert.ToDouble(readStringArray_1[9]);
                    readStringArray_Out[9] = DoubleSort_3(temp).ToString();

                    OutPutFile.WriteLine(readStringArray_Out[0] + " " + readStringArray_Out[1] + " " + readStringArray_Out[2] + " " + readStringArray_Out[3] + " " + readStringArray_Out[4] + " " + readStringArray_Out[5]
                        + " " + readStringArray_Out[6] + " " + readStringArray_Out[7] + " " + readStringArray_Out[8] + " " + readStringArray_Out[9]);
                }
            }
            OutPutFile.Close();
            this.Close();
        }


        private double DoubleSort_3(double[] array)
        {
            double outnum = 0.0;

            if (array[0] <= array[1] && array[1] <= array[2] || array[2] <= array[1] && array[1] <= array[0]) outnum = array[1];
            if (array[1] <= array[0] && array[0] <= array[2] || array[2] <= array[0] && array[0] <= array[1]) outnum = array[0];
            if (array[0] <= array[2] && array[2] <= array[1] || array[1] <= array[2] && array[2] <= array[0]) outnum = array[2];

            return outnum;
        }







        public int dimFilt = 151;
        private void Smoothing_Click(object sender, EventArgs e)
        {
            StreamReader Imitator_Telemetric = new StreamReader("D://SINS Solution//MovingImitator_Azimut//Imitator_data//TelemetricData_Median.txt");
            StreamWriter OutPutFile = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//Imitator_data//TelemetricData_Smoothed.txt");

            string readString = "";
            string[,] readStringArray = new string[dimFilt, 10];
            string[] readStringArray_Out = new string[dimFilt];

            for (int i = 0; ; i++)
            {
                readString = Imitator_Telemetric.ReadLine();
                if (Imitator_Telemetric.EndOfStream == true) break;

                if (i < dimFilt)
                {
                    string[] tempstr = readString.Split(' ');
                    for (int j = 0; j < 10; j++)
                        readStringArray[i, j] = tempstr[j];
                    continue;
                }
                else
                {
                    for (int j = 0; j < dimFilt - 1; j++)
                    {
                        for (int t = 0; t < 10; t++)
                            readStringArray[j, t] = readStringArray[j + 1, t];
                    }
                    string[] tempstr = readString.Split(' ');
                    for (int j = 0; j < 10; j++)
                        readStringArray[dimFilt - 1, j] = tempstr[j];

                    int k = (dimFilt - 1) / 2;
                    double temp = 0.0;
                    readStringArray_Out[0] = readStringArray[dimFilt - 1 - k, 0];
                    readStringArray_Out[3] = readStringArray[dimFilt - 1 - k, 3];
                    readStringArray_Out[6] = readStringArray[dimFilt - 1 - k, 6];

                    temp = 0.0;
                    for (int j = 0; j < dimFilt; j++) temp += Convert.ToDouble(readStringArray[j, 1]) * (1 + Math.Cos((j - k) * Math.PI / k)) / 2.0 / k;
                    readStringArray_Out[1] = temp.ToString();
                    temp = 0.0;
                    for (int j = 0; j < dimFilt; j++) temp += Convert.ToDouble(readStringArray[j, 2]) * (1 + Math.Cos((j - k) * Math.PI / k)) / 2.0 / k;
                    readStringArray_Out[2] = temp.ToString();

                    temp = 0.0;
                    for (int j = 0; j < dimFilt; j++) temp += Convert.ToDouble(readStringArray[j, 4]) * (1 + Math.Cos((j - k) * Math.PI / k)) / 2.0 / k;
                    readStringArray_Out[4] = temp.ToString();
                    temp = 0.0;
                    for (int j = 0; j < dimFilt; j++) temp += Convert.ToDouble(readStringArray[j, 5]) * (1 + Math.Cos((j - k) * Math.PI / k)) / 2.0 / k;
                    readStringArray_Out[5] = temp.ToString();

                    temp = 0.0;
                    for (int j = 0; j < dimFilt; j++) temp += Convert.ToDouble(readStringArray[j, 7]) * (1 + Math.Cos((j - k) * Math.PI / k)) / 2.0 / k;
                    readStringArray_Out[7] = temp.ToString();
                    temp = 0.0;
                    for (int j = 0; j < dimFilt; j++) temp += Convert.ToDouble(readStringArray[j, 8]) * (1 + Math.Cos((j - k) * Math.PI / k)) / 2.0 / k;
                    readStringArray_Out[8] = temp.ToString();
                    temp = 0.0;
                    for (int j = 0; j < dimFilt; j++) temp += Convert.ToDouble(readStringArray[j, 9]) * (1 + Math.Cos((j - k) * Math.PI / k)) / 2.0 / k;
                    readStringArray_Out[9] = temp.ToString();

                    if (i > 100)
                        OutPutFile.WriteLine(readStringArray_Out[0] + " " + readStringArray_Out[1] + " " + readStringArray_Out[2] + " " + readStringArray_Out[3] + " " + readStringArray_Out[4] + " " + readStringArray_Out[5]
                            + " " + readStringArray_Out[6] + " " + readStringArray_Out[7] + " " + readStringArray_Out[8] + " " + readStringArray_Out[9]);
                }
            }
            OutPutFile.Close();
            this.Close();
        }



        private void RelativeV_Click(object sender, EventArgs e)
        {
            StreamReader Imitator_Telemetric = new StreamReader("D://SINS Solution//MovingImitator_Azimut//Imitator_data//TelemetricData_Smoothed.txt");
            StreamWriter OutPutFile = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//Imitator_data//TelemetricData_Condition.txt");

            string readString = "";
            string[] readStringArray = new string[10];

            SINS_State SINSstate = new SINS_State();

            for (int i = 0; ; i++)
            {
                readString = Imitator_Telemetric.ReadLine();
                if (Imitator_Telemetric.EndOfStream == true) break;

                readStringArray = readString.Split(' ');

                SINSstate.Count++;

                SINSstate.Time = Convert.ToDouble(readStringArray[0]);

                if (i == 0 && SINSstate.Count == 1)
                {
                    i = -1;
                    SINSstate.Time_prev = SINSstate.Time;
                    continue;
                }

                SINSstate.Latitude = Convert.ToDouble(readStringArray[1]);
                SINSstate.Longitude = Convert.ToDouble(readStringArray[2]);
                SINSstate.Altitude = Convert.ToDouble(readStringArray[3]);
                SINSstate.Heading = Convert.ToDouble(readStringArray[7]);
                SINSstate.Roll = Convert.ToDouble(readStringArray[8]);
                SINSstate.Pitch = Convert.ToDouble(readStringArray[9]);

                SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);

                if (i == 0)
                {
                    SINSstate.timeStep = SINSstate.Time - SINSstate.Time_prev;
                    SINSstate.Latitude_prev = SINSstate.Latitude;
                    SINSstate.Longitude_prev = SINSstate.Longitude;
                    SINSstate.Altitude_prev = SINSstate.Altitude;
                }

                SINSstate.Vx_0[0] = (SINSstate.Longitude - SINSstate.Longitude_prev) / SINSstate.timeStep
                    * (SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude) + SINSstate.Altitude + SimpleOperations.RadiusE(SINSstate.Latitude_prev, SINSstate.Altitude_prev) + SINSstate.Altitude_prev) / 2.0
                    * Math.Cos((SINSstate.Latitude + SINSstate.Latitude_prev) / 2.0);
                SINSstate.Vx_0[1] = (SINSstate.Latitude - SINSstate.Latitude_prev) / SINSstate.timeStep
                    * (SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude) + SINSstate.Altitude + SimpleOperations.RadiusN(SINSstate.Latitude_prev, SINSstate.Altitude_prev) + SINSstate.Altitude_prev) / 2.0;
                SINSstate.Vx_0[2] = (SINSstate.Altitude - SINSstate.Altitude_prev) / SINSstate.timeStep;


                OutPutFile.WriteLine(SINSstate.Time + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Altitude + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[1] + " " + SINSstate.Vx_0[2]
                        + " " + SINSstate.Heading + " " + SINSstate.Roll + " " + SINSstate.Pitch);


                SINSstate.Time_prev = SINSstate.Time;
                SINSstate.Latitude_prev = SINSstate.Latitude;
                SINSstate.Longitude_prev = SINSstate.Longitude;
                SINSstate.Altitude_prev = SINSstate.Altitude;
            }

            OutPutFile.Close();
            Imitator_Telemetric.Close();
            this.Close();
        }







        private void TelemetricImitator_Click(object sender, EventArgs e)
        {
            StreamReader Imitator_Telemetric = new StreamReader("D://SINS Solution//MovingImitator_Azimut//Imitator_data//TelemetricData_Condition.txt");
            StreamWriter OutPutFile = new StreamWriter(SimpleData.PathInputString + "Imitator_Telemetric.txt");

            SINS_State SINSstate = new SINS_State();

            string readString = "";
            string[] readStringArray;

            double[] n_ = new double[3], z_ = new double[3], z_prev = new double[3];
            double[] g0_x0 = new double[3], g0_z = new double[3];
            double[] v_x0 = new double[3], v_z = new double[3], v_z_prev = new double[3];
            double odometer_left = 0.0;

            for (int i = 0; ; i++)
            {
                readString = Imitator_Telemetric.ReadLine();
                if (Imitator_Telemetric.EndOfStream == true) break;
                readStringArray = readString.Split(' ');

                SINSstate.Count++;

                SINSstate.Time = Convert.ToDouble(readStringArray[0]);

                if (i == 0 && SINSstate.Count == 1)
                {
                    i = -1;
                    SINSstate.Time_prev = SINSstate.Time;
                    continue;
                }

                SINSstate.Latitude = Convert.ToDouble(readStringArray[1]);
                SINSstate.Longitude = Convert.ToDouble(readStringArray[2]);
                SINSstate.Altitude = Convert.ToDouble(readStringArray[3]);
                SINSstate.Vx_0[0] = Convert.ToDouble(readStringArray[4]);
                SINSstate.Vx_0[1] = Convert.ToDouble(readStringArray[5]);
                SINSstate.Vx_0[2] = Convert.ToDouble(readStringArray[6]);
                SINSstate.Heading = Convert.ToDouble(readStringArray[7]);
                SINSstate.Roll = Convert.ToDouble(readStringArray[8]);
                SINSstate.Pitch = Convert.ToDouble(readStringArray[9]);

                SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Altitude);
                SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Altitude);

                if (i == 0)
                {
                    SINSstate.timeStep = SINSstate.Time - SINSstate.Time_prev;
                    SINSstate.Latitude_Start = SINSstate.Latitude;
                    SINSstate.Longitude_Start = SINSstate.Longitude;
                    SINSstate.A_sx0_prev = SimpleOperations.A_sx0(SINSstate);
                    SINSstate.A_x0n_prev = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                    SINSstate.A_nxi_prev = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                    SimpleOperations.CopyMatrix(SINSstate.AT_prev, SINSstate.A_sx0_prev * SINSstate.A_x0n_prev * SINSstate.A_nxi_prev);

                    SINSstate.Latitude_prev = SINSstate.Latitude;
                    SINSstate.Longitude_prev = SINSstate.Longitude;
                    SINSstate.Altitude_prev = SINSstate.Altitude;

                    v_x0[0] = SINSstate.Vx_0[0] + SimpleData.U * (SINSstate.R_e + SINSstate.Altitude) * Math.Cos(SINSstate.Latitude);
                    v_x0[1] = SINSstate.Vx_0[1];
                    v_x0[2] = SINSstate.Vx_0[2];
                    SimpleOperations.CopyArray(v_z, SINSstate.A_sx0_prev * v_x0);
                    SimpleOperations.CopyArray(v_z_prev, v_z);
                }


                //-------------------------------------------------------------------------------------------------------------------------------------------------------//
                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
                SimpleOperations.CopyMatrix(SINSstate.AT, SINSstate.A_sx0 * SINSstate.A_x0n * SINSstate.A_nxi);

                Matrix B = new Matrix(3, 3);
                SimpleOperations.CopyMatrix(B, SINSstate.AT * SINSstate.AT_prev.Transpose() - Matrix.UnitMatrix(3));

                double gamma = 0.0;
                if (Math.Abs(B[0, 0] + B[1, 1] - B[2, 2]) <= 10E-16 && Math.Abs(B[0, 0] - B[1, 1] + B[2, 2]) <= 10E-16 && Math.Abs(-B[0, 0] + B[1, 1] + B[2, 2]) <= 10E-16) gamma = 0.0;
                else if (Math.Abs(B[0, 0] + B[1, 1] - B[2, 2]) >= Math.Abs(B[0, 0] - B[1, 1] + B[2, 2]) && Math.Abs(B[0, 0] + B[1, 1] - B[2, 2]) >= Math.Abs(-B[0, 0] + B[1, 1] + B[2, 2]))
                    gamma = Math.Asin(0.5 * Math.Abs(B[0, 1] - B[1, 0]) * Math.Sqrt((B[0, 0] + B[1, 1] + B[2, 2]) / (B[0, 0] + B[1, 1] - B[2, 2])));
                else if (Math.Abs(B[0, 0] - B[1, 1] + B[2, 2]) >= Math.Abs(B[0, 0] + B[1, 1] - B[2, 2]) && Math.Abs(B[0, 0] - B[1, 1] + B[2, 2]) >= (-B[0, 0] + B[1, 1] + B[2, 2]))
                    gamma = Math.Asin(0.5 * Math.Abs(B[2, 0] - B[0, 2]) * Math.Sqrt((B[0, 0] + B[1, 1] + B[2, 2]) / (B[0, 0] - B[1, 1] + B[2, 2])));
                else if (Math.Abs(-B[0, 0] + B[1, 1] + B[2, 2]) >= Math.Abs(B[0, 0] - B[1, 1] + B[2, 2]) && Math.Abs(-B[0, 0] + B[1, 1] + B[2, 2]) >= (B[0, 0] + B[1, 1] - B[2, 2]))
                    gamma = Math.Asin(0.5 * Math.Abs(B[1, 2] - B[2, 1]) * Math.Sqrt((B[0, 0] + B[1, 1] + B[2, 2]) / (-B[0, 0] + B[1, 1] + B[2, 2])));

                if (gamma == 0.0)
                {
                    SINSstate.W_z[0] = 0.0;
                    SINSstate.W_z[1] = 0.0;
                    SINSstate.W_z[2] = 0.0;
                }
                else
                {
                    SINSstate.W_z[0] = 0.5 * gamma * (B[1, 2] - B[2, 1]) / Math.Sin(gamma) / SINSstate.timeStep;
                    SINSstate.W_z[1] = 0.5 * gamma * (B[2, 0] - B[0, 2]) / Math.Sin(gamma) / SINSstate.timeStep;
                    SINSstate.W_z[2] = 0.5 * gamma * (B[0, 1] - B[1, 0]) / Math.Sin(gamma) / SINSstate.timeStep;
                }




                n_[0] = (SINSstate.R_e + SINSstate.Altitude) * Math.Cos(SINSstate.Latitude) * Math.Cos(SINSstate.Longitude);
                n_[1] = (SINSstate.R_e + SINSstate.Altitude) * Math.Cos(SINSstate.Latitude) * Math.Sin(SINSstate.Longitude);
                n_[2] = ((1 - SimpleData.Ex_Squared) * SINSstate.R_e + SINSstate.Altitude) * Math.Sin(SINSstate.Latitude);
                SimpleOperations.CopyArray(z_, SINSstate.A_sx0 * SINSstate.A_x0n * n_);

                if (i == 0)
                {
                    //SimpleOperations.CopyArray(z_prev, z_);
                    //continue;
                    for (int j = 0; j < 3; j++)
                        z_prev[j] = z_[j] - v_z[j] * SINSstate.timeStep;
                }

                SINSstate.g = SimpleOperations.GilmertGravityForce(SINSstate.Latitude, SINSstate.Altitude);
                g0_x0[0] = 0.0;
                g0_x0[1] = SimpleData.U * SimpleData.U * (SINSstate.R_e + SINSstate.Altitude) * Math.Sin(SINSstate.Latitude) * Math.Cos(SINSstate.Latitude);
                g0_x0[2] = -SINSstate.g - SimpleData.U * SimpleData.U * (SINSstate.R_e + SINSstate.Altitude) * Math.Cos(SINSstate.Latitude) * Math.Cos(SINSstate.Latitude);
                SimpleOperations.CopyArray(g0_z, SINSstate.A_sx0 * g0_x0);

                Matrix C_1 = new Matrix(3, 3), C_2 = new Matrix(3, 3), C_3 = new Matrix(3, 3);
                if (gamma < 10E-8)
                {
                    SimpleOperations.CopyMatrix(C_1, Matrix.UnitMatrix(3) + SINSstate.timeStep * Matrix.SkewSymmetricMatrix(SINSstate.W_z) + 0.5 * Math.Pow(SINSstate.timeStep, 2) * Matrix.SkewSymmetricMatrixSquare(SINSstate.W_z));
                    SimpleOperations.CopyMatrix(C_2, 0.5 * Matrix.UnitMatrix(3) + 1.0 / 3.0 * SINSstate.timeStep * Matrix.SkewSymmetricMatrix(SINSstate.W_z) + 1.0 / 8.0 * Math.Pow(SINSstate.timeStep, 2) * Matrix.SkewSymmetricMatrixSquare(SINSstate.W_z));
                    SimpleOperations.CopyMatrix(C_3, Matrix.UnitMatrix(3) + 1.0 / 2.0 * SINSstate.timeStep * Matrix.SkewSymmetricMatrix(SINSstate.W_z) + 1.0 / 6.0 * Math.Pow(SINSstate.timeStep, 2) * Matrix.SkewSymmetricMatrixSquare(SINSstate.W_z));
                }
                else
                {
                    SimpleOperations.CopyMatrix(C_1, Matrix.UnitMatrix(3) + Math.Sin(gamma) * SINSstate.timeStep / gamma * Matrix.SkewSymmetricMatrix(SINSstate.W_z) +
                            (1.0 - Math.Cos(gamma)) * Math.Pow(SINSstate.timeStep, 2) / gamma / gamma * Matrix.SkewSymmetricMatrixSquare(SINSstate.W_z));
                    SimpleOperations.CopyMatrix(C_2, 0.5 * Matrix.UnitMatrix(3) + (Math.Sin(gamma) - gamma * Math.Cos(gamma)) * SINSstate.timeStep / Math.Pow(gamma, 3) * Matrix.SkewSymmetricMatrix(SINSstate.W_z) +
                            (1.0 + 0.5 * gamma * gamma - Math.Cos(gamma) - gamma * Math.Sin(gamma)) * Math.Pow(SINSstate.timeStep, 2) / Math.Pow(gamma, 4) * Matrix.SkewSymmetricMatrixSquare(SINSstate.W_z));
                    SimpleOperations.CopyMatrix(C_3, Matrix.UnitMatrix(3) + (1.0 - Math.Cos(gamma)) * SINSstate.timeStep / gamma / gamma * Matrix.SkewSymmetricMatrix(SINSstate.W_z) +
                            (gamma - Math.Sin(gamma)) * Math.Pow(SINSstate.timeStep, 2) / gamma / gamma / gamma * Matrix.SkewSymmetricMatrixSquare(SINSstate.W_z));
                }

                double[] temp_1 = new double[3], temp_2 = new double[3], temp_3 = new double[3];

                if (gamma > 10E-8)
                {
                    SimpleOperations.CopyArray(temp_1, (Matrix.UnitMatrix(3) + SINSstate.timeStep * Math.Sin(gamma / 2.0) / gamma * Matrix.SkewSymmetricMatrix(SINSstate.W_z) + Math.Pow(SINSstate.timeStep, 2) * (1.0 - Math.Cos(gamma / 2.0)) / gamma / gamma * Matrix.SkewSymmetricMatrixSquare(SINSstate.W_z)).GetInvertible() * g0_z);
                    SimpleOperations.CopyArray(g0_z, temp_1);
                }

                if (false)
                {
                    temp_1 = C_2.GetInvertible() * z_;
                    temp_2 = C_2.GetInvertible() * C_1 * z_prev;
                    temp_3 = SINSstate.timeStep * C_2.GetInvertible() * C_1 * v_z_prev;

                    for (int j = 0; j < 3; j++)
                        SINSstate.F_z[j] = 1.0 / Math.Pow(SINSstate.timeStep, 2) * (temp_1[j] - temp_2[j] - temp_3[j]) - g0_z[j];

                    if (gamma > 10E-13)
                    {
                        SimpleOperations.CopyArray(temp_1, (Matrix.UnitMatrix(3) + SINSstate.timeStep * Math.Sin(gamma / 2.0) / gamma * Matrix.SkewSymmetricMatrix(SINSstate.W_z) + Math.Pow(SINSstate.timeStep, 2) * (1.0 - Math.Cos(gamma / 2.0)) / gamma / gamma * Matrix.SkewSymmetricMatrixSquare(SINSstate.W_z)).GetInvertible() * SINSstate.F_z);
                        SimpleOperations.CopyArray(SINSstate.F_z, temp_1);
                    }

                    SimpleOperations.CopyArray(temp_1, C_1 * v_z_prev);
                    SimpleOperations.CopyArray(temp_2, C_3 * SINSstate.F_z);
                    SimpleOperations.CopyArray(temp_3, C_3 * g0_z);
                    for (int j = 0; j < 3; j++) v_z[j] = temp_1[j] + SINSstate.timeStep * (temp_2[j] + temp_3[j]);

                }
                else if (false)
                {
                    v_x0[0] = SINSstate.Vx_0[0] + SimpleData.U * (SINSstate.R_e + SINSstate.Altitude) * Math.Cos(SINSstate.Latitude);
                    v_x0[1] = SINSstate.Vx_0[1];
                    v_x0[2] = SINSstate.Vx_0[2];
                    SimpleOperations.CopyArray(v_z, SINSstate.A_sx0 * v_x0);

                    SimpleOperations.CopyArray(temp_1, Matrix.SkewSymmetricMatrix(SINSstate.W_z) * v_z_prev);

                    for (int j = 0; j < 3; j++)
                        SINSstate.F_z[j] = 1.0 / SINSstate.timeStep * (v_z[j] - v_z_prev[j] - SINSstate.timeStep * temp_1[j]) - g0_z[j];
                }
                else
                {
                    v_x0[0] = SINSstate.Vx_0[0] + SimpleData.U * (SINSstate.R_e + SINSstate.Altitude) * Math.Cos(SINSstate.Latitude);
                    v_x0[1] = SINSstate.Vx_0[1];
                    v_x0[2] = SINSstate.Vx_0[2];
                    SimpleOperations.CopyArray(v_z, SINSstate.A_sx0 * v_x0);

                    temp_1 = C_2.GetInvertible() * v_z;
                    temp_2 = C_2.GetInvertible() * C_1 * v_z_prev;

                    for (int j = 0; j < 3; j++)
                        SINSstate.F_z[j] = (temp_1[j] - temp_2[j]) / SINSstate.timeStep - g0_z[j];
                }


                //if (Math.Abs(SINSstate.F_z[0]) > 5.0) SINSstate.F_z[0] = Math.Sign(SINSstate.F_z[0]) * (5.0);
                //if (Math.Abs(SINSstate.F_z[1]) > 5.0) SINSstate.F_z[1] = Math.Sign(SINSstate.F_z[1]) * (5.0);
                //if (Math.Abs(SINSstate.F_z[2]) - 9.81 > 5.0) SINSstate.F_z[2] = Math.Sign(SINSstate.F_z[2]) * (Math.Abs(SINSstate.F_z[2]) - 9.81 - 5.0);


                //---ОДОМЕТР---//
                if (SINSstate.Count <= 19000)
                    SINSstate.FLG_Stop = 1;
                else SINSstate.FLG_Stop = 0;

                if (SINSstate.FLG_Stop != 1)
                {
                    odometer_left += Math.Sqrt(Math.Pow((SINSstate.Latitude - SINSstate.Latitude_prev) * (SINSstate.R_n + SINSstate.Altitude), 2) +
                                Math.Pow((SINSstate.Longitude - SINSstate.Longitude_prev) * (SINSstate.R_e + SINSstate.Altitude) * Math.Cos(SINSstate.Latitude), 2) + Math.Pow(SINSstate.Altitude - SINSstate.Altitude_prev, 2));

                    if (i % 5 == 0)
                    {
                        SINSstate.OdometerData.odometer_left.isReady = 1;
                        SINSstate.OdometerData.odometer_left.Value = odometer_left;
                    }
                    else
                        SINSstate.OdometerData.odometer_left.isReady = 2;
                }


                //---ДЖПС---//
                SINSstate.GPS_Data.gps_Longitude.Value = SINSstate.Longitude;
                SINSstate.GPS_Data.gps_Latitude.Value = SINSstate.Latitude;
                SINSstate.GPS_Data.gps_Altitude.Value = SINSstate.Altitude;

                if (i % 50 == 0)
                {
                    SINSstate.GPS_Data.gps_Longitude.isReady = 1;
                    SINSstate.GPS_Data.gps_Latitude.isReady = 1;
                    SINSstate.GPS_Data.gps_Altitude.isReady = 1;
                }
                else
                {
                    SINSstate.GPS_Data.gps_Longitude.isReady = 2;
                    SINSstate.GPS_Data.gps_Latitude.isReady = 2;
                    SINSstate.GPS_Data.gps_Altitude.isReady = 2;
                }


                //---ВЫВОД---//
                OutPutFile.WriteLine(SINSstate.Count + " " + SINSstate.F_z[1] + " " + SINSstate.F_z[2] + " " + SINSstate.F_z[0] + " " + SINSstate.W_z[1] + " " + SINSstate.W_z[2] + " " + SINSstate.W_z[0]
                                 + " " + SINSstate.GPS_Data.gps_Latitude.Value.ToString() + " " + SINSstate.GPS_Data.gps_Latitude.isReady.ToString() + " " + SINSstate.GPS_Data.gps_Longitude.Value.ToString()
                                 + " " + SINSstate.GPS_Data.gps_Longitude.isReady.ToString() + " " + SINSstate.GPS_Data.gps_Altitude.Value.ToString() + " " + SINSstate.GPS_Data.gps_Altitude.isReady.ToString()
                                 + " " + SINSstate.GPS_Data.gps_Vn.Value.ToString() + " " + SINSstate.GPS_Data.gps_Vn.isReady.ToString() + " " + SINSstate.GPS_Data.gps_Ve.Value.ToString() + " " + SINSstate.GPS_Data.gps_Ve.isReady.ToString()
                                 + " " + SINSstate.FLG_Stop.ToString() + " " + SINSstate.OdometerData.odometer_left.Value.ToString() + " " + SINSstate.OdometerData.odometer_left.isReady.ToString()
                                 + " " + SINSstate.OdometerData.odometer_right.Value.ToString() + " " + SINSstate.OdometerData.odometer_right.isReady.ToString()
                                 + " " + SINSstate.Heading.ToString() + " " + SINSstate.Roll.ToString() + " " + SINSstate.Pitch.ToString());


                //-------------------------------------------------------------------------------------------------------------------------------------------------------//
                SimpleOperations.CopyMatrix(SINSstate.A_sx0_prev, SINSstate.A_sx0);
                SimpleOperations.CopyMatrix(SINSstate.A_x0n_prev, SINSstate.A_x0n);
                SimpleOperations.CopyMatrix(SINSstate.A_nxi_prev, SINSstate.A_nxi);
                SimpleOperations.CopyMatrix(SINSstate.AT_prev, SINSstate.AT);
                SimpleOperations.CopyArray(z_prev, z_);
                SimpleOperations.CopyArray(v_z_prev, v_z);
                SINSstate.Latitude_prev = SINSstate.Latitude;
                SINSstate.Longitude_prev = SINSstate.Longitude;
                SINSstate.Altitude_prev = SINSstate.Altitude;
            }

            Imitator_Telemetric.Close();
            OutPutFile.Close();
            this.Close();
        }













        private void button2_Click(object sender, EventArgs e)
        {
            StreamReader _45_16_09_13_TLM_1zaezd = new StreamReader("D://LavNavSolution_2//Азимут-Т//Заезд 18.10.2013//данные 18.10.2013//2013_10_18(12_55)_TLM.txt");
            StreamWriter _45_16_09_13_TLM_1zaezd_OUT = new StreamWriter("D://LavNavSolution_2//Азимут-Т//Заезд 18.10.2013//данные 18.10.2013//2013_10_18(12_55)_TLM_FILLED.txt");
            StreamWriter help = new StreamWriter("D://LavNavSolution_2//Азимут-Т//Заезд 18.10.2013//данные 18.10.2013//help.txt");

            string hat_string = "";
            hat_string = "Time;Mode;DUS X;DUS Y;DUS Z;Accs X;Accs Y;Accs Z;Impuls ARM -0.1268;Temperat DUS 1;Temperat DUS 2;Temperat DUS 3;Temperat ACCS X;Temperat ACCS Y;Temperat ACCS Z;SNS Lat;SNS Long;SNS h;SNS Vn;SNS Ve;SNS Vup;Solution Quality;Lat SINS;Long SINS;h SINS;True Heading SINS;Roll SINS;Pitch SINS;IsOdoReady;IsSNS_Ready;TimeIncrement";
            //hat_string += "IsOdoReady;IsSNS_Ready;TimeIncrement";
            _45_16_09_13_TLM_1zaezd_OUT.WriteLine(hat_string);

            _45_16_09_13_TLM_1zaezd.ReadLine();
            string str, temp_str = "";
            string[] str_array;
            double time, time_prev = 0.0;


            int j = 0;
            int[] Odo_is_Ready = new int[200];
            double Time = 0.0, Time_Prev = 0.0, dT = 0.0;
            string EndOfLongString = "";
            string[] str_2 = new string[200];
            for (int i = 0; ; i++)
            {
                if (_45_16_09_13_TLM_1zaezd.EndOfStream == true) break;

                str_2[j] = _45_16_09_13_TLM_1zaezd.ReadLine();
                str_array = str_2[j].Split(new Char[] { ';' }, 11);

                //Time = Convert.ToDouble(str_array[0].Split(':')[0]) * 3600.0 + Convert.ToDouble(str_array[0].Split(':')[1]) * 60 + Convert.ToDouble(str_array[0].Split(':')[2]);
                Time = Convert.ToDouble(str_array[0]);

                if (i % 10 == 0)
                    Odo_is_Ready[j] = 1;
                else
                    Odo_is_Ready[j] = 2;

                if (i == 0)
                {
                    EndOfLongString = str_array[9] + ";" + str_array[10];
                    //if (str_array[1] != "0")
                    //_45_16_09_13_TLM_1zaezd_OUT.WriteLine(Time.ToString() + ";" + str_2[j].Substring(str_array[0].Length + 1) + Odo_is_Ready[j].ToString() + ";1;" + dT.ToString());
                    Time_Prev = Time;
                }
                else
                    j++;

                if (i != 0 && str_array.Length == 11)
                {
                    //dT = (Time - Time_Prev) / j;
                    for (int k = 0; k < j - 1; k++)
                    {
                        if (k != 0)
                            dT = Convert.ToDouble(str_2[k].Split(';')[0]) - Convert.ToDouble(str_2[k - 1].Split(';')[0]);
                        //if (str_2[k].Split(';')[1] != "0")
                        //{
                        if (k == j - 2)
                            _45_16_09_13_TLM_1zaezd_OUT.WriteLine(str_2[k] + EndOfLongString + Odo_is_Ready[k].ToString() + ";1;" + dT.ToString());
                        else
                            //_45_16_09_13_TLM_1zaezd_OUT.WriteLine((Time_Prev + dT * (k + 1)).ToString() + ";" + str_2[k].Substring(str_array[0].Length + 1) + EndOfLongString + Odo_is_Ready[k].ToString() + ";0;" + dT.ToString());
                            _45_16_09_13_TLM_1zaezd_OUT.WriteLine(str_2[k] + EndOfLongString + Odo_is_Ready[k].ToString() + ";0;" + dT.ToString());
                        //}
                    }
                    j = 0;
                    Time_Prev = Time;
                    EndOfLongString = str_array[9] + ";" + str_array[10];
                }
            }


            //for (int i = 0; ; i++)
            //{
            //    if (_45_16_09_13_TLM_1zaezd.EndOfStream == true) break;

            //    str = _45_16_09_13_TLM_1zaezd.ReadLine();
            //    str_array = str.Split(new Char[] { ';' }, 10);

            //    time = Convert.ToDouble(str_array[0].Split(':')[0]) * 3600 + Convert.ToDouble(str_array[0].Split(':')[1]) * 60 + Convert.ToDouble(str_array[0].Split(':')[2]);
            //    if (i == 0) time_prev = time;

            //    help.WriteLine(i.ToString() + " " + (time - time_prev).ToString());
            //    if (str_array.Length == 10)
            //    {
            //        temp_str = str_array[8] + ";" + str_array[9];
            //        _45_16_09_13_TLM_1zaezd_OUT.WriteLine(str);
            //    }
            //    else
            //        _45_16_09_13_TLM_1zaezd_OUT.WriteLine(str + temp_str);

            //    time_prev = time;

            //}
            _45_16_09_13_TLM_1zaezd.Close(); _45_16_09_13_TLM_1zaezd_OUT.Close();
            Close();

        }







    }
}
