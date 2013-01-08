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
        public double StartHeading = 45.0 * SimpleData.ToRadian, StartRoll = 0.0 * SimpleData.ToRadian, StartPitch = 0.0 * SimpleData.ToRadian;
        public double StartLatitude = 55.0 * SimpleData.ToRadian, StartLongitude = 37.0 * SimpleData.ToRadian, StartAltitude = 100.0;

        public Imitation()
        {
            InitializeComponent();
        }

        public void button1_Click(object sender, EventArgs e)
        {
            //Объявление необходимых переменных
            double dT = 0.1, Gravity, CurrentTime = 0.0, OdometerMeasure, kappa1, kappa3;
            double[] OrientationAngles = new double[3], dOrientationAngles = new double[3], Position = new double[3];
            double[] temp = new double[3], Velocity_s = new double[3], Velocity_x0 = new double[3], Acceleration_s = new double[3], Acceleration_x0 = new double[3],
                     Velocity_x0_prev = new double[3], RelativeAngular_sx0 = new double[3], RelativeAngular_x0s = new double[3], RelativeAngular_x0 = new double[3],
                     U_x0 = new double[3], F_x0 = new double[3], F_s = new double[3], W_s = new double[3];

            double[] OdometerCoordinate_s = new double[3], OdometerCoordinate_x0 = new double[3], OdometerPosition = new double[3], OdometerVelocity_x0 = new double[3];

            Matrix A_x0s = new Matrix(3, 3), A_sx0 = new Matrix(3,3), A_odoZ = new Matrix(3,3);

            FileStream FileExitInfoClear = new FileStream("ExitInfoClear.dat", FileMode.OpenOrCreate, FileAccess.Write);
            StreamWriter ExitInfoClear = new StreamWriter(FileExitInfoClear);
            ExitInfoClear.WriteLine("Time \t f_s1 \t f_s2 \t f_s3 \t W_s1 \t W_s2 \t W_s3 \t OdoMeasure \t Heading \t Roll \t Pitch \t Latitude \t Longitude \t Altitude \t OdoLatitude \t OdoLongitude \t OdoAltitude");

            //Выставление начальных условий для вектора углов ориентации
            OrientationAngles[0] = StartHeading;        Position[0] = StartLatitude;            OdometerCoordinate_s[0] = -0.5;// В осях приборного трехгранника
            OrientationAngles[1] = StartRoll;           Position[1] = StartLongitude;           OdometerCoordinate_s[1] =  2.0;
            OrientationAngles[2] = StartPitch;          Position[2] = StartAltitude;            OdometerCoordinate_s[2] = -0.3;
            //Углы несоосности оси одометра и приборных
            kappa1 = 3 * SimpleData.ToRadian;
            kappa3 = 2 * SimpleData.ToRadian;


            ///////////////////////////////////////////// Рабочий цикл /////////////////////////////////////////////////
            while (CurrentTime < 200.0)
            {
                CurrentTime += dT;
                //Задание углов ориентации Курса, крена, тангажа и их производных
                OrientationAngles = SimpleOperations.AnglesHRP(CurrentTime, StartHeading, StartRoll, StartPitch);
                dOrientationAngles = SimpleOperations.DerivativeOfAnglesHRP(CurrentTime);

                //Задание матриц ориентации между основными трехгранниками
                A_sx0 = SimpleOperations.A_sx0(OrientationAngles);
                A_x0s = A_sx0.Transpose();
                A_odoZ = SimpleOperations.A_odoZ(kappa1, kappa3);

                //Задание относительной линейной скорости в различных проекциях, а так же их производных
                Velocity_s = SimpleOperations.Velocity_s(CurrentTime);
                Velocity_x0 = Matrix.Multiply(A_x0s, Velocity_s);

                //Формирование позиционной информации
                Position = SimpleOperations.PositionIntegration_x0(dT, Position, Velocity_x0);

                //Относительные угловые скорости и модуля сила тяжести по формуле Гильмерта.
                RelativeAngular_sx0 = SimpleOperations.RelativeAngular_sx0(CurrentTime, OrientationAngles);
                RelativeAngular_x0s = SimpleOperations.RelativeAngular_x0s(CurrentTime, OrientationAngles);
                RelativeAngular_x0 =  SimpleOperations.RelativeAngular_x0(Velocity_x0, Position[0], Position[2]);
                Gravity = SimpleOperations.GilmertGravityForce(Position[0], Position[2]);

                //Задание угловой скорости географического трехгранника
                U_x0 = SimpleOperations.U_x0(Position[0]);

                //Формирование вектора удельной силы тяжести
                F_x0 = SimpleOperations.Force_x0(dT, Matrix.Multiply(A_x0s, SimpleOperations.Velocity_s(CurrentTime+dT)), Velocity_x0, RelativeAngular_x0, U_x0, Gravity);
                F_s = Matrix.Multiply(A_sx0, F_x0); ///////////!!!///////////

                //Формирование Абсолютной угловой скорости приборного трехгранника
                for (int i = 0; i < 3; i++ )
                    W_s[i] = RelativeAngular_sx0[i] + Matrix.Multiply(A_sx0, RelativeAngular_x0)[i] + Matrix.Multiply(A_sx0, U_x0)[i]; ///////////!!!///////////

                //-----------------------------------------Информации о местоположении одометра---------------------------------------------
                OdometerCoordinate_x0 = Matrix.Multiply(A_x0s, OdometerCoordinate_s);
                OdometerPosition[0] = Position[0] + OdometerCoordinate_x0[1] / SimpleOperations.RadiusN(Position[0], Position[2]);
                OdometerPosition[1] = Position[1] + OdometerCoordinate_x0[0] / SimpleOperations.RadiusE(Position[0], Position[2]) / Math.Cos(Position[0]);
                OdometerPosition[2] = Position[2] + OdometerCoordinate_x0[2];

                temp = Matrix.Multiply(Matrix.SkewSymmetricMatrix(RelativeAngular_x0s), OdometerCoordinate_x0);
                for (int i = 0; i < 3; i++)
                    OdometerVelocity_x0[i] = Velocity_x0[i] + temp[i];

                //Добавление несоосности оси одометра и оси Oz1
                temp = Matrix.Multiply(A_odoZ, Matrix.Multiply(A_sx0, OdometerVelocity_x0));

                //---Формирование самого измерения пройденного пути---//
                OdometerMeasure = SimpleOperations.AbsoluteVectorValue(temp);///////////!!!///////////

                //Вывод сформированных данных в файл
                ExitInfoClear.WriteLine(CurrentTime.ToString() + "    " + F_s[0].ToString() + "    " + F_s[1].ToString() + "    " + F_s[2].ToString() + "    " +
                                W_s[0].ToString() + "    " + W_s[1].ToString() + "    " + W_s[2].ToString() + "    " +
                                OdometerMeasure.ToString() + "    " + OrientationAngles[0].ToString() + "    " + OrientationAngles[1].ToString() + "    " + OrientationAngles[2].ToString() + "    " +
                                Position[0].ToString() + "    " + Position[1].ToString() + "    " + Position[2].ToString() + "    " + OdometerPosition[0].ToString() + "    " + OdometerPosition[1].ToString() + "    "
                                 + OdometerPosition[2].ToString() + "    " + (Math.Sqrt(F_s[0] * F_s[0] + F_s[1] * F_s[1] + F_s[2] * F_s[2])).ToString());
            }
            ///////////////////////////////////////////// Конец рабочего цикла ////////////////////////////////////////////
            ExitInfoClear.Close();
            MessageBox.Show("     Done                             ", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
            Close();
            //Авто копирование выходного файла в папку с обработкой
        }
    }
}
