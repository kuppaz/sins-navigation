using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class SimpleOperations
    {
        public static double RadiusE(double Latitude, double Altitude) //R_1
        {
            return SimpleData.A / Math.Pow(1 - SimpleData.Ex * Math.Sin(Latitude) * Math.Sin(Latitude), 0.5) + Altitude;
        }

        public static double RadiusN(double Latitude, double Altitude) //R_2
        {
            return SimpleData.A * (1 - SimpleData.Ex) / Math.Pow(1 - SimpleData.Ex * Math.Sin(Latitude) * Math.Sin(Latitude), 1.5) + Altitude;
        }

        public static double GilmertGravityForce(double Latitude, double Altitude)
        {
            return 9.780318 * (1 + 0.005302 * Math.Sin(Latitude) * Math.Sin(Latitude) - 0.000007 * Math.Sin(2 * Latitude) * Math.Sin(2 * Latitude)) - 0.00014 - 2 * Altitude * 0.000001543;
        }
        public static double NormalGravity_g0(double Latitude, double Altitude)
        {
            return 9.780318 * (1 + 0.005302 * Math.Sin(Latitude) * Math.Sin(Latitude) - 0.0000059 * Math.Sin(2 * Latitude) * Math.Sin(2 * Latitude)) ;
        }

        public static double AbsoluteVectorValue(double[] Vect)
        {
            return Math.Sqrt(Vect[0] * Vect[0] + Vect[1] * Vect[1] + Vect[2] * Vect[2]);
        }

        public static double SkalyarProduct(double[] Vect1, double[] Vect2)
        {
            return (Vect1[0] * Vect2[0] + Vect1[1] * Vect2[1] + Vect1[2] * Vect2[2]);
        }

        //---------------------------------Задается модель поведения движения-------------------------------------------
        public static double[] Velocity_s(double CurrentTime)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = 0.0;
            ResultVect[1] = 0.2 * CurrentTime;
            ResultVect[2] = 0.0;
            return ResultVect;
        }

        public static double[] Acceleration_s(double CurrentTime)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = 0.0;
            ResultVect[1] = 0.2;
            ResultVect[2] = 0.0;
            return ResultVect;
        }

        public static double[] AnglesHRP(double CurrentTime, double StartHeading, double StartRoll, double StartPitch)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = StartHeading + 30.0 * SimpleData.ToRadian * Math.Sin(0.052 * CurrentTime);
            ResultVect[1] = StartRoll + 5.0 * SimpleData.ToRadian * Math.Sin(0.021 * CurrentTime);
            ResultVect[2] = StartPitch + 15.0 * SimpleData.ToRadian * Math.Sin(0.035 * CurrentTime);
            return ResultVect;
        }

        public static double[] DerivativeOfAnglesHRP(double CurrentTime)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = 30.0 * 0.052 * SimpleData.ToRadian * Math.Cos(0.052 * CurrentTime);
            ResultVect[1] = 5.0 * 0.021 * SimpleData.ToRadian * Math.Cos(0.021 * CurrentTime);
            ResultVect[2] = 15.0 * 0.035 * SimpleData.ToRadian * Math.Cos(0.035 * CurrentTime);
            return ResultVect;
        }
        //---------------------------------------------------------------------------------------------------------------

        public static double[] U_x0(double Latitude)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = 0.0;
            ResultVect[1] = SimpleData.U * Math.Cos(Latitude);
            ResultVect[2] = SimpleData.U * Math.Sin(Latitude);
            return ResultVect;
        }

        public static double[] RelativeAngular_sx0(double CurrentTime, double[] OrientationAngles)
        {
            double[] ResultVect = new double[3];
            double[] dAngular = DerivativeOfAnglesHRP(CurrentTime);

            ResultVect[0] = dAngular[2] * Math.Cos(OrientationAngles[1]) + dAngular[0] * Math.Cos(OrientationAngles[2]) * Math.Sin(OrientationAngles[1]);
            ResultVect[1] = dAngular[1] - dAngular[0] * Math.Sin(OrientationAngles[2]);
            ResultVect[2] = dAngular[2] * Math.Sin(OrientationAngles[1]) - dAngular[0] * Math.Cos(OrientationAngles[2]) * Math.Cos(OrientationAngles[1]);
            return ResultVect;
        }

        public static double[] RelativeAngular_x0s(double CurrentTime, double[] OrientationAngles)
        {
            double[] ResultVect = new double[3];
            double[] dAngular = DerivativeOfAnglesHRP(CurrentTime);

            ResultVect[0] = -dAngular[2] * Math.Cos(OrientationAngles[0]) - dAngular[1] * Math.Sin(OrientationAngles[0]) * Math.Cos(OrientationAngles[2]);
            ResultVect[1] = dAngular[2] * Math.Sin(OrientationAngles[0]) - dAngular[1] * Math.Cos(OrientationAngles[0]) * Math.Cos(OrientationAngles[2]);
            ResultVect[2] = dAngular[0] - dAngular[1] * Math.Sin(OrientationAngles[2]);
            return ResultVect;
        }

        public static double[] RelativeAngular_x0(double[] Velocity_x0, double Latitude, double Altitude)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = -Velocity_x0[1] / RadiusN(Latitude, Altitude);
            ResultVect[1] = Velocity_x0[0] / RadiusE(Latitude, Altitude);
            ResultVect[2] = ResultVect[1] * Math.Tan(Latitude);
            return ResultVect;
        }

        public static double[] Force_x0(double dT, double[] Velocity_x0, double[] Velocity_prev, double[] RelativeAngular_x0, double[] U_x0, double Gravity)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = (Velocity_x0[0] - Velocity_prev[0]) / dT - (RelativeAngular_x0[2] + 2 * U_x0[2]) * Velocity_prev[1] + (RelativeAngular_x0[1] + 2 * U_x0[1]) * Velocity_prev[2];
            ResultVect[1] = (Velocity_x0[1] - Velocity_prev[1]) / dT + (RelativeAngular_x0[2] + 2 * U_x0[2]) * Velocity_prev[0] - (RelativeAngular_x0[0] + 2 * U_x0[0]) * Velocity_prev[2];
            ResultVect[2] = (Velocity_x0[2] - Velocity_prev[2]) / dT - (RelativeAngular_x0[1] + 2 * U_x0[1]) * Velocity_prev[0] + (RelativeAngular_x0[0] + 2 * U_x0[0]) * Velocity_prev[1] + Gravity;
            return ResultVect;
        }

        public static double[] PositionIntegration_x0(double dT, double[] Position_prev, double[] Velocity_x0)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = Position_prev[0] + dT * (Velocity_x0[1] / RadiusN(Position_prev[0], Position_prev[2]));
            ResultVect[1] = Position_prev[1] + dT * (Velocity_x0[0] / RadiusE(Position_prev[0], Position_prev[2]) / Math.Cos(Position_prev[0]));
            ResultVect[2] = Position_prev[2] + dT * Velocity_x0[2];
            return ResultVect;
        }


        //-----------------------------------------------------------------Матрицы----------------------------------------------------------
        public static Matrix A_sx0(double[] Angles)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(Angles[0]) * Math.Cos(Angles[1]) + Math.Sin(Angles[0]) * Math.Sin(Angles[2]) * Math.Sin(Angles[1]);
            MatrixResult[0, 1] = -Math.Sin(Angles[0]) * Math.Cos(Angles[1]) + Math.Cos(Angles[0]) * Math.Sin(Angles[2]) * Math.Sin(Angles[1]);
            MatrixResult[0, 2] = -Math.Cos(Angles[2]) * Math.Sin(Angles[1]);
            MatrixResult[1, 0] = Math.Sin(Angles[0]) * Math.Cos(Angles[2]);
            MatrixResult[1, 1] = Math.Cos(Angles[0]) * Math.Cos(Angles[2]);
            MatrixResult[1, 2] = Math.Sin(Angles[2]);
            MatrixResult[2, 0] = Math.Cos(Angles[0]) * Math.Sin(Angles[1]) - Math.Sin(Angles[0]) * Math.Sin(Angles[2]) * Math.Cos(Angles[1]);
            MatrixResult[2, 1] = -Math.Sin(Angles[0]) * Math.Sin(Angles[1]) - Math.Cos(Angles[0]) * Math.Sin(Angles[2]) * Math.Cos(Angles[1]);
            MatrixResult[2, 2] = Math.Cos(Angles[2]) * Math.Cos(Angles[1]);
            return MatrixResult;
        }
        public static Matrix A_sx0(SINS_State SINSState)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SINSState.Heading) * Math.Cos(SINSState.Roll) + Math.Sin(SINSState.Heading) * Math.Sin(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[0, 1] = -Math.Sin(SINSState.Heading) * Math.Cos(SINSState.Roll) + Math.Cos(SINSState.Heading) * Math.Sin(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[0, 2] = -Math.Cos(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[1, 0] = Math.Sin(SINSState.Heading) * Math.Cos(SINSState.Pitch);
            MatrixResult[1, 1] = Math.Cos(SINSState.Heading) * Math.Cos(SINSState.Pitch);
            MatrixResult[1, 2] = Math.Sin(SINSState.Pitch);
            MatrixResult[2, 0] = Math.Cos(SINSState.Heading) * Math.Sin(SINSState.Roll) - Math.Sin(SINSState.Heading) * Math.Sin(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            MatrixResult[2, 1] = -Math.Sin(SINSState.Heading) * Math.Sin(SINSState.Roll) - Math.Cos(SINSState.Heading) * Math.Sin(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            MatrixResult[2, 2] = Math.Cos(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            return MatrixResult;
        }
        public static Matrix A_xs(SINS_State SINSState)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SINSState.Roll);
            MatrixResult[0, 1] = 0.0;
            MatrixResult[0, 2] = Math.Sin(SINSState.Roll);
            MatrixResult[1, 0] = Math.Sin(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[1, 1] = Math.Cos(SINSState.Pitch);
            MatrixResult[1, 2] = -Math.Sin(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            MatrixResult[2, 0] = -Math.Cos(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[2, 1] = Math.Sin(SINSState.Pitch);
            MatrixResult[2, 2] = Math.Cos(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            return MatrixResult;
        }
        public static Matrix A_x0n(double Latitude, double Longitude)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = -Math.Sin(Longitude);
            MatrixResult[0, 1] = Math.Cos(Longitude);
            MatrixResult[0, 2] = 0.0;
            MatrixResult[1, 0] = -Math.Cos(Longitude) * Math.Sin(Latitude);
            MatrixResult[1, 1] = -Math.Sin(Longitude) * Math.Sin(Latitude);
            MatrixResult[1, 2] = Math.Cos(Latitude);
            MatrixResult[2, 0] = Math.Cos(Longitude) * Math.Cos(Latitude);
            MatrixResult[2, 1] = Math.Sin(Longitude) * Math.Cos(Latitude);
            MatrixResult[2, 2] = Math.Sin(Latitude);
            return MatrixResult;
        }
        public static Matrix A_ne(double CurrentTime, double StartLongitude)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SimpleData.U * CurrentTime + StartLongitude);
            MatrixResult[0, 1] = Math.Sin(SimpleData.U * CurrentTime + StartLongitude);
            MatrixResult[0, 2] = 0.0;
            MatrixResult[1, 0] = -Math.Sin(SimpleData.U * CurrentTime + StartLongitude);
            MatrixResult[1, 1] = Math.Cos(SimpleData.U * CurrentTime + StartLongitude);
            MatrixResult[1, 2] = 0.0;
            MatrixResult[2, 0] = 0.0;
            MatrixResult[2, 1] = 0.0;
            MatrixResult[2, 2] = 1.0;
            return MatrixResult;
        }

        public static void CopyArray(double[] p, double[] p_2)
        {
            for (int ii = 0; ii < p.Length; ii++)
                p[ii] = p_2[ii];
        }
        public static void CopyMatrix(Matrix p, Matrix p_2)
        {
            for (int ii = 0; ii < 3; ii++)
                for (int jj = 0; jj < 3; jj++)
                    p[ii,jj] = p_2[ii,jj];
        }

        public static void MakeMatrixFromVector(Matrix Matrix_out, double[] Vector_in, int dim)
        {
            for (int i = 0; i < dim; i++)
            {
                for (int j = 0; j < dim; j++)
                {
                    Matrix_out[i, j] = Vector_in[i * dim + j];
                }
            }
        }

        public static Matrix A_odoZ(double kappa1, double kappa3)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = 1;
            MatrixResult[0, 1] = kappa3;
            MatrixResult[0, 2] = 0;
            MatrixResult[1, 0] = -kappa3;
            MatrixResult[1, 1] = 1;
            MatrixResult[1, 2] = kappa1;
            MatrixResult[2, 0] = 0;
            MatrixResult[2, 1] = -kappa1;
            MatrixResult[2, 2] = 1;
            return MatrixResult;
        }

        

        
       
    }
}
