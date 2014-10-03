using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace Common_Namespace
{
    public class SimpleOperations
    {
        public static double RadiusE(double Latitude, double Altitude) //R_1
        {
            return SimpleData.A / Math.Pow(1 - SimpleData.Ex_Squared * Math.Sin(Latitude) * Math.Sin(Latitude), 0.5) + Altitude;
        }

        public static double RadiusN(double Latitude, double Altitude) //R_2
        {
            return SimpleData.A * (1 - SimpleData.Ex_Squared) / Math.Pow(1 - SimpleData.Ex_Squared * Math.Sin(Latitude) * Math.Sin(Latitude), 1.5) + Altitude;
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

        public static double[] NullingOfArray(double[] Vect)
        {
            for (int i = 0; i < Vect.Length; i++)
                Vect[i] = 0.0;

            return Vect;
        }
        public static double[] NullingOfNotDiagMatrixElements(double[] Vect, int matrixDim)
        {
            for (int i = 0; i < matrixDim; i++)
            {
                for (int j = 0; j < matrixDim; j++)
                {
                    if (i != j)
                        Vect[i * matrixDim + j] = 0.0;
                }
            }

            return Vect;
        }
        public static double[] NullingOfBottomMatrix(double[] Vect, int matrixDim)
        {
            for (int i = 0; i < matrixDim; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    Vect[i * matrixDim + j] = 0.0;
                }
            }

            return Vect;
        }

        //---------------------------------Задается модель поведения движения-------------------------------------------

        public static double CalculateDistanceBtwDots(double Lat1, double Lon1, double Alt1, double Lat2, double Lon2, double Alt2)
        {
            double lat_dif_true = (Lat2 - Lat1) * SimpleOperations.RadiusN(Lat2, Alt2);
            double long_dif_true = (Lon2 - Lon1) * SimpleOperations.RadiusE(Lat2, Alt2) * Math.Cos(Lat2);

            return Math.Sqrt(lat_dif_true * lat_dif_true + long_dif_true * long_dif_true);
        }
        public static double CalculateHeadingByTwoDots(double Lat1, double Lon1, double Alt1, double Lat2, double Lon2, double Alt2)
        {
            double lat_dif_true = (Lat2 - Lat1) * SimpleOperations.RadiusN(Lat2, Alt2);
            double long_dif_true = (Lon2 - Lon1) * SimpleOperations.RadiusE(Lat2, Alt2) * Math.Cos(Lat2);

            return Math.Atan2(long_dif_true, lat_dif_true);
        }


        public static double[] Velocity_s(double CurrentTime)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = 0.0;
            ResultVect[1] = 0.02 * CurrentTime;
            ResultVect[2] = 0.0;
            return ResultVect;
        }

        public static double[] Acceleration_s(double CurrentTime)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = 0.0;
            ResultVect[1] = 0.02;
            ResultVect[2] = 0.0;
            return ResultVect;
        }

        public static void AnglesHRP(double CurrentTime, SINS_State SINSstate, double StartHeading, double StartRoll, double StartPitch)
        {
            double[] ResultVect = new double[3];
            SINSstate.Heading = StartHeading + 30.0 * SimpleData.ToRadian * Math.Sin(0.04 * CurrentTime);
            SINSstate.Roll = StartRoll + 5.0 * SimpleData.ToRadian * Math.Sin(0.021 * CurrentTime);
            SINSstate.Pitch = StartPitch + 0.0 * SimpleData.ToRadian * Math.Sin(0.025 * CurrentTime);
        }

        public static double[] DerivativeOfAnglesHRP(double dT, SINS_State SINSstate)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = (SINSstate.Heading - SINSstate.Heading_prev) / dT;
            ResultVect[1] = (SINSstate.Roll - SINSstate.Roll_prev) / dT;
            ResultVect[2] = (SINSstate.Pitch - SINSstate.Pitch_prev) / dT;

            return ResultVect;
        }

        public static double[] DerivativeOfAnglesHRP_analitic(double CurrentTime)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = 30.0 * 0.052 * SimpleData.ToRadian * Math.Cos(0.052 * CurrentTime);
            ResultVect[1] = 5.0 * 0.021 * SimpleData.ToRadian * Math.Cos(0.021 * CurrentTime);
            ResultVect[2] = 0.01 * 0.035 * SimpleData.ToRadian * Math.Cos(0.035 * CurrentTime);
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

        public static double[] RelativeAngular_sx0(double CurrentTime, SINS_State SINSstate, double[] dAngular)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = dAngular[2] * Math.Cos(SINSstate.Roll) + dAngular[0] * Math.Cos(SINSstate.Pitch) * Math.Sin(SINSstate.Roll);
            ResultVect[1] = dAngular[1] - dAngular[0] * Math.Sin(SINSstate.Pitch);
            ResultVect[2] = dAngular[2] * Math.Sin(SINSstate.Roll) - dAngular[0] * Math.Cos(SINSstate.Pitch) * Math.Cos(SINSstate.Roll);
            return ResultVect;
        }

        public static double[] RelativeAngular_x0s(double CurrentTime, SINS_State SINSstate, double[] dAngular)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = -dAngular[2] * Math.Cos(SINSstate.Heading) - dAngular[1] * Math.Sin(SINSstate.Heading) * Math.Cos(SINSstate.Pitch);
            ResultVect[1] = dAngular[2] * Math.Sin(SINSstate.Heading) - dAngular[1] * Math.Cos(SINSstate.Heading) * Math.Cos(SINSstate.Pitch);
            ResultVect[2] = dAngular[0] - dAngular[1] * Math.Sin(SINSstate.Pitch);
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

        public static void PositionIntegration_x0(double dT, SINS_State SINSstate)
        {
            SINSstate.Latitude = SINSstate.Latitude_prev + dT * (SINSstate.Vx_0[1] / RadiusN(SINSstate.Latitude_prev, SINSstate.Altitude_prev));
            SINSstate.Longitude = SINSstate.Longitude_prev + dT * (SINSstate.Vx_0[0] / RadiusE(SINSstate.Latitude_prev, SINSstate.Altitude_prev) / Math.Cos(SINSstate.Latitude_prev));
            SINSstate.Altitude = SINSstate.Altitude_prev + dT * SINSstate.Vx_0[2];
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
        public static Matrix A_sx0_Gyro(SINS_State SINSState)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SINSState.GyroHeading) * Math.Cos(SINSState.Roll) + Math.Sin(SINSState.GyroHeading) * Math.Sin(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[0, 1] = -Math.Sin(SINSState.GyroHeading) * Math.Cos(SINSState.Roll) + Math.Cos(SINSState.GyroHeading) * Math.Sin(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[0, 2] = -Math.Cos(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[1, 0] = Math.Sin(SINSState.GyroHeading) * Math.Cos(SINSState.Pitch);
            MatrixResult[1, 1] = Math.Cos(SINSState.GyroHeading) * Math.Cos(SINSState.Pitch);
            MatrixResult[1, 2] = Math.Sin(SINSState.Pitch);
            MatrixResult[2, 0] = Math.Cos(SINSState.GyroHeading) * Math.Sin(SINSState.Roll) - Math.Sin(SINSState.GyroHeading) * Math.Sin(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            MatrixResult[2, 1] = -Math.Sin(SINSState.GyroHeading) * Math.Sin(SINSState.Roll) - Math.Cos(SINSState.GyroHeading) * Math.Sin(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            MatrixResult[2, 2] = Math.Cos(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            return MatrixResult;
        }
        public static Matrix A_cx0(SINS_State SINSState)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SINSState.CourseHeading);
            MatrixResult[0, 1] = -Math.Sin(SINSState.CourseHeading);
            MatrixResult[0, 2] = 0.0;
            MatrixResult[1, 0] = Math.Sin(SINSState.CourseHeading) * Math.Cos(SINSState.CoursePitch);
            MatrixResult[1, 1] = Math.Cos(SINSState.CourseHeading) * Math.Cos(SINSState.CoursePitch);
            MatrixResult[1, 2] = Math.Sin(SINSState.CoursePitch);
            MatrixResult[2, 0] = -Math.Sin(SINSState.CourseHeading) * Math.Sin(SINSState.CoursePitch);
            MatrixResult[2, 1] = -Math.Cos(SINSState.CourseHeading) * Math.Sin(SINSState.CoursePitch);
            MatrixResult[2, 2] = Math.Cos(SINSState.CoursePitch);
            return MatrixResult;
        }
        public static Matrix A_x0c_by_V(double[] V_x0)
        {
            double V_hor = Math.Sqrt(V_x0[0] * V_x0[0] + V_x0[1] * V_x0[1]);
            double V = Math.Sqrt(V_x0[0] * V_x0[0] + V_x0[1] * V_x0[1] + V_x0[2] * V_x0[2]);

            double sin_psi = V_x0[0] / V_hor;
            double cos_psi = V_x0[1] / V_hor;
            double cos_theta = V_hor / V;
            double sin_theta = V_x0[2] / V;

            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = cos_psi;
            MatrixResult[0, 1] = sin_psi * cos_theta;
            MatrixResult[0, 2] = -sin_psi * sin_theta;
            MatrixResult[1, 0] = -sin_psi;
            MatrixResult[1, 1] = cos_psi * cos_theta;
            MatrixResult[1, 2] = -cos_psi * sin_theta;
            MatrixResult[2, 0] = 0.0;
            MatrixResult[2, 1] = sin_theta;
            MatrixResult[2, 2] = cos_theta;
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
        public static Matrix A_xs(double Heading, double Roll, double Pitch)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(Roll);
            MatrixResult[0, 1] = 0.0;
            MatrixResult[0, 2] = Math.Sin(Roll);
            MatrixResult[1, 0] = Math.Sin(Pitch) * Math.Sin(Roll);
            MatrixResult[1, 1] = Math.Cos(Pitch);
            MatrixResult[1, 2] = -Math.Sin(Pitch) * Math.Cos(Roll);
            MatrixResult[2, 0] = -Math.Cos(Pitch) * Math.Sin(Roll);
            MatrixResult[2, 1] = Math.Sin(Pitch);
            MatrixResult[2, 2] = Math.Cos(Pitch) * Math.Cos(Roll);
            return MatrixResult;
        }


        public static Matrix C_convultion_iMx(SINS_State SINSstate)
        {
            Matrix MatrixResult = new Matrix(SimpleData.iMxSmthd, SimpleData.iMx);

            if (SimpleData.iMxSmthd == 2)
            {
                MatrixResult[0, 1] = 1.0 / SINSstate.R_n;
                MatrixResult[1, 0] = 1.0 / SINSstate.R_e / Math.Cos(SINSstate.Latitude);
            }
            else if (SimpleData.iMxSmthd == 7)
            {
                MatrixResult[0, 1] = 1.0 / SINSstate.R_n;
                MatrixResult[1, 0] = 1.0 / SINSstate.R_e / Math.Cos(SINSstate.Latitude);
                MatrixResult[2, 2] = 1.0;
                MatrixResult[2, 6] = SINSstate.Vx_0[1];
                MatrixResult[3, 3] = 1.0;
                MatrixResult[3, 6] = -SINSstate.Vx_0[0];
                MatrixResult[4, 4] = -Math.Cos(SINSstate.Heading);
                MatrixResult[4, 5] = Math.Sin(SINSstate.Heading);
                MatrixResult[5, 4] = -Math.Sin(SINSstate.Heading) / Math.Cos(SINSstate.Pitch);
                MatrixResult[5, 5] = -Math.Cos(SINSstate.Heading) / Math.Cos(SINSstate.Pitch);
                MatrixResult[6, 4] = -Math.Sin(SINSstate.Heading) * Math.Tan(SINSstate.Pitch);
                MatrixResult[6, 5] = -Math.Cos(SINSstate.Heading) * Math.Tan(SINSstate.Pitch);
                MatrixResult[6, 6] = 1.0;
                MatrixResult[6, 0] = 1.0 / SINSstate.R_e * Math.Tan(SINSstate.Latitude);
            }
            return MatrixResult;
        }
        public static Matrix C_convultion_iMx_r3(SINS_State SINSstate)
        {
            Matrix MatrixResult = new Matrix(1, SimpleData.iMx);
            MatrixResult[0, SINSstate.iMx_r3_dV3] = 1.0;
            return MatrixResult;
        }



        public static Matrix ArrayToMatrix(double[] array)
        {
            Matrix MatrixResult = new Matrix(Convert.ToInt32(Math.Sqrt(array.Length)), Convert.ToInt32(Math.Sqrt(array.Length)));
            for (int i = 0; i < Convert.ToInt32(Math.Sqrt(array.Length)); i++)
            {
                for (int j = 0; j < Convert.ToInt32(Math.Sqrt(array.Length)); j++)
                {
                    MatrixResult[i, j] = array[i * Convert.ToInt32(Math.Sqrt(array.Length)) + j];
                }
            }
            return MatrixResult;
        }
        public static double[] MatrixToArray(Matrix matrix)
        {
            double[] array = new double[matrix.Cols*matrix.Rows];
            for (int i = 0; i < matrix.Rows; i++)
            {
                for (int j = 0; j < matrix.Cols; j++)
                {
                    array[i * matrix.Cols + j] = matrix[i, j];
                }
            }
            return array;
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
        
        public static Matrix A_x0n_Gyro(SINS_State SINSState)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            //MatrixResult[0, 0] = -Math.Sin(SINSState.Longitude);
            //MatrixResult[1, 0] = -Math.Cos(SINSState.Longitude) * Math.Sin(SINSState.Latitude);
            //MatrixResult[2, 0] = Math.Cos(SINSState.Longitude) * Math.Cos(SINSState.Latitude);

            MatrixResult[0, 1] = Math.Cos(SINSState.Azimth) * Math.Cos(SINSState.Longitude) - Math.Sin(SINSState.Azimth) * Math.Sin(SINSState.Longitude) * Math.Sin(SINSState.Latitude);
            MatrixResult[1, 1] = -Math.Sin(SINSState.Azimth) * Math.Cos(SINSState.Longitude) - Math.Cos(SINSState.Azimth) * Math.Sin(SINSState.Longitude) * Math.Sin(SINSState.Latitude);
            MatrixResult[2, 1] = Math.Sin(SINSState.Longitude) * Math.Cos(SINSState.Latitude);

            MatrixResult[0, 2] = Math.Sin(SINSState.Azimth) * Math.Cos(SINSState.Latitude);
            MatrixResult[1, 2] = Math.Cos(SINSState.Azimth) * Math.Cos(SINSState.Latitude);
            MatrixResult[2, 2] = Math.Sin(SINSState.Latitude);

            MatrixResult[0, 0] = MatrixResult[1, 1] * MatrixResult[2, 2] - MatrixResult[1, 2] * MatrixResult[2, 1];
            MatrixResult[1, 0] = MatrixResult[2, 1] * MatrixResult[0, 2] - MatrixResult[2, 2] * MatrixResult[0, 1];
            MatrixResult[2, 0] = MatrixResult[0, 1] * MatrixResult[1, 2] - MatrixResult[0, 2] * MatrixResult[1, 1];

            return MatrixResult;
        }
        public static Matrix A_ne(double CurrentTime, double StartLongitude)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SimpleData.U * CurrentTime);// + StartLongitude);
            MatrixResult[0, 1] = Math.Sin(SimpleData.U * CurrentTime);// + StartLongitude);
            MatrixResult[0, 2] = 0.0;
            MatrixResult[1, 0] = -Math.Sin(SimpleData.U * CurrentTime);// + StartLongitude);
            MatrixResult[1, 1] = Math.Cos(SimpleData.U * CurrentTime);// + StartLongitude);
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
        public static void CopyArray(double[,] p, int i, double[] p_2)
        {
            for (int ii = 0; ii < p_2.Length; ii++)
                p[i,ii] = p_2[ii];
        }
        public static void CopyArray(double[] p, int i, double[,] p_2)
        {
            for (int ii = 0; ii < p.Length; ii++)
                p[ii] = p_2[i,ii];
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


        public static Matrix MakeC_forSmoothing(double[] ErrVect, SINS_State SINSstate)
        {
            Matrix MatrixResult = new Matrix(7, SimpleData.iMx);

            MatrixResult[0, 1] = 1.0 / SINSstate.R_n;
            MatrixResult[1, 0] = 1.0 / SINSstate.R_e / Math.Cos(SINSstate.Latitude);
            MatrixResult[2, 2] = 1.0;
            MatrixResult[2, 7] = SINSstate.Vx_0[1];
            MatrixResult[3, 3] = 1.0;
            MatrixResult[3, 7] = -SINSstate.Vx_0[0];

            return MatrixResult;
        }


        public static void PrintMatrixToFile(double[] Matrix, int dim_str, int dim_colls)
        {
            StreamWriter PrintMatrix = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//PrintMatrix.txt");
            for (int j = 0; j < dim_str; j++)
            {
                string str_odo = null;
                for (int i = 0; i < dim_colls; i++)
                {
                    str_odo = str_odo + " " + Matrix[j * dim_colls + i].ToString();
                }

                PrintMatrix.WriteLine(str_odo);
            }
            PrintMatrix.Close();
        }

        public static void PrintVectorToFile(double[] Vector, int dim)
        {
            StreamWriter PrintVector = new StreamWriter("D://SINS Solution//MovingImitator_Azimut//SINS motion processing_new data//Output//PrintVector.txt");
            for (int j = 0; j < dim; j++)
            {
                string str_odo = null;

                str_odo = str_odo + " " + Vector[j].ToString();

                PrintVector.WriteLine(str_odo);
            }
            PrintVector.Close();
        }
        
       
    }
}
