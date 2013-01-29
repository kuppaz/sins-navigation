using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    class WorkClasses
    {
    }

    public class DataWithIsReady
    {
        public double Value;
        public int isReady;
    }

    public class gps_data
    {
        public DataWithIsReady gps_Latitude = new DataWithIsReady(), gps_Longitude = new DataWithIsReady(), gps_Altitude = new DataWithIsReady();
        public DataWithIsReady gps_Ve = new DataWithIsReady(), gps_Vn = new DataWithIsReady();
    }

    public class odometer_data
    {
        public DataWithIsReady odometer_left = new DataWithIsReady(), odometer_right = new DataWithIsReady();
    }

    public class start_data
    {
        public DataWithIsReady start_Latitude = new DataWithIsReady(), start_Longitude = new DataWithIsReady(), start_Altitude = new DataWithIsReady();
    }

    public class SINS_State
    {
        public string StartErrorString;

        public double alpha_z, alpha_x, alpha_y;
        public bool usinganglecorrection = false, UsingCorrection = false, KNS_flg = false, Alignment = false, Autonomous = false, UsingAvegering = false, UsingAltitudeCorrection = false, feedbackExist = false;
        public bool UsingClasAlignment = false, UsingNavAlignment = false;
        public bool Odometr_SINS_case = false;
        public bool UseOdoVelocity_In_Oz = false;

        public double temp1, temp2, tempLat;
        public double Count, initCount;
        public double[] tempVect = new double[3];
        public double timeStep;
        public bool usingSNS;
        public double[] Nu = new double[3];
        public double FLG_Stop;
        public double Freq, dV_q;
        public int FreqOutput;

        public string Global_file = "";

        public bool iMx_r3_dV3 = false, iMx_kappa_13_ds = false;
        public int value_iMx_r3_dV3 = 0, value_iMx_r_odo_12 = 0, value_iMx_kappa_13_ds = 0;

        public bool UsingOdoPosition = false, UsingOdoVelocity = false;
        public bool Use_3_Measure = false, Use_1_Measure = false;

        public gps_data GPS_Data = new gps_data();
        public odometer_data OdometerData = new odometer_data();
        public int OdoTimeStepCount, OdoTimeStepCount_2;

        public bool UseLastMinusOneOdo = false;
        public double OdometerLeftPrev, OdometerRightPrev, V_norm;
        public double OdometerLeftPrev_2, OdometerRightPrev_2;
        public start_data StartData = new start_data();
        public double LongSNS, LatSNS, AltSNS;
        public double[] Vx_0_prev = new double[3];

        public double V_abs, DistanceModeled = 0.0, Time_Alignment = 0;

        public double OdoAbsSpeed, OdoSpeedPrev, OdoSpeedPrev_2;
        public double[] OdometerVector = new double[3];

        public double Time;
        public double Latitude, Longitude, Altitude, Latitude_Start, Longitude_Start, Altitude_Start;
        public double[] Vx_0 = new double[3];
        public double[] Omega_x = new double[3];
        public double[] OdoSpeed = new double[3];
        public double Heading, Roll, Pitch, Azimth;
        public double[] F_z = new double[3], F_x = new double[3];
        public double[] F_z_prev = new double[3];
        public double[] W_z_prev = new double[3];
        public double[] W_z = new double[3];
        public double[] W_x = new double[3];
        public double[] u_x = new double[3];
        public double g, F_mod;
        public double g_0; // Нормальная сила тяжести? по неполной формуле гильмерта?
        public double[] g_x = new double[3];
        public Matrix A_sx0 = new Matrix(3, 3), A_x0s = new Matrix(3, 3), A_nx0 = new Matrix(3, 3), A_x0n = new Matrix(3, 3), A_x0n_prev = new Matrix(3, 3);
        public Matrix AT = new Matrix(3, 3), A_nxi = new Matrix(3,3);
        public double R_e, R_n;
        public double[] GK_Latitude = new double[50];
        public double[] GK_Longitude = new double[50];
        public double DirectionalAngle;
        public double Odo_Limit_Measures;

        public double DeltaLatitude, DeltaLongitude, DeltaV_1, DeltaV_2, DeltaV_3, DeltaHeading, DeltaRoll, DeltaPitch, DeltaAltitude;

        public double Latitude_Point;

        public double Altitude_prev, Latitude_prev, Longitude_prev;

        public int LastCountForRead;
    }

    public class Kalman_Vars
    {
        public int cnt_measures;
        public double DynamicNoiseFactor_f = 1e-5, DynamicNoiseFactor_nu = 1e-8, OdoNoise;

        public double[] Measure = new double[SimpleData.iMz];
        public double[] Noize_Z = new double[SimpleData.iMz];

        public double[] ErrorConditionVector_m = new double[SimpleData.iMx];
        public double[] ErrorConditionVector_p = new double[SimpleData.iMx];
        public double[] StringOfMeasure = new double[SimpleData.iMx];
        public double[] KalmanFactor = new double[SimpleData.iMx];

        public double[] Matrix_A = new double[SimpleData.iMx * SimpleData.iMx];
        public double[] Matrix_H = new double[SimpleData.iMx * SimpleData.iMz];
        public double[] CovarianceMatrixS_m = new double[SimpleData.iMx * SimpleData.iMx];
        public double[] CovarianceMatrixS_p = new double[SimpleData.iMx * SimpleData.iMx];
        public double[] CovarianceMatrixNoise = new double[SimpleData.iMx * SimpleData.iMq];
        public double[] TransitionMatrixF = new double[SimpleData.iMx * SimpleData.iMx];

        public double Noise_Pos;
        public double Noise_Vel;
        public double Noise_Angl;
        public double Noise_Drift;
        public double Noise_Accel;

        public double OdoNoise_STOP;

        public class BufferArray
        {
            public int[] Count = new int[60];

            public double[] odometer_left = new double[60], odometer_right = new double[60];
            public int[] odo_left_IsReady = new int[60], odo_right_IsReady = new int[60];
            public int OdoTimeStepCount;
            public double OdometerLeftPrev, OdometerRightPrev;

            public double[,] V_x = new double[60, 3];
            public double[,] F_z = new double[60, 3];
            public double[,] W_z = new double[60, 3];
        }
    }

    public class Proc_Help
    {
        public bool initCount = false;
        public string datastring;

        public double distance, distance_from_start;

        public double[] distance_GK_Sarat = new double[46];

        public double LatSNS, LongSNS, AltSNS, SpeedSNS, Ve_SNS, Vn_SNS;

        public int corrected = 0, AlgnCnt = 0;
    }
}

