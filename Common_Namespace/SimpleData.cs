using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class SimpleData
    {
        public static string PathInputString = "D://SINS Solution//MovingImitator_Azimut 3//trunk//SINS motion processing_new data//All_data//",
                             PathOutputString = "D://SINS Solution//MovingImitator_Azimut 3//trunk//SINS motion processing_new data//Output//",
                             PathTelemetricString = "D://SINS Solution//Imitator_Kompas_temp//Work//DataForImitator//Imitator_";

        //public static string PathInputString = "",
        //                     PathOutputString = "",
        //                     PathTelemetricString = "Imitator_";

        public static int iMx;
        public static int iMxSmthd;
        public static int iMz;
        public static int iMq;

        public static int iMz_Align = 7, iMx_Align = 9, iMq_Align = 3;

        public static double A = 6378137.0; // a - большая полуось
        public static double Ex_Squared = 0.0066943799901413;
        public static double U = 0.000072921151467;

        public static double ToRadian = Math.PI / 180.0;
        public static double ToDegree = 180.0 / Math.PI;
        public static double ToRadian_min = ToRadian / 60.0;
        public static double ToRadian_sec = ToRadian_min / 60.0;
        public static double ToDegree_min = ToDegree * 60.0;
        public static double ToDegree_sec = ToDegree_min * 60.0;

        public static double Gravity_Normal = 9.78049;

        public static Matrix Identity = new Matrix(3, 3);

        public static int StartPosNum;
    }
}
