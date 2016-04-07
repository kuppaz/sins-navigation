using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class GeodesicVsGreenwich : SimpleData
    {
        //---------------------------------------------------------------------------------------
        //
        // по географическим координатам dLambda, dPhi, dHeight вычисляются гринвичские R
        // A - большая полуось, e2 - квадрат 1-го эксцентриситета 
        static double[] Geodesic2Greenwich(double dLambda, double dPhi, double dHeight, double A, double e2)
        {
            double[] R = new double[3];
            double R2 = 0.0;
            double sin_phi = 0.0, cos_phi = 0.0;

            sin_phi = Math.Sin(dPhi);
            cos_phi = Math.Cos(dPhi);

            R2 = A / Math.Sqrt(1.0 - e2 * sin_phi * sin_phi);

            R[0] = (R2 + dHeight) * Math.Cos(dLambda) * cos_phi;
            R[1] = (R2 + dHeight) * Math.Sin(dLambda) * cos_phi;
            R[2] = ((1.0 - e2) * R2 + dHeight) * sin_phi;

            return R;
        }


        //---------------------------------------------------
        //
        // по гринвичским R координатам   вычисляются географические dLambda, dPhi, dHeight 
        // A - большая полуось, e2 - квадрат 1-го эксцентриситета 
        static double[] Greenwich2Geodesic(double[] R, double A, double e2)
        {
            double[] PhiLambdaH = new double[3];

            double dRxy = 0.0, dRxy2 = 0.0;
            double dR2 = 0.0;
            double sin_phi = 0.0, cos_phi = 0.0;
            double dDelta_Rxy = 0.0, dDelta_R3 = 0.0;

            double dDelta_phi = 0.0, dDelta_h = 0.0;
            double c = 0.0, c1 = 0.0, dDet = 0.0;

            double dR2_phi = 0.0, dF1_phi = 0.0, dF2_phi = 0.0, dF1_h = 0.0, dF2_h = 0.0;
            double dC = 0.0;
            int iIterations = 0;

            // Simple Check
            dC = Math.Sqrt(R[0] * R[0] + R[1] * R[1] + R[2] * R[2]);
            //if (Math.Abs(dC) < 1.0) return;

            PhiLambdaH[1] = Math.Atan2(R[1], R[0]);


            dRxy2 = R[0] * R[0] + R[1] * R[1];
            dRxy = Math.Sqrt(dRxy2);


            PhiLambdaH[0] = Math.Atan2(R[2], dRxy);
            sin_phi = Math.Sin(PhiLambdaH[0]);

            PhiLambdaH[2] = Math.Sqrt(dRxy2 + R[2] * R[2]) - A + A * e2 * sin_phi * sin_phi * 0.5;
            PhiLambdaH[0] += 0.5 * e2 * (1.0 - PhiLambdaH[2] / A) * sin_phi * Math.Cos(PhiLambdaH[0]);


            //--------------

            for (; ; )
            {
                cos_phi = Math.Cos(PhiLambdaH[0]);
                sin_phi = Math.Sin(PhiLambdaH[0]);

                c = 1.0 / (1.0 - e2 * sin_phi * sin_phi);
                c1 = Math.Sqrt(c);
                dR2 = A * c1;

                dDelta_Rxy = dRxy - (dR2 + PhiLambdaH[2]) * cos_phi;
                dDelta_R3 = R[2] - ((1.0 - e2) * dR2 + PhiLambdaH[2]) * sin_phi;

                dR2_phi = -dR2 * e2 * sin_phi * cos_phi * c;
                dF1_phi = -dR2_phi * cos_phi + (dR2 + PhiLambdaH[2]) * sin_phi;
                dF2_phi = -(1.0 - e2) * dR2_phi * sin_phi - ((1.0 - e2) * dR2 + PhiLambdaH[2]) * cos_phi;
                dF1_h = -cos_phi;
                dF2_h = -sin_phi;
                dDet = 1.0 / (dF1_phi * dF2_h - dF2_phi * dF1_h);


                dDelta_phi = -dDet * (dF2_h * dDelta_Rxy - dF1_h * dDelta_R3);
                dDelta_h = -dDet * (-dF2_phi * dDelta_Rxy + dF1_phi * dDelta_R3);

                PhiLambdaH[0] += dDelta_phi;
                PhiLambdaH[2] += dDelta_h;

                if ((dDelta_phi * dDelta_phi * A * A + dDelta_h * dDelta_h) < 0.001 * 0.001) break;
                iIterations++;
                if (iIterations > 10) break;
            }

            return PhiLambdaH;
        }

        //
        // по гринвичским R_input координатам одном эллипсоиде вычисляются  
        // гринвичские R_output координаты в другом  эллипсоиде 
        // iType определяет направление перехода 
        //-------------------------------------------------------------------
        static double[] Greenwich2Greenwich(double[] pdR_input, int iType)
        {
            double[] pdR_output = new double[3];

            // iType = 5  from  WGS84 to PE90
            // iType = 4  from  PE90  to WGS84
            // iType = 3  from  PE90  to 42
            // iType = 2  from  42  to PE90
            // iType = 1  from  42  to WGS84
            // iType = 0  from  WGS84  to 42


            double dM = 0.0, dDx = 0.0, dDy = 0.0, dDz = 0.0, dOmega_x = 0.0, dOmega_y = 0.0, dOmega_z = 0.0;

            double c;

            switch (iType)
            {
                case 0:   // WGS84 to 42
                    dM = 0.12e-6;
                    dDx = -23.9; dDy = 141.3; dDz = 80.9;
                    dOmega_x = 0.0; dOmega_y = +1.8e-6; dOmega_z = +4.12e-6;
                    break;
                case 1:   //  42 to WGS84
                    dM = -0.12e-6;
                    dDx = 23.9; dDy = -141.3; dDz = -80.9;
                    dOmega_x = 0.0; dOmega_y = -1.8e-6; dOmega_z = -4.12e-6;
                    break;
                case 2:    // iType = 2  from  42  to PE90
                    dM = 0.0;
                    dDx = 25.0; dDy = -141.0; dDz = -80.0;
                    dOmega_x = 0.0; dOmega_y = 1.6992e-6; dOmega_z = -3.12e-6;
                    break;
                case 3:   // iType = 3  from  PE90  to 42
                    dM = 0.0;
                    dDx = -25.0; dDy = 141.0; dDz = 80.0;
                    dOmega_x = 0.0; dOmega_y = -1.6992e-6; dOmega_z = +3.12e-6;
                    break;
                case 4:  // iType = 4  from  PE90  to WGS84
                    dM = -0.12e-6;
                    dDx = -1.1; dDy = -0.3; dDz = -0.9;
                    dOmega_x = 0.0; dOmega_y = 0.0; dOmega_z = -0.82e-6;
                    break;
                case 5:   // iType = 5  from  WGS84 to PE90
                    dM = 0.12e-6;
                    dDx = 1.1; dDy = 0.3; dDz = 0.9;
                    dOmega_x = 0.0; dOmega_y = 0.0; dOmega_z = +0.969627e-6;
                    break;
            }

            //
            //                     |                               |            |     |
            //                     |   1       dOmega_z  -dOmega_y |            | dDx |
            //                     |                               |            |     |
            //   R(new)  = (1 + dM)|-dOmega_z    1        dOmega_x |*R(old)   + | dDy |
            //                     |                               |            |     |
            //                     | dOmega_y  -dOmega_x     1     |            | dDz |
            //                     |                               |            |     |
            //
            //

            c = 1.0 + dM;


            pdR_output[0] = c * (pdR_input[0] + dOmega_z * pdR_input[1] -
                                                dOmega_y * pdR_input[2]) + dDx;
            pdR_output[1] = c * (pdR_input[1] - dOmega_z * pdR_input[0] +
                                                dOmega_x * pdR_input[2]) + dDy;
            pdR_output[2] = c * (pdR_input[2] + dOmega_y * pdR_input[0] -
                                                dOmega_x * pdR_input[1]) + dDz;


            return pdR_output;
        }

        //-------------------------------------------------------------------
        // по географическим *_in координатам одном эллипсоиде вычисляются  
        // географические *_out координаты в другом  эллипсоиде 
        // iType определяет направление перехода 
        public static double[] Geodesic2Geodesic(double dPhi_in, double dLambda_in, double dHeight_in, int iType)
        {
            double[] PhiLambdaH_out = new double[3];

            // iType = 5  from  WGS84 to PE90
            // iType = 4  from  PE90  to WGS84
            // iType = 3  from  PE90  to 42
            // iType = 2  from  42  to PE90
            // iType = 1  from  42  to WGS84
            // iType = 0  from  WGS84  to 42

            double A_Old = 0.0, E2_Old = 0.0, A_New = 0.0, E2_New = 0.0, Alpha_New = 0.0, Alpha_Old = 0.0;
            double[] dR = new double[3], dR_new = new double[3], A = new double[2], e2 = new double[2];


            switch (iType)
            {

                case 0:
                    A_Old = A_84; A_New = A_42;
                    Alpha_Old = Alpha_84; Alpha_New = Alpha_42;
                    //E2_Old = E2_84;  E2_New = E2_42;
                    // iType = 0  from  WGS84  to 42
                    break;

                case 1:
                    A_Old = A_42; A_New = A_84;
                    Alpha_Old = Alpha_42; Alpha_New = Alpha_84;
                    //E2_Old = E2_42;  E2_New = E2_84;
                    // iType = 1  from  42  to WGS84

                    break;

                case 2:
                    A_Old = A_42; A_New = A_90;
                    Alpha_Old = Alpha_42; Alpha_New = Alpha_90;
                    E2_Old = E2_42; E2_New = E2_90;
                    // iType = 2  from  42  to PE90

                    break;

                case 3:
                    A_Old = A_90; A_New = A_42;
                    Alpha_Old = Alpha_90; Alpha_New = Alpha_42;
                    //E2_Old = E2_90;  E2_New = E2_42;
                    // iType = 3  from  PE90  to 42

                    break;

                case 4:
                    A_Old = A_90; A_New = A_84;
                    Alpha_Old = Alpha_90; Alpha_New = Alpha_84;
                    //E2_Old = E2_90;  E2_New = E2_84;
                    // iType = 4  from  PE90  to WGS84
                    break;

                case 5:
                    A_Old = A_84; A_New = A_90;
                    Alpha_Old = Alpha_84; Alpha_New = Alpha_90;
                    //E2_Old = E2_84;  E2_New = E2_90;
                    // iType = 5  from  WGS84 to PE90

                    break;

            }

            E2_Old = Math.Pow(2.0 * Alpha_Old - Alpha_Old * Alpha_Old, 2.0);
            E2_New = Math.Pow(2.0 * Alpha_New - Alpha_New * Alpha_New, 2.0);

            A[0] = A_Old; e2[0] = E2_Old;
            A[1] = A_New; e2[1] = E2_New;

            dR = Geodesic2Greenwich(dLambda_in, dPhi_in, dHeight_in, A[0], e2[0]);
            dR_new = Greenwich2Greenwich(dR, iType);
            PhiLambdaH_out = Greenwich2Geodesic(dR_new, A[1], e2[1]);

            return PhiLambdaH_out;
        }


    }
}
