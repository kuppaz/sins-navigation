using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace SINSAlignment
{
    public class SINSAlignment_Classical
    {
        public static int SINS_Alignment_Classical(Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod, StreamReader myFile, Kalman_Vars KalmanVars)
        {
            int i = 0, t = 0;

            int iMx = 9;
            int iMq = 3;
            int iMz = 7;

            StreamWriter Alignment_Errors = new StreamWriter(SimpleData.PathOutputString + "Alignment//Alignment_Errors.txt");
            StreamWriter Alignment_SINSstate = new StreamWriter(SimpleData.PathOutputString + "Alignment//Alignment_SINSstate.txt");
            StreamWriter Alignment_Corrected_State = new StreamWriter(SimpleData.PathOutputString + "Alignment//Alignment_Corrected_State.txt");
            StreamWriter Alignment_StateErrorsVector = new StreamWriter(SimpleData.PathOutputString + "Alignment//Alignment_StateErrorsVector.txt");
            StreamWriter Alignment_STD_Data = new StreamWriter(SimpleData.PathOutputString + "Alignment//Alignment_STD_Data.txt");

            Alignment_Errors.WriteLine("DeltaHeading DeltaRoll DeltaPitch");
            Alignment_StateErrorsVector.WriteLine("Time Beta1 Beta2 Beta3  dF1  dF2  dF3 Nu1  Nu2  Nu3  ");
            Alignment_SINSstate.WriteLine("Time  Count Lat Long Altitude V1 V2 Heading HeadingCor Roll RollCor Pitch PitchCor");


            //---Этап грубой выставки---

            //int temp_AlgnCnt = ProcHelp.AlgnCnt;
            //ProcHelp.AlgnCnt = Convert.ToInt32(120.0 / SINSstate.Freq);

            i = Alignment.RougthAlignment(ProcHelp, SINSstate, myFile, KalmanVars, SINSstate_OdoMod);

            //ProcHelp.AlgnCnt = temp_AlgnCnt;
            SINSstate.flag_Alignment = true;


            

            if (false)
            {
                Kalman_Align KalmanAlign = new Kalman_Align();

                SimpleOperations.CopyArray(KalmanAlign.Noise_Vel, KalmanVars.Noise_Vel);
                SimpleOperations.CopyArray(KalmanAlign.Noise_Angl, KalmanVars.Noise_Angl);
                Alignment_Classical.InitOfCovarianceMatrixes(KalmanAlign);

                for (int j = i; j < ProcHelp.AlgnCnt; j++)
                {
                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);


                    Alignment_Classical.AlignStateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate_OdoMod);
                    Alignment_Classical.Make_A(SINSstate, KalmanAlign);
                    Alignment_Classical.MatrixNoise_ReDef(SINSstate, KalmanAlign);

                    KalmanProcs.Make_F_Align(SINSstate.timeStep, KalmanAlign);
                    KalmanProcs.KalmanForecast_Align(KalmanAlign);

                    Alignment_Classical.Make_H_and_Correction(SINSstate, KalmanAlign);

                    i = j;

                    if (j % 200 == 0)
                        Console.WriteLine(SINSstate.Count.ToString() + ",  " + (SINSstate.Latitude * SimpleData.ToDegree - ProcHelp.LatSNS).ToString() + ",  "  + SINSstate.F_x[2].ToString().ToString());

                    SINSstate.DeltaRoll = -(KalmanAlign.ErrorConditionVector_p[0] * Math.Sin(SINSstate.Heading) + KalmanAlign.ErrorConditionVector_p[1] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
                    SINSstate.DeltaPitch = -KalmanAlign.ErrorConditionVector_p[0] * Math.Cos(SINSstate.Heading) + KalmanAlign.ErrorConditionVector_p[1] * Math.Sin(SINSstate.Heading);
                    SINSstate.DeltaHeading = KalmanAlign.ErrorConditionVector_p[2] + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);

                    Alignment_Errors.WriteLine(SINSstate.DeltaHeading * 180.0 / 3.141592 + " " + SINSstate.DeltaRoll * 180.0 / 3.141592 + " " + SINSstate.DeltaPitch * 180.0 / 3.141592);

                    Alignment_STD_Data.WriteLine(KalmanProcs.Sigmf_Disp(0, KalmanAlign) + " " + KalmanProcs.Sigmf_Disp(1, KalmanAlign) + " " + KalmanProcs.Sigmf_Disp(2, KalmanAlign) + " " + KalmanProcs.Sigmf_Disp(3, KalmanAlign) + " " +
                                KalmanProcs.Sigmf_Disp(4, KalmanAlign) + " " + KalmanProcs.Sigmf_Disp(5, KalmanAlign) + " " + KalmanProcs.Sigmf_Disp(6, KalmanAlign) + " " + KalmanProcs.Sigmf_Disp(7, KalmanAlign) + " " + KalmanProcs.Sigmf_Disp(8, KalmanAlign));

                    Alignment.OutPutInfo_Class_Alignment(ProcHelp, SINSstate, SINSstate2, myFile, KalmanAlign, Alignment_Errors, Alignment_SINSstate, Alignment_Corrected_State, Alignment_StateErrorsVector);

                }

                //SimpleOperations.PrintMatrixToFile(KalmanAlign.CovarianceMatrixS_p, SimpleData.iMx_Align, SimpleData.iMx_Align);

                SINSstate.Heading = SINSstate.Heading - SINSstate.DeltaHeading;
                SINSstate.Roll = SINSstate.Roll - SINSstate.DeltaRoll;
                SINSstate.Pitch = SINSstate.Pitch - SINSstate.DeltaPitch;

                //KalmanVars.CovarianceMatrixS_m[4 * SimpleData.iMx + 4] = KalmanVars.CovarianceMatrixS_p[4 * SimpleData.iMx + 4] = KalmanProcs.Sigmf_Disp(0, KalmanAlign);
                //KalmanVars.CovarianceMatrixS_m[5 * SimpleData.iMx + 5] = KalmanVars.CovarianceMatrixS_p[5 * SimpleData.iMx + 5] = KalmanProcs.Sigmf_Disp(1, KalmanAlign);
                //KalmanVars.CovarianceMatrixS_m[6 * SimpleData.iMx + 6] = KalmanVars.CovarianceMatrixS_p[6 * SimpleData.iMx + 6] = KalmanProcs.Sigmf_Disp(2, KalmanAlign);

                //KalmanVars.CovarianceMatrixS_m[10 * SimpleData.iMx + 10] = KalmanVars.CovarianceMatrixS_p[10 * SimpleData.iMx + 10] = KalmanProcs.Sigmf_Disp(3, KalmanAlign);
                //KalmanVars.CovarianceMatrixS_m[11 * SimpleData.iMx + 11] = KalmanVars.CovarianceMatrixS_p[11 * SimpleData.iMx + 11] = KalmanProcs.Sigmf_Disp(4, KalmanAlign);
                //KalmanVars.CovarianceMatrixS_m[12 * SimpleData.iMx + 12] = KalmanVars.CovarianceMatrixS_p[12 * SimpleData.iMx + 12] = KalmanProcs.Sigmf_Disp(5, KalmanAlign);

                //KalmanVars.CovarianceMatrixS_m[7 * SimpleData.iMx + 7] = KalmanVars.CovarianceMatrixS_p[7 * SimpleData.iMx + 7] = KalmanProcs.Sigmf_Disp(6, KalmanAlign);
                //KalmanVars.CovarianceMatrixS_m[8 * SimpleData.iMx + 8] = KalmanVars.CovarianceMatrixS_p[8 * SimpleData.iMx + 8] = KalmanProcs.Sigmf_Disp(7, KalmanAlign);
                //KalmanVars.CovarianceMatrixS_m[9 * SimpleData.iMx + 9] = KalmanVars.CovarianceMatrixS_p[9 * SimpleData.iMx + 9] = KalmanProcs.Sigmf_Disp(8, KalmanAlign);


                SINSstate.Time_Alignment = SINSstate.Time;

                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
                if (SINSstate.Global_file == "Azimuth_minsk_race_4_3to6to2")
                {
                    //SINSstate.Heading = -3.0504734;
                }
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //


                SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
                SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
                SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
                SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time - SINSstate.Time_Alignment, SINSstate.Longitude_Start);
                SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
                SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);




                Alignment_Errors.Close(); Alignment_Corrected_State.Close(); Alignment_SINSstate.Close(); Alignment_StateErrorsVector.Close();
                /*----------------------------------------------------------------------------------------*/
            }

            ProcHelp.initCount = false;
            SINSstate.flag_Alignment = false;
            return i;
        }
    }
}
