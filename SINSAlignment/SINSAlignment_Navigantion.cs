using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace SINSAlignment
{
    public class SINSAlignment_Navigantion
    {
        public static int SINS_Alignment_Navigation(Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate2, SINS_State SINSstate_OdoMod, StreamReader myFile, Kalman_Vars KalmanVars, StreamWriter GRTV_output)
        {
            int i = 0, t = 0;

            SimpleData.iMx = 13;
            SimpleData.iMq = 5;
            SimpleData.iMz = 7;

            StreamWriter Alignment_Errors = new StreamWriter(SimpleData.PathOutputString + "Alignment_Errors.txt");
            StreamWriter Alignment_SINSstate = new StreamWriter(SimpleData.PathOutputString + "Alignment_SINSstate.txt");
            StreamWriter Alignment_Corrected_State = new StreamWriter(SimpleData.PathOutputString + "Alignment_Corrected_State.txt");
            StreamWriter Alignment_StateErrorsVector = new StreamWriter(SimpleData.PathOutputString + "Alignment_StateErrorsVector.txt");

            Alignment_Errors.WriteLine("dR1  dR2  dV1  dV2  Alpha1 Alpha2 Beta3  Nu1  Nu2  Nu3  dF1  dF2  dF3");
            Alignment_Corrected_State.WriteLine("Time  Count  LatCrtd Lat  LongCrtd    Long  AltitudeCrtd V1 V2 V3 Heading HeadingCor Roll RollCor  Pitch PitchCor");


            //---Этап грубой выставки---

            //int temp_AlgnCnt = ProcHelp.AlgnCnt;
            //ProcHelp.AlgnCnt = Convert.ToInt32(200.0 / SINSstate.Freq);

            i = Alignment.RougthAlignment(ProcHelp, SINSstate, myFile, KalmanVars, SINSstate_OdoMod, GRTV_output);

            //ProcHelp.AlgnCnt = temp_AlgnCnt;
            SINSstate.flag_Alignment = true;

            if (false)
            {
                Alignment_Navigation.InitOfCovarianceMatrixes(KalmanVars);     //---Инициализация ковариационных матриц матриц вектора ошибок---//
                /*----------------------------------------------------------------------------------------*/

                while (true)
                {
                    if (SINSstate.FLG_Stop == 0 || (ProcHelp.AlignmentCounts != 0 && i == ProcHelp.AlignmentCounts))
                        break;

                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, null, SINSstate, SINSstate_OdoMod, true);
                    if (t == 0) { SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z); SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z); t = 1; }

                    SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate2, SINSstate2);

                    Alignment_Navigation.MatrixNoise_ReDef(SINSstate, KalmanVars);   //изменить все эти функции
                    Alignment_Navigation.Make_A_easy(SINSstate2, KalmanVars);
                    KalmanProcs.Make_F(SINSstate.timeStep, KalmanVars, SINSstate);
                    KalmanProcs.KalmanForecast(KalmanVars, SINSstate);

                    Alignment_Navigation.Make_H(KalmanVars, SINSstate);

                    KalmanProcs.KalmanCorrection(KalmanVars, SINSstate, SINSstate);

                    Alignment_Navigation.CalcStateErrors(KalmanVars.ErrorConditionVector_p, SINSstate);
                    Alignment_Navigation.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate2);
                    

                    Alignment.OutPutInfo_Nav_Alignment(ProcHelp, SINSstate, SINSstate2, myFile, KalmanVars, Alignment_Errors, Alignment_SINSstate, Alignment_Corrected_State, Alignment_StateErrorsVector);

                    SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
                    SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z);

                    if (i > 100 && i % 500 == 0)
                        Console.WriteLine(SINSstate.Count.ToString() + ",  " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + ",  " + (SINSstate.Heading * SimpleData.ToDegree).ToString() + ",  " +
                                (SINSstate2.Heading * SimpleData.ToDegree).ToString() + ",  " + KalmanVars.ErrorConditionVector_p[0].ToString() + ",  " + KalmanVars.ErrorConditionVector_p[1].ToString());
                    i++;
                }



                SINSstate.Heading = SINSstate.Heading - SINSstate.DeltaHeading;
                SINSstate.Roll = SINSstate.Roll - SINSstate.DeltaRoll;
                SINSstate.Pitch = SINSstate.Pitch - SINSstate.DeltaPitch;


                KalmanVars.CovarianceMatrixS_m[0 * SimpleData.iMx + 0] = KalmanVars.CovarianceMatrixS_p[0 * SimpleData.iMx + 0] = KalmanProcs.Sigmf_Disp(0, KalmanVars);
                KalmanVars.CovarianceMatrixS_m[1 * SimpleData.iMx + 1] = KalmanVars.CovarianceMatrixS_p[1 * SimpleData.iMx + 1] = KalmanProcs.Sigmf_Disp(1, KalmanVars);
                KalmanVars.CovarianceMatrixS_m[2 * SimpleData.iMx + 2] = KalmanVars.CovarianceMatrixS_p[2 * SimpleData.iMx + 2] = KalmanProcs.Sigmf_Disp(2, KalmanVars);
                KalmanVars.CovarianceMatrixS_m[3 * SimpleData.iMx + 3] = KalmanVars.CovarianceMatrixS_p[3 * SimpleData.iMx + 3] = KalmanProcs.Sigmf_Disp(3, KalmanVars);

                KalmanVars.CovarianceMatrixS_m[10 * SimpleData.iMx + 10] = KalmanVars.CovarianceMatrixS_p[10 * SimpleData.iMx + 10] = KalmanProcs.Sigmf_Disp(10, KalmanVars);
                KalmanVars.CovarianceMatrixS_m[11 * SimpleData.iMx + 11] = KalmanVars.CovarianceMatrixS_p[11 * SimpleData.iMx + 11] = KalmanProcs.Sigmf_Disp(11, KalmanVars);
                KalmanVars.CovarianceMatrixS_m[12 * SimpleData.iMx + 12] = KalmanVars.CovarianceMatrixS_p[12 * SimpleData.iMx + 12] = KalmanProcs.Sigmf_Disp(12, KalmanVars);

                KalmanVars.CovarianceMatrixS_m[7 * SimpleData.iMx + 7] = KalmanVars.CovarianceMatrixS_p[7 * SimpleData.iMx + 7] = KalmanProcs.Sigmf_Disp(7, KalmanVars);
                KalmanVars.CovarianceMatrixS_m[8 * SimpleData.iMx + 8] = KalmanVars.CovarianceMatrixS_p[8 * SimpleData.iMx + 8] = KalmanProcs.Sigmf_Disp(8, KalmanVars);
                KalmanVars.CovarianceMatrixS_m[9 * SimpleData.iMx + 9] = KalmanVars.CovarianceMatrixS_p[9 * SimpleData.iMx + 9] = KalmanProcs.Sigmf_Disp(9, KalmanVars);


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
