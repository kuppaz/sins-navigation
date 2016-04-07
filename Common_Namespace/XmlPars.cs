using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using System.Xml;
using System.IO; 


namespace Common_Namespace
{
    public class XmlPars
    {
        public static void ReadDataForAAStudent()
        {
            StreamWriter Result_DataForAAStudent = new StreamWriter("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//DataForAAStudent_Result.txt");
            StreamReader read = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//DataForAAStudent.txt");

            SINS_State SINSstate = new SINS_State();
            string datastring = "";
            double[] dS_x = new double[3];
            string[] tt;
            char[] separator = new char[] {' '};


            List<double> b = new List<double>();

            double[] a = new double[] {1, 4, 8};
            a = a.Where(o => o < 6).ToArray();
            b = b.Where(o => o < 6).ToList();

            var myFile = new StreamReader("D://Ботва//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//MovingImitator_Azimut//SINS motion processing_new data//AzimutB_210530_Other_120814_Autolab_10-31-26_2.dat");


            Matrix A_sx0 = new Matrix(3,3);
            Matrix A_x0s = new Matrix(3, 3);

            datastring = read.ReadLine();

            SINSstate.Latitude = 57.9987987 * SimpleData.ToRadian;
            SINSstate.Longitude = 56.268466 * SimpleData.ToRadian;
            SINSstate.Height = 173.8157;

            datastring = myFile.ReadLine();

            for (int i = 0; i < 200000; i++ )
            {
                

                datastring = myFile.ReadLine();

                //datastring = read.ReadLine();
                tt = datastring.Split(separator, StringSplitOptions.RemoveEmptyEntries);

                SINSstate.Time = Convert.ToDouble(tt[0]);
                SINSstate.Heading = Convert.ToDouble(tt[1]) * SimpleData.ToRadian;
                SINSstate.Roll = Convert.ToDouble(tt[2]) * SimpleData.ToRadian;
                SINSstate.Pitch = Convert.ToDouble(tt[3]) * SimpleData.ToRadian;
                SINSstate.OdometerData.odometer_left.Value = Convert.ToDouble(tt[4]);

                b.Add(SINSstate.Time);
                b.Add(SINSstate.Heading);
                b.Add(SINSstate.Roll);
                b.Add(SINSstate.Pitch);
                b.Add(Convert.ToDouble(tt[5]));
                b.Add(SINSstate.OdometerData.odometer_left.Value);
                b.Add(Convert.ToDouble(tt[6]));
                b.Add(Convert.ToDouble(tt[7]));
                b.Add(Convert.ToDouble(tt[8]));
                b.Add(Convert.ToDouble(tt[9]));
                b.Add(Convert.ToDouble(tt[10]));
                b.Add(Convert.ToDouble(tt[11]));
                b.Add(Convert.ToDouble(tt[12]));
                b.Add(Convert.ToDouble(tt[13]));
                b.Add(SINSstate.Time);
                b.Add(SINSstate.Heading);
                b.Add(SINSstate.Roll);
                b.Add(SINSstate.Pitch);
                b.Add(Convert.ToDouble(tt[5]));
                b.Add(SINSstate.OdometerData.odometer_left.Value);
                b.Add(Convert.ToDouble(tt[6]));
                b.Add(Convert.ToDouble(tt[7]));
                b.Add(Convert.ToDouble(tt[8]));
                b.Add(Convert.ToDouble(tt[9]));
                b.Add(Convert.ToDouble(tt[10]));
                b.Add(Convert.ToDouble(tt[11]));
                b.Add(Convert.ToDouble(tt[12]));
                b.Add(Convert.ToDouble(tt[13]));
                b.Add(SINSstate.Time);
                b.Add(SINSstate.Heading);
                b.Add(SINSstate.Roll);
                b.Add(SINSstate.Pitch);
                b.Add(Convert.ToDouble(tt[5]));
                b.Add(SINSstate.OdometerData.odometer_left.Value);
                b.Add(Convert.ToDouble(tt[6]));
                b.Add(Convert.ToDouble(tt[7]));
                b.Add(Convert.ToDouble(tt[8]));
                b.Add(Convert.ToDouble(tt[9]));
                b.Add(Convert.ToDouble(tt[10]));
                b.Add(Convert.ToDouble(tt[11]));
                b.Add(Convert.ToDouble(tt[12]));
                b.Add(Convert.ToDouble(tt[13]));
                b.Add(SINSstate.Time);
                b.Add(SINSstate.Heading);
                b.Add(SINSstate.Roll);
                b.Add(SINSstate.Pitch);
                b.Add(Convert.ToDouble(tt[5]));
                b.Add(SINSstate.OdometerData.odometer_left.Value);
                b.Add(Convert.ToDouble(tt[6]));
                b.Add(Convert.ToDouble(tt[7]));
                b.Add(Convert.ToDouble(tt[8]));
                b.Add(Convert.ToDouble(tt[9]));
                b.Add(Convert.ToDouble(tt[10]));
                b.Add(Convert.ToDouble(tt[11]));
                b.Add(Convert.ToDouble(tt[12]));
                b.Add(Convert.ToDouble(tt[13]));


                A_sx0 = SimpleOperations.A_sx0(SINSstate);
                A_x0s = A_sx0.Transpose();

                SINSstate.R_e = SimpleOperations.RadiusE(SINSstate.Latitude, SINSstate.Height);
                SINSstate.R_n = SimpleOperations.RadiusN(SINSstate.Latitude, SINSstate.Height);

                SINSstate.OdometerVector[1] = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;
                SimpleOperations.CopyArray(dS_x, A_x0s * SINSstate.OdometerVector);

                SINSstate.Latitude = SINSstate.Latitude + dS_x[1] / SINSstate.R_n;
                SINSstate.Longitude = SINSstate.Longitude + dS_x[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);
                SINSstate.Height = SINSstate.Height + dS_x[2];

                SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;

                string temp = (SINSstate.Latitude * SimpleData.ToDegree).ToString() + " " + (SINSstate.Longitude * SimpleData.ToDegree).ToString() + " " + SINSstate.Height;

                Result_DataForAAStudent.WriteLine(temp);
            }
            Result_DataForAAStudent.Close();
            
            int y = b.Count(o => o < 1000);
            DateTime t1 = DateTime.Now;
            List<double> bb = b.Where(o => o * o / 7 < 1000).ToList();
            DateTime t2 = DateTime.Now;
            List<double> bbb = b.AsParallel().Where(o => o * o / 7 < 1000).ToList();
            DateTime t3 = DateTime.Now;
            Console.WriteLine((t2 - t1).ToString());
            Console.WriteLine((t3 - t2).ToString());
            
        }

        public static void ReadPDBDocument()
        {
            string word_, translations_, examples_; // Новые переменные имени и пароля  
            FileStream fs1 = new FileStream("D://OutXML.csv", FileMode.OpenOrCreate, FileAccess.Write, FileShare.None);
            StreamWriter OutXML = new StreamWriter(fs1, System.Text.UnicodeEncoding.Unicode);

            // Объявляем и забиваем файл в документ  
            StreamReader fs = new StreamReader("D://grammar2.pdb");

            int j = 0, jt = 0;

            for (int i = 0; i < 100; i++)
            {
                word_ = fs.ReadLine();
                int t = word_.Length;
                bool flg = word_.Contains("arisen");
                if (flg == true)
                {
                    flg = flg;

                    string[] str_split = word_.Split(' ');


                }
            }
            // Закрываем поток  
            OutXML.Close();
            fs1.Close();
            fs.Close();
        }  

        public static void ReadXMLDocument()
        {
            string word_, translations_, examples_; // Новые переменные имени и пароля  
            FileStream fs1 = new FileStream("D://OutXML.csv", FileMode.OpenOrCreate, FileAccess.Write, FileShare.None);
            StreamWriter OutXML = new StreamWriter(fs1, System.Text.UnicodeEncoding.Unicode);

            // Объявляем и забиваем файл в документ  
            XmlDocument xd = new XmlDocument();
            //FileStream fs = new FileStream("D://kuppaz(En-Ru).xml", FileMode.Open);
            //FileStream fs = new FileStream("D://kuppaz vol.2(En-Ru).xml", FileMode.Open);
            FileStream fs = new FileStream("D://grammar2.pdb", FileMode.Open);
            xd.Load(fs);

            int j = 0, jt = 0;
            XmlNodeList list = xd.GetElementsByTagName("card");

            for (int i = 0; i < list.Count; i++, j++, jt++)
            {
                XmlElement list_ = (XmlElement)xd.GetElementsByTagName("card")[i];

                translations_ = "";
                examples_ = "";
                word_ = "";

                int flg_translation = 0, flg_word = 0, flg_example = 0;
                int cnt_translation = 0, cnt_word = 0, cnt_example = 0;

                string[] tt = list_.InnerXml.ToString().Split('<', '>');
                for (int y = 0; y < tt.Length; y++)
                {
                    if (flg_word == 1)
                    {
                        word_ = tt[y];
                        flg_word = 0;
                    }
                    if (tt[y].Contains("word wordId") == true)
                        flg_word = 1;


                    if (word_ == "gust")
                        word_ = word_;


                    if (flg_translation == 1)
                    {
                        cnt_translation++;
                        if (cnt_translation == 3)
                        {
                            translations_ += "\t ";
                            cnt_translation = 0;
                        }
                        translations_ += tt[y] + "; ";
                        flg_translation = 0;
                    }
                    if (tt[y] == "word")
                    {
                        flg_translation = 1;
                    }


                    if (flg_example == 1)
                    {
                        examples_ += tt[y] + "; ";
                        flg_example = 0;
                    }
                    if (tt[y] == "example")
                    {
                        flg_example = 1;
                    }
                }

                string outstring = "";
                if (examples_ != "")
                    outstring = word_ + "\t" + translations_ + "\t" + examples_;
                else
                    outstring = word_ + "\t" + translations_;

                OutXML.WriteLine(outstring);


            }
            // Закрываем поток  
            OutXML.Close();
            fs1.Close();
            fs.Close();
        }  


        public static void ReadXMLDocument_old()
        {
            string word_, translations_, examples_; // Новые переменные имени и пароля  
            FileStream fs1 = new FileStream("D://OutXML.csv", FileMode.OpenOrCreate, FileAccess.Write, FileShare.None);
            StreamWriter OutXML = new StreamWriter(fs1, System.Text.UnicodeEncoding.Unicode);

            // Объявляем и забиваем файл в документ  
            XmlDocument xd = new XmlDocument();
            //FileStream fs = new FileStream("D://kuppaz(En-Ru).xml", FileMode.Open);
            FileStream fs = new FileStream("D://kuppaz vol.2(En-Ru).xml", FileMode.Open);
            xd.Load(fs);

            int j = 0, jt =0;
            XmlNodeList list = xd.GetElementsByTagName("card");

            for (int i = 0; i < list.Count; i++, j++, jt++)
            {
                XmlElement word = (XmlElement)xd.GetElementsByTagName("word")[j];
                XmlElement list_ = (XmlElement)xd.GetElementsByTagName("card")[i];
                XmlNodeList meaning = list_.GetElementsByTagName("meaning");

                if (meaning.Count > 1)
                    j = j;

                word_ = word.InnerText;

                XmlElement translations = (XmlElement)xd.GetElementsByTagName("translations")[jt];   // Забиваем password в переменную  
                XmlElement examples = (XmlElement)xd.GetElementsByTagName("examples")[i];   // Забиваем password в переменную 
                XmlNodeList asdf = translations.GetElementsByTagName("word");

                translations_ = null;
                int flg = 0;

                string[] tt = translations.InnerXml.ToString().Split('<', '>');
                for (int y = 0; y < tt.Length; y++)
                {
                    if (flg == 1)
                    {
                        translations_ += tt[y];
                        if (tt.Length > 5)
                            translations_ = translations_ + "; ";
                    }
                    if (tt[y] == "word")
                    {
                        flg = 1;
                    }
                    else flg = 0;
                }

                for (int t = 0; t < asdf.Count; t++)
                {
                    j++;
                    word = (XmlElement)xd.GetElementsByTagName("word")[j];
                }
                for (int t = 0; t < meaning.Count - 1; t++)
                {
                    jt++;
                    translations = (XmlElement)xd.GetElementsByTagName("translations")[jt];   // Забиваем password в переменную  
                    j++;
                    word = (XmlElement)xd.GetElementsByTagName("word")[j];
                }
                examples_ = examples.InnerText;


                OutXML.WriteLine(word_ + "\t" + translations_ + "\t" + examples_);


            }
            // Закрываем поток  
            fs.Close();
        }  
    }
}
