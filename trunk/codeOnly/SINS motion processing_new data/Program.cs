using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace SINS_motion_processing_new_data
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new SINS_Processing());
        }
    }
}
