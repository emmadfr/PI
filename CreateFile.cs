using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading.Tasks;

namespace Motion
{
    public class CreateFile
    {
        public string myfile { get; set; }
        public string filename { get; set; }

        public void createOutput(string sourceFile, string format)
        {
            string year = DateTime.Now.Year.ToString().Substring(2);
            string month = DateTime.Now.Month.ToString("00");
            string day = DateTime.Now.Day.ToString("00");
            string hour = DateTime.Now.Hour.ToString("00");
            string minute = DateTime.Now.Minute.ToString("00");
            string second = DateTime.Now.Second.ToString("00");
            string timestamp = year + month + day + "_" + hour + minute + second;
            
            string destinationFile = @"../../Output/acquisition_"+timestamp+format;
            if (File.Exists(destinationFile))
            {
                File.Delete(destinationFile);
            }
            File.Copy(sourceFile, destinationFile);
        }
    }
}