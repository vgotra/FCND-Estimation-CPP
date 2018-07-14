using System;
using System.IO;
using System.Linq;

namespace SensorNoise
{
    class Program
    {
        static void Main(string[] args)
        {
            // for copied files into the binary folder
            var gps = @"Graph1.txt";
            var acc = @"Graph2.txt";

            // for project
            //var gps = @"/Users/vgotra/projects/#udacity/FCND-Estimation-CPP/config/log/Graph1.txt";
            //var acc = @"/Users/vgotra/projects/#udacity/FCND-Estimation-CPP/config/log/Graph2.txt";
            
            var gpsData = File.ReadAllLines(gps);
            var accData = File.ReadAllLines(acc);
            
            var gpsList = gpsData.Skip(1).Select(x => x.Split(',')).Select(x => double.Parse(x[1])).ToList();
            var accList = accData.Skip(1).Select(x => x.Split(',')).Select(x => double.Parse(x[1])).ToList();
            
            var avgGps = gpsList.Average();
            var avgAcc = accList.Average();
            
            var gpsSd = Math.Sqrt(gpsList.Select(x => x - avgGps).Select(x => Math.Pow(x, 2)).Sum() / (gpsList.Count - 1));
            var accSd = Math.Sqrt(accList.Select(x => x - avgAcc).Select(x => Math.Pow(x, 2)).Sum() / (accList.Count - 1));
            
            Console.WriteLine($"Avg GPS: {avgGps}, GPS Standard Deviation: {gpsSd}");
            Console.WriteLine($"Avg ACC: {avgAcc}, ACC Standard Deviation: {accSd}");
        }
    }
}
