using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.IO;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Xml.Linq;
using System.Reflection;
using System.Security.Policy;
using System.Threading;
using System.Windows.Threading;
using System.Numerics;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TaskbarClock;
using System.Collections;

namespace Motion
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Variables

        // dynamic array of data
        List<double[]> Data = new List<double[]>();

        // dynamic array of time
        List<double> Time = new List<double>();

        // bool variables for the instructions
        private bool starting = true;
        private bool recording = false;
        private bool stopping = false;

        // bool variables for the record method 
        private bool drawstart = true;
        private bool drawstop = false;

        // variables for the files
        public int counter = 0;
        public string fileTRC = @"../../kinectTRC.txt";
        public string fileMOT = @"../../kinectMOT.txt";

        // variables for the coordinates
        public double originX { get; set; }
        public double originY { get; set; }
        public double originZ { get; set; }
        public double coordinateX { get; set; }
        public double coordinateY { get; set; }
        public double coordinateZ { get; set; }

        // for the initialisiation of the kinect
        private KinectSensor kinectSensor = null;

        // array with the bodies
        IList<Body> bodies;

        // for all data streams
        private MultiSourceFrameReader msfr;

        // for the image control
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private CoordinateMapper coordinateMapper = null;

    #endregion

    public MainWindow()
        {
            // change dots format from , to .
            System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)Thread.CurrentThread.CurrentCulture.Clone();
            customCulture.NumberFormat.NumberDecimalSeparator = ".";
            Thread.CurrentThread.CurrentCulture = customCulture;

            // create instances for the image control
            drawingGroup = new DrawingGroup();
            imageSource = new DrawingImage(this.drawingGroup);
            
            // connection to the Kinect
            kinectSensor = KinectSensor.GetDefault();
            
            // open SoureFrameReader with color and body streams
            msfr = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Body);
            // start eventhandler of SourceFrameReader
            msfr.MultiSourceFrameArrived += msfr_MultiSourceFrameArrived;

            // create instances for coordinate mapping
            coordinateMapper = this.kinectSensor.CoordinateMapper;

            // start the Kinect streams
            kinectSensor.Open();

            // create the time axis
            for (int i = 0; i <= 1500; i++)
            {
                double value = Math.Round(i / 30.0, 2, MidpointRounding.ToEven);
                Time.Add(value); // ajoute la valeur à la liste
            }

            // initialize all
            InitializeComponent();
        }

        public void msfr_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e) // stream manager
        {
            var reference = e.FrameReference.AcquireFrame();

            using (var frame = reference.ColorFrameReference.AcquireFrame()) // Kinect Color Stream
            {
                if (frame != null)
                {
                    camera.Source = frame.ToBitmap(); // display the color stream
                }
            }

            using (var frame = reference.BodyFrameReference.AcquireFrame()) // Kinect Body Stream
            {
                if (frame != null)
                {
                    using (DrawingContext dc = this.drawingGroup.Open())
                    {
                        canvas.Children.Clear();
                        bodies = new Body[frame.BodyFrameSource.BodyCount];
                        frame.GetAndRefreshBodyData(bodies);

                        foreach (var body in bodies)
                        {
                            if (body.IsTracked)
                            {
                                // coordinate mapping
                                foreach (Joint joint in body.Joints.Values)
                                {
                                    if (joint.TrackingState == TrackingState.Tracked)
                                    {
                                        // 3D space point
                                        CameraSpacePoint jointPosition = joint.Position;
                                        // 2D space point
                                        Point point1 = new Point();

                                        ColorSpacePoint colorPoint = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(jointPosition);
                                        point1.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
                                        point1.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;

                                        // draw junction
                                        Ellipse ellipse = new Ellipse
                                        {
                                            Fill = Brushes.LimeGreen,
                                            Width = 15,
                                            Height = 15
                                        };
                                        Canvas.SetLeft(ellipse, point1.X - ellipse.Width / 2);
                                        Canvas.SetTop(ellipse, point1.Y - ellipse.Height / 2);
                                        canvas.Children.Add(ellipse);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            using (var frame = reference.BodyFrameReference.AcquireFrame()) // Message Stream
            {
                if (frame != null)
                {
                    if (starting == true && recording == false && stopping == false) // Put your right hand in the circle to start recording
                    {
                        var uri = new Uri("../../Images/Image_Starting.png", UriKind.Relative);
                        var bitmap = new BitmapImage(uri);
                        instructions.Source = bitmap;
                    }
                    if (starting == true && recording == true && stopping == false) // Put your right hand in the circle to stop recording
                    {
                        var uri = new Uri("../../Images/Image_Recording.png", UriKind.Relative);
                        var bitmap = new BitmapImage(uri);
                        instructions.Source = bitmap;
                    }
                    if (starting == true && recording == false && stopping == true) // Processing data
                    {
                        var uri = new Uri("../../Images/Image_Stopping.png", UriKind.Relative);
                        var bitmap = new BitmapImage(uri);
                        instructions.Source = bitmap;

                        Processing();
                    }

                    frame.GetAndRefreshBodyData(bodies);
                    
                    foreach (var body in bodies)
                    {
                        if (body.IsTracked)
                        {
                            if (drawstart) // draw the circle to begin recording
                            {
                                DrawStartArea(body, JointType.SpineMid);
                            }
                            if (drawstop) // draw the circle to stop recording
                            {
                                DrawStopArea(body, JointType.SpineShoulder);
                            }

                            bool startRecord = IsHandOverStart(body);
                            if (startRecord)
                            {
                                drawstart = false;
                                drawstop = true;

                                recording = true;
                            }

                            bool stopRecord = IsHandOverStop(body);
                            if (stopRecord)
                            {
                                drawstart = true;
                                drawstop = false;

                                recording = false;
                                stopping = true;
                                break;
                            }

                            // define the origin point
                            originX = body.Joints[JointType.SpineBase].Position.X;
                            originY = body.Joints[JointType.SpineBase].Position.Y;
                            originZ = body.Joints[JointType.SpineBase].Position.Z;
                            
                            if (recording)
                            {
                                int n = 0;
                                double[] element = new double[75];
                                
                                // for each joint
                                foreach (Joint joint in body.Joints.Values)
                                {
                                    if (joint.TrackingState == TrackingState.Tracked) // if the joint is tracked
                                    {
                                        // change the origin point
                                        double jointX = joint.Position.X - originX;
                                        double jointY = joint.Position.Y - originY;
                                        double jointZ = joint.Position.Z - originZ;

                                        // rotate the axis
                                        coordinateX = jointX * Math.Cos(-90) + jointZ * Math.Sin(-90);
                                        coordinateY = jointY;
                                        coordinateZ = jointX * -Math.Sin(-90) + jointZ * Math.Cos(-90);
                                        
                                        // convert to millimeters with 3 digits after the comma
                                        coordinateX = Math.Round(coordinateX * 1000, 4, MidpointRounding.ToEven);
                                        coordinateY = Math.Round(coordinateY * 1000, 4, MidpointRounding.ToEven);
                                        coordinateZ = Math.Round(coordinateZ * 1000, 4, MidpointRounding.ToEven);
                                        
                                        // add the data to the array
                                        element[n] = coordinateX;
                                        element[n + 1] = coordinateY;
                                        element[n + 2] = coordinateZ;
                                    }
                                    else // if the joint is not tracked
                                    {
                                        // add the data to the array
                                        element[n] = double.NaN;
                                        element[n + 1] = double.NaN;
                                        element[n + 2] = double.NaN;
                                    }
                                    n+=3;
                                }
                                Data.Add(element);
                                counter++;
                            }
                        }
                    }
                }
            }
        }

        public void Processing() // process data
        {
            WriteTRC();
            WriteMOT();

            // create the output file
            var instance = new CreateFile();
            instance.createOutput(fileTRC, ".trc");
            instance.createOutput(fileMOT, ".mot");
            Application.Current.Shutdown();
        }

        private void WriteTRC() // create the TRC file
        {
            File.WriteAllText(fileTRC, ""); // overwrite the file
            using (StreamWriter file = new StreamWriter(fileTRC, true))
            {
                // write header in the file
                file.WriteLine("PathFileType" + "\t" + "4" + "\t" + "(X/Y/Z)" + "\t" + "kinectCoordinates");
                file.WriteLine("DataRate" + "\t" + "CameraRate" + "\t" + "NumFrames" + "\t" + "NumMarkers" + "\t" + "Units" + "\t" + "OrigDataRate" + "\t" + "OrigDataStartFrame" + "\t" + "OrigNumFrames");
                file.WriteLine("30.00" + "\t" + "30.00" + "\t" + counter.ToString() + "\t" + "25" + "\t" + "mm" + "\t" + "30.00" + "\t" + "1" + "\t" + "1");
                file.WriteLine("Frame#" + "\t" + "Time" + "\t" +
                    "HipCenter" + "\t" + "\t" + "\t" +
                    "SpineMid" + "\t" + "\t" + "\t" +
                    "Neck" + "\t" + "\t" + "\t" +
                    "Head" + "\t" + "\t" + "\t" +
                    "L.Shoulder" + "\t" + "\t" + "\t" +
                    "L.Elbow" + "\t" + "\t" + "\t" +
                    "L.Wrist" + "\t" + "\t" + "\t" +
                    "L.Hand" + "\t" + "\t" + "\t" +
                    "R.Shoulder" + "\t" + "\t" + "\t" +
                    "R.Elbow" + "\t" + "\t" + "\t" +
                    "R.Wrist" + "\t" + "\t" + "\t" +
                    "R.Hand" + "\t" + "\t" + "\t" +
                    "L.Hip" + "\t" + "\t" + "\t" +
                    "L.Knee" + "\t" + "\t" + "\t" +
                    "L.Ankle" + "\t" + "\t" + "\t" +
                    "L.Foot" + "\t" + "\t" + "\t" +
                    "R.Hip" + "\t" + "\t" + "\t" +
                    "R.Knee" + "\t" + "\t" + "\t" +
                    "R.Ankle" + "\t" + "\t" + "\t" +
                    "R.Foot" + "\t" + "\t" + "\t" +
                    "ShoulderMid" + "\t" + "\t" + "\t" +
                    "L.HandTip" + "\t" + "\t" + "\t" +
                    "L.Thumb" + "\t" + "\t" + "\t" +
                    "R.HandTip" + "\t" + "\t" + "\t" +
                    "R.Thumb");
                file.Write("\t" + "\t");
                for (int i = 1; i <= 25; i++)
                {
                    file.Write($"X{i}\tY{i}\tZ{i}\t");
                }
                file.WriteLine();
                file.WriteLine();

                // write the coordinates
                int number = 0;
                foreach (double[] arr in Data)
                {
                    file.Write((number + 1).ToString() + "\t" + Time[number] + "\t");
                    for (int i = 0; i < arr.Length; i++)
                    {
                        arr[i] = Math.Round(arr[i], 4, MidpointRounding.ToEven);
                    }
                    string line = string.Join("\t", arr);
                    file.WriteLine(line);
                    number++;
                }
            }
        }

        private void WriteMOT() // create the MOT file
        {
            System.IO.File.WriteAllText(fileMOT, ""); // overwrite the file
            // write header in the file
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(fileMOT, true))
            {
                file.WriteLine("kinectAngles");
                file.WriteLine("version = 1");
                file.WriteLine("nRows=" + counter.ToString());
                file.WriteLine("nColumns=22");
                file.WriteLine("inDegrees = yes");
                file.WriteLine("endheader");
                file.WriteLine();
                file.WriteLine("Time" + "\t" +
                    "pelvis_tx" + "\t" +
                    "pelvis_ty" + "\t" +
                    "pelvis_tz" + "\t" +
                    "hip_adduction_l" + "\t" +
                    "hip_rotation_l" + "\t" +
                    "hip_flexion_l" + "\t" +
                    "hip_adduction_r" + "\t" +
                    "hip_rotation_r" + "\t" +
                    "hip_flexion_r" + "\t" +
                    "knee_angle_l" + "\t" +
                    "ankle_angle_l" + "\t" +
                    "elbow_flexion_l" + "\t" +
                    "knee_angle_r" + "\t" +
                    "ankle_angle_r" + "\t" +
                    "elbow_flexion_r" + "\t" +
                    "arm_flexion_l" + "\t" +
                    "arm_adduction_l" + "\t" +
                    "arm_flexion_r" + "\t" +
                    "arm_adduction_r" + "\t" +
                    "pelvis_list" + "\t" +
                    "pelvis_tilt" + "\t" +
                    "pelvis_rotation");
                file.WriteLine();

                // write the angles
                int number = 0;
                foreach (double[] arr in Data)
                {
                    var ang = new CalculateAngles(arr);
                    file.Write(Time[number] + "\t");
                    for (int i = 0; i < ang.angleList.Length; i++)
                    {
                        ang.angleList[i] = Math.Round(ang.angleList[i], 4, MidpointRounding.ToEven);
                    }
                    string line = string.Join("\t", ang.angleList);
                    file.WriteLine(line);
                    number++;
                }
            }
        }

        private void DrawStartArea(Body body, JointType jointType)
        {
            Joint spineMid = body.Joints[JointType.SpineMid];
            CameraSpacePoint startArea = spineMid.Position;
            Point point2 = new Point();
            ColorSpacePoint colorPoint = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(startArea);
            point2.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
            point2.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;
            // draw start
            Ellipse ellipse = new Ellipse
            {
                Stroke = Brushes.Lime,
                StrokeThickness = 5,
                Width = 100,
                Height = 100,
            };
            Canvas.SetLeft(ellipse, (point2.X) - ellipse.Width / 2);
            Canvas.SetTop(ellipse, (point2.Y) - ellipse.Height / 2);
            canvas.Children.Add(ellipse);
        }

        private void DrawStopArea(Body body, JointType jointType)
        {
            Joint headjoint = body.Joints[JointType.SpineShoulder];
            CameraSpacePoint jointPosition = headjoint.Position;
            Point point2 = new Point();
            ColorSpacePoint colorPoint = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(jointPosition);
            point2.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
            point2.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;
            // draw stop
            Ellipse ellipse = new Ellipse
            {
                Stroke = Brushes.Lime,
                StrokeThickness = 5,
                Width = 100,
                Height = 100,
            };
            Canvas.SetLeft(ellipse, (point2.X) - ellipse.Width / 2);
            Canvas.SetTop(ellipse, (point2.Y) - ellipse.Height / 2);
            canvas.Children.Add(ellipse);
        }

        private bool IsHandOverStart(Body body)
        {
            if (drawstart == false)
            {
                return false;
            }
            // start on SpineMid
            double goalX = body.Joints[JointType.SpineMid].Position.X;
            double goalY = body.Joints[JointType.SpineMid].Position.Y;

            double handX = body.Joints[JointType.HandRight].Position.X;
            double handY = body.Joints[JointType.HandRight].Position.Y;

            handX = Math.Round(handX, 1, MidpointRounding.ToEven);
            goalX = Math.Round(goalX, 1, MidpointRounding.ToEven);

            handY = Math.Round(handY, 1, MidpointRounding.ToEven);
            goalY = Math.Round(goalY, 1, MidpointRounding.ToEven);

            bool isOverStart = handX == goalX && handY == goalY;
            return isOverStart;
        }

        private bool IsHandOverStop(Body body)
        {
            // stop on SpineShoulder
            double goalX = body.Joints[JointType.SpineShoulder].Position.X;
            double goalY = body.Joints[JointType.SpineShoulder].Position.Y;

            double handX = body.Joints[JointType.HandRight].Position.X;
            double handY = body.Joints[JointType.HandRight].Position.Y;

            handX = Math.Round(handX, 1, MidpointRounding.ToEven);
            goalX = Math.Round(goalX, 1, MidpointRounding.ToEven);

            handY = Math.Round(handY, 1, MidpointRounding.ToEven);
            goalY = Math.Round(goalY, 1, MidpointRounding.ToEven);

            bool isOverStart = handX == goalX && handY == goalY;
            return isOverStart;
        }
    }
}
