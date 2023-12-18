using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading.Tasks;
using System.Numerics;

namespace Motion
{
    public class CalculateAngles
    {
        public static double[] values { get; set; }
        public double[] angleList = new double[22];

        #region Coordinates

        //1
        float SpineBaseX;
        float SpineBaseY;
        float SpineBaseZ;
        //2
        float SpineMidX;
        float SpineMidY;
        float SpineMidZ;
        //3 
        float NeckX;
        float NeckY;
        float NeckZ;
        //4
        float HeadX;
        float HeadY;
        float HeadZ;
        //5
        float ShoulderLeftX;
        float ShoulderLeftY;
        float ShoulderLeftZ;
        //6
        float ElbowLeftX;
        float ElbowLeftY;
        float ElbowLeftZ;
        //7
        float WristLeftX;
        float WristLeftY;
        float WristLeftZ;
        //8
        float HandLeftX;
        float HandLeftY;
        float HandLeftZ;
        //9
        float ShoulderRightX;
        float ShoulderRightY;
        float ShoulderRightZ;
        //10
        float ElbowRightX;
        float ElbowRightY;
        float ElbowRightZ;
        //11
        float WristRightX;
        float WristRightY;
        float WristRightZ;
        //12
        float HandRightX;
        float HandRightY;
        float HandRightZ;
        //13
        float HipLeftX;
        float HipLeftY;
        float HipLeftZ;
        //14
        float KneeLeftX;
        float KneeLeftY;
        float KneeLeftZ;
        //15
        float AnkleLeftX;
        float AnkleLeftY;
        float AnkleLeftZ;
        //16
        float FootLeftX;
        float FootLeftY;
        float FootLeftZ;
        //17
        float HipRightX;
        float HipRightY;
        float HipRightZ;
        //18
        float KneeRightX;
        float KneeRightY;
        float KneeRightZ;
        //19
        float AnkleRightX;
        float AnkleRightY;
        float AnkleRightZ;
        //20
        float FootRightX;
        float FootRightY;
        float FootRightZ;
        //21
        float SpineShoulderX;
        float SpineShoulderY;
        float SpineShoulderZ;
        //22
        float HandTipLeftX;
        float HandTipLeftY;
        float HandTipLeftZ;
        //23
        float ThumbLeftX;
        float ThumbLeftY;
        float ThumbLeftZ;
        //24
        float HandTipRightX;
        float HandTipRightY;
        float HandTipRightZ;
        //25
        float ThumbRightX;
        float ThumbRightY;
        float ThumbRightZ;

        #endregion

        public CalculateAngles(double[] values)
        {
            #region retrieve the coordinates

            // 1
            SpineBaseX = Convert.ToSingle(values[0]);
            SpineBaseY = Convert.ToSingle(values[1]);
            SpineBaseZ = Convert.ToSingle(values[2]);
            // 2
            SpineMidX = Convert.ToSingle(values[3]);
            SpineMidY = Convert.ToSingle(values[4]);
            SpineMidZ = Convert.ToSingle(values[5]);
            // 3
            NeckX = Convert.ToSingle(values[6]);
            NeckY = Convert.ToSingle(values[7]);
            NeckZ = Convert.ToSingle(values[8]);
            // 4
            HeadX = Convert.ToSingle(values[9]);
            HeadY = Convert.ToSingle(values[10]);
            HeadZ = Convert.ToSingle(values[11]);
            // 5
            ShoulderLeftX = Convert.ToSingle(values[12]);
            ShoulderLeftY = Convert.ToSingle(values[13]);
            ShoulderLeftZ = Convert.ToSingle(values[14]);
            // 6
            ElbowLeftX = Convert.ToSingle(values[15]);
            ElbowLeftY = Convert.ToSingle(values[16]);
            ElbowLeftZ = Convert.ToSingle(values[17]);
            // 7
            WristLeftX = Convert.ToSingle(values[18]);
            WristLeftY = Convert.ToSingle(values[19]);
            WristLeftZ = Convert.ToSingle(values[20]);
            // 8
            HandLeftX = Convert.ToSingle(values[21]);
            HandLeftY = Convert.ToSingle(values[22]);
            HandLeftZ = Convert.ToSingle(values[23]);
            // 9
            ShoulderRightX = Convert.ToSingle(values[24]);
            ShoulderRightY = Convert.ToSingle(values[25]);
            ShoulderRightZ = Convert.ToSingle(values[26]);
            // 10
            ElbowRightX = Convert.ToSingle(values[27]);
            ElbowRightY = Convert.ToSingle(values[28]);
            ElbowRightZ = Convert.ToSingle(values[29]);
            // 11
            WristRightX = Convert.ToSingle(values[30]);
            WristRightY = Convert.ToSingle(values[31]);
            WristRightZ = Convert.ToSingle(values[32]);
            // 12
            HandRightX = Convert.ToSingle(values[33]);
            HandRightY = Convert.ToSingle(values[34]);
            HandRightZ = Convert.ToSingle(values[35]);
            // 13
            HipLeftX = Convert.ToSingle(values[36]);
            HipLeftY = Convert.ToSingle(values[37]);
            HipLeftZ = Convert.ToSingle(values[38]);
            // 14
            KneeLeftX = Convert.ToSingle(values[39]);
            KneeLeftY = Convert.ToSingle(values[40]);
            KneeLeftZ = Convert.ToSingle(values[41]);
            // 15
            AnkleLeftX = Convert.ToSingle(values[42]);
            AnkleLeftY = Convert.ToSingle(values[43]);
            AnkleLeftZ = Convert.ToSingle(values[44]);
            // 16
            FootLeftX = Convert.ToSingle(values[45]);
            FootLeftY = Convert.ToSingle(values[46]);
            FootLeftZ = Convert.ToSingle(values[47]);
            // 17
            HipRightX = Convert.ToSingle(values[48]);
            HipRightY = Convert.ToSingle(values[49]);
            HipRightZ = Convert.ToSingle(values[50]);
            // 18
            KneeRightX = Convert.ToSingle(values[51]);
            KneeRightY = Convert.ToSingle(values[52]);
            KneeRightZ = Convert.ToSingle(values[53]);
            // 19
            AnkleRightX = Convert.ToSingle(values[54]);
            AnkleRightY = Convert.ToSingle(values[55]);
            AnkleRightZ = Convert.ToSingle(values[56]);
            // 20
            FootRightX = Convert.ToSingle(values[57]);
            FootRightY = Convert.ToSingle(values[58]);
            FootRightZ = Convert.ToSingle(values[59]);
            // 21
            SpineShoulderX = Convert.ToSingle(values[60]);
            SpineShoulderY = Convert.ToSingle(values[61]);
            SpineShoulderZ = Convert.ToSingle(values[62]);
            // 22
            HandTipLeftX = Convert.ToSingle(values[63]);
            HandTipLeftY = Convert.ToSingle(values[64]);
            HandTipLeftZ = Convert.ToSingle(values[65]);
            // 23
            ThumbLeftX = Convert.ToSingle(values[66]);
            ThumbLeftY = Convert.ToSingle(values[67]);
            ThumbLeftZ = Convert.ToSingle(values[68]);
            // 24
            HandTipRightX = Convert.ToSingle(values[69]);
            HandTipRightY = Convert.ToSingle(values[70]);
            HandTipRightZ = Convert.ToSingle(values[71]);
            // 25
            ThumbRightX = Convert.ToSingle(values[72]);
            ThumbRightY = Convert.ToSingle(values[73]);
            ThumbRightZ = Convert.ToSingle(values[74]);

            #endregion

            #region calculate the angles

            angleList[0] = SpineBaseX;
            angleList[1] = SpineBaseY;
            angleList[2] = SpineBaseZ;

            angleList[3] = calcul_hip_adduction_l(SpineBaseX, SpineBaseY, SpineBaseZ, HipLeftX, HipLeftY, HipLeftZ);
            angleList[4] = calcul_hip_rotation_l(SpineBaseX, SpineBaseY, SpineBaseZ, HipLeftX, HipLeftY, HipLeftZ);
            angleList[5] = calcul_hip_flexion_l(SpineBaseX, SpineBaseY, SpineBaseZ, HipLeftX, HipLeftY, HipLeftZ);

            angleList[6] = calcul_hip_adduction_r(SpineBaseX, SpineBaseY, SpineBaseZ, HipRightX, HipRightY, HipRightZ);
            angleList[7] = calcul_hip_rotation_r(SpineBaseX, SpineBaseY, SpineBaseZ, HipRightX, HipRightY, HipRightZ);
            angleList[8] = calcul_hip_flexion_r(SpineBaseX, SpineBaseY, SpineBaseZ, HipRightX, HipRightY, HipRightZ);

            angleList[9] = calcul_knee_angle_l(HipLeftX, HipLeftY, HipLeftZ, KneeLeftX, KneeLeftY, KneeLeftZ, AnkleLeftX, AnkleLeftY, AnkleLeftZ);
            angleList[10] = calcul_ankle_angle_l(KneeLeftX, KneeLeftY, KneeLeftZ, AnkleLeftX, AnkleLeftY, AnkleLeftZ, FootLeftX, FootLeftY, FootLeftZ);
            angleList[11] = calcul_elbow_flex_l(ShoulderLeftX, ShoulderLeftY, ShoulderLeftZ, ElbowLeftX, ElbowLeftY, ElbowLeftZ, HandLeftX, HandLeftY, HandLeftZ);

            angleList[12] = calcul_knee_angle_r(HipRightX, HipRightY, HipRightZ, KneeRightX, KneeRightY, KneeRightZ, AnkleRightX, AnkleRightY, AnkleRightZ);
            angleList[13] = calcul_ankle_angle_r(KneeRightX, KneeRightY, KneeRightZ, AnkleRightX, AnkleRightY, AnkleRightZ, FootRightX, FootRightY, FootRightZ);
            angleList[14] = calcul_elbow_flex_r(ShoulderRightX, ShoulderRightY, ShoulderRightZ, ElbowRightX, ElbowRightY, ElbowRightZ, HandRightX, HandRightY, HandRightZ);

            angleList[15] = calcul_arm_flex_l(ShoulderLeftX, ShoulderLeftY, ShoulderLeftZ, HipLeftX, HipLeftY, HipLeftZ, ElbowLeftX);
            angleList[16] = calcul_arm_add_l(ShoulderLeftX, ShoulderLeftY, ShoulderLeftZ, HipLeftX, HipLeftY, HipLeftZ, ElbowLeftZ);
            angleList[17] = calcul_arm_flex_r(ShoulderRightX, ShoulderRightY, ShoulderRightZ, HipRightX, HipRightY, HipRightZ, ElbowRightX);
            angleList[18] = calcul_arm_add_r(ShoulderRightX, ShoulderRightY, ShoulderRightZ, HipRightX, HipRightY, HipRightZ, ElbowRightZ);

            angleList[19] = calcul_pelvis_list(SpineBaseX, SpineBaseY, SpineBaseZ, SpineShoulderX, SpineShoulderY, SpineShoulderZ);
            angleList[20] = calcul_pelvis_tilt(SpineBaseX, SpineBaseY, SpineBaseZ, SpineShoulderX, SpineShoulderY, SpineShoulderZ);
            angleList[21] = calcul_pelvis_rot(SpineBaseX, SpineBaseY, SpineBaseZ, SpineShoulderX, SpineShoulderY, SpineShoulderZ);

            #endregion
        }

        public static double Angle(Vector3 vector1, Vector3 vector2) // take two vectors and calculate the angle in degree between them
        {
            float dotProduct = Vector3.Dot(vector1, vector2);
            float magnitude1 = vector1.Length();
            float magnitude2 = vector2.Length();
            float cosAngle = dotProduct / (magnitude1 * magnitude2);
            float angleRadians = (float)Math.Acos(cosAngle);
            float angleDegrees = angleRadians * 180 / (float)Math.PI;
            return Convert.ToDouble(angleDegrees);
        }

        // 3 hip_adduction_l
        public static double calcul_hip_adduction_l(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float HipLeftX, float HipLeftY, float HipLeftZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(HipLeftX, HipLeftY, HipLeftZ);
            Vector3 point3 = new Vector3(SpineBaseX, HipLeftY, SpineBaseZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 4 hip_rotation_l
        public static double calcul_hip_rotation_l(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float HipLeftX, float HipLeftY, float HipLeftZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(HipLeftX, HipLeftY, HipLeftZ);
            Vector3 point3 = new Vector3(HipLeftX, SpineBaseY, SpineBaseZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 5 hip_flexion_l
        public static double calcul_hip_flexion_l(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float HipLeftX, float HipLeftY, float HipLeftZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(HipLeftX, HipLeftY, HipLeftZ);
            Vector3 point3 = new Vector3(HipLeftX, HipLeftY, SpineBaseZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 6 hip_adduction_r
        public static double calcul_hip_adduction_r(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float HipRightX, float HipRightY, float HipRightZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(HipRightX, HipRightY, HipRightZ);
            Vector3 point3 = new Vector3(SpineBaseX, HipRightY, SpineBaseZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 7 hip_rotation_r
        public static double calcul_hip_rotation_r(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float HipRightX, float HipRightY, float HipRightZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(HipRightX, HipRightY, HipRightZ);
            Vector3 point3 = new Vector3(HipRightX, SpineBaseY, SpineBaseZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 8 hip_flexion_r
        public static double calcul_hip_flexion_r(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float HipRightX, float HipRightY, float HipRightZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(HipRightX, HipRightY, HipRightZ);
            Vector3 point3 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point4 = new Vector3(HipRightX, HipRightY, SpineBaseZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point4 - point3;

            return Angle(vector1, vector2);
        }

        // 9 knee_angle_l
        public static double calcul_knee_angle_l(float HipLeftX, float HipLeftY, float HipLeftZ, float KneeLeftX, float KneeLeftY, float KneeLeftZ, float AnkleLeftX, float AnkleLeftY, float AnkleLeftZ)
        {
            // define points
            Vector3 point1 = new Vector3(HipLeftX, HipLeftY, HipLeftZ);
            Vector3 point2 = new Vector3(KneeLeftX, KneeLeftY, KneeLeftZ);
            Vector3 point3 = new Vector3(AnkleLeftX, AnkleLeftY, AnkleLeftZ);

            // create vectors from the points
            Vector3 vector1 = point1 - point2;
            Vector3 vector2 = point3 - point2;

            return Angle(vector1, vector2);
        }

        // 10 ankle_angle_l
        public static double calcul_ankle_angle_l(float KneeLeftX, float KneeLeftY, float KneeLeftZ, float AnkleLeftX, float AnkleLeftY, float AnkleLeftZ, float FootLeftX, float FootLeftY, float FootLeftZ)
        {
            // define points
            Vector3 point1 = new Vector3(KneeLeftX, KneeLeftY, KneeLeftZ);
            Vector3 point2 = new Vector3(AnkleLeftX, AnkleLeftY, AnkleLeftZ);
            Vector3 point3 = new Vector3(FootLeftX, FootLeftY, FootLeftZ);

            // create vectors from the points
            Vector3 vector1 = point1 - point2;
            Vector3 vector2 = point3 - point2;

            return Angle(vector1, vector2);
        }

        // 11 elbow_flex_l
        public static double calcul_elbow_flex_l(float ShoulderLeftX, float ShoulderLeftY, float ShoulderLeftZ, float ElbowLeftX, float ElbowLeftY, float ElbowLeftZ, float HandLeftX, float HandLeftY, float HandLeftZ)
        {
            // define points
            Vector3 point1 = new Vector3(ShoulderLeftX, ShoulderLeftY, ShoulderLeftZ);
            Vector3 point2 = new Vector3(ElbowLeftX, ElbowLeftY, ElbowLeftZ);
            Vector3 point3 = new Vector3(HandLeftX, HandLeftY, HandLeftZ);

            // create vectors from the points
            Vector3 vector1 = point1 - point2;
            Vector3 vector2 = point3 - point2;

            return Angle(vector1, vector2);
        }

        // 12 knee_angle_r
        public static double calcul_knee_angle_r(float HipRightX, float HipRightY, float HipRightZ, float KneeRightX, float KneeRightY, float KneeRightZ, float AnkleRightX, float AnkleRightY, float AnkleRightZ)
        {
            // define points
            Vector3 point1 = new Vector3(HipRightX, HipRightY, HipRightZ);
            Vector3 point2 = new Vector3(KneeRightX, KneeRightY, KneeRightZ);
            Vector3 point3 = new Vector3(AnkleRightX, AnkleRightY, AnkleRightZ);

            // create vectors from the points
            Vector3 vector1 = point1 - point2;
            Vector3 vector2 = point3 - point2;

            return Angle(vector1, vector2);
        }

        // 13 ankle_angle_r
        public static double calcul_ankle_angle_r(float KneeRightX, float KneeRightY, float KneeRightZ, float AnkleRightX, float AnkleRightY, float AnkleRightZ, float FootRightX, float FootRightY, float FootRightZ)
        {
            // define points
            Vector3 point1 = new Vector3(KneeRightX, KneeRightY, KneeRightZ);
            Vector3 point2 = new Vector3(AnkleRightX, AnkleRightY, AnkleRightZ);
            Vector3 point3 = new Vector3(FootRightX, FootRightY, FootRightZ);

            // create vectors from the points
            Vector3 vector1 = point1 - point2;
            Vector3 vector2 = point3 - point2;

            return Angle(vector1, vector2);
        }

        // 14 elbow_flex_r
        public static double calcul_elbow_flex_r(float ShoulderRightX, float ShoulderRightY, float ShoulderRightZ, float ElbowRightX, float ElbowRightY, float ElbowRightZ, float HandRightX, float HandRightY, float HandRightZ)
        {
            // define points
            Vector3 point1 = new Vector3(ShoulderRightX, ShoulderRightY, ShoulderRightZ);
            Vector3 point2 = new Vector3(ElbowRightX, ElbowRightY, ElbowRightZ);
            Vector3 point3 = new Vector3(HandRightX, HandRightY, HandRightZ);

            // create vectors from the points
            Vector3 vector1 = point1 - point2;
            Vector3 vector2 = point3 - point2;

            return Angle(vector1, vector2);
        }

        // 15 arm_flex_l
        public static double calcul_arm_flex_l(float ShoulderLeftX, float ShoulderLeftY, float ShoulderLeftZ, float HipLeftX, float HipLeftY, float HipLeftZ, float ElbowLeftX)
        {
            // define points
            Vector3 point1 = new Vector3(ShoulderLeftX, ShoulderLeftY, ShoulderLeftZ);
            Vector3 point2 = new Vector3(HipLeftX, HipLeftY, HipLeftZ);
            Vector3 point3 = new Vector3(ElbowLeftX, HipLeftY, HipLeftZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 16 arm_add_l
        public static double calcul_arm_add_l(float ShoulderLeftX, float ShoulderLeftY, float ShoulderLeftZ, float HipLeftX, float HipLeftY, float HipLeftZ, float ElbowLeftZ)
        {
            // define points
            Vector3 point1 = new Vector3(ShoulderLeftX, ShoulderLeftY, ShoulderLeftZ);
            Vector3 point2 = new Vector3(HipLeftX, HipLeftY, HipLeftZ);
            Vector3 point3 = new Vector3(HipLeftX, HipLeftY, ElbowLeftZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 17 arm_flex_r
        public static double calcul_arm_flex_r(float ShoulderRightX, float ShoulderRightY, float ShoulderRightZ, float HipRightX, float HipRightY, float HipRightZ, float ElbowRightX)
        {
            // define points
            Vector3 point1 = new Vector3(ShoulderRightX, ShoulderRightY, ShoulderRightZ);
            Vector3 point2 = new Vector3(HipRightX, HipRightY, HipRightZ);
            Vector3 point3 = new Vector3(ElbowRightX, HipRightY, HipRightZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 18 arm_add_r
        public static double calcul_arm_add_r(float ShoulderRightX, float ShoulderRightY, float ShoulderRightZ, float HipRightX, float HipRightY, float HipRightZ, float ElbowRightZ)
        {
            // define points
            Vector3 point1 = new Vector3(ShoulderRightX, ShoulderRightY, ShoulderRightZ);
            Vector3 point2 = new Vector3(HipRightX, HipRightY, HipRightZ);
            Vector3 point3 = new Vector3(HipRightX, HipRightY, ElbowRightZ);

            // create vectors from the points
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 19 pelvis_list
        public static double calcul_pelvis_list(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float SpineShoulderX, float SpineShoulderY, float SpineShoulderZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(SpineShoulderX, SpineShoulderY, SpineShoulderZ);
            Vector3 point3 = new Vector3(SpineBaseX, SpineShoulderY, SpineShoulderZ);

            // create vectors from the points  
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 20 pelvis_tilt
        public static double calcul_pelvis_tilt(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float SpineShoulderX, float SpineShoulderY, float SpineShoulderZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(SpineShoulderX, SpineShoulderY, SpineShoulderZ);
            Vector3 point3 = new Vector3(SpineShoulderX, SpineShoulderY, SpineBaseZ);

            // create vectors from the points  
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }

        // 21 pelvis_rot
        public static double calcul_pelvis_rot(float SpineBaseX, float SpineBaseY, float SpineBaseZ, float SpineShoulderX, float SpineShoulderY, float SpineShoulderZ)
        {
            // define points
            Vector3 point1 = new Vector3(SpineBaseX, SpineBaseY, SpineBaseZ);
            Vector3 point2 = new Vector3(SpineShoulderX, SpineShoulderY, SpineShoulderZ);
            Vector3 point3 = new Vector3(SpineShoulderX, SpineBaseY, SpineShoulderZ);

            // create vectors from the points  
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point1;

            return Angle(vector1, vector2);
        }
    }
}