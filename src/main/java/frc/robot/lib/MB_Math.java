package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class MB_Math {
    public static double maxValueCutoff(double input, double max){ //This probably exists somewhere else 😋
        max = Math.abs(max);
        if(Math.abs(input)>max){
            if(input>0) input = max;
            else        input = -max;
        }
        return input;
    }

    public static double angleDiffDeg(double ang1, double ang2){
       if(Math.abs(ang1-ang2)>180){
           if(ang1>ang2)
               return -ang1+ang2+360; //(ang1+180)-(180-ang2) <- that's just wrong
           else
               return -ang1+ang2-360;
       }
       return ang2-ang1;
    }



    public static boolean isWithinRangeOf(double point, double setpoint, double tolerance) {
        return Math.abs(point - setpoint) < Math.abs(tolerance);
    }

        public static int[] randomIndexes ( int size){
            int[] out = new int[size];
            for (int i = 0; i < out.length; i++)
                out[i] = i;

            for (int i = 0; i < out.length; i++) {
                int rand1 = (int) (Math.random() * out.length);
                int rand2 = (int) (Math.random() * out.length);
                int held = out[rand1];
                out[rand1] = out[rand2];
                out[rand2] = held;
            }
            return out;
        }

        public static double poseDist (Pose2d pose1, Pose2d pose2){
            if(pose1 == null || pose2 == null ) return 0;
            return Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));

        }

        public static int indexWrap(int i, int max){ //keeps i within 0 to (max-1), wraps back to the beginning/end if gone too high/below zero
        if(max == 0) return 0;
        return (int)(i + Math.abs(i*(Math.floor(max))))%max;
        }

}
