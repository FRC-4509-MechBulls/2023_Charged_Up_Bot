package frc.robot.lib;

public class MathThings {
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

}
