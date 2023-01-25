package frc.robot.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class MathThings {
    public static double absMax(double input, double max){ //This probably exists somewhere else 😋
        max = Math.abs(max);
        if(Math.abs(input)>max){
            if(input>0) input = max;
            else        input = -max;
        }
        return input;
    }

    public static double angleDiffDeg(double ang1, double ang2){
        //if(ang1>0 != ang2>0){
            if(Math.abs(ang1)+Math.abs(ang2)>180){
                double absAngDiff = Math.abs(180-ang1)+Math.abs(180-ang2);
                if(ang1<ang2)
                    return -absAngDiff;
                else
                    return absAngDiff;
            }
      //  }
return ang1-ang2;
    }

}
