package frc.robot.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.PathingTelemetrySub;
import frc.robot.subsystems.SwerveSubsystem;

import java.awt.*;
import java.awt.geom.Line2D;
import java.util.ArrayList;

public class NavigationField extends SubsystemBase {

static ArrayList<Line2D.Double> barriers = new ArrayList<Line2D.Double>();
private static double desiredX = 2;
private static double desiredY = 0.3;


PathingTelemetrySub pTelemetrySub;
SwerveSubsystem swerveSubsystem;
public NavigationField(PathingTelemetrySub telemetrySub, SwerveSubsystem swerveSubsystem){
    this.pTelemetrySub = telemetrySub;
    this.swerveSubsystem = swerveSubsystem;

    barriers.add(new Line2D.Double(1,0.5,1,-0.5));
    this.pTelemetrySub.updateBarriers(barriers);
}

public void periodic(){
pTelemetrySub.updateRobotPose(swerveSubsystem.getEstimatedPosition());
}

public static boolean clearPathOnLine(Line2D.Double line){
    double lineDir = Math.atan2(line.getY2() - line.getY1() , line.getX2() - line.getX1());
    double lineDist = Math.sqrt(Math.pow(line.getX1() - line.getX2(),2) + Math.pow(line.getY1() - line.getY2(),2));

    double edgePtX1 = line.getX1() + Math.cos(lineDir - Math.PI) * Constants.PathingConstants.kRobotRadius;
    double edgePtY1 = line.getX1() + Math.sin(lineDir - Math.PI) * Constants.PathingConstants.kRobotRadius;

    double edgePtX2 = line.getX1() + Math.cos(lineDir + Math.PI) * Constants.PathingConstants.kRobotRadius;
    double edgePtY2 = line.getX1() + Math.sin(lineDir + Math.PI) * Constants.PathingConstants.kRobotRadius;

    double destEdgePtX1 = edgePtX1 + (line.getX2() - line.getX1());
    double destEdgePtY1 = edgePtY1 + (line.getY2() - line.getY1());

    double destEdgePtX2 = edgePtX2 + (line.getX2() - line.getX1());
    double destEdgePtY2 = edgePtY2 + (line.getY2() - line.getY1());

    LineIntersection.Point l1p1 = new LineIntersection.Point(line.getX1(), line.getY1());
    LineIntersection.Point l1p2 = new LineIntersection.Point(line.getX2(), line.getY2());

    LineIntersection.Point l2p1 = new LineIntersection.Point(edgePtX1, edgePtY1);
    LineIntersection.Point l2p2 = new LineIntersection.Point(destEdgePtX1, destEdgePtY1);

    LineIntersection.Point l3p1 = new LineIntersection.Point(edgePtX2, edgePtY2);
    LineIntersection.Point l3p2 = new LineIntersection.Point(destEdgePtX2, destEdgePtY2);


    boolean pathClear = true;
    for(Line2D barrier : barriers){
        LineIntersection.Point bp1 = new LineIntersection.Point(barrier.getX1(), barrier.getY1());
        LineIntersection.Point bp2 = new LineIntersection.Point(barrier.getX2(), barrier.getY2());

        if(LineIntersection.doIntersect(bp1,bp2,l1p1,l1p2) || LineIntersection.doIntersect(bp1,bp2,l2p1,l2p2) || LineIntersection.doIntersect(bp1,bp2,l3p1,l3p2)){
            pathClear = false;
        }
    }

    return pathClear;

}
    static public double getDesiredX(){return desiredX;}
    public static  double getDesiredY(){return desiredY;}
    public static double[] getDesiredXY(){return new double[]{getDesiredX(),getDesiredY()};}

}
