package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.FieldObjects.FieldLine;
import frc.robot.lib.LineIntersection;
import frc.robot.lib.MB_Math;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;


import static frc.robot.Constants.EFPathingConstants.*;


public class EFNavSystem extends SubsystemBase {
Point2D.Double pivotPoint = new Point2D.Double(0,0.3);
EFPathingTelemetrySub telemetrySub;
Pose2d efPose = new Pose2d();

    Pose2d desiredPose = new Pose2d(0.6,0.075,Rotation2d.fromRotations(0));
    ArrayList<Pose2d> navPoses = new ArrayList<>();

    public EFNavSystem(EFPathingTelemetrySub telemetrySub){
        this.telemetrySub = telemetrySub;
    }


    Pose2d lastUsedPose = new Pose2d();
    boolean engaged = true;
    public void engageNav(){engaged = true;}
    public void disengageNav(){engaged = false;}


    private Pose2d pointToPose2D(Point2D.Double in){
        return new Pose2d(in.getX(),in.getY(),Rotation2d.fromDegrees(0));
    }


    public void periodic(){
        telemetrySub.updateDestinationPose(desiredPose);
    }


    public void updatePivotPoint(Point2D.Double pivotPoint){
        this.pivotPoint = pivotPoint;
        this.efPose = new Pose2d(pivotPoint.getX() + CENTER_OFFSET_FROM_PIVOT_POINT_X, pivotPoint.getY()+CENTER_OFFSET_FROM_PIVOT_POINT_Y, Rotation2d.fromDegrees(0));
       // SmartDashboard.putString("yDebug","getY():"+pivotPoint.getY()+", OFFSETVAR:"+CENTER_OFFSET_FROM_PIVOT_POINT_Y);
        telemetrySub.updatePivotPoint(this.pivotPoint);
        telemetrySub.updateEFPose(this.efPose);
    }
    public void updatePivotPoint(double[] pivotPoint){
        if(pivotPoint== null) return;
        updatePivotPoint(new Point2D.Double(pivotPoint[0],pivotPoint[1]));
    }


    public Pose2d getNextNavPoint(){
        return desiredPose;
    }
    public void updateDesiredPose(double[] desiredPose){
        this.desiredPose = new Pose2d(desiredPose[0],desiredPose[1],Rotation2d.fromDegrees(0));
    }
    public Pose2d getDesiredPose(){
        return desiredPose;
    }




}
