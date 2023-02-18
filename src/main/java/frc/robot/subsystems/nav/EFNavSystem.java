package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.FieldObjects.FieldLine;
import frc.robot.lib.LineIntersection;
import frc.robot.lib.MB_Math;
import org.opencv.core.Scalar;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;


import static frc.robot.Constants.EFPathingConstants.*;


public class EFNavSystem extends SubsystemBase {
Point2D.Double pivotPoint = new Point2D.Double(0,0);
EFPathingTelemetrySub telemetrySub;
Pose2d efPose = new Pose2d();

    ArrayList<FieldLine> fieldLines = new ArrayList<FieldLine>();
    ArrayList<Pose2d> cornerPoints = new ArrayList<Pose2d>();
    Thread pathingThread;
    boolean poseChanged = false;
    Pose2d desiredPose = new Pose2d(0.6,0.075,Rotation2d.fromRotations(0));
    ArrayList<Pose2d> navPoses = new ArrayList<>();

    public EFNavSystem(EFPathingTelemetrySub telemetrySub){
        this.telemetrySub = telemetrySub;
        createBarriers();
        createCornerPoints();
        telemetrySub.updateBarriers(fieldLines);
        createAndStartPathingThread();
        SmartDashboard.putNumber("EFPivotX",0);
        SmartDashboard.putNumber("EFPivotY",0.3);
        SmartDashboard.putNumber("EFDesiredX",0.6);
        SmartDashboard.putNumber("EFDesiredY",0.075);
    }
    private void createAndStartPathingThread(){
        pathingThread =
                new Thread(
                        () -> {
                            try {
                                while(!pathingThread.isInterrupted()){
                                    double startTime = Timer.getFPGATimestamp()*1000;
                                    //    if(Timer.getFPGATimestamp()-engageTime> 10)
                                    //        disengageNav(); //disengage if engaged for >10s

                                    updateNavPoses();
                                    
                                    double compTime = Timer.getFPGATimestamp()*1000 - startTime;
                                    SmartDashboard.putNumber("EFpathingCompTime",compTime);
                                    telemetrySub.updateNavPoses(navPoses);

                                    Thread.sleep(minPathingDelay);
                                }
                            } catch (InterruptedException e) {throw new RuntimeException(e);}
                        });
        pathingThread.setDaemon(true);
        pathingThread.setName("MB_EF_Pathing");
        pathingThread.setPriority(1); //low priority I hope?
        pathingThread.start();
    }

    Pose2d lastUsedPose = new Pose2d();
    boolean engaged = false;
    public void updateNavPoses(){
        boolean poseChangedOld = poseChanged;
        poseChanged = false;
        if(desiredPose == null)
            return;
        Pose2d[] outNavPoses = findNavPoses(efPose,desiredPose,0,Timer.getFPGATimestamp(),5);
        if(outNavPoses.length<1)
            return;
        boolean poseCloseToLast = MB_Math.poseDist(lastUsedPose,efPose)<Constants.PathingConstants.recalcThreshold;

        if(getPathLengthFromArm(outNavPoses)> getPathLengthFromArm(navPoses) && ((engaged || poseCloseToLast ) && !poseChangedOld)){
            //    SmartDashboard.putNumber("lastEngagedDrop",Timer.getFPGATimestamp());
            return;
        }


        lastUsedPose = efPose;
        navPoses.clear();
        for(Pose2d  i : outNavPoses)
            navPoses.add(i);
        telemetrySub.updateDestinationPose(this.desiredPose);
        //    pTelemetrySub.updateNavPoses(navPoses);
    }

    private double getPathLengthFromArm(Pose2d[] path){ //getPathLengthFromBot rename
        if(path.length==0 || path[0] == null)
            return Integer.MAX_VALUE;
        double botX = efPose.getX();
        double botY = efPose.getY();
        double length = Math.sqrt(Math.pow(botX - path[0].getX(),2)+Math.pow(botY-path[0].getY(),2));
        for(int i = 1; i<path.length; i++)
            length+=Math.sqrt(Math.pow(path[i].getX() - path[i-1].getX(),2)+Math.pow(path[i].getY() - path[i-1].getY(),2));

        return length;
    }

    private  double getPathLengthFromArm(ArrayList<Pose2d> path){
        Pose2d[] newPath = new Pose2d[path.size()];
        for(int i = 0; i<Math.min(path.size(),newPath.length); i++)
            newPath[i] = path.get(i);
        return getPathLengthFromArm(newPath);
    }

    public void periodic(){
        updatePivotPoint(new Point2D.Double(SmartDashboard.getNumber("EFPivotX",0),SmartDashboard.getNumber("EFPivotY",0)));
        desiredPose = new Pose2d(SmartDashboard.getNumber("EFDesiredX",0.6),SmartDashboard.getNumber("EFDesiredY",0.075),Rotation2d.fromDegrees(0));
        telemetrySub.updateDestinationPose(desiredPose);
    }

    private void createBarriers(){
        fieldLines.clear();
        fieldLines.add(new FieldLine(new Line2D.Double(-15,0,15,0),true,new Scalar(100,100,100))); //ground
        fieldLines.add(new FieldLine(new Line2D.Double(-BUMPER_X_FROM_ORIGIN,BUMPER_Y_FROM_ORIGIN,BUMPER_X_FROM_ORIGIN,BUMPER_Y_FROM_ORIGIN))); //bumper top
        fieldLines.add(new FieldLine(new Line2D.Double(-BUMPER_X_FROM_ORIGIN,BUMPER_Y_FROM_ORIGIN,-BUMPER_X_FROM_ORIGIN,0))); //bumper left
        fieldLines.add(new FieldLine(new Line2D.Double(BUMPER_X_FROM_ORIGIN,BUMPER_Y_FROM_ORIGIN,BUMPER_X_FROM_ORIGIN,0))); //bumper right
        fieldLines.add(new FieldLine(new Line2D.Double(-BUMPER_X_FROM_ORIGIN,0,BUMPER_X_FROM_ORIGIN,0))); //bumper bottom
    }


    public void updatePivotPoint(Point2D.Double pivotPoint){
        this.pivotPoint = pivotPoint;
        this.efPose = new Pose2d(pivotPoint.getX() + CENTER_OFFSET_FROM_PIVOT_POINT_X, pivotPoint.getY()+CENTER_OFFSET_FROM_PIVOT_POINT_Y, Rotation2d.fromDegrees(0));
        SmartDashboard.putString("yDebug","getY():"+pivotPoint.getY()+", OFFSETVAR:"+CENTER_OFFSET_FROM_PIVOT_POINT_Y);
        telemetrySub.updatePivotPoint(this.pivotPoint);
        telemetrySub.updateEFPose(this.efPose);
    }

    public  boolean barrierOnLine(Line2D.Double line){
        //shift the line to check from the center of the box. The pivot point to pivot point is what's being passed in
        line.setLine(line.getX1() + CENTER_OFFSET_FROM_PIVOT_POINT_X, line.getY1() + CENTER_OFFSET_FROM_PIVOT_POINT_Y, line.getX2() + CENTER_OFFSET_FROM_PIVOT_POINT_X, line.getY2() + CENTER_OFFSET_FROM_PIVOT_POINT_Y);

        double lineDir = Math.atan2(line.getY2() - line.getY1() , line.getX2() - line.getX1());
        double lineDist = Math.sqrt(Math.pow(line.getX1() - line.getX2(),2) + Math.pow(line.getY1() - line.getY2(),2));

        double edgePtX1 = line.getX1() + Math.cos(lineDir - Math.PI/2) * EF_RADIUS; //why do I need to multiply by 2??
        double edgePtY1 = line.getY1() + Math.sin(lineDir - Math.PI/2) * EF_RADIUS; //because your visualization was not to scale 😉

        double edgePtX2 = line.getX1() + Math.cos(lineDir + Math.PI/2) * EF_RADIUS;
        double edgePtY2 = line.getY1() + Math.sin(lineDir + Math.PI/2) * EF_RADIUS;

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


        boolean pathObstructed = false;
        for(int i = 0; i<innerLineTestCount; i++)
            for(FieldLine fieldLine : fieldLines){
                if(!fieldLine.isABarrier()) continue;
                LineIntersection.Point bp1 = new LineIntersection.Point(fieldLine.getLine().getX1(), fieldLine.getLine().getY1());
                LineIntersection.Point bp2 = new LineIntersection.Point(fieldLine.getLine().getX2(), fieldLine.getLine().getY2());

                // if(LineIntersection.doIntersect(bp1,bp2,l1p1,l1p2) || LineIntersection.doIntersect(bp1,bp2,l2p1,l2p2) || LineIntersection.doIntersect(bp1,bp2,l3p1,l3p2)){
                //     pathClear = true;
                // }
                //         LineIntersection.Point linePoint1 = new
                double x1 = l2p1.x + (l3p1.x - l2p1.x)*((double)i/ innerLineTestCount);
                double y1 = l2p1.y + (l3p1.y - l2p1.y)*((double)i/ innerLineTestCount);

                double x2 = l2p2.x + (l3p2.x - l2p2.x)*((double)i/ innerLineTestCount);
                double y2 = l2p2.y + (l3p2.y - l2p2.y)*((double)i/ innerLineTestCount);

                LineIntersection.Point lp1New = new LineIntersection.Point(x1,y1);
                LineIntersection.Point lp2New = new LineIntersection.Point(x2,y2);

                if(LineIntersection.doIntersect(bp1,bp2,lp1New,lp2New)){
                    pathObstructed = true;
                }

            }

        return pathObstructed;

    }

    public Pose2d[] findNavPoses(Pose2d myPose, Pose2d desiredPose, int recursionDepth, double startTime, double timeout){
        if(Timer.getFPGATimestamp() - startTime >timeout)
            return new Pose2d[]{};
     //   myPose = new Pose2d(myPose.getX() + CENTER_OFFSET_FROM_PIVOT_POINT_X, myPose.getY() + CENTER_OFFSET_FROM_PIVOT_POINT_Y, myPose.getRotation());

        int[] randIndexes = MB_Math.randomIndexes(cornerPoints.size());
        if(!barrierOnLine(new Line2D.Double(myPose.getX(),myPose.getY(),desiredPose.getX(),desiredPose.getY())))
            return new Pose2d[] {myPose,desiredPose};
        if(recursionDepth>maxRecursionDepth)
            return new Pose2d[] {};

        for (double dist = lineDistIterator; dist < maxLineDist; dist += lineDistIterator)
            for (int angI = -cornerPoints.size(); angI < moveAngles; angI += 1) {

                double ang = angI * (Math.PI * 2 / (moveAngles));
                double branchHeadX = myPose.getX() + dist*Math.cos(ang);
                double branchHeadY = myPose.getY() + dist*Math.sin(ang);

                if(angI<0 && cornerPoints.size()>0 && randIndexes.length>0){ //iterate through every corner point before doing anything else - notice how angI starts at -size
                    //for indexes, use [cornerPoints.size()-ang]
                    int cornerIndex = angI + cornerPoints.size();
                    branchHeadX = cornerPoints.get(randIndexes[cornerIndex]).getX();
                    branchHeadY = cornerPoints.get(randIndexes[cornerIndex]).getY();

                }


                Line2D.Double lineToTestPoint = new Line2D.Double(myPose.getX(), myPose.getY(),branchHeadX ,branchHeadY );

                if(barrierOnLine(lineToTestPoint)) continue;

                Pose2d[] lowerLevelOut = findNavPoses(new Pose2d(branchHeadX,branchHeadY,desiredPose.getRotation()),desiredPose,recursionDepth+1,startTime,timeout);

                if(lowerLevelOut.length>0){
                    Pose2d[] myOut = new Pose2d[lowerLevelOut.length + 1];
                    myOut[0] = new Pose2d(branchHeadX,branchHeadY,desiredPose.getRotation());
                    for(int i = 1; i<myOut.length; i++)
                        myOut[i] = lowerLevelOut[i-1];
                    return myOut;
                }

            }



        //if you've gone too deep, original pose should be returned
        return new Pose2d[] {};
    }

    void createCornerPoints(){
        cornerPoints.clear();

        cornerPoints.add(new Pose2d(0.6,0.3,Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(1.5,1.5,Rotation2d.fromDegrees(0)));


        telemetrySub.updateCornerPoints(cornerPoints);
    }


}
