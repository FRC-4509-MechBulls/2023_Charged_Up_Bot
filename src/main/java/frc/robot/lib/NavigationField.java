package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.PathingTelemetrySub;
import frc.robot.subsystems.SwerveSubsystem;
import org.opencv.core.Scalar;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import static frc.robot.Constants.FieldConstants.*;


public class NavigationField extends SubsystemBase {

 ArrayList<FieldLine> fieldLines = new ArrayList<FieldLine>();
 ArrayList<Node> nodes = new ArrayList<Node>();
 ArrayList<Pose2d> setPoints = new ArrayList<Pose2d>();

 ArrayList<Pose2d> cornerPoints = new ArrayList<Pose2d>();


PathingTelemetrySub pTelemetrySub;
 private SwerveSubsystem swerveSubsystem;
 Thread pathingThread;
 FMSGetter fmsGetter;
public NavigationField(PathingTelemetrySub telemetrySub, SwerveSubsystem swerveSubsystem, FMSGetter fmsGetter){
    this.pTelemetrySub = telemetrySub;
    this.swerveSubsystem = swerveSubsystem;
    this.fmsGetter = fmsGetter;

    createAndStartPathingThread();
    createBarriers();

    this.pTelemetrySub.updateBarriers(fieldLines);
    this.pTelemetrySub.updateNodes(nodes);
    this.pTelemetrySub.updateSetPoints(setPoints);
}

boolean wasOnRedAlliance = true;
double lastAllianceCheck = Timer.getFPGATimestamp();
boolean queueNodeReset = true;
public void periodic(){
pTelemetrySub.updateRobotPose(swerveSubsystem.getEstimatedPosition());
Line2D.Double testLine = new Line2D.Double(SmartDashboard.getNumber("x1",0),SmartDashboard.getNumber("y1",0),SmartDashboard.getNumber("x2",0),SmartDashboard.getNumber("y2",0));
    SmartDashboard.putBoolean("barrierOnLine", barrierOnLine(testLine));
    if(Timer.getFPGATimestamp() - lastAllianceCheck>3){
        if(wasOnRedAlliance!= fmsGetter.isRedAlliance() || queueNodeReset)
        {
            resetNodes();
            placeCornerPoints();
            wasOnRedAlliance = fmsGetter.isRedAlliance();
            queueNodeReset = false;
        }
        lastAllianceCheck = Timer.getFPGATimestamp();
    }
    SmartDashboard.putBoolean("poseChanged",poseChanged);
}

public  boolean barrierOnLine(Line2D.Double line){
    double lineDir = Math.atan2(line.getY2() - line.getY1() , line.getX2() - line.getX1());
    double lineDist = Math.sqrt(Math.pow(line.getX1() - line.getX2(),2) + Math.pow(line.getY1() - line.getY2(),2));

    double edgePtX1 = line.getX1() + Math.cos(lineDir - Math.PI/2) * Constants.PathingConstants.kRobotRadius; //why do I need to multiply by 2??
    double edgePtY1 = line.getY1() + Math.sin(lineDir - Math.PI/2) * Constants.PathingConstants.kRobotRadius; //because your visualization was not to scale 😉

    double edgePtX2 = line.getX1() + Math.cos(lineDir + Math.PI/2) * Constants.PathingConstants.kRobotRadius;
    double edgePtY2 = line.getY1() + Math.sin(lineDir + Math.PI/2) * Constants.PathingConstants.kRobotRadius;

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
    for(int i = 0; i<Constants.PathingConstants.innerLineTestCount; i++)
        for(FieldLine fieldLine : fieldLines){
            if(!fieldLine.isABarrier()) continue;
            LineIntersection.Point bp1 = new LineIntersection.Point(fieldLine.getLine().getX1(), fieldLine.getLine().getY1());
            LineIntersection.Point bp2 = new LineIntersection.Point(fieldLine.getLine().getX2(), fieldLine.getLine().getY2());

           // if(LineIntersection.doIntersect(bp1,bp2,l1p1,l1p2) || LineIntersection.doIntersect(bp1,bp2,l2p1,l2p2) || LineIntersection.doIntersect(bp1,bp2,l3p1,l3p2)){
           //     pathClear = true;
           // }
            //         LineIntersection.Point linePoint1 = new
            double x1 = l2p1.x + (l3p1.x - l2p1.x)*((double)i/ Constants.PathingConstants.innerLineTestCount);
            double y1 = l2p1.y + (l3p1.y - l2p1.y)*((double)i/ Constants.PathingConstants.innerLineTestCount);

            double x2 = l2p2.x + (l3p2.x - l2p2.x)*((double)i/ Constants.PathingConstants.innerLineTestCount);
            double y2 = l2p2.y + (l3p2.y - l2p2.y)*((double)i/ Constants.PathingConstants.innerLineTestCount);

            LineIntersection.Point lp1New = new LineIntersection.Point(x1,y1);
            LineIntersection.Point lp2New = new LineIntersection.Point(x2,y2);

            if(LineIntersection.doIntersect(bp1,bp2,lp1New,lp2New)){
                pathObstructed = true;
            }

    }

    return pathObstructed;

}



 public Pose2d[] findNavPoses(Pose2d myPose, Pose2d desiredPose, int recursionDepth){
    double random = Math.random(); // :)
    if(!barrierOnLine(new Line2D.Double(myPose.getX(),myPose.getY(),desiredPose.getX(),desiredPose.getY())))
        return new Pose2d[] {myPose,desiredPose};
    if(recursionDepth>Constants.PathingConstants.maxRecursionDepth)
        return new Pose2d[] {};

    for (double dist = Constants.PathingConstants.lineDistIterator; dist < Constants.PathingConstants.maxLineDist; dist += Constants.PathingConstants.lineDistIterator)
        for (int angI = -cornerPoints.size(); angI < Constants.PathingConstants.moveAngles; angI += 1) {

            double ang = angI * (Math.PI * 2 / (Constants.PathingConstants.moveAngles));
            double branchHeadX = myPose.getX() + dist*Math.cos(ang);
            double branchHeadY = myPose.getY() + dist*Math.sin(ang);

            if(angI<0){ //iterate through every corner point before doing anything else - notice how angI starts at -size
                //for indexes, use [cornerPoints.size()-ang]
                int cornerIndex = angI + cornerPoints.size();
                if(Math.random()>0.5) cornerIndex = -angI - 1;
                branchHeadX = cornerPoints.get(cornerIndex).getX();
                branchHeadY = cornerPoints.get(cornerIndex).getY();

            }


            Line2D.Double lineToTestPoint = new Line2D.Double(myPose.getX(), myPose.getY(),branchHeadX ,branchHeadY );

            if(barrierOnLine(lineToTestPoint)) continue;

            Pose2d[] lowerLevelOut = findNavPoses(new Pose2d(branchHeadX,branchHeadY,desiredPose.getRotation()),desiredPose,recursionDepth+1);

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

private ArrayList<Pose2d> navPoses = new ArrayList<Pose2d>();
private Pose2d desiredPose;

    public void setNavPoint(Pose2d desiredPose){
        this.desiredPose = desiredPose;
        poseChanged = true;
        pTelemetrySub.updateDestinationPose(this.desiredPose);
        //updateNavPoses();
    }

    public void updateNavPoses(){
        boolean poseChangedOld = poseChanged;
        poseChanged = false;
        if(desiredPose == null)
            return;
        Pose2d[] outNavPoses = findNavPoses(swerveSubsystem.getEstimatedPosition(),desiredPose,0);
        if(outNavPoses.length<1)
            return;
        if(getPathLengthFromBot(outNavPoses)>getPathLengthFromBot(navPoses) && (engaged && !poseChangedOld))
            return;


        navPoses.clear();
        for(Pose2d  i : outNavPoses)
            navPoses.add(i);
        pTelemetrySub.updateDestinationPose(this.desiredPose);
        pTelemetrySub.updateNavPoses(navPoses);
    }
    boolean poseChanged = false;
    private double getPathLengthFromBot(Pose2d[] path){
        if(path.length==0)
            return Integer.MAX_VALUE;
        double botX = swerveSubsystem.getEstimatedPosition().getX();
        double botY = swerveSubsystem.getEstimatedPosition().getY();
        double length = Math.sqrt(Math.pow(botX - path[0].getX(),2)+Math.pow(botY-path[0].getY(),2));
        for(int i = 1; i<path.length; i++)
            length+=Math.sqrt(Math.pow(path[i].getX() - path[i-1].getX(),2)+Math.pow(path[i].getY() - path[i-1].getY(),2));

        return length;
    }
    private  double getPathLengthFromBot(ArrayList<Pose2d> path){
        Pose2d[] newPath = new Pose2d[path.size()];
        for(int i = 0; i<newPath.length; i++)
            newPath[i] = path.get(i);
        return getPathLengthFromBot(newPath);
    }


public Pose2d getNextNavPoint(){
    if(navPoses.size()<1) return swerveSubsystem.getEstimatedPosition();
    pTelemetrySub.updateNavPoses(navPoses);
    Pose2d botPose = swerveSubsystem.getEstimatedPosition();
    while(navPoses.size()>1 && Math.sqrt(Math.pow(botPose.getX() - navPoses.get(0).getX(),2)+Math.pow(botPose.getY() - navPoses.get(0).getY(),2))<Constants.PathingConstants.reachedInBetweenPointThreshold)
        navPoses.remove(0);
    if(navPoses.size()>0)
        return navPoses.get(0);
    return swerveSubsystem.getEstimatedPosition();

}

public void setNavTrajectory(ArrayList<Pose2d> navPoses){
        this.navPoses = navPoses;
}

private void createBarriers(){
    Scalar redBarrierColor = new Scalar(100,100,255);
    Scalar blueBarrierColor = new Scalar(255,100,100);

    //outer walls
    fieldLines.add(new FieldLine(new Line2D.Double(leftWallPos,topWallPos,rightWallPos,topWallPos)));
    fieldLines.add(new FieldLine(new Line2D.Double(leftWallPos,bottomWallPos,rightWallPos,bottomWallPos)));
    fieldLines.add(new FieldLine(new Line2D.Double(leftWallPos,topWallPos,leftWallPos,bottomWallPos)));
    fieldLines.add(new FieldLine(new Line2D.Double(rightWallPos,topWallPos,rightWallPos,bottomWallPos)));

    //node long sides
    fieldLines.add(new FieldLine(new Line2D.Double(leftWallPos+nodesWidth, topWallPos, leftWallPos+nodesWidth, topWallPos - nodesHeight),true, redBarrierColor));
    fieldLines.add(new FieldLine(new Line2D.Double(rightWallPos-nodesWidth, topWallPos, rightWallPos-nodesWidth, topWallPos - nodesHeight),true, blueBarrierColor));
    //node short sides
    fieldLines.add(new FieldLine(new Line2D.Double(rightWallPos, topWallPos - nodesHeight, rightWallPos-nodesWidth, topWallPos - nodesHeight)));
    fieldLines.add(new FieldLine(new Line2D.Double(leftWallPos, topWallPos - nodesHeight, leftWallPos+nodesWidth, topWallPos - nodesHeight)));

    //barriers
    fieldLines.add(new FieldLine(new Line2D.Double(leftWallPos+nodesWidth, topWallPos - nodesHeight, leftWallPos+nodesWidth + barrierLength, topWallPos - nodesHeight),true));
    fieldLines.add(new FieldLine(new Line2D.Double(rightWallPos-nodesWidth, topWallPos - nodesHeight, rightWallPos-nodesWidth - barrierLength, topWallPos - nodesHeight),true));

    //charge stations
    fieldLines.add(new FieldLine(new Line2D.Double(chargeStationFarX, chargeStationTopY, chargeStationCloseX, chargeStationTopY),true, blueBarrierColor));
    fieldLines.add(new FieldLine(new Line2D.Double(chargeStationFarX, chargeStationBottomY, chargeStationCloseX, chargeStationBottomY),true, blueBarrierColor));

    fieldLines.add(new FieldLine(new Line2D.Double(-chargeStationFarX, chargeStationTopY, -chargeStationCloseX, chargeStationTopY),true, redBarrierColor));
    fieldLines.add(new FieldLine(new Line2D.Double(-chargeStationFarX, chargeStationBottomY, -chargeStationCloseX, chargeStationBottomY),true, redBarrierColor));

    //little charge station edge things
/*
    barriers.add(new Line2D.Double(chargeStationFarX, chargeStationTopY+chargeStationEdgeLength/2, chargeStationFarX, chargeStationTopY-chargeStationEdgeLength/2));
    barriers.add(new Line2D.Double(chargeStationCloseX, chargeStationTopY+chargeStationEdgeLength/2, chargeStationCloseX, chargeStationTopY-chargeStationEdgeLength/2));
    barriers.add(new Line2D.Double(chargeStationFarX, chargeStationBottomY+chargeStationEdgeLength/2, chargeStationFarX, chargeStationBottomY-chargeStationEdgeLength/2));
    barriers.add(new Line2D.Double(chargeStationCloseX, chargeStationBottomY+chargeStationEdgeLength/2, chargeStationCloseX, chargeStationBottomY-chargeStationEdgeLength/2));

    barriers.add(new Line2D.Double(-chargeStationFarX, chargeStationTopY+chargeStationEdgeLength/2, -chargeStationFarX, chargeStationTopY-chargeStationEdgeLength/2));
    barriers.add(new Line2D.Double(-chargeStationCloseX, chargeStationTopY+chargeStationEdgeLength/2, -chargeStationCloseX, chargeStationTopY-chargeStationEdgeLength/2));
    barriers.add(new Line2D.Double(-chargeStationFarX, chargeStationBottomY+chargeStationEdgeLength/2, -chargeStationFarX, chargeStationBottomY-chargeStationEdgeLength/2));
    barriers.add(new Line2D.Double(-chargeStationCloseX, chargeStationBottomY+chargeStationEdgeLength/2, -chargeStationCloseX, chargeStationBottomY-chargeStationEdgeLength/2));
*/
    //block off charge stations

   // barriers.add(new Line2D.Double(chargeStationFarX, chargeStationTopY, chargeStationFarX, chargeStationBottomY));
    fieldLines.add(new FieldLine(new Line2D.Double(chargeStationCloseX, chargeStationTopY, chargeStationCloseX, chargeStationBottomY),true, new Scalar(20,20,20)));

  //  barriers.add(new Line2D.Double(-chargeStationFarX, chargeStationTopY, -chargeStationFarX, chargeStationBottomY));
    fieldLines.add(new FieldLine(new Line2D.Double(-chargeStationCloseX, chargeStationTopY, -chargeStationCloseX, chargeStationBottomY),true, new Scalar(20,20,20)));


    //double substations
    fieldLines.add(new FieldLine(new Line2D.Double(leftWallPos+doubleSubstationDepth, topWallPos - nodesHeight, leftWallPos+doubleSubstationDepth, bottomWallPos),true, blueBarrierColor));
    fieldLines.add(new FieldLine(new Line2D.Double(rightWallPos-doubleSubstationDepth, topWallPos -nodesHeight, rightWallPos-doubleSubstationDepth, bottomWallPos),true, redBarrierColor));

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
                                SmartDashboard.putNumber("pathingCompTime",compTime);
                              //  Thread.sleep((int) (Math.max(compTime/Constants.PathingConstants.maxCPUTime, Constants.PathingConstants.minPathingDelay)));
                                Thread.sleep(Constants.PathingConstants.minPathingDelay);
                            }
                        } catch (InterruptedException e) {throw new RuntimeException(e);}
                    });
    pathingThread.setDaemon(true);
    pathingThread.setPriority(1); //low priority I hope?
    pathingThread.start();
}
private boolean engaged = false;
    private double engageTime;

    public void engageNav(){ //nudge th
        engaged = true;
        engageTime = Timer.getFPGATimestamp();
    }
    public void disengageNav(){
        engaged = false;
    }

private void resetNodes(){
        nodes.clear();
        setPoints.clear();
        //for(int revX = -1; revX<=1; revX+=2)
    int revX = 1;  //used to make nodes mirror on both sides
    if(fmsGetter.isRedAlliance())
        revX = -1;

            for(int x = 0; x<=2; x++)
                for(int y = 0; y<9; y++){
                    if(x==2){
                        nodes.add(new Node(nodeX1*revX, topNodeY-y* yDistBetweenNodes, Node.NodeType.HYBRID, Node.Level.GROUND));
                        continue;
                    }
                    Node.Level level = Node.Level.LVL1;
                    double myNodeX = nodeX2;
                    if(x==1) {
                        level = Node.Level.LVL2;
                        myNodeX = nodeX3;
                    }
                    Node.NodeType type = Node.NodeType.CONE;
                    if(y%3==1)
                        type = Node.NodeType.CUBE;

                    nodes.add(new Node(myNodeX*revX,topNodeY-y*yDistBetweenNodes, type, level));
            }

            for(int y = 0; y<9; y++){
                double yPos = topNodeY - y*yDistBetweenNodes;
                double xPos = width1/2 - nodesWidth - Constants.PathingConstants.kRobotWidth/2 - Units.inchesToMeters(3); //3 inches from nodes barrier?
                xPos*=revX;
                Rotation2d myRotation = Rotation2d.fromDegrees(180); //red
                if(revX>0)
                    myRotation = Rotation2d.fromDegrees(0); //blue
                setPoints.add(new Pose2d(xPos,yPos,myRotation));


            }

}
int setPointIndex = 0;
    public void iterateSetPoint(){
        setPointIndex++;
        updateSetPoint();
    }
    public void decimateSetPoint(){
        setPointIndex--;
        updateSetPoint();
    }
    public void updateSetPoint(){
        if(setPointIndex>=setPoints.size())
            setPointIndex = 0;
        if(setPointIndex<0)
            setPointIndex = setPoints.size()-1;

        if(setPoints.size()>0)
            setNavPoint(setPoints.get(setPointIndex));
    }

    public void placeCornerPoints(){
        cornerPoints.clear();

        cornerPoints.add(new Pose2d(-3,3, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(-3,-0.8, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(-6,3, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(-6,-0.8, Rotation2d.fromDegrees(0)));
        
        cornerPoints.add(new Pose2d(3,-0.8, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(6,3, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(6,-0.8, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(3,3, Rotation2d.fromDegrees(0)));


        pTelemetrySub.updateCornerPoints(cornerPoints);
    }


}
