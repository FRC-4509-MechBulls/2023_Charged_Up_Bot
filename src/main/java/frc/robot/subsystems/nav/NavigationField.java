package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.*;
import frc.robot.lib.FieldObjects.FieldLine;
import frc.robot.lib.FieldObjects.Node;
import frc.robot.subsystems.state.FMSGetter;
import frc.robot.subsystems.arm.Grabber;
import frc.robot.subsystems.state.StateControllerSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import org.opencv.core.Scalar;

import java.awt.geom.Line2D;
import java.util.ArrayList;

import static frc.robot.Constants.FieldConstants.*;


public class NavigationField extends SubsystemBase {

    StateControllerSubsystem stateControllerSub;
 ArrayList<FieldLine> fieldLines = new ArrayList<FieldLine>();
 ArrayList<Node> nodes = new ArrayList<Node>();
 ArrayList<Pose2d> setPoints = new ArrayList<Pose2d>();

 ArrayList<Pose2d> cornerPoints = new ArrayList<Pose2d>();


PathingTelemetrySub pTelemetrySub;
 private SwerveSubsystem swerveSubsystem;
 Thread pathingThread;
 FMSGetter fmsGetter;
public NavigationField(PathingTelemetrySub telemetrySub, SwerveSubsystem swerveSubsystem, FMSGetter fmsGetter, StateControllerSubsystem stateControllerSub){
    this.pTelemetrySub = telemetrySub;
    this.swerveSubsystem = swerveSubsystem;
    this.fmsGetter = fmsGetter;
    this.stateControllerSub = stateControllerSub;

    createAndStartPathingThread();
    createBarriers();

    this.pTelemetrySub.updateBarriers(fieldLines);
    this.pTelemetrySub.updateNodes(nodes);
    this.pTelemetrySub.updateSetPoints(setPoints);
}

boolean wasOnRedAlliance = true;
double lastAllianceCheck = Timer.getFPGATimestamp();
boolean queueNodeReset = true;
int oldSetpointIndex = 0;
public void periodic(){
pTelemetrySub.updateRobotPose(swerveSubsystem.getEstimatedPosition());

//Line2D.Double testLine = new Line2D.Double(SmartDashboard.getNumber("x1",0),SmartDashboard.getNumber("y1",0),SmartDashboard.getNumber("x2",0),SmartDashboard.getNumber("y2",0));
//    SmartDashboard.putBoolean("barrierOnLine", barrierOnLine(testLine));
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
  //  SmartDashboard.putBoolean("poseChanged",poseChanged);
    if(oldSetpointIndex != stateControllerSub.getSetpointIndex()){
        updateSetPoint(stateControllerSub.getSetpointIndex());
        oldSetpointIndex = stateControllerSub.getSetpointIndex();
    }

    //set the setpoint to the closest setpoint to the robot
    Pose2d robotPose = swerveSubsystem.getEstimatedPosition();
    double minDist = Double.MAX_VALUE;
    int minIndex = -1;
    for(int i = 0; i<setPoints.size(); i++){
        double dist = Math.sqrt(Math.pow(robotPose.getTranslation().getX() - setPoints.get(i).getX(),2) + Math.pow(robotPose.getTranslation().getY() - setPoints.get(i).getY(),2));
        if(dist<minDist){
            minDist = dist;
            minIndex = i;
        }
    }
    if(minIndex!=-1)
        closestSetpoint = new Pose2d(setPoints.get(minIndex).getX(),setPoints.get(minIndex).getY(), setPoints.get(minIndex).getRotation());

}
Pose2d closestSetpoint = new Pose2d();
public Pose2d getClosestSetpoint(){
    return closestSetpoint;
}

public  boolean barrierOnLine(Line2D.Double line){
    double lineDir = Math.atan2(line.getY2() - line.getY1() , line.getX2() - line.getX1());
    double lineDist = Math.sqrt(Math.pow(line.getX1() - line.getX2(),2) + Math.pow(line.getY1() - line.getY2(),2));

    double edgePtX1 = line.getX1() + Math.cos(lineDir - Math.PI/2) * Constants.PathingConstants.ROBOT_RADIUS; //why do I need to multiply by 2??
    double edgePtY1 = line.getY1() + Math.sin(lineDir - Math.PI/2) * Constants.PathingConstants.ROBOT_RADIUS; //because your visualization was not to scale 😉

    double edgePtX2 = line.getX1() + Math.cos(lineDir + Math.PI/2) * Constants.PathingConstants.ROBOT_RADIUS;
    double edgePtY2 = line.getY1() + Math.sin(lineDir + Math.PI/2) * Constants.PathingConstants.ROBOT_RADIUS;

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



 public Pose2d[] findNavPoses(Pose2d myPose, Pose2d desiredPose, int recursionDepth, double startTime, double timeout){
     if(Timer.getFPGATimestamp() - startTime >timeout)
         return new Pose2d[]{};

    double random = Math.random(); // :)
     int[] randIndexes = MB_Math.randomIndexes(cornerPoints.size());
    if(!barrierOnLine(new Line2D.Double(myPose.getX(),myPose.getY(),desiredPose.getX(),desiredPose.getY())))
        return new Pose2d[] {myPose,desiredPose};
    if(recursionDepth>Constants.PathingConstants.maxRecursionDepth)
        return new Pose2d[] {};

    for (double dist = Constants.PathingConstants.lineDistIterator; dist < Constants.PathingConstants.maxLineDist; dist += Constants.PathingConstants.lineDistIterator)
        for (int angI = -cornerPoints.size(); angI < Constants.PathingConstants.moveAngles; angI += 1) {

            double ang = angI * (Math.PI * 2 / (Constants.PathingConstants.moveAngles));
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

private ArrayList<Pose2d> navPoses = new ArrayList<Pose2d>();
private Pose2d desiredPose;

public Pose2d getDesiredPose(){
    return desiredPose;
}

    public void setNavPoint(Pose2d desiredPose){
        if(desiredPose!= null && this.desiredPose != null)
        if(MB_Math.poseDist(desiredPose,this.desiredPose) != 0 || this.desiredPose.getRotation().getDegrees() != desiredPose.getRotation().getDegrees()){
            poseChanged = true;
        }
        this.desiredPose = desiredPose;
        pTelemetrySub.updateDestinationPose(this.desiredPose);
        //updateNavPoses();
    }

    Pose2d lastUsedPose = new Pose2d();
    public void updateNavPoses(){
        boolean poseChangedOld = poseChanged;
        poseChanged = false;
        if(desiredPose == null)
            return;
        Pose2d[] outNavPoses = findNavPoses(swerveSubsystem.getEstimatedPosition(),desiredPose,0,Timer.getFPGATimestamp(),5);
        if(outNavPoses.length<1)
            return;
        boolean poseCloseToLast = MB_Math.poseDist(lastUsedPose,swerveSubsystem.getEstimatedPosition())<Constants.PathingConstants.recalcThreshold;
        if(getPathLengthFromBot(outNavPoses)>getPathLengthFromBot(navPoses) && ((engaged || poseCloseToLast ) && !poseChangedOld)){
        //    SmartDashboard.putNumber("lastEngagedDrop",Timer.getFPGATimestamp());
            return;
        }

        lastUsedPose = swerveSubsystem.getEstimatedPosition();
        navPoses.clear();
        for(Pose2d  i : outNavPoses)
            navPoses.add(i);
        pTelemetrySub.updateDestinationPose(this.desiredPose);
    //    pTelemetrySub.updateNavPoses(navPoses);
    }
    boolean poseChanged = false;
    private double getPathLengthFromBot(Pose2d[] path){
        if(path.length==0 || path[0] == null)
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
        for(int i = 0; i<Math.min(path.size(),newPath.length); i++)
            newPath[i] = path.get(i);
        return getPathLengthFromBot(newPath);
    }


public Pose2d getNextNavPoint(){
    if(navPoses.size()<1) return swerveSubsystem.getEstimatedPosition();
//    pTelemetrySub.updateNavPoses(navPoses);
    Pose2d botPose = swerveSubsystem.getEstimatedPosition();
    while(navPoses.size()>1 && Math.sqrt(Math.pow(botPose.getX() - navPoses.get(0).getX(),2)+Math.pow(botPose.getY() - navPoses.get(0).getY(),2))<Constants.PathingConstants.reachedInBetweenPointThreshold)
        if(navPoses.size()>1)
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
                                //SmartDashboard.putNumber("pathingCompTime",compTime);
                              //  Thread.sleep((int) (Math.max(compTime/Constants.PathingConstants.maxCPUTime, Constants.PathingConstants.minPathingDelay)));
                                pTelemetrySub.updateNavPoses(navPoses);
                                Thread.sleep(Constants.PathingConstants.minPathingDelay);
                            }
                        } catch (InterruptedException e) {throw new RuntimeException(e);}
                    });
    pathingThread.setDaemon(true);
    pathingThread.setName("MB_Pathing");
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
                        nodes.add(new Node(nodeX1*revX, topNodeY-y* yDistBetweenNodes, Node.NodeType.HYBRID, StateControllerSubsystem.Level.POS1));
                        continue;
                    }
                    StateControllerSubsystem.Level level = StateControllerSubsystem.Level.POS2;
                    double myNodeX = nodeX2;
                    if(x==1) {
                        level = StateControllerSubsystem.Level.POS3;
                        myNodeX = nodeX3;
                    }
                    Node.NodeType type = Node.NodeType.CONE;
                    if(y%3==1)
                        type = Node.NodeType.CUBE;

                    nodes.add(new Node(myNodeX*revX,topNodeY-y*yDistBetweenNodes, type, level));
            }

            for(int y = 0; y<9; y++){
                double yPos = topNodeY - y*yDistBetweenNodes;
                //double xPos = width1/2 - nodesWidth - Constants.PathingConstants.ROBOT_LENGTH /2 - Units.inchesToMeters(12); //3 inches from nodes barrier?
                double xPos = width1/2 - nodesWidth - Units.inchesToMeters(37.375/2 + 2); //3 inches from nodes barrier?
                xPos*=revX;
                Rotation2d myRotation = Rotation2d.fromDegrees(180); //red
                if(revX>0)
                    myRotation = Rotation2d.fromDegrees(0); //blue
                setPoints.add(new Pose2d(xPos,yPos,myRotation));


            }

}
int setPointIndex = 0;
    private int placingLvlIndex = 0;
    private StateControllerSubsystem.Level placingLevel = StateControllerSubsystem.Level.POS1;
    private static final StateControllerSubsystem.Level[] nodeLevels = {StateControllerSubsystem.Level.POS1, StateControllerSubsystem.Level.POS2,StateControllerSubsystem.Level.POS3};



    public void iteratePlacingLevel(){
        placingLvlIndex++;
        updateNodeLevel();
    }
    public void decimatePlacingLevel(){
     placingLvlIndex--;
     updateNodeLevel();
    }


    public void updateSetPoint(int i){
        if(setPoints.size()==0) return;
        int adjustedIndex = MB_Math.indexWrap(i, setPoints.size());
        if(setPoints.size()>0)
            setNavPoint(setPoints.get(adjustedIndex));
    }

    public void updateNodeLevel(){
        if(placingLvlIndex >nodeLevels.length-1)
            placingLvlIndex = nodeLevels.length - 1;
        if(placingLvlIndex <0)
            placingLvlIndex = 0;
        setPlacingLevel(nodeLevels[placingLvlIndex]);
    }
    public void setPlacingLevel(StateControllerSubsystem.Level placingLevel){
        this.placingLevel = placingLevel;
    }
    public StateControllerSubsystem.Level getPlacingNodeLevel(){
        return this.placingLevel;
    }
    public Grabber.ArmModes getPlacingArmModeCube(){
        switch(this.placingLevel){
            case POS1: return Grabber.ArmModes.PLACING_CUBE_LVL1;
            case POS2: return Grabber.ArmModes.PLACING_CUBE_LVL2;
            case POS3: return Grabber.ArmModes.PLACING_CUBE_LVL3;
        }
        return null;
    }
    public Grabber.ArmModes getPlacingArmModeCone(){
        switch(this.placingLevel){
            case POS1: return Grabber.ArmModes.PLACING_CONE_LVL1;
            case POS2: return Grabber.ArmModes.PLACING_CONE_LVL2;
            case POS3: return Grabber.ArmModes.PLACING_CONE_LVL3;
        }
        return null;
    }

    public void placeCornerPoints(){
        cornerPoints.clear();

        cornerPoints.add(new Pose2d(-2.75,3.25, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(-2.75,-0.7, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(-6,3.25, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(-6,-0.7, Rotation2d.fromDegrees(0)));
        
        cornerPoints.add(new Pose2d(2.75,-0.7, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(6,3.25, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(6,-0.7, Rotation2d.fromDegrees(0)));
        cornerPoints.add(new Pose2d(2.75,3.25, Rotation2d.fromDegrees(0)));


        pTelemetrySub.updateCornerPoints(cornerPoints);
    }


}
