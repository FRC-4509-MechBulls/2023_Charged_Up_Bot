package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.NavigationField;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PathingTelemetrySub extends GraphicalTelemetrySubsystem{

    public PathingTelemetrySub() {super("Pathing");}


    protected void drawThings(Mat mat){
       /* List<MatOfPoint> pointList2 = new ArrayList<MatOfPoint>();
        pointList2.add(new MatOfPoint(
                new Point(50,50),
                new Point(25,100),
                new Point(75,100)
        ));
        Imgproc.polylines (mat, pointList2, true, new Scalar(255, 255, 255), 1);*/


        //draws a small point at each meter
        for(int markX = -5; markX<=5; markX++)
            for(int markY = -5; markY<=5; markY++){
                double[] pt = metersPosToPixelsPos(new double[] {markX,markY});
                Imgproc.rectangle(mat,new Point(pt[0],pt[1]),new Point(pt[0]+0,pt[1]+0), new Scalar(0,255,0),3);
            }

        //draws all barriers
        for(Line2D.Double line: barriers){
            double[] pt1Pix = metersPosToPixelsPos(new double[] {line.getX1(), line.getY1()});
            double[] pt2Pix = metersPosToPixelsPos(new double[] {line.getX2(), line.getY2()});
            Imgproc.line(mat,new Point(pt1Pix[0],pt1Pix[1]),new Point(pt2Pix[0],pt2Pix[1]), new Scalar(255,255,255),4);
        }


        //get robot corners in meters
        double ang1 = Math.atan2(Constants.PathingConstants.kRobotLength, Constants.PathingConstants.kRobotWidth);
        double ang2 = Math.atan2(Constants.PathingConstants.kRobotLength, -Constants.PathingConstants.kRobotWidth);
        double cornerDist = Math.sqrt(Math.pow(Constants.PathingConstants.kRobotLength/2,2) + Math.pow(Constants.PathingConstants.kRobotWidth/2,2));
        double robotAngle = robotPose.getRotation().getRadians();
        Point[] robotPts = new Point[]{
                new Point(cornerDist*Math.cos(robotAngle+ang1) + robotPose.getX(), cornerDist*Math.sin(robotAngle+ang1) + robotPose.getY()),
                new Point(cornerDist*Math.cos(robotAngle-ang1) + robotPose.getX(), cornerDist*Math.sin(robotAngle-ang1) + robotPose.getY()),
                new Point(cornerDist*Math.cos(robotAngle-ang2) + robotPose.getX(), cornerDist*Math.sin(robotAngle-ang2) + robotPose.getY()),
                new Point(cornerDist*Math.cos(robotAngle+ang2) + robotPose.getX(), cornerDist*Math.sin(robotAngle+ang2) + robotPose.getY())

        };
        //convert corner points to pixels
        for(int i = 0; i<robotPts.length; i++)
            robotPts[i] = metersPosToPixelsPos(robotPts[i]);
        //draw robot to screen
        List<MatOfPoint> pointList2 = new ArrayList<MatOfPoint>();
        pointList2.add(new MatOfPoint(robotPts[0],robotPts[1],robotPts[2],robotPts[3])); //ew gross
        Imgproc.polylines(mat,pointList2 ,true,new Scalar(0,0,255),2);

        double camX = robotPose.getX() + Math.cos(robotAngle+ Constants.VisionConstants.camDirFromCenter) * Constants.VisionConstants.camDistFromCenter;
        double camY = robotPose.getY() + Math.sin(robotAngle + Constants.VisionConstants.camDirFromCenter)* Constants.VisionConstants.camDistFromCenter;
        Imgproc.circle(mat,metersPosToPixelsPos(new Point(camX,camY)),4,new Scalar(255,255,255),2);


//        //draw desired point
//        double[] navDesiredPointInPixels = metersPosToPixelsPos(NavigationField.getDesiredXY());
//        Point navPointAsPoint = new Point(navDesiredPointInPixels[0],navDesiredPointInPixels[1]);
//        Imgproc.line(mat, navPointAsPoint,navPointAsPoint,new Scalar(236,144,0),10);


        //draw navigation lines
        for(int i = 1; i<navPoses.size(); i++)
            Imgproc.line(mat, metersPosToPixelsPos(new Point(navPoses.get(i-1).getX(), navPoses.get(i-1).getY())),metersPosToPixelsPos(new Point(navPoses.get(i).getX(), navPoses.get(i).getY())),new Scalar(255,0,255),2);
        SmartDashboard.putNumber("navPosesInPathingTelemetry",navPoses.size());
        String out = "";
        for(Pose2d pose : navPoses){
            out+= "(" + pose.getX() + ", " + pose.getY() + "), ";
        }
        SmartDashboard.putString("navPoses", out);
        if(navPoses.size()>0){
            Pose2d lastPose = navPoses.get(navPoses.size()-1);
            Point lastPosePix = metersPosToPixelsPos(new Point(lastPose.getX(),lastPose.getY()));
            Imgproc.circle(mat,lastPosePix,1,new Scalar(0,255,255),4);

        }

        //draw test line
        Imgproc.line(mat,metersPosToPixelsPos(new Point(SmartDashboard.getNumber("x1",0),SmartDashboard.getNumber("y1",0))), metersPosToPixelsPos(new Point(SmartDashboard.getNumber("x2",0),SmartDashboard.getNumber("y2",0))),new Scalar(255,255,255),2);


        //coords and heading
        Imgproc.putText(mat,"("+(Math.floor(robotPose.getX()*100)/100.0)+", "+(Math.floor(robotPose.getY()*100)/100.0)+")",new Point(0,20),5,1,new Scalar(255,255,255));
        Imgproc.putText(mat,(Math.floor(robotPose.getRotation().getDegrees()*10)/10.0) + "*",new Point(0,40),5,1,new Scalar(255,255,255));

        //   SmartDashboard.putNumber("point?",barriers.size());

    }

    private ArrayList<Line2D.Double> barriers = new ArrayList<>();
    public void updateBarriers(ArrayList<Line2D.Double> barriers){
        this.barriers.clear();
        for(Line2D.Double line : barriers)
            this.barriers.add(line);
    }

    Pose2d robotPose = new Pose2d();
    boolean robotOrientedView = false;
public void updateRobotPose(Pose2d newPose){
robotPose = newPose;
}

ArrayList<Pose2d> navPoses = new ArrayList<Pose2d>();
public void updateNavPoses(ArrayList<Pose2d> navPoses){this.navPoses = navPoses;}

    public  Point metersPosToPixelsPos(Point posInMeters){
    posInMeters.x = -posInMeters.x;
        posInMeters.x += SmartDashboard.getNumber("TCamX",0);
        posInMeters.y += SmartDashboard.getNumber("TCamY",0);

        if(robotOrientedView) {
            posInMeters.x += robotPose.getX();
            posInMeters.y += robotPose.getY();
        }


        double centerX = 640/2;
        double centerY = 480/2;
        double ang = Math.toRadians(90);
        double zoom = SmartDashboard.getNumber("TCamZoom",1);

        ang+= Math.toRadians(SmartDashboard.getNumber("TCamAngle",0));
        if(robotOrientedView)
            ang+= robotPose.getRotation().getRadians();

        double outX = centerX;
        double outY = centerY;
        outX+= posInMeters.x * 60;
        outY+= posInMeters.y * 60;

        double dist = Math.sqrt(Math.pow(outX - centerX ,2) + Math.pow(outY -centerY,2)) * zoom;
        double dir = Math.atan2(outY -centerY, outX - centerX);
        outX = centerX + dist * Math.cos(dir+ang);
        outY = centerY + dist * Math.sin(dir+ang);

        return new Point (outX,outY);
    }

    public double[] metersPosToPixelsPos(double[] posInMeters){
        Point point = new Point(posInMeters[0],posInMeters[1]);
        Point newPoint = metersPosToPixelsPos(point);
        return new double[]{newPoint.x,newPoint.y};
    }

    public void init() {
   //     SmartDashboard.putNumber("TCamAngle",0);
   //     SmartDashboard.putNumber("TCamZoom",1.0);
   //     SmartDashboard.putNumber("TCamX",0);
   //     SmartDashboard.putNumber("TCamY",0);

    }

    public void periodic(){
    Line2D.Double pathLine = new Line2D.Double(robotPose.getX(), robotPose.getY(), 2, 0.3);
  //  SmartDashboard.putBoolean("Clear path to (2, 0.3)", NavigationField.clearPathOnLine(pathLine));
    }
}
