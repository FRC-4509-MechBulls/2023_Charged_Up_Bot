package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PathingTelemetrySub extends GraphicalTelemetrySubsystem{

    public PathingTelemetrySub() {
        super("Pathing");
    }


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
        double cornerDist = Math.sqrt(Math.pow(Constants.PathingConstants.kRobotLength,2) + Math.pow(Constants.PathingConstants.kRobotWidth,2));
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


        //   SmartDashboard.putNumber("point?",barriers.size());

    }

    private ArrayList<Line2D.Double> barriers = new ArrayList<>();
    public void updateBarriers(ArrayList<Line2D.Double> barriers){
        this.barriers.clear();
        for(Line2D.Double line : barriers)
            this.barriers.add(line);
    }

    Pose2d robotPose = new Pose2d();
public void updateRobotPose(Pose2d newPose){
robotPose = newPose;
}

    public static double[] metersPosToPixelsPos(double[] posInMeters){
        Point point = new Point(posInMeters[0],posInMeters[1]);
        Point newPoint = metersPosToPixelsPos(point);
        return new double[]{newPoint.x,newPoint.y};
    }
    public static Point metersPosToPixelsPos(Point posInMeters){
        double outX = 640/2;
        double outY = 480/2;
        outX+= posInMeters.x * 60;
        outY+= posInMeters.y * 60;
        return new Point (outX,outY);
    }

}
