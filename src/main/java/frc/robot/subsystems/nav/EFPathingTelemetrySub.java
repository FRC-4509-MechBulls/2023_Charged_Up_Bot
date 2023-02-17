package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.FieldObjects.FieldLine;
import frc.robot.lib.FieldObjects.FieldTag;
import frc.robot.lib.FieldObjects.Node;
import frc.robot.subsystems.state.StateControllerSubsystem;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class EFPathingTelemetrySub extends GraphicalTelemetrySubsystem {

    public EFPathingTelemetrySub() {
        super("EFNav",360,360,2);
    }


    protected void drawThings(Mat mat){
        clearScreen(mat);
        dontTouchMe = true;

        dontTouchMe = false;
    }


    public void drawArrow(Mat mat, double x, double y, double angle, double length, Scalar color){
        double startX = x- length/2*Math.cos(angle);
        double startY = y+ length/2*Math.sin(angle);
        double endX = x+ length/2*Math.cos(angle);
        double endY = y- length/2*Math.sin(angle);
        double branch1X = endX + length/2.5*Math.cos(angle+Math.toRadians(135));
        double branch1Y = endY - length/2.5*Math.sin(angle+Math.toRadians(135));
        double branch2X = endX + length/2.5*Math.cos(angle-Math.toRadians(135));
        double branch2Y = endY - length/2.5*Math.sin(angle-Math.toRadians(135));

        Imgproc.line(mat, new Point(startX,startY),new Point(endX,endY),color,2);
        Imgproc.line(mat, new Point(endX,endY),new Point(branch1X,branch1Y),color,2);
        Imgproc.line(mat, new Point(endX,endY),new Point(branch2X,branch2Y),color,2);
    }

    public void drawRotatedRect(Mat mat, double centerX, double centerY, double l, double w, Rotation2d angle, Scalar color, int thickness){
        double ang1 = Math.atan2(l, w);
        double ang2 = Math.atan2(l, -w);
        double cornerDist = Math.sqrt(Math.pow(l/2,2) + Math.pow(w/2,2));
        double robotAngle = angle.getRadians();
        Point[] robotPts = new Point[]{
                new Point(cornerDist*Math.cos(robotAngle+ang1) + centerX, cornerDist*Math.sin(robotAngle+ang1) + centerY),
                new Point(cornerDist*Math.cos(robotAngle-ang1) + centerX, cornerDist*Math.sin(robotAngle-ang1) + centerY),
                new Point(cornerDist*Math.cos(robotAngle-ang2) + centerX, cornerDist*Math.sin(robotAngle-ang2) + centerY),
                new Point(cornerDist*Math.cos(robotAngle+ang2) + centerX, cornerDist*Math.sin(robotAngle+ang2) + centerY)

        };
        //convert corner points to pixels
        for(int i = 0; i<robotPts.length; i++)
            robotPts[i] = metersPosToPixelsPos(robotPts[i]);
        //draw robot to screen
        List<MatOfPoint> pointList2 = new ArrayList<MatOfPoint>();
        pointList2.add(new MatOfPoint(robotPts[0],robotPts[1],robotPts[2],robotPts[3])); //ew gross
        Imgproc.polylines(mat,pointList2 ,true,color,thickness);
    }

    void clearScreen(Mat mat){
        List<MatOfPoint> pointList1 = new ArrayList<MatOfPoint>();
        pointList1.add(new MatOfPoint(
                new Point(0,0),
                new Point(0,480),
                new Point(768,480),
                new Point(768,0)
        ));
        Imgproc.fillPoly(mat,pointList1,new Scalar(0,0,0));
        
    }


    public  Point metersPosToPixelsPos(Point posInMeters){
        double centerX = 640/2;
        double centerY = 480/2;
        double ang = Math.toRadians(180);
        posInMeters.x = -posInMeters.x; //what?
   //     posInMeters.x += SmartDashboard.getNumber("TCamX",0);
   //     posInMeters.y += SmartDashboard.getNumber("TCamY",0);

     //   double zoom = SmartDashboard.getNumber("TCamZoom",1) * 0.6  ;
        double zoom = 1 * 0.6;
     //   ang+= Math.toRadians(SmartDashboard.getNumber("TCamAngle",0));
        double outX = centerX;
        double outY = centerY;
        outX+= posInMeters.x * 60;
        outY+= posInMeters.y * 60;

        double dist = Math.sqrt(Math.pow(outX - centerX ,2) + Math.pow(outY -centerY,2)) * zoom;
        double dir = Math.atan2(outY -centerY, outX - centerX);
        outX = centerX + dist * Math.cos(dir+ang);
        outY = centerY + dist * Math.sin(dir+ang);

        outX+=128;
        if(outX<128)
            outX = 128;
        return new Point (outX,outY);
    }

    public double[] metersPosToPixelsPos(double[] posInMeters){
        Point point = new Point(posInMeters[0],posInMeters[1]);
        Point newPoint = metersPosToPixelsPos(point);
        return new double[]{newPoint.x,newPoint.y};
    }

    public void init() {


    }

    public void periodic(){

    }

}
