package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.FieldObjects.FieldLine;
import frc.robot.lib.FieldObjects.FieldTag;
import frc.robot.lib.FieldObjects.Node;
import frc.robot.lib.LineIntersection;
import frc.robot.subsystems.state.StateControllerSubsystem;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.EFPathingConstants.*;

public class EFPathingTelemetrySub extends GraphicalTelemetrySubsystem {

    public EFPathingTelemetrySub() {
        super("EFNav",480,360,2);
    }
        Point2D.Double pivotPoint = new Point2D.Double(1,1);

    Point2D.Double efCenter = new Point2D.Double(1,1);
    ArrayList<FieldLine> fieldLines = new ArrayList<>();

    protected void drawThings(Mat mat){
        clearScreen(mat);
        dontTouchMe = true;
        drawOrigin(mat);
        drawEFPivotPointAndBox(mat);
        drawBarriers(mat);
        dontTouchMe = false;
    }


    void drawEFPivotPointAndBox(Mat mat){
        Point pivotPointMeters = metersPosToPixelsPos(new Point(this.pivotPoint.x,this.pivotPoint.y));
        Imgproc.circle(mat,pivotPointMeters,3,new Scalar(255,255,255),2); //pivot point

        Point efCenterMeters = metersPosToPixelsPos(new Point(efCenter.x,efCenter.y));
        drawRotatedRect(mat,efCenterMeters.x,efCenterMeters.y,EF_WIDTH,EF_HEIGHT,Rotation2d.fromDegrees(0),new Scalar(0,0,255),2);
    }

    void drawOrigin(Mat mat){
        Point origin = metersPosToPixelsPos(new Point());
        Imgproc.line(mat,new Point(origin.x-3,origin.y),new Point(origin.x+3,origin.y),new Scalar(255,255,255),2);
        Imgproc.line(mat,new Point(origin.x,origin.y-3),new Point(origin.x,origin.y+3),new Scalar(255,255,255),2);
    }
    void drawBarriers(Mat mat){
        for(FieldLine line: fieldLines){
            double[] pt1Pix = metersPosToPixelsPos(new double[] {line.getLine().getX1(), line.getLine().getY1()});
            double[] pt2Pix = metersPosToPixelsPos(new double[] {line.getLine().getX2(), line.getLine().getY2()});
            Imgproc.line(mat,new Point(pt1Pix[0],pt1Pix[1]),new Point(pt2Pix[0],pt2Pix[1]), line.getColor(),4);
        }
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

    public void drawRotatedRect(Mat mat, double centerX, double centerY, double width, double height, Rotation2d angle, Scalar color, int thickness){
        double ang1 = Math.atan2(width, height);
        double ang2 = Math.atan2(width, -height);
        double cornerDist = Math.sqrt(Math.pow(width/2,2) + Math.pow(height/2,2));
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


    public  boolean barrierOnLine(Line2D.Double line){
        double lineDir = Math.atan2(line.getY2() - line.getY1() , line.getX2() - line.getX1());
        double lineDist = Math.sqrt(Math.pow(line.getX1() - line.getX2(),2) + Math.pow(line.getY1() - line.getY2(),2));

        double edgePtX1 = line.getX1() + Math.cos(lineDir - Math.PI/2) * Constants.EFPathingConstants.EF_RADIUS;
        double edgePtY1 = line.getY1() + Math.sin(lineDir - Math.PI/2) * Constants.EFPathingConstants.EF_RADIUS;

        double edgePtX2 = line.getX1() + Math.cos(lineDir + Math.PI/2) * Constants.EFPathingConstants.EF_RADIUS;
        double edgePtY2 = line.getY1() + Math.sin(lineDir + Math.PI/2) * Constants.EFPathingConstants.EF_RADIUS;

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
        for(int i = 0; i<Constants.EFPathingConstants.innerLineTestCount; i++)
            for(FieldLine fieldLine : fieldLines){
                if(!fieldLine.isABarrier()) continue;
                LineIntersection.Point bp1 = new LineIntersection.Point(fieldLine.getLine().getX1(), fieldLine.getLine().getY1());
                LineIntersection.Point bp2 = new LineIntersection.Point(fieldLine.getLine().getX2(), fieldLine.getLine().getY2());

                // if(LineIntersection.doIntersect(bp1,bp2,l1p1,l1p2) || LineIntersection.doIntersect(bp1,bp2,l2p1,l2p2) || LineIntersection.doIntersect(bp1,bp2,l3p1,l3p2)){
                //     pathClear = true;
                // }
                //         LineIntersection.Point linePoint1 = new
                double x1 = l2p1.x + (l3p1.x - l2p1.x)*((double)i/ Constants.EFPathingConstants.innerLineTestCount);
                double y1 = l2p1.y + (l3p1.y - l2p1.y)*((double)i/ Constants.EFPathingConstants.innerLineTestCount);

                double x2 = l2p2.x + (l3p2.x - l2p2.x)*((double)i/ Constants.EFPathingConstants.innerLineTestCount);
                double y2 = l2p2.y + (l3p2.y - l2p2.y)*((double)i/ Constants.EFPathingConstants.innerLineTestCount);

                LineIntersection.Point lp1New = new LineIntersection.Point(x1,y1);
                LineIntersection.Point lp2New = new LineIntersection.Point(x2,y2);

                if(LineIntersection.doIntersect(bp1,bp2,lp1New,lp2New)){
                    pathObstructed = true;
                }

            }

        return pathObstructed;

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
        double centerX = 480/2;
        double centerY = 360/2;
        double ang = Math.toRadians(180);
        posInMeters.x = -posInMeters.x; //what?
   //     posInMeters.x += SmartDashboard.getNumber("TCamX",0);
     //   posInMeters.y += SmartDashboard.getNumber("TCamY",0);

        posInMeters.x += 1.25;
        posInMeters.y += -0.9;

     //   double zoom = SmartDashboard.getNumber("TCamZoom",1) * 0.6  ;
        double zoom = 3;
     //   ang+= Math.toRadians(SmartDashboard.getNumber("TCamAngle",0));
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

    public Point pointToCVPoint(Point2D.Double pointIn){
        return new Point(pointIn.x, pointIn.y);
    }

    public void updateBarriers(ArrayList<FieldLine> fieldLines){
        if(dontTouchMe) return;
        this.fieldLines.clear();
        for(FieldLine line : fieldLines)
            this.fieldLines.add(line);
    }

    public void init() {
    //    SmartDashboard.putNumber("TCamX",0);
    //    SmartDashboard.putNumber("TCamY",0);

    }

    public void periodic(){

    }

}
