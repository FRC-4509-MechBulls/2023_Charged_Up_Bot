package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PathingTelemetrySub extends GraphicalTelemetrySubsystem{

    public PathingTelemetrySub(String name) {
        super(name);
    }


    protected void drawThings(Mat mat){
        List<MatOfPoint> pointList2 = new ArrayList<MatOfPoint>();
        pointList2.add(new MatOfPoint(
                new Point(50,50),
                new Point(25,100),
                new Point(75,100)
        ));
        Imgproc.polylines (mat, pointList2, true, new Scalar(255, 255, 255), 1);
    }
}
