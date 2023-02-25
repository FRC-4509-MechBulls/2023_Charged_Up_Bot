// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class GraphicalTelemetrySubsystem extends SubsystemBase {
  /** Creates a new GraphicalTelemetrySubsystem. */
  Thread m_visionThread;
  public GraphicalTelemetrySubsystem(String name, int width, int height, int targetFramerate) {

    m_visionThread =
            new Thread(
                    () -> {

                      // Setup a CvSource. This will send images back to the Dashboard
                      CvSource outputStream = CameraServer.putVideo(name, width, height);
                      // Mats are very memory expensive. Lets reuse this Mat.
                      Mat mat = new Mat(height,width,16);

                      // This cannot be 'true'. The program will never exit if it is. This
                      // lets the robot stop this thread when restarting robot code or
                      // deploying.
                      while (!Thread.interrupted()) {
                            this.drawThings(mat);

                        // Give the output stream a new image to display
                        outputStream.putFrame(mat);
                          try {Thread.sleep((int)(1000.0/targetFramerate));} catch (InterruptedException e) {}
                      }
                    });
    m_visionThread.setName("MB_Telem-"+name);
    m_visionThread.setPriority(1);
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  protected boolean dontTouchMe = false;

  protected void drawThings(Mat mat){
      List<MatOfPoint> pointList2 = new ArrayList<MatOfPoint>();
      pointList2.add(new MatOfPoint(
              new Point(0,0),
              new Point(0,25),
              new Point(25,25),
              new Point(25,0)
      ));
      Imgproc.polylines (mat, pointList2, true, new Scalar(255, 255, 255), 1);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void init(){

  }
}
