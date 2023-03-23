// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.FieldObjects.FieldTag;
import frc.robot.lib.MB_Math;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.nav.PathingTelemetrySub;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FieldConstants.*;

import java.util.ArrayList;

//meaga cool super epic coding time I love code so much UwU -Isaac
public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("gloworm");


    private ArrayList<FieldTag> fieldTags = new ArrayList<FieldTag>();
    private SwerveSubsystem swerveSubsystem;
    public VisionSubsystem(SwerveSubsystem swerveSubsystem, PathingTelemetrySub pathingTelemetrySub) {
        this.swerveSubsystem = swerveSubsystem;


        for(int i = 0; i<8; i++){
            int id = i+1;
            double x = aprilTagOriginX  -  Units.inchesToMeters(aprilTagYDiffsFromOriginInches[i]);
            double y = aprilTagOriginY - Units.inchesToMeters(aprilTagXDiffsFromOriginInches[i]);

            Rotation2d rotation2d = Rotation2d.fromDegrees(0);
            if(i>=4)
                rotation2d = Rotation2d.fromDegrees(180);

            fieldTags.add(new FieldTag(id, new Pose2d(x,y,rotation2d)));
        }

    pathingTelemetrySub.updateFieldTags(fieldTags);
    }




    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        processCamResult(camera.getLatestResult());


    }

    public void processCamResult(PhotonPipelineResult result){
        if(result.getTargets().size() == 1 && result.getTargets().get(0).getPoseAmbiguity() < Constants.VisionConstants.MAX_AMBIGUITY){
            SmartDashboard.putNumber("visionAmbiguity",result.getTargets().get(0).getPoseAmbiguity());
            PhotonTrackedTarget target = result.getTargets().get(0);
            int id = target.getFiducialId();
            Transform3d transform = target.getBestCameraToTarget();
            Pose2d botPose = tagPoseFromCameraToBotPose(fieldTags.get(id-1), transform); //just trusting the id-1
            swerveSubsystem.getOdometry().addVisionMeasurement(botPose, result.getTimestampSeconds());
        }
    }

    public Pose2d tagPoseFromCameraToBotPose(FieldTag fieldTag, Transform3d transform){
        //1. calculate X and Y position of camera based on X and Y components of tag and create a pose from that
        Rotation2d newRotation = new Rotation2d( ( Math.IEEEremainder((-transform.getRotation().getZ() - fieldTag.getPose().getRotation().getRadians()+4*Math.PI),2*Math.PI)));
        double newY = 0-( transform.getY()* Math.cos(-newRotation.getRadians()) + transform.getX() * Math.cos(Math.PI/2 - newRotation.getRadians())  ) + fieldTag.getPose().getY();
        double newX = 0- ( transform.getY()*Math.sin(-newRotation.getRadians()) + transform.getX() * Math.sin(Math.PI/2 - newRotation.getRadians())  ) + fieldTag.getPose().getX();
        Pose2d newPose = new Pose2d(newX,newY, newRotation);

        double camXOffset =Math.cos(newRotation.getRadians()+ Constants.VisionConstants.camDirFromCenter) * Constants.VisionConstants.camDistFromCenter;
        double camYOffset =  Math.sin(newRotation.getRadians() + Constants.VisionConstants.camDirFromCenter)* Constants.VisionConstants.camDistFromCenter;
        newX-=camXOffset;
        newY-=camYOffset;

        //  SmartDashboard.putNumber("new_x", newX);
        // SmartDashboard.putNumber("new_y", newY);

        //rotate the robot about the camera's position to account for the camera's angle (needs testing)
        //might be broken
    //    double camHeading = newRotation.getDegrees() + Constants.VisionConstants.camHeading;
    //    double[] rotated = MB_Math.rotatePoint(newX,newY,newX - camXOffset,newY - camYOffset,Constants.VisionConstants.camHeading); //should we add the bot's xy to the camXOffset and camYOffset? also the other coords?
    //    newX = rotated[0];
    //    newY = rotated[1];
     //   newRotation.rotateBy(Rotation2d.fromRadians(Constants.VisionConstants.camHeading));

        return new Pose2d(newX,newY, newRotation);

    }


    boolean lastSeenOnRight;
    Transform3d lastTransform = new Transform3d();
    double lastSeenTime;
    @Deprecated
    public double[] getDesiredSpeeds(){
        double[] out = new double[3];
        double timeSinceSeen = Timer.getFPGATimestamp() - lastSeenTime;

        if(timeSinceSeen>0.5){
            out[2] = -0.1;
            if(lastSeenOnRight)
                out[2] = 0.1;
            return out;
        }

        out[0] = (lastTransform.getX()-1) *0.5; // get one meter from target
        out[1] = lastTransform.getY() * 0.5;


        // SmartDashboard.putNumber("out0",out[0]);

        double rotationFromCamera = Math.IEEEremainder(lastTransform.getRotation().getZ() + Math.PI, 2*Math.PI);
        out[2] =  rotationFromCamera;//this might be the wrong axis, uncomment this for rotation tracking

        out[0] = MB_Math.maxValueCutoff(out[0], 0.2);
        out[1] = MB_Math.maxValueCutoff(out[1], 0.2);
        if(Math.abs(out[2])<0.1) out[2] = 0;
        out[2] = MB_Math.maxValueCutoff(out[2], 0.05);


        return out;
    }

}
