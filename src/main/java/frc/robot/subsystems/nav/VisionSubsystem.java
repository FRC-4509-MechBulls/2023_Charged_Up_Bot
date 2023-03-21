// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.FieldObjects.FieldTag;
import frc.robot.lib.MB_Math;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.nav.PathingTelemetrySub;
import org.photonvision.PhotonCamera;
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
        for(PhotonTrackedTarget target :camera.getLatestResult().getTargets()){
            int id = target.getFiducialId();
            Transform3d transform = target.getBestCameraToTarget();
            Transform3d alternateTransform = target.getAlternateCameraToTarget();
            Rotation3d rotation = transform.getRotation();

            for(int i = 0; i<fieldTags.size(); i++){
                if(fieldTags.get(i).getID() != id) continue;
                Transform3d sentTransform = new Transform3d(new Translation3d(transform.getX()*-1,transform.getY()*-1,transform.getZ()),transform.getRotation());
                swerveSubsystem.fieldTagSpotted(fieldTags.get(i), transform, camera.getLatestResult().getLatencyMillis(), target.getPoseAmbiguity());
      //          SmartDashboard.putNumber("lastPoseAmbiguity",camera.getLatestResult().getBestTarget().getPoseAmbiguity());


            }



            double rotationFromCamera = Math.IEEEremainder(lastTransform.getRotation().getZ() + Math.PI, 2*Math.PI);


            if(id==0){
                lastSeenOnRight = transform.getY()>0;
                lastTransform = transform;
                lastSeenTime = Timer.getFPGATimestamp();
            }

        }

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
