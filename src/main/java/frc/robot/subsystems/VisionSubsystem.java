// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.lib.FieldTag;
import frc.robot.lib.MathThings;
import frc.robot.lib.NavigationField;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.FieldConstants.*;

import java.util.ArrayList;

//meaga cool super epic coding time I love code so much UwU -Isaac
public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("gloworm");


    private ArrayList<FieldTag> fieldTags = new ArrayList<FieldTag>();
    private SwerveSubsystem swerveSubsystem;
    public VisionSubsystem(SwerveSubsystem swerveSubsystem, PathingTelemetrySub pathingTelemetrySub) {
        this.swerveSubsystem = swerveSubsystem;
        //fieldTags.add(new FieldTag(0, new Pose2d(-1, 0, new Rotation2d(Math.PI))));
        //fieldTags.add(new FieldTag(1, new Pose2d(1.7, 0.8, new Rotation2d(-Math.PI/2))));

//        Pose2d tag1Pose = new Pose2d(-aprilTagX,centerTagY+distBetweenTags,new Rotation2d(Math.PI));
//        Pose2d tag2Pose = new Pose2d(-aprilTagX,centerTagY,new Rotation2d(Math.PI));
//        Pose2d tag3Pose = new Pose2d(-aprilTagX,centerTagY-distBetweenTags,new Rotation2d(Math.PI));
//
//        Pose2d tag6Pose = new Pose2d(aprilTagX,centerTagY-distBetweenTags,new Rotation2d());
//        Pose2d tag7Pose = new Pose2d(aprilTagX,centerTagY,new Rotation2d());
//        Pose2d tag8Pose = new Pose2d(aprilTagX,centerTagY+distBetweenTags,new Rotation2d());
//
//        Pose2d tag4Pose = new Pose2d(-lonesomeAprilTagX, lonesomeAprilTagY, new Rotation2d(Math.PI));
//        Pose2d tag5Pose = new Pose2d(lonesomeAprilTagX, lonesomeAprilTagY, new Rotation2d());
//
//        fieldTags.add(new FieldTag(1,tag1Pose));
//        fieldTags.add(new FieldTag(2,tag2Pose));
//        fieldTags.add(new FieldTag(3,tag3Pose));
//        fieldTags.add(new FieldTag(4,tag4Pose));
//        fieldTags.add(new FieldTag(5,tag5Pose));
//        fieldTags.add(new FieldTag(6,tag6Pose));
//        fieldTags.add(new FieldTag(7,tag7Pose));
//        fieldTags.add(new FieldTag(8,tag8Pose));


        for(int i = 0; i<8; i++){
            int id = i+1;
            double x = aprilTagOriginX  -  Units.inchesToMeters(aprilTagYDiffsFromOriginInches[i]);
            double y = aprilTagOriginY - Units.inchesToMeters(aprilTagXDiffsFromOriginInches[i]);
//            SmartDashboard.putNumber("tagOriginX",aprilTagOriginX);
//            SmartDashboard.putNumber("tagOriginY",aprilTagOriginY);
//            SmartDashboard.putNumber("tagDiffX",Units.inchesToMeters(aprilTagXDiffsFromOriginInches[i]));
//            SmartDashboard.putNumber("tagDiffY",Units.inchesToMeters(aprilTagYDiffsFromOriginInches[i]));



            Rotation2d rotation2d = Rotation2d.fromDegrees(0);
            if(i>=4)
                rotation2d = Rotation2d.fromDegrees(180);

            fieldTags.add(new FieldTag(id, new Pose2d(x,y,rotation2d)));
        }

//        fieldTags.add(new FieldTag(1,new Pose2d()));
//        fieldTags.add(new FieldTag(2,new Pose2d(5,0, new Rotation2d())));
//        fieldTags.add(new FieldTag(3,new Pose2d(0,5, new Rotation2d())));
    pathingTelemetrySub.updateFieldTags(fieldTags);
    }




    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        for(PhotonTrackedTarget target :camera.getLatestResult().getTargets()){
            int id = target.getFiducialId();
            Transform3d transform = target.getBestCameraToTarget();
            Rotation3d rotation = transform.getRotation();

            for(int i = 0; i<fieldTags.size(); i++){
                if(fieldTags.get(i).getID() != id) continue;
                Transform3d sentTransform = new Transform3d(new Translation3d(transform.getX()*-1,transform.getY()*-1,transform.getZ()),transform.getRotation());
                swerveSubsystem.fieldTagSpotted(fieldTags.get(i), transform, camera.getLatestResult().getLatencyMillis(), target.getPoseAmbiguity());
      //          SmartDashboard.putNumber("lastPoseAmbiguity",camera.getLatestResult().getBestTarget().getPoseAmbiguity());

            }

//      SmartDashboard.putNumber("tag"+id+".X", transform.getX());
//      SmartDashboard.putNumber("tag"+id+".Y", transform.getY());
//      SmartDashboard.putNumber("tag"+id+".Z", transform.getZ());
//
//      SmartDashboard.putNumber("tag"+id+".rX", rotation.getX());
//      SmartDashboard.putNumber("tag"+id+".rY", rotation.getY());
//      SmartDashboard.putNumber("tag"+id+".rZ", rotation.getZ());


            double rotationFromCamera = Math.IEEEremainder(lastTransform.getRotation().getZ() + Math.PI, 2*Math.PI);
            // SmartDashboard.putNumber("rotationFromCamera", rotationFromCamera);


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

        out[0] = MathThings.absMax(out[0], 0.2);
        out[1] = MathThings.absMax(out[1], 0.2);
        if(Math.abs(out[2])<0.1) out[2] = 0;
        out[2] = MathThings.absMax(out[2], 0.05);


        return out;
    }

}
