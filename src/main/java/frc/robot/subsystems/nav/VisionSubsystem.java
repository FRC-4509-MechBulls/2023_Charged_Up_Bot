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
        if(result.getTargets().size() == 1){
            SmartDashboard.putString("visionAmbiguity",""+result.getTargets().get(0).getPoseAmbiguity());
            PhotonTrackedTarget target = result.getTargets().get(0);
            Transform3d transform = target.getBestCameraToTarget();
            Transform3d transformAlternative = target.getBestCameraToTarget();
            int id = target.getFiducialId();
            if(!(id>0 && id<=8)){
                SmartDashboard.putString("visionError","Invalid ID: "+id);
                return;
            }
            Pose2d botPose = tagPoseFromCameraToBotPose(fieldTags.get(id - 1), transform);
            Pose2d botPoseAlternative = tagPoseFromCameraToBotPose(fieldTags.get(id - 1), transformAlternative);


            if(result.getTargets().get(0).getPoseAmbiguity() <= Constants.VisionConstants.MAX_AMBIGUITY){
                //choose better option
                swerveSubsystem.getOdometry().addVisionMeasurement(botPose, result.getTimestampSeconds());
            }
            else {
                //choose option closest to current estimation
                Pose2d currentPose = swerveSubsystem.getOdometry().getEstimatedPosition();
                if (botPose.getTranslation().getDistance(currentPose.getTranslation()) < botPoseAlternative.getTranslation().getDistance(currentPose.getTranslation()))
                    swerveSubsystem.getOdometry().addVisionMeasurement(botPose, result.getTimestampSeconds());
                else
                    swerveSubsystem.getOdometry().addVisionMeasurement(botPoseAlternative, result.getTimestampSeconds());
            }
        }

        if(result.getTargets().size()>=2){
            double minDistance = Double.MAX_VALUE;
            ArrayList<PhotonTrackedTarget> targetsSorted = new ArrayList<PhotonTrackedTarget>();
            for(PhotonTrackedTarget target : result.getTargets())
                    targetsSorted.add(target);
            //sort targets by distance
            for(int i = 0; i<targetsSorted.size(); i++){
                for(int j = i+1; j<targetsSorted.size(); j++){
                    if(targetsSorted.get(i).getBestCameraToTarget().getTranslation().getDistance(new Translation3d()) > targetsSorted.get(j).getBestCameraToTarget().getTranslation().getDistance(new Translation3d())){
                        PhotonTrackedTarget temp = targetsSorted.get(i);
                        targetsSorted.set(i, targetsSorted.get(j));
                        targetsSorted.set(j, temp);
                    }
                }
            }

            if(!(targetsSorted.get(0).getFiducialId() > 0 && targetsSorted.get(0).getFiducialId() <= 8 && targetsSorted.get(1).getFiducialId() > 0 && targetsSorted.get(1).getFiducialId() <= 8)){
                SmartDashboard.putString("visionError","Invalid ID: "+targetsSorted.get(0).getFiducialId()+", "+targetsSorted.get(1).getFiducialId());
                return;
            }
            Pose2d target1Pose = tagPoseFromCameraToBotPose(fieldTags.get(targetsSorted.get(0).getFiducialId() - 1), targetsSorted.get(0).getBestCameraToTarget());
            Pose2d target1PoseAlternative = tagPoseFromCameraToBotPose(fieldTags.get(targetsSorted.get(0).getFiducialId() - 1), targetsSorted.get(0).getAlternateCameraToTarget());
            Pose2d target2Pose = tagPoseFromCameraToBotPose(fieldTags.get(targetsSorted.get(1).getFiducialId() - 1), targetsSorted.get(1).getBestCameraToTarget());
            Pose2d target2PoseAlternative = tagPoseFromCameraToBotPose(fieldTags.get(targetsSorted.get(1).getFiducialId() - 1), targetsSorted.get(1).getAlternateCameraToTarget());
            Pose2d[] target1Poses = {target1Pose,target1PoseAlternative};
            Pose2d[] target2Poses = {target2Pose,target2PoseAlternative};

            double minSolvedBotDist = Double.MAX_VALUE;
            Pose2d solvedBotPose = new Pose2d(1000,1000,Rotation2d.fromDegrees(0)); //something outlandish
            for(int i = 0; i<target1Poses.length; i++){
                for(int j = 0; j<target2Poses.length; j++){
                    double dist = target1Poses[i].getTranslation().getDistance(target2Poses[j].getTranslation());
                        if(dist<minSolvedBotDist){
                            minSolvedBotDist = dist;
                            solvedBotPose = new Pose2d((target1Poses[i].getX()+target2Poses[j].getX())/2, (target1Poses[i].getY()+target2Poses[j].getY())/2, Rotation2d.fromRadians((target1Poses[i].getRotation().getRadians()+target2Poses[j].getRotation().getRadians())/2));
                        }
                    }
                }
            swerveSubsystem.getOdometry().addVisionMeasurement(solvedBotPose, result.getTimestampSeconds());
            SmartDashboard.putString("visionAmbiguity","N/A");
            }

        }



    public Pose2d tagPoseFromCameraToBotPose(FieldTag fieldTag, Transform3d transform){
        //1. calculate X and Y position of camera based on X and Y components of tag and create a pose from that
        Rotation2d newRotation = new Rotation2d( ( Math.IEEEremainder((-transform.getRotation().getZ() - fieldTag.getPose().getRotation().getRadians()+4*Math.PI),2*Math.PI)));
        double newY = 0-( transform.getY()* Math.cos(-newRotation.getRadians()) + transform.getX() * Math.cos(Math.PI/2 - newRotation.getRadians())  ) + fieldTag.getPose().getY();
        double newX = 0- ( transform.getY()*Math.sin(-newRotation.getRadians()) + transform.getX() * Math.sin(Math.PI/2 - newRotation.getRadians())  ) + fieldTag.getPose().getX();
        Pose2d newPose = new Pose2d(newX,newY, newRotation);

        double camXOffset =Math.cos(newRotation.getRadians()+ Constants.VisionConstants.camDirFromCenter + Constants.VisionConstants.camHeading) * Constants.VisionConstants.camDistFromCenter;
        double camYOffset =  Math.sin(newRotation.getRadians() + Constants.VisionConstants.camDirFromCenter + Constants.VisionConstants.camHeading)* Constants.VisionConstants.camDistFromCenter;
        newX-=camXOffset;
        newY-=camYOffset;
        newRotation = Rotation2d.fromRadians(newRotation.getRadians() + Constants.VisionConstants.camHeading);

        //  SmartDashboard.putNumber("new_x", newX);
        // SmartDashboard.putNumber("new_y", newY);

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
