// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConstants;

public class SwerveSubsystem extends SubsystemBase {
  //Modules
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
  //Gyro
    private WPI_Pigeon2 gyro;
  //Odometry
    private SwerveDrivePoseEstimator odometry;
    private Pose2d initialPose;
    private ChassisSpeeds chassisSpeeds;
  //Values
    static boolean fieldOriented;
  //Dashboard
    //Tabs
      private ShuffleboardTab tabSwerveSubsystem;
    //Entries
    private NetworkTableEntry dashboardOdometryHeading;
    private NetworkTableEntry dashboardOdometryY;
    private NetworkTableEntry dashboardOdometryX;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //pose
      initialPose = new Pose2d();
    //Modules
      frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, 
                                   DriveConstants.kFrontLeftTurningMotorPort, 
                                   DriveConstants.kFrontLeftDriveEncoderReversed, 
                                   DriveConstants.kFrontLeftTurningEncoderReversed, 
                                   DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
                                   DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
                                   DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
      frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, 
                                    DriveConstants.kFrontRightTurningMotorPort, 
                                    DriveConstants.kFrontRightDriveEncoderReversed, 
                                    DriveConstants.kFrontRightTurningEncoderReversed, 
                                    DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
                                    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
                                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
      backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, 
                                  DriveConstants.kBackLeftTurningMotorPort, 
                                  DriveConstants.kBackLeftDriveEncoderReversed, 
                                  DriveConstants.kBackLeftTurningEncoderReversed, 
                                  DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
                                  DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
                                  DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
      backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, 
                                   DriveConstants.kBackRightTurningMotorPort, 
                                   DriveConstants.kBackRightDriveEncoderReversed, 
                                   DriveConstants.kBackRightTurningEncoderReversed, 
                                   DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
                                   DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
                                   DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    //gyro
      gyro = new WPI_Pigeon2(DriveConstants.kPigeonPort);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 3001, 1000);      
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 3003, 1000); 
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 3011, 1000); 
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 3017, 1000);           
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 3023, 1000);
      gyro.configAllSettings(Robot.ctreConfigs.gyro, 1000);
      gyro.configMountPose(AxisDirection.NegativeY, AxisDirection.PositiveZ, 1000);                       
      zeroHeading();                                                               
    //Odometry
      constructOdometry(); //custructs odometry with newly corrct gyro values
    //Dashboard
      tabSwerveSubsystem = Shuffleboard.getTab("SwerveSubsystem");
    //dashboard
      debugInit(); //initialize debug outputs
  }

  //Configuration
    public void zeroHeading() { //reset gyroscope to have it set the current direction as the forward direction of field when robot boots up
      gyro.setYaw(initialPose.getRotation().getDegrees(), 1000);
    }
    public void constructOdometry() { //constructs odometry object
      odometry = new SwerveDrivePoseEstimator(getRotation2d(), 
      initialPose, 
      DriveConstants.kDriveKinematics, 
      DriveConstants.kSDOdo, 
      DriveConstants.kSDState, 
      DriveConstants.kSDVision, 
      RobotConstants.kMainLoopPeriod);  
    }
  //Getters
    //A number equal to x - (y Q), where Q is the quotient of x / y rounded to the nearest integer
    //(if x / y falls halfway between two integers, the even integer is returned)
    public double getHeading() {
          return Math.IEEEremainder(gyro.getYaw(), 360); //clamps value between -/+ 180 deg where zero is forward
    }
    public Rotation2d getRotation2d() { //since wpilib often wants heading in format of Rotation2d
      return Rotation2d.fromDegrees(getHeading());
    }
    public double getAngularVelocity() { //get rotational velocity for closed loop
        return -gyro.getRate() * DriveConstants.kDegreesToRadians;
    }
    public SwerveModuleState[] getStates() { //module states
      return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    }
    public ChassisSpeeds getChassisSpeeds() { //chassis speeds
      chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
      return fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, getRotation2d()) :
                             chassisSpeeds;
    }
  //Setters 
    public void fieldOriented(boolean isFieldOriented) {
      fieldOriented = isFieldOriented;
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //normalizes wheel speeds in case max speed reached
      frontLeft.setDesiredState(desiredStates[0]);
      frontRight.setDesiredState(desiredStates[1]);
      backLeft.setDesiredState(desiredStates[2]);
      backRight.setDesiredState(desiredStates[3]);
    }
    public void stopModules() {
      frontLeft.stop();
      frontRight.stop();
      backLeft.stop();
      backRight.stop();
    }
    public void updateOdometry() {
      odometry.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getStates());
    }
    public void updateOdometryVision(Pose2d visionRobotPoseMeters, double timestampSeconds, Vector<N3> visionMeasurementStdDevs) {
      odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
    //Dashboard
    //Debugging
      //Odometry
        public void debugOdometryInit () {
          //Odometry
            dashboardOdometryHeading = tabSwerveSubsystem.add("OdometryHeading", odometry.getEstimatedPosition().getRotation().getDegrees()).getEntry();
            dashboardOdometryY = tabSwerveSubsystem.add("OdometryY", odometry.getEstimatedPosition().getY()).getEntry();
            dashboardOdometryX = tabSwerveSubsystem.add("OdometryX", odometry.getEstimatedPosition().getX()).getEntry();
        }
        public void debugOdometryPeriodic () {
          //Odometry
            dashboardOdometryHeading.setDouble(odometry.getEstimatedPosition().getRotation().getDegrees());
            dashboardOdometryY.setDouble(odometry.getEstimatedPosition().getY());
            dashboardOdometryX.setDouble(odometry.getEstimatedPosition().getX());
        }
      //Other
        public void debugInit() {
          //Odometry
            debugOdometryInit();
        }
        public void debugPeriodic() {
          //Chassis Speeds
            //debug output: tabSwerveSubsystem.add("CSH", getChassisSpeeds().omegaRadiansPerSecond);
            //debug output: tabSwerveSubsystem.add("CSY", getChassisSpeeds().vyMetersPerSecond);
            //debug output: tabSwerveSubsystem.add("CSX", getChassisSpeeds().vxMetersPerSecond);
          //Odometry
            debugOdometryPeriodic();
          //Gryo
            //debug output: tabSwerveSubsystem.add("gyroH", getHeading());
        }

    @Override
    public void periodic() {
      //update odometry
        updateOdometry();
      //dashboard outputs
        debugPeriodic();
    }        
}