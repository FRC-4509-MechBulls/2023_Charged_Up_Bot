// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
  //Modules
    private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, 
                                                            DriveConstants.kFrontLeftTurningMotorPort, 
                                                            DriveConstants.kFrontLeftDriveEncoderReversed, 
                                                            DriveConstants.kFrontLeftTurningEncoderReversed, 
                                                            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
                                                            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
                                                            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, 
                                                            DriveConstants.kFrontRightTurningMotorPort, 
                                                            DriveConstants.kFrontRightDriveEncoderReversed, 
                                                            DriveConstants.kFrontRightTurningEncoderReversed, 
                                                            DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
                                                            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
                                                            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    private final SwerveModule backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, 
                                                          DriveConstants.kBackLeftTurningMotorPort, 
                                                          DriveConstants.kBackLeftDriveEncoderReversed, 
                                                          DriveConstants.kBackLeftTurningEncoderReversed, 
                                                          DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
                                                          DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
                                                          DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, 
                                                            DriveConstants.kBackRightTurningMotorPort, 
                                                            DriveConstants.kBackRightDriveEncoderReversed, 
                                                            DriveConstants.kBackRightTurningEncoderReversed, 
                                                            DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
                                                            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
                                                            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
  //Gyro
    private WPI_Pigeon2 gyro = new WPI_Pigeon2(0);

  //Odometry
    private SwerveDrivePoseEstimator odometry;
    private Pose2d initialPose;
    private ChassisSpeeds chassisSpeeds;

  //Values
    static boolean fieldOriented;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    initialPose = new Pose2d();
    odometry = new SwerveDrivePoseEstimator(getRotation2d(), 
    initialPose, 
    DriveConstants.kDriveKinematics, 
    //VecBuilder.fill(0.5, 0.5, 5 * DriveConstants.kDegreesToRadians), 
    //VecBuilder.fill(0.01 * DriveConstants.kDegreesToRadians), 
    //VecBuilder.fill(0.5, 0.5, 30 * DriveConstants.kDegreesToRadians), 
    VecBuilder.fill(Units.feetToMeters(.5), Units.feetToMeters(.5), 10 * DriveConstants.kDegreesToRadians), 
    VecBuilder.fill(1 * DriveConstants.kDegreesToRadians), 
    VecBuilder.fill(1000000, 100000, 1000000), 
    0.02);

        //put in thread so it doesn't stop the rest of our code from running
        new Thread(() -> {
                try {
                        Thread.sleep(1000);
                        gyro.configFactoryDefault();
                        gyro.configMountPose(AxisDirection.NegativeY, AxisDirection.PositiveZ);
                        zeroHeading();
                } catch (Exception e) {
                }
        
        }).start();
        //allows gyro to calibrate for 1 sec before requesting to reset^^
  }

  //reset gyroscope to have it set the current direction as the forward direction of field when robot boots up
  public void zeroHeading() {
        gyro.zeroGyroBiasNow();
        gyro.setYaw(0);
  }

  //A number equal to x - (y Q), where Q is the quotient of x / y rounded to the nearest integer
  //(if x / y falls halfway between two integers, the even integer is returned)
  public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), 360); //clamps value between -/+ 180 deg where zero is forward
  }

  //module states
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
  }

  //chassis speeds
  public ChassisSpeeds getChassisSpeeds() {
    chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
    return fieldOriented ? 
    ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, getRotation2d()) :
    chassisSpeeds;
  }

  //get rotational velocity for closed loop
  public double getAngularVelocity() {
      return -gyro.getRate() * DriveConstants.kDegreesToRadians;
  }

  //since wpilib often wants heading in format of Rotation2d
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    //update odometry
      odometry.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getStates());
      //debug output: 
      SmartDashboard.putNumber("OdoH", odometry.getEstimatedPosition().getRotation().getDegrees());
      //debug output: 
      SmartDashboard.putNumber("gyroH", getHeading());
      //debug output: 
      SmartDashboard.putNumber("OdoY", Units.metersToFeet(odometry.getEstimatedPosition().getY()));
      //debug output: 
      SmartDashboard.putNumber("OdoX", Units.metersToFeet(odometry.getEstimatedPosition().getX()));
    SmartDashboard.putNumber("CSH", getChassisSpeeds().omegaRadiansPerSecond);
    SmartDashboard.putNumber("CSY", getChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("CSX", getChassisSpeeds().vxMetersPerSecond);
  }

  public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
  }

  public void fieldOriented(boolean isFieldOriented) {
    fieldOriented = isFieldOriented;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //normalizes wheel speeds in case max speed reached^^
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
  }
        
}