// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort, 
          DriveConstants.kFrontLeftTurningMotorPort, 
          DriveConstants.kFrontLeftDriveEncoderReversed, 
          DriveConstants.kFrontLeftTurningEncoderReversed, 
          DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
          DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
          DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,"FL");
  
  private final SwerveModule frontRight = new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort, 
          DriveConstants.kFrontRightTurningMotorPort, 
          DriveConstants.kFrontRightDriveEncoderReversed, 
          DriveConstants.kFrontRightTurningEncoderReversed, 
          DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
          DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
          DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,"FR");

  private final SwerveModule backLeft = new SwerveModule(
          DriveConstants.kBackLeftDriveMotorPort, 
          DriveConstants.kBackLeftTurningMotorPort, 
          DriveConstants.kBackLeftDriveEncoderReversed, 
          DriveConstants.kBackLeftTurningEncoderReversed, 
          DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
          DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
          DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,"RL");


  private final SwerveModule backRight = new SwerveModule(
          DriveConstants.kBackRightDriveMotorPort, 
          DriveConstants.kBackRightTurningMotorPort, 
          DriveConstants.kBackRightDriveEncoderReversed, 
          DriveConstants.kBackRightTurningEncoderReversed, 
          DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
          DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
          DriveConstants.kBackRightDriveAbsoluteEncoderReversed,"RR");

  //private AHRS gyro = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
  private Pigeon2 gyro = new Pigeon2(0);
  

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
        //put in thread so it doesn't stop the rest of our code from running
        new Thread(() -> {
                try {
                        Thread.sleep(1000);
                        zeroHeading();
                } catch (Exception e) {
                }
        
        }).start();
        //allows gyro to calibrate for 1 sec before requesting to reset^^
  }

  //reset gyroscope to have it set the current direction as the forward direction of field when robot boots up
  public void zeroHeading() {
        gyro.zeroGyroBiasNow();
  }

  public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), 360); //clamps value between -/+ 180 deg
  }

  //since wpilib often wants heading in format of Rotation2d
  public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
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