// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConstants;

/** Add your docs here. */
public class CTREConfigs {
  public TalonFXConfiguration swerveDriveMotor;
  public TalonFXConfiguration swerveTurnMotor;
  public Pigeon2Configuration gyro;

  public CTREConfigs() {
    swerveDriveMotor = new TalonFXConfiguration();
    swerveTurnMotor = new TalonFXConfiguration();
    gyro = new Pigeon2Configuration();

    configSwerveDriveMotor();
    configSwerveTurnMotor();
    configGyro();
  }

  //Setters
  public void configSwerveDriveMotor() {
    swerveDriveMotor.slot0.kF = ModuleConstants.kFDrive;
    swerveDriveMotor.slot0.kP = ModuleConstants.kPDrive;
    swerveDriveMotor.voltageCompSaturation = RobotConstants.ROBOT_NOMINAL_VOLTAGE;
    swerveDriveMotor.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    swerveDriveMotor.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveDriveMotor.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 40, 40, 0);
    swerveDriveMotor.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 45, 55, 50);
    swerveDriveMotor.neutralDeadband = ModuleConstants.NEUTRAL_DEADBAND;
  }
  public void configSwerveTurnMotor() {
    swerveTurnMotor.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    swerveTurnMotor.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveTurnMotor.slot0.kP = ModuleConstants.kPTurning;
    swerveTurnMotor.voltageCompSaturation = RobotConstants.ROBOT_NOMINAL_VOLTAGE;
    swerveTurnMotor.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 40, 40, 0);
    swerveTurnMotor.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 50);
    swerveTurnMotor.neutralDeadband = ModuleConstants.NEUTRAL_DEADBAND;
  }
  public void configGyro() {
    gyro.ZAxisGyroError = DriveConstants.GYRO_Z_ERROR;
    gyro.MountPosePitch = DriveConstants.GYRO_MOUNT_POSE_PITCH;
    gyro.MountPoseYaw = DriveConstants.GYRO_MOUNT_POSE_YAW;
    gyro.MountPoseRoll = DriveConstants.GYRO_MOUNT_POSE_ROLL;
  }
}
