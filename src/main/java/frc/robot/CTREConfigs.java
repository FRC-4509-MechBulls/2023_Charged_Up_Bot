// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
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
    swerveDriveMotor.voltageCompSaturation = RobotConstants.kRobotNominalVoltage;
    swerveDriveMotor.primaryPID = new BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor);
    swerveDriveMotor.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveDriveMotor.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 40, 40, 0);
    swerveDriveMotor.supplyCurrLimit = new SupplyCurrentLimitConfiguration(false, 40, 40, 0);
    swerveDriveMotor.neutralDeadband = 0.01;
  }
  public void configSwerveTurnMotor() {
    swerveTurnMotor.primaryPID = new BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor);
    swerveTurnMotor.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveTurnMotor.slot0.kP = ModuleConstants.kPTurning;
    swerveTurnMotor.voltageCompSaturation = RobotConstants.kRobotNominalVoltage;
    swerveTurnMotor.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 40, 40, 0);
    swerveTurnMotor.supplyCurrLimit = new SupplyCurrentLimitConfiguration(false, 40, 40, 0);
    swerveTurnMotor.neutralDeadband = 0.01;
  }
  public void configGyro() {
    gyro.ZAxisGyroError = DriveConstants.kGyroZError;
    gyro.MountPosePitch = DriveConstants.kGyroMountPosePitch;
    gyro.MountPoseYaw = DriveConstants.kGyroMountPoseYaw;
    gyro.MountPoseRoll = DriveConstants.kGyroMountPoseRoll;
  }
}
