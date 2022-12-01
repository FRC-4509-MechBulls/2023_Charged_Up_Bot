// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConstants;

/** Add your docs here. */
public class CTREConfigs {
  public TalonFXConfiguration swerveDriveMotor;
  public TalonFXConfiguration swerveTurnMotor;

  public CTREConfigs() {
    swerveDriveMotor = new TalonFXConfiguration();
    swerveTurnMotor = new TalonFXConfiguration();

    configSwerveDriveMotor();
    configSwerveTurnMotor();
  }

  //Setters
  public void configSwerveDriveMotor() {
    swerveDriveMotor.slot0.kF = ModuleConstants.kFDrive;
    swerveDriveMotor.slot0.kP = ModuleConstants.kPDrive;
    swerveDriveMotor.voltageCompSaturation = RobotConstants.kRobotNominalVoltage;
    swerveDriveMotor.neutralDeadband = 0.01;
  }
  public void configSwerveTurnMotor() {
    swerveTurnMotor.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveTurnMotor.slot0.kP = ModuleConstants.kPTurning;
    swerveTurnMotor.voltageCompSaturation = RobotConstants.kRobotNominalVoltage;
  }
}
