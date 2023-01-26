// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  TalonSRX efMotorTop;
  TalonSRX efMotorBottom;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    efMotorTop = new TalonSRX(EndEffectorConstants.kEFMotorTopID);
    efMotorBottom = new TalonSRX(EndEffectorConstants.kEFMotorBottomID);

    // setting motors to brake mode so they immediately stop moving when voltage stops being sent
    efMotorTop.setNeutralMode(NeutralMode.Brake);
    efMotorBottom.setNeutralMode(NeutralMode.Brake);
    
    // limits power going to motor to prevent burnout
    // values need to be changed
    efMotorTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      true, // enabled
      10, // Limit (amp)
      15, // Trigger Threshold (amp)
      0.5)); // Trigger Threshold Time(s)

    // limits power going to motor to prevent burnout
    // values need to be changed
    efMotorTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      true, // enabled
      10, // Limit (amp)
      15, // Trigger Threshold (amp)
      0.5)); // Trigger Threshold Time(s)
  }

  public void intakeCone() {
    // sets motors to run opposite of each other at set percent output
    // intakes cone into the end effector
    // the direction of the motors might need to be reversed, we'll see
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kIntakeConeOutput);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, -EndEffectorConstants.kIntakeConeOutput);
  }

  public void intakeCube() {
    // sets motors to run in the same direction at set percent output
    // intakes cube into the end effector
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kIntakeCubeOutput);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kIntakeCubeOutput);
  }

  public void holdCone() {
    // sets motors at enough percent output to hold cone in end effector
    // motors run in opposite directions
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kHoldConeOutput);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, -EndEffectorConstants.kHoldConeOutput);
  }

  public void holdCube() {
    // sets motors at enough percent output to hold cube in end effector
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kHoldCubeOutput);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kHoldCubeOutput);
  }

  public void placeCone() {
    // reverses direction of motors to place cone
    // motors run in opposite directions
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kPlaceConeOutput);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, -EndEffectorConstants.kPlaceConeOutput);
  }

  public void placeCube() {
    // reverses direction of motors to place cube
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kPlaceCubeOutput);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.kPlaceCubeOutput);
  }

  public void stopMotors() {
    efMotorTop.set(TalonSRXControlMode.PercentOutput, 0);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
