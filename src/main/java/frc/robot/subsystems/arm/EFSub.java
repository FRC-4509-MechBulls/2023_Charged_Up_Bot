// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;

import static frc.robot.Constants.EndEffectorConstants.HOLD_CUBE_TOP_OUTPUT;

public class EFSub extends SubsystemBase {
  private TalonSRX efMotorTop;
  private TalonSRX efMotorBottom;

  private double[] cGCoordinateRelativeToPivot;
  private double mass;

  /** Creates a new EndEffectorSubsystem. */
  public EFSub() {
    efMotorTop = new TalonSRX(EndEffectorConstants.EF_MOTOR_TOP_ID);
    efMotorBottom = new TalonSRX(EndEffectorConstants.EF_MOTOR_BOTTOM_ID);
    configMotorStatusFrames();

    // setting motors to brake mode so they immediately stop moving when voltage stops being sent
    efMotorTop.setNeutralMode(NeutralMode.Coast);
    efMotorBottom.setNeutralMode(NeutralMode.Coast);

    efMotorBottom.setInverted(false);
    efMotorTop.setInverted(true);
    
    // limits power going to motor to prevent burnout
    // values need to be changed
    efMotorTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      true, // enabled
      10, // Limit (amp)
      10, // Trigger Threshold (amp)
      0)); // Trigger Threshold Time(s)

    // limits power going to motor to prevent burnout
    // values need to be changed
    efMotorTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      true, // enabled
      10, // Limit (amp)
      10, // Trigger Threshold (amp)
      0)); // Trigger Threshold Time(s)
    
      cGCoordinateRelativeToPivot = ArmConstants.eFCGCoordinateRelativeToPivot;
      mass = ArmConstants.eFMass;
  }
  public void configMotorStatusFrames() {
    //top
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 197, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 199, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 211, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 223, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 227, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 229, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 241, 1000);
    efMotorTop.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 251, 1000);
    //bottom
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 197, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 199, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 211, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 223, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 227, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 229, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 241, 1000);
    efMotorBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 251, 1000);
  }

  public void calculateStageData() {
  }
  public double[] getCGCoordinateRelativeToPivot() {
    return cGCoordinateRelativeToPivot;
  }
  public double getMass() {
    return mass;
  }

  public void intakeCone() {
    // sets motors to run opposite of each other at set percent output
    // intakes cone into the end effector
    // the direction of the motors might need to be reversed, we'll see
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.INTAKE_CONE_TOP_OUTPUT);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.INTAKE_CONE_BOTTOM_OUTPUT);
  }

  public void intakeCube() {
    // sets motors to run in the same direction at set percent output
    // intakes cube into the end effector
    efMotorTop.set(TalonSRXControlMode.PercentOutput,  EndEffectorConstants.INTAKE_CUBE_TOP_OUTPUT);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.INTAKE_CUBE_BOTTOM_OUTPUT);
  }

  public void holdCone() {
    // sets motors at enough percent output to hold cone in end effector
    // motors run in opposite directions
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.HOLD_CONE_TOP_OUTPUT);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.HOLD_CONE_BOTTOM_OUTPUT);
  }

  public void holdCube() {
    // sets motors at enough percent output to hold cube in end effector
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.HOLD_CUBE_TOP_OUTPUT);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.HOLD_CUBE_BOTTOM_OUTPUT);
  }

  public void placeCone() {
    // reverses direction of motors to place cone
    // motors run in opposite directions
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.PLACE_CONE_TOP_OUTPUT);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.PLACE_CONE_BOTTOM_OUTPUT);
  }

  public void placeCubeBottom() {
    // reverses direction of motors to place cube
    efMotorTop.set(TalonSRXControlMode.PercentOutput, -EndEffectorConstants.PLACE_CUBE_TOP_OUTPUT);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, -EndEffectorConstants.PLACE_CUBE_BOTTOM_OUTPUT);
  }
  public void placeCubeTop() {
    // reverses direction of motors to place cube
    efMotorTop.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.PLACE_CUBE_TOP_OUTPUT);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, EndEffectorConstants.PLACE_CUBE_BOTTOM_OUTPUT);
  }

  public void stopMotors() {
    efMotorTop.set(TalonSRXControlMode.PercentOutput, 0);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    calculateStageData();
    // This method will be called once per scheduler run
  }
}
