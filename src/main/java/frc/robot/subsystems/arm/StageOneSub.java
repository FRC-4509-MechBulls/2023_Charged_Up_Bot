// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import static frc.robot.Constants.ArmConstants;

public class StageOneSub extends SubsystemBase {
  private TalonSRX armMotorPrimary;
  private TalonSRX armMotorSecondary;
  private double setpoint;
  private double angle;
  private double AFF;
  private double length;
  private double springConstant;
  private double encoderRatio;
  private double mass;
  private double[] defaultCGCoordinateRelativeToPivot;
  private double[] defaultSpringStartCoordinateRelativeToPivot;
  private double[] defaultSpringEndCoordinateRelativeToPivot;
  private double restingSpringLength;
  private double voltsPerTorque;
  private double[] pivotCoordinate;
  private double softLimitForward;
  private double softLimitReverse;
  private double continuousCurrentLimit;
  private double peakCurrentLimit;
  private double peakCurrentTime;

  /** Creates a new ArmStageOne. */
  public StageOneSub() {
    instantiateConstants();
    instantiateMotorControllers();
    resetMotorControllers();
    configMotorControllers();
    configEncoder();
  }
  //Config
  private void instantiateConstants() {
    length = ArmConstants.stageOneLength;
    springConstant = ArmConstants.stageOneSpringConstant; 
    encoderRatio = ArmConstants.stageOneEncoderRatio; 
    mass = ArmConstants.stageOneMass; 
    defaultCGCoordinateRelativeToPivot = ArmConstants.stageOneDefaultCGCoordinateRelativeToPivot;
    defaultSpringStartCoordinateRelativeToPivot = ArmConstants.stageOneDefaultSpringStartCoordinateRelativeToPivot;
    defaultSpringEndCoordinateRelativeToPivot = ArmConstants.stageOneDefaultSpringEndCoordinateRelativeToPivot;
    restingSpringLength = ArmConstants.stageOneRestingSpringLength;
    voltsPerTorque = ArmConstants.stageOneOutputVoltsPerTorque;
    pivotCoordinate = ArmConstants.stageOnePivotCoordinate;
    softLimitForward = ArmConstants.stageOneSoftLimitForward;
    softLimitReverse = ArmConstants.stageOneSoftLimitReverse;
    continuousCurrentLimit = ArmConstants.stageOneContinuousCurrentLimit;
    peakCurrentLimit = ArmConstants.stageOnePeakCurrentLimit;
    peakCurrentTime = ArmConstants.stageOnePeakCurrentTime;
  }
  private void instantiateMotorControllers() {
    armMotorPrimary = new TalonSRX(ArmConstants.stageTwoSparkRightID);
    armMotorSecondary = new TalonSRX(ArmConstants.stageTwoSparkLeftID);
  }
  private void resetMotorControllers() {
    armMotorPrimary.configFactoryDefault(1000);
    armMotorSecondary.configFactoryDefault(1000);
  }
  private void configMotorControllers() {
    //current limit
    armMotorPrimary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, continuousCurrentLimit, peakCurrentLimit, peakCurrentTime), 1000);
    //soft limit
    armMotorPrimary.configForwardSoftLimitThreshold(softLimitForward, 1000);
    armMotorPrimary.configReverseSoftLimitThreshold(softLimitReverse, 1000);
    armMotorPrimary.configForwardSoftLimitEnable(true, 1000);
    armMotorPrimary.configReverseSoftLimitEnable(true, 1000);
    //encoder
    armMotorPrimary.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,1000);
    armMotorPrimary.configSelectedFeedbackCoefficient(ArmConstants.stageOneEncoderTicksToRadians * encoderRatio);
    //PID
    armMotorPrimary.config_kP(0,ArmConstants.stageOne_kP,1000);
    armMotorPrimary.config_kI(0,ArmConstants.stageOne_kI,1000);
    armMotorPrimary.config_kD(0,ArmConstants.stageOne_kD,1000);
    armMotorPrimary.configClosedLoopPeriod(0, 1, 1000);
    //voltage compensation
    armMotorPrimary.configVoltageCompSaturation(RobotConstants.ROBOT_NOMINAL_VOLTAGE, 1000);
    armMotorPrimary.enableVoltageCompensation(true);
    //Neutral Mode
    armMotorPrimary.setNeutralMode(NeutralMode.Coast);
    //second motor
    armMotorSecondary.follow(armMotorPrimary);
    armMotorSecondary.setInverted(InvertType.OpposeMaster);
    armMotorSecondary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, continuousCurrentLimit, peakCurrentLimit, peakCurrentTime), 1000);
    armMotorSecondary.configVoltageCompSaturation(RobotConstants.ROBOT_NOMINAL_VOLTAGE, 1000);
    armMotorSecondary.enableVoltageCompensation(true);
    armMotorSecondary.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000, 1000);
    armMotorSecondary.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, 1000);
    armMotorSecondary.setNeutralMode(NeutralMode.Coast);
  }
  private void configEncoder() {
    setSensorPosition(ArmConstants.stageOneStartAngle);
  }
  //Getters
  public double getLength() {
    return length;
  }
  public double getAngle() {
    return angle;
  }
  public double getSpringConstant() {
    return springConstant;
  }
  public double getMass() {
    return mass;
  }
  public double[] getDefaultCGCoordinateRelativeToPivot() {
    return defaultCGCoordinateRelativeToPivot;
  }
  public double[] getDefaultSpringStartCoordinateRelativeToPivot() {
    return defaultSpringStartCoordinateRelativeToPivot;
  }
  public double[] getDefaultSpringEndCoordinateRelativeToPivot() {
    return defaultSpringEndCoordinateRelativeToPivot;
  }
  public double getRestingSpringLength() {
    return restingSpringLength;
  }
  public double getVoltsPerTorque() {
    return voltsPerTorque;
  }
  public double[] getPivotCoordinate() {
    return pivotCoordinate;
  }
  //Setters
  public void setAFF(double AFF){
    this.AFF = AFF;
  }
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }
  public void setSensorPosition(double position){
    armMotorPrimary.setSelectedSensorPosition(position);
  }
  //Util
  public void calculateStageData() {
    angle = getEncoder();
    SmartDashboard.putNumber("stageOneAngle", angle);
  }
  private void setArmPosition(){
    armMotorPrimary.set(TalonSRXControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, (AFF/12));
  }
  private double getEncoder() {
    return armMotorPrimary.getSelectedSensorPosition();
  }
  @Override
  public void periodic() {
    calculateStageData();
    setArmPosition();
  }
}
