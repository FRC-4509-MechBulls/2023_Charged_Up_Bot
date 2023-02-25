// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import static frc.robot.Constants.ArmConstants;

public class StageTwoSub extends SubsystemBase {
  private CANSparkMax armMotorPrimary;
  private CANSparkMax armMotorSecondary;

  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  private DigitalInput limitSwitch;

  private double setpoint;
  private double AFF;
  private double angle;
  private double length;
  private double springConstant;
  private double encoderRatio;
  private double mass;
  private double[] defaultCGCoordinateRelativeToPivot;
  private double[] defaultSpringStartCoordinateRelativeToPivot;
  private double[] defaultSpringEndCoordinateRelativeToPivot;
  private double restingSpringLength;
  private double voltsPerTorque;
  private double softLimitForward;
  private double softLimitReverse;
  private int smartCurrentLimit;
  private double secondaryCurrentLimit;
  private double velocity;
  private boolean limitSwitchValue;
  private boolean lastInLimitZone = true;

  /** Creates a new ArmStageTwo. */
  public StageTwoSub() {
    instantiateConstants();
    instantiateMotorControllers();
    resetMotorControllers();
    instantiateEncoder();
    configEncoder();
    configMotorControllers();
    configPIDController();
    burnConfigs();
    instantiateLimitSwitch();
  }
  //config
  private void instantiateConstants() {
    length = ArmConstants.stageTwoLength;
    springConstant = ArmConstants.stageTwoSpringConstant;
    encoderRatio = ArmConstants.stageTwoEncoderRatio;
    mass = ArmConstants.stageTwoMass;
    defaultCGCoordinateRelativeToPivot = ArmConstants.stageTwoDefaultCGCoordinateRelativeToPivot;
    defaultSpringStartCoordinateRelativeToPivot = ArmConstants.stageTwoDefaultSpringStartCoordinateRelativeToPivot;
    defaultSpringEndCoordinateRelativeToPivot = ArmConstants.stageTwoDefaultSpringEndCoordinateRelativeToPivot;
    restingSpringLength = ArmConstants.stageTwoRestingSpringLength;
    voltsPerTorque = ArmConstants.stageTwoOutputVoltsPerTorque;
    softLimitForward = ArmConstants.stageTwoSoftLimitForward;
    softLimitReverse = ArmConstants.stageTwoSoftLimitReverse;
    smartCurrentLimit = ArmConstants.stageTwoSmartCurrentLimit;
    secondaryCurrentLimit = ArmConstants.stageTwoSecondaryCurrentLimit;  
  }
  private void instantiateMotorControllers() {
    armMotorPrimary = new CANSparkMax(ArmConstants.stageTwoSparkLeftID, CANSparkMaxLowLevel.MotorType.kBrushless);
    armMotorSecondary = new CANSparkMax(ArmConstants.stageTwoSparkRightID, CANSparkMaxLowLevel.MotorType.kBrushless);
  }
  private void resetMotorControllers() {
    armMotorPrimary.restoreFactoryDefaults();
    armMotorSecondary.restoreFactoryDefaults();
    armMotorPrimary.setCANTimeout(1000);
    armMotorSecondary.setCANTimeout(1000);
  }
  private void instantiateEncoder() {
    encoder = armMotorPrimary.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,8192);
  }
  private void configEncoder() {
    encoder.setPositionConversionFactor((2 * Math.PI) / encoderRatio);
    encoder.setVelocityConversionFactor(((2 * Math.PI) / encoderRatio) / 60);
    encoder.setInverted(false);
    setSensorPosition(ArmConstants.stageTwoStartAngle);
  }
  private void configMotorControllers() {
    armMotorPrimary.setSoftLimit(SoftLimitDirection.kForward, (float) softLimitForward);
    armMotorPrimary.setSoftLimit(SoftLimitDirection.kReverse, (float) softLimitReverse);
    armMotorPrimary.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotorPrimary.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotorPrimary.enableVoltageCompensation(RobotConstants.ROBOT_NOMINAL_VOLTAGE);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    armMotorPrimary.setSecondaryCurrentLimit(secondaryCurrentLimit);
    armMotorPrimary.setSmartCurrentLimit(smartCurrentLimit);
    armMotorPrimary.setIdleMode(CANSparkMax.IdleMode.kCoast);
    armMotorSecondary.setIdleMode(CANSparkMax.IdleMode.kCoast);
    armMotorSecondary.enableVoltageCompensation(RobotConstants.ROBOT_NOMINAL_VOLTAGE);
    armMotorSecondary.follow(armMotorPrimary, true);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    armMotorSecondary.setSecondaryCurrentLimit(secondaryCurrentLimit);
    armMotorSecondary.setSmartCurrentLimit(smartCurrentLimit);
  }
  private void configPIDController() {
    pidController = armMotorPrimary.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(ArmConstants.stageTwo_kP);
    pidController.setI(ArmConstants.stageTwo_kI);
    pidController.setD(ArmConstants.stageTwo_kD);
    pidController.setOutputRange(-12,12);
    pidController.setPositionPIDWrappingEnabled(false);
  }
  private void burnConfigs() {
    armMotorPrimary.burnFlash();
    armMotorSecondary.burnFlash();
  }
  private void instantiateLimitSwitch() {
    limitSwitch = new DigitalInput(5);
  }
  //getters
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
  public double[] getdefaultCGCoordinateRelativeToPivot() {
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
  public boolean getLimitSwitchValue() {
    return limitSwitchValue;
  }
  //setters
  public void setAFF(double AFF) {
    this.AFF = AFF;
  }
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }
  public void setSensorPosition(double position) {
    encoder.setPosition(position);
  }
  public double getVelocity() {
    return velocity;
  }
  //util
  private void calculateStageData() {
    angle = getEncoderPosition();
    velocity = getEncoderVelocity();
    limitSwitchValue = getLimitSwitch();
  }
  private void setArmPosition(){
    pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0, AFF, ArbFFUnits.kVoltage);
  }
  private double getEncoderPosition() {
    return encoder.getPosition();
  }
  private double getEncoderVelocity() {
    return encoder.getVelocity();
  }
  private boolean getLimitSwitch() {
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    calculateStageData();
    setArmPosition();
    SmartDashboard.putNumber("stageTwoAngle", Units.radiansToDegrees(angle));
    SmartDashboard.putBoolean("stageTwoLimitSwitch", limitSwitchValue);
    SmartDashboard.putNumber("stageTwoVelocity", velocity);
    if (!getLimitSwitch()) {
      lastInLimitZone = false;
    }
    if(getLimitSwitch() && velocity > 0 && !lastInLimitZone) {
      setSensorPosition(ArmConstants.stageTwoLimitSwitchLeadingAngle);
      SmartDashboard.putNumber("stageTwoLSAngle", Units.radiansToDegrees(angle));
      lastInLimitZone = true;
    }/*
    if(getLimitSwitch() && velocity < 0 && !lastInLimitZone) {
      setSensorPosition(ArmConstants.stageTwoLimitSwitchTrailingAngle);
      lastInLimitZone = true;
    }*/
  }
}
