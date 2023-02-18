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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import static frc.robot.Constants.ArmConstants;

public class StageTwoSub extends SubsystemBase {
  private CANSparkMax armMotorPrimary;
  private CANSparkMax armMotorSecondary;

  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  private double setpoint;
  private double AFF;
  private double angle;
  private double length;
  private double springConstant;
  private double kEncoderRatio;
  private double mass;
  private double[] defaultCGCoordinateRelativeToPivot;
  private double[] defaultSpringStartCoordinateRelativeToPivot;
  private double[] defaultSpringEndCoordinateRelativeToPivot;
  private double restingSpringLength;
  private double voltsPerTorque;

  /** Creates a new ArmStageTwo. */
  public StageTwoSub() {
    length = ArmConstants.stageTwoLength;
    springConstant = ArmConstants.stageTwoSpringConstant;
    kEncoderRatio = ArmConstants.stageTwoEncoderRatio;
    mass = ArmConstants.stageTwoMass; 
    defaultCGCoordinateRelativeToPivot = ArmConstants.stageTwoDefaultCGCoordinateRelativeToPivot;
    defaultSpringStartCoordinateRelativeToPivot = ArmConstants.stageTwoDefaultSpringStartCoordinateRelativeToPivot;
    defaultSpringEndCoordinateRelativeToPivot = ArmConstants.stageTwoDefaultSpringEndCoordinateRelativeToPivot;
    restingSpringLength = ArmConstants.stageTwoRestingSpringLength;
    voltsPerTorque = ArmConstants.stageTwoOutputVoltsPerTorque;

    armMotorPrimary = new CANSparkMax(ArmConstants.STAGE_TWO_MOTOR_LEFT_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless); //"right" motor
    //armMotorPrimary.clearFaults();
    armMotorSecondary = new CANSparkMax(ArmConstants.STAGE_TWO_MOTOR_RIGHT_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
    //armMotorSecondary.clearFaults();
    armMotorPrimary.setCANTimeout(2000);
    armMotorSecondary.setCANTimeout(2000);

    armMotorPrimary.restoreFactoryDefaults();
    armMotorSecondary.restoreFactoryDefaults();
    armMotorPrimary.setCANTimeout(2000);
    armMotorSecondary.setCANTimeout(2000);

    encoder = armMotorPrimary.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,8192); //the Alternate Encoder is automatically configured when the Alternate Encoder object is instantiated
    encoder.setPositionConversionFactor((2 * Math.PI) / kEncoderRatio);

    armMotorPrimary.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRadians(-10));
    armMotorPrimary.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.degreesToRadians(-170));
    armMotorPrimary.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotorPrimary.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotorPrimary.enableVoltageCompensation(RobotConstants.ROBOT_NOMINAL_VOLTAGE);
    armMotorSecondary.enableVoltageCompensation(RobotConstants.ROBOT_NOMINAL_VOLTAGE);
    armMotorSecondary.follow(armMotorPrimary, true);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    //armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    //armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    armMotorPrimary.setSecondaryCurrentLimit(60);
    armMotorSecondary.setSecondaryCurrentLimit(60);
    armMotorPrimary.setSmartCurrentLimit(40);
    armMotorSecondary.setSmartCurrentLimit(40);
    armMotorPrimary.setIdleMode(CANSparkMax.IdleMode.kCoast);
    armMotorSecondary.setIdleMode(CANSparkMax.IdleMode.kCoast);

    setSensorPosition(ArmConstants.stageTwoStartAngle);

    pidController = armMotorPrimary.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(ArmConstants.stageTwo_kP);
    pidController.setI(ArmConstants.stageTwo_kI);
    pidController.setD(ArmConstants.stageTwo_kD);
    pidController.setOutputRange(-12,12);
    pidController.setPositionPIDWrappingEnabled(false);

    armMotorPrimary.burnFlash();
    armMotorSecondary.burnFlash();
  }

  private void calculateStageData() {
    double ticks = getEncoder();
    angle = calculateAngle(ticks);
    SmartDashboard.putNumber("stageTwoAngle", angle);
  }
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
  

  public void setAFF(double AFF) {
    this.AFF = AFF;
  }
  void setArmPositionRad(double setpoint){
    //pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0, AFF, ArbFFUnits.kVoltage);
    //pidController.setReference(Units.degreesToRadians(-ArmConstants.stageOneStartAngle), CANSparkMax.ControlType.kPosition, 0, AFF, ArbFFUnits.kVoltage);
    pidController.setReference(ArmConstants.stageTwoStartAngle, CANSparkMax.ControlType.kPosition, 0, AFF, ArbFFUnits.kVoltage);
  }
  private double getEncoder() {
    return encoder.getPosition();
  }
  private double calculateAngle(double encoder){
    return encoder;
  }
  private void setSensorPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public void periodic() {
    calculateStageData();
    setArmPositionRad(setpoint);
    SmartDashboard.putNumber("stageTwoAFF", AFF);
    // This method will be called once per scheduler run
    //armMotorPrimary.set(TalonSRXControlMode.PercentOutput,pid.calculate(getAbsoluteEncoderRad())); //replace this with internal PID
  }
}
