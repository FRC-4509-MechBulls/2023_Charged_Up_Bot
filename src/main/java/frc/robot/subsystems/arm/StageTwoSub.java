// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

import static frc.robot.Constants.ArmConstants;

public class StageTwoSub extends SubsystemBase {
  private CANSparkMax armMotorPrimary;
  private CANSparkMax armMotorSecondary;

  private SparkMaxPIDController pidController;
  private AbsoluteEncoder encoder;

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
  private double simulatedAngleRad = 0;

  /** Creates a new ArmStageTwo. */
  public StageTwoSub() {
    //SmartDashboard.putNumber("stageTwoP", ArmConstants.stageTwo_kP);
    SmartDashboard.putNumber("stageTwoI", ArmConstants.stageTwo_kI);
    instantiateConstants();
    instantiateMotorControllers();
    resetMotorControllers();
    instantiateEncoder();
    configEncoder();
    configMotorControllers();
    configPIDController();
    burnConfigs();
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
    encoder = armMotorPrimary.getAbsoluteEncoder(Type.kDutyCycle);
  }
  private void configEncoder() {
    encoder.setPositionConversionFactor((2 * Math.PI) / encoderRatio);
    encoder.setVelocityConversionFactor(((2 * Math.PI) / encoderRatio) / 60);
    encoder.setInverted(true);
    System.out.println(encoder.setZeroOffset(ArmConstants.stageTwoEncoderOffset));
  }
  private void configMotorControllers() {
    armMotorPrimary.setSoftLimit(SoftLimitDirection.kForward, (float) (softLimitForward + Units.degreesToRadians(180)));
    armMotorPrimary.setSoftLimit(SoftLimitDirection.kReverse, (float) (softLimitReverse + Units.degreesToRadians(180)));
    armMotorPrimary.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotorPrimary.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotorPrimary.enableVoltageCompensation(RobotConstants.ROBOT_NOMINAL_VOLTAGE);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    System.out.println(armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20));
    if (armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20) != REVLibError.kOk) {
      System.out.println(armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20));
    }
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
    pidController.setP(ArmConstants.stageTwo_kP, 0);
    if (pidController.setI(ArmConstants.stageTwo_kI, 0) != REVLibError.kOk) {
      System.out.println(pidController.setI(ArmConstants.stageTwo_kI, 0));
    }
    pidController.setD(ArmConstants.stageTwo_kD, 0);
    pidController.setOutputRange(-12,12, 0);
    pidController.setPositionPIDWrappingEnabled(false);
    pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    pidController.setSmartMotionMaxAccel((Units.degreesToRadians(130/1) * 60) / .25, 0);
    pidController.setSmartMotionMaxVelocity(Units.degreesToRadians(130/1) * 60, 0);
    //pidController.setSmartMotionAllowedClosedLoopError(Units.degreesToRotations(0.2), 0);
  }
  private void burnConfigs() {
    armMotorPrimary.burnFlash();
    armMotorSecondary.burnFlash();
  }
  //getters
  public double getLength() {
    return length;
  }
  public double getAngle() {
    if(Constants.SimulationConstants.simulationEnabled) return simulatedAngleRad;
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
  //setters
  public void setAFF(double AFF) {
    this.AFF = AFF;
  }
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }
  public double getVelocity() {
    return velocity;
  }
  //util
  private void calculateStageData() {
    angle = getEncoderPosition();
    velocity = getEncoderVelocity();
  }
  private void setArmPosition(){
    SmartDashboard.putNumber("stageTwoSet", Units.radiansToDegrees(setpoint));
    double convertedSetpoint = setpoint + Units.degreesToRadians(180);
    if (angle > setpoint + Units.degreesToRadians(2) || angle < setpoint - Units.degreesToRadians(2)) {
      pidController.setIAccum(0);
    }
    pidController.setReference(convertedSetpoint, CANSparkMax.ControlType.kSmartMotion, 0, AFF, ArbFFUnits.kVoltage);
  }
  private double getEncoderPosition() {
    return encoder.getPosition() - Units.degreesToRadians(180);
  }
  private double getEncoderVelocity() {
    return encoder.getVelocity();
  }
  double timeSinceLastSimUpdate = Timer.getFPGATimestamp();

  @Override
  public void periodic() {
    if(Constants.SimulationConstants.simulationEnabled){
      simulatedAngleRad += ((Timer.getFPGATimestamp() - timeSinceLastSimUpdate) * (setpoint - simulatedAngleRad) * Constants.SimulationConstants.armStageTwoSpeedMultiplier);
      timeSinceLastSimUpdate = Timer.getFPGATimestamp();
    }

    calculateStageData();
    setArmPosition();
    SmartDashboard.putNumber("stageTwoAngle", Units.radiansToDegrees(angle));
    if (pidController.getI() < SmartDashboard.getNumber("stageTwoI", ArmConstants.stageTwo_kI) * 0.9 || pidController.getI() > SmartDashboard.getNumber("stageTwoI", ArmConstants.stageTwo_kI) * 1.1) {
      System.out.println(pidController.setI(SmartDashboard.getNumber("stageTwoI", ArmConstants.stageTwo_kI), 0));
    }
    SmartDashboard.putNumber("stageTwoISpark", pidController.getI(0));
    SmartDashboard.putNumber("stageTwoImaxaccumSpark", pidController.getIMaxAccum(0));
    SmartDashboard.putNumber("stageTwoIaccumSpark", pidController.getIAccum());
    SmartDashboard.putNumber("stageTwoIzoneSpark", pidController.getIZone(0));
    //SmartDashboard.putBoolean("stageTwoLimitSwitch", limitSwitchValue);
    //SmartDashboard.putNumber("stageTwoVelocity", velocity);
  }
}