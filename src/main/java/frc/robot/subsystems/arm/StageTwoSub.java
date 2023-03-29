// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotConstants;

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
  private boolean changedSides = false;
  private double lastAngle = 0;

  /** Creates a new ArmStageTwo. */
  public StageTwoSub() {
    
    SmartDashboard.putNumber("stageTwoP", ArmConstants.stageTwo_kP);
    SmartDashboard.putNumber("stageTwoI", ArmConstants.stageTwo_kI);
    SmartDashboard.putNumber("stageTwoD", ArmConstants.stageTwo_kD);
    SmartDashboard.putNumber("stageTwoRange", ArmConstants.stageTwoOutputRange);
    SmartDashboard.putNumber("stageTwoEfficiencyMultiplier", 1);

    /*
    SmartDashboard.putNumber("stageTwoAllowedError", 0.2);
    SmartDashboard.putNumber("stageTwoAccel", .6);
    SmartDashboard.putNumber("stageTwoVelocity", .6);
    */
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
    encoder.setZeroOffset(ArmConstants.stageTwoEncoderOffset);
  }
  private void configMotorControllers() {
    configMotorStatusFrames();
    armMotorPrimary.setSoftLimit(SoftLimitDirection.kForward, (float) (softLimitForward + Units.degreesToRadians(180)));
    armMotorPrimary.setSoftLimit(SoftLimitDirection.kReverse, (float) (softLimitReverse + Units.degreesToRadians(180)));
    armMotorPrimary.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotorPrimary.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotorPrimary.enableVoltageCompensation(RobotConstants.ROBOT_NOMINAL_VOLTAGE);
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
    armMotorSecondary.setSecondaryCurrentLimit(secondaryCurrentLimit);
    armMotorSecondary.setSmartCurrentLimit(smartCurrentLimit);
  }
  private void configPIDController() {
    pidController = armMotorPrimary.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(ArmConstants.stageTwo_kP, 0);
    pidController.setI(ArmConstants.stageTwo_kI, 0);
    pidController.setD(ArmConstants.stageTwo_kD, 0);
    pidController.setOutputRange(-ArmConstants.stageTwoOutputRange, ArmConstants.stageTwoOutputRange, 0);
    pidController.setPositionPIDWrappingEnabled(false);
    /*
    pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    pidController.setSmartMotionMaxAccel((Units.degreesToRadians(130/.6) * 60) / .6, 0);
    pidController.setSmartMotionMaxVelocity(Units.degreesToRadians(130/.6) * 60, 0);
    pidController.setSmartMotionAllowedClosedLoopError(Units.degreesToRotations(0.2), 0);
    */
  }
  private void configMotorStatusFrames() {
    //primary
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65521);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65519);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);
    armMotorPrimary.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 15);
    //secondary
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65521);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65519);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65497);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65479);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65449);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65447);
    armMotorSecondary.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65437);
    
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
    /*
    if (angle > setpoint && lastAngle < setpoint || angle < setpoint && lastAngle > setpoint) {
      changedSides = true;
    }
    else changedSides = false;
    */
    if (angle > setpoint + Units.degreesToRadians(ArmConstants.stageTwo_kIZone) || angle < setpoint - Units.degreesToRadians(ArmConstants.stageTwo_kIZone)) {
      pidController.setIAccum(0);
    }
    //lastAngle = angle;
    pidController.setReference(convertedSetpoint, CANSparkMax.ControlType.kPosition, 0, AFF * (1/SmartDashboard.getNumber("stageTwoEfficiencyMultiplier", 1)), ArbFFUnits.kVoltage);
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
    
    pidController.setP(SmartDashboard.getNumber("stageTwoP", ArmConstants.stageTwo_kP), 0);
    pidController.setI(SmartDashboard.getNumber("stageTwoI", ArmConstants.stageTwo_kI), 0);
    pidController.setD(SmartDashboard.getNumber("stageTwoD", ArmConstants.stageTwo_kD), 0);
    double outputRange = SmartDashboard.getNumber("stageTwoRange", ArmConstants.stageTwoOutputRange);
    pidController.setOutputRange(-outputRange, outputRange, 0);

    /*
    pidController.setSmartMotionAllowedClosedLoopError(Units.degreesToRadians(SmartDashboard.getNumber("stageTwoAllowedError", 0)), 0);
    pidController.setSmartMotionMaxAccel((Units.degreesToRadians(130/.6) * 60) / SmartDashboard.getNumber("stageTwoAccel", 0), 0);
    pidController.setSmartMotionMaxVelocity(Units.degreesToRadians(130/SmartDashboard.getNumber("stageTwoVelocity", 0)) * 60, 0);
    */
    /*
    SmartDashboard.putNumber("stageTwoISpark", pidController.getI(0));
    SmartDashboard.putNumber("stageTwoImaxaccumSpark", pidController.getIMaxAccum(0));
    SmartDashboard.putNumber("stageTwoIaccumSpark", pidController.getIAccum());
    SmartDashboard.putNumber("stageTwoIzoneSpark", pidController.getIZone(0));
    */
    //SmartDashboard.putBoolean("stageTwoLimitSwitch", limitSwitchValue);
    //SmartDashboard.putNumber("stageTwoVelocity", velocity);
  }
}