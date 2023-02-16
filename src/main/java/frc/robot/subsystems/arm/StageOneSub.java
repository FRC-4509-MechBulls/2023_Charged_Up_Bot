// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants;

public class StageOneSub extends SubsystemBase {
  private TalonSRX armMotorPrimary;
  private TalonSRX armMotorSecondary;
  private double setpointRad;
  private double kCG[];
  private double kCB[];
  private double cG[];
  private double angle;
  private double kTransmissionData[];
  private double AFF;
  private double kLength;
  private double kPivotCoordinate[];
  private boolean kRedirected;
  private double kSpringMountCoordinate[];
  private double kSpringRedirectCoordinate[];
  private double kSpringRestLength;
  private double kCBCoordinate[];
  private double kSpringConstant;
  private double kEncoderRatio;

  /** Creates a new ArmStageOne. */
  public StageOneSub() {
    kCG = ArmConstants.stageOneCG;
    kTransmissionData = ArmConstants.stageOneTransmissionData;
    kLength = ArmConstants.stageOneLength;
    kPivotCoordinate = ArmConstants.stageOnePivotCoordinate;
    kRedirected = ArmConstants.stageOneRedirected;
    kSpringMountCoordinate = ArmConstants.stageOneSpringMountCoordinate;
    kSpringRedirectCoordinate = ArmConstants.stageOneSpringRedirectCoordinate;
    kSpringRestLength = ArmConstants.stageOneSpringRestLength;
    kCBCoordinate = ArmConstants.stageOneCBCoordinate;
    kSpringConstant = ArmConstants.stageOneSpringConstant; 
    kEncoderRatio = ArmConstants.stageOneEncoderRatio; 

    armMotorPrimary = new TalonSRX(ArmConstants.STAGE_ONE_MOTOR_RIGHT_ID);
    armMotorSecondary = new TalonSRX(ArmConstants.STAGE_ONE_MOTOR_LEFT_ID);

    armMotorPrimary.configFactoryDefault(1000);
    armMotorSecondary.configFactoryDefault(1000);

    armMotorPrimary.setNeutralMode(NeutralMode.Coast);
    armMotorSecondary.setNeutralMode(NeutralMode.Coast);

    armMotorSecondary.follow(armMotorPrimary);
    armMotorSecondary.setInverted(InvertType.OpposeMaster);

    armMotorPrimary.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,1000);

    setSensorPosition(ArmConstants.stageOneStartAngle);

    armMotorPrimary.config_kP(0,ArmConstants.stageOne_kP,1000);
    armMotorPrimary.config_kI(0,ArmConstants.stageOne_kI,1000);
    armMotorPrimary.config_kD(0,ArmConstants.stageOne_kD,1000);

    armMotorPrimary.configClosedLoopPeriod(0, 1, 1000);
  }
  //Config
  //Getters
  public void calculateStageData() {
    double ticks = getEncoder();
    angle = calculateAngle(ticks);
    SmartDashboard.putNumber("stageOneAngle", angle);

    cG = calculateCG();
  }
  public double[] getCG() {
    return cG;
  }
  public double getLength() {
    return kLength;
  }
  public double[] getTransmissionData() {
    return kTransmissionData;
  }
  public double[] getPivotCoordinate() {
    return kPivotCoordinate;
  }
  public double getAngle() {
    return angle;
  }
  public boolean getRedirected() {
    return kRedirected;
  }
  public double[] getSpringMountCoordinate() {
    return kSpringMountCoordinate;
  }
  public double[] getSpringRedirectCoordinate() {
    return kSpringRedirectCoordinate;
  }
  public double getSpringRestLength() {
    return kSpringRestLength;
  }
  public double[] getkCBCoordinate() {
    return kCBCoordinate;
  }
  public double getSpringConstant() {
    return kSpringConstant;
  }
  //Setters
  public void setEncoderPosition(double position){
    armMotorPrimary.setSelectedSensorPosition(position);
  }
  public void setAFF(double AFF){
    this.AFF = AFF;
  }
  public void setArmPositionRad(double setpoint){
    armMotorPrimary.set(TalonSRXControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, (AFF/12));
  }
  //Util
  private void setSensorPosition(double position) {
    armMotorPrimary.setSelectedSensorPosition(position * ArmConstants.STAGE_ONE_ENCODER_TICKS_TO_RADIANS * kEncoderRatio);
  }
  private double getEncoder() {
    return armMotorPrimary.getSelectedSensorPosition();
  }
  private double calculateAngle(double encoder){
    double ticks = encoder;
    double outputTicks = ticks / kEncoderRatio;
    angle = calculateRadiansFromTicks(outputTicks);
    return angle;
  }
  private double calculateRadiansFromTicks(double ticks) {
    double radians = ticks / ArmConstants.STAGE_ONE_ENCODER_TICKS_TO_RADIANS;
    return radians;
  }
  private double[] calculateCG() {
    /*
    x, y -> angle
    angle + angle
    angle -> x, y
    x, y * magnitude
    */
    Rotation2d cGAngle = new Rotation2d(new Rotation2d(kCG[0], kCG[1]).getRadians() + angle);
    double magnitude = Math.sqrt(Math.pow(kCG[0], 2) + Math.pow(kCG[1], 2));
    return new double[] {cGAngle.getCos() * magnitude, cGAngle.getSin() * magnitude, kCG[2]};
  }

  @Override
  public void periodic() {
    calculateStageData();
    setArmPositionRad(setpointRad);
    SmartDashboard.putNumber("stageOneAFF", AFF);
    // This method will be called once per scheduler run
  }
}
