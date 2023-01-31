// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ArmConstants;

public class ArmStageTwo extends SubsystemBase {
  private CANSparkMax armMotorPrimary;
  private CANSparkMax armMotorSecondary;

  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  private double encoderOffset = ArmConstants.kStageTwo_AbsEncoderInitialOffset;
  private double setpointRad = encoderOffset;
  private double AFF;

  /** Creates a new ArmStageTwo. */
  public ArmStageTwo() {
    armMotorPrimary = new CANSparkMax(ArmConstants.kStageTwo_MotorLeftChannel, CANSparkMaxLowLevel.MotorType.kBrushless); //"right" motor
    armMotorSecondary = new CANSparkMax(ArmConstants.kStageTwo_MotorRightChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

    encoder =  armMotorPrimary.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,8192); //the Alternate Encoder is automatically configured when the Alternate Encoder object is instantiated

    armMotorPrimary.restoreFactoryDefaults();
    armMotorSecondary.restoreFactoryDefaults();


    armMotorSecondary.follow(armMotorPrimary);
    armMotorSecondary.setInverted(true);

    pidController = armMotorPrimary.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(ArmConstants.stageTwo_kP);
    pidController.setI(ArmConstants.stageTwo_kI);
    pidController.setD(ArmConstants.stageTwo_kD);
    pidController.setOutputRange(-1,1);




  }
  public void calculateStageData() {
    relativeCG = calculateRelativeCG();
  }
  public double[] calculateRelativeCG() {
    /*
    x, y -> angle
    angle + angle
    angle -> x, y
    x, y * magnitude
    */
    Rotation2d relativeCGAngle = new Rotation2d(new Rotation2d(kcG[0], kcG[1]).getRadians() + angle);
    double magnitude = Math.sqrt(Math.pow(kcG[0], 2) + Math.pow(kcG[1], 2));
    return new double[] {relativeCGAngle.getCos() * magnitude, relativeCGAngle.getSin() * magnitude, kcG[2]};
  }
  public double[] getRelativeCG() {
    return relativeCG;
  }
  public void setAFF(double AFF) {
    this.AFF = AFF;
  }
  public double getEncoderRad() {
    return armMotorPrimary.getEncoder().getPosition() * ArmConstants.kstageTwo_encoderTicksToRadians;
  }


  public void limitSwitchPassed(){
    armMotorPrimary.getEncoder().setPosition(ArmConstants.kStageTwo_LimitSwitchAngleRad / Math.PI / 2.0);
  }

  public void setArmPositionRad(double setpoint){
    pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //armMotorPrimary.set(TalonSRXControlMode.PercentOutput,pid.calculate(getAbsoluteEncoderRad())); //replace this with internal PID
  }
}
