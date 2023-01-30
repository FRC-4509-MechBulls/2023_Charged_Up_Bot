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

  private double encoderOffset = ArmConstants.kStageTwo_AbsEncoderInitialOffset;
  private PIDController pid = new PIDController(ArmConstants.stageTwo_kP,ArmConstants.stageTwo_kI,ArmConstants.stageTwo_kD);
  private double setpointRad = encoderOffset;

  /** Creates a new ArmStageTwo. */
  public ArmStageTwo() {
    armMotorPrimary = new CANSparkMax(ArmConstants.kStageTwo_MotorLeftChannel, CANSparkMaxLowLevel.MotorType.kBrushless); //"right" motor
    armMotorSecondary = new CANSparkMax(ArmConstants.kStageTwo_MotorRightChannel, CANSparkMaxLowLevel.MotorType.kBrushless);


    armMotorPrimary.restoreFactoryDefaults();
    armMotorSecondary.restoreFactoryDefaults();


    armMotorPrimary.follow(armMotorSecondary);
    armMotorPrimary.setInverted(true);

    pidController = armMotorPrimary.getPIDController();
    pidController.setP(ArmConstants.stageTwo_kP);
    pidController.setI(ArmConstants.stageTwo_kI);
    pidController.setD(ArmConstants.stageTwo_kD);
    pidController.setOutputRange(-1,1);




  }

  public double getAbsoluteEncoderRad() {
    double angle = armMotorPrimary.getEncoder().getPosition(); //range 0-1
    angle *= Math.PI*2; //convert to radians
    angle += encoderOffset; //add the offset
    return angle * (ArmConstants.kStageTwo_AbsEncoderReversed ? -1.0 : 1.0); //multiply -1 if reversed
  }

  public double getSetpointRaw(){
    double angle = armMotorPrimary.getEncoder().getPosition();
    angle -= encoderOffset/Math.PI/2;
    angle += setpointRad/Math.PI/2;
    return angle;
  }


  public void limitSwitchPassed(){
    double encoderAng = armMotorPrimary.getEncoder().getPosition() * Math.PI * 2;
    encoderOffset =  ArmConstants.kStageTwo_LimitSwitchAngleRad - encoderAng;
  }

  public void setArmPositionRad(double setpoint){
    pid.setSetpoint(setpoint);
    pidController.setReference(getSetpointRaw(), CANSparkMax.ControlType.kPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //armMotorPrimary.set(TalonSRXControlMode.PercentOutput,pid.calculate(getAbsoluteEncoderRad())); //replace this with internal PID
  }
}
