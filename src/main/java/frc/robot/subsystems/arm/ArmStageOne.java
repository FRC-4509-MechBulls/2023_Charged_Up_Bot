// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ArmConstants;

public class ArmStageOne extends SubsystemBase {
  private TalonSRX armMotorPrimary;
  private TalonSRX armMotorSecondary;
  private double encoderOffset = ArmConstants.kStageOne_AbsEncoderInitialOffset;
  private PIDController pid = new PIDController(ArmConstants.stageOne_kP,ArmConstants.stageOne_kI,ArmConstants.stageOne_kD);
  private double setpointRad = encoderOffset;

  /** Creates a new ArmStageOne. */
  public ArmStageOne() {
    armMotorPrimary = new TalonSRX(ArmConstants.kStageOne_MotorLeftID);
    armMotorSecondary = new TalonSRX(ArmConstants.kStageOne_MotorRightID);

    armMotorPrimary.configFactoryDefault(1000);
    armMotorSecondary.configFactoryDefault(1000);

    armMotorSecondary.follow(armMotorPrimary);
    armMotorSecondary.setInverted(InvertType.OpposeMaster);

    armMotorPrimary.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,1000);

    armMotorPrimary.config_kP(0,ArmConstants.stageOne_kP,30);
    armMotorPrimary.config_kI(0,ArmConstants.stageOne_kI,30);
    armMotorPrimary.config_kD(0,ArmConstants.stageOne_kD,30);

    armMotorPrimary.configClosedLoopPeriod(0, 1, 30);

    //magEncoder = new DutyCycleEncoder(ArmConstants.kStageOne_MagEncoderID);



  }

  public double getEncoderRad() {
    double angle = armMotorPrimary.getSelectedSensorPosition(); //range 0-1
    angle *= Math.PI*2; //convert to radians
    angle += encoderOffset; //add the offset
    return angle * (ArmConstants.kStageOne_AbsEncoderReversed ? -1.0 : 1.0); //multiply -1 if reversed
  }

  public double getSetpointRaw(){
    double angle = armMotorPrimary.getSelectedSensorPosition();
    angle -= encoderOffset/Math.PI/2;
    angle += setpointRad/Math.PI/2;
    return angle;
  }


  public void limitSwitchPassed(){
    double encoderAng = armMotorPrimary.getSelectedSensorPosition() * Math.PI * 2;
    encoderOffset =  ArmConstants.kStageOne_LimitSwitchAngleRad - encoderAng;
  }

  public void setArmPositionRad(double setpoint){
    pid.setSetpoint(setpoint);
    armMotorPrimary.set(TalonSRXControlMode.Position, getSetpointRaw());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //armMotorPrimary.set(TalonSRXControlMode.PercentOutput,pid.calculate(getAbsoluteEncoderRad())); //replace this with internal PID
  }
}
