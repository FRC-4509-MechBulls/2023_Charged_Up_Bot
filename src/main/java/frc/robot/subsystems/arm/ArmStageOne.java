// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants;

public class ArmStageOne extends SubsystemBase {
  private TalonSRX armMotorPrimary;
  private TalonSRX armMotorSecondary;
  private DutyCycleEncoder magEncoder;
  private double encoderOffset = ArmConstants.kStageOne_AbsEncoderInitialOffset;
  private PIDController pid = new PIDController(ArmConstants.stageOne_kP,ArmConstants.stageOne_kI,ArmConstants.stageOne_kD);

  /** Creates a new ArmStageOne. */
  public ArmStageOne() {
    armMotorPrimary = new TalonSRX(ArmConstants.kStageOne_MotorLeftID); //left is master
    armMotorSecondary = new TalonSRX(ArmConstants.kStageOne_MotorRightID);

    armMotorPrimary.configFactoryDefault();
    armMotorSecondary.configFactoryDefault();

    armMotorPrimary.follow(armMotorSecondary);
    armMotorPrimary.setInverted(InvertType.OpposeMaster);

    magEncoder = new DutyCycleEncoder(ArmConstants.kStageOne_MagEncoderID);

    magEncoder.setDistancePerRotation(1);

  }

  public double getAbsoluteEncoderRad() {
    double angle = magEncoder.getAbsolutePosition(); //range 0-1
    angle *= Math.PI*2; //convert to radians
    angle += encoderOffset; //add the offset
    return angle * (ArmConstants.kStageOne_AbsEncoderReversed ? -1.0 : 1.0); //multiply -1 if reversed
  }


  public void limitSwitchPassed(){
    double encoderAng = magEncoder.getAbsolutePosition() * Math.PI * 2;
    encoderOffset =  ArmConstants.kStageOne_LimitSwitchAngleRad - encoderAng;
  }

  public void setArmPositionRad(double setpoint){
    pid.setSetpoint(setpoint);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armMotorPrimary.set(TalonSRXControlMode.PercentOutput,pid.calculate(getAbsoluteEncoderRad())); //replace this with internal PID
  }
}
