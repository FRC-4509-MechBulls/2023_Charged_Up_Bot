// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.IMUProtocol.YPRUpdate;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ArmConstants;

public class ArmStageOne extends SubsystemBase {
  private TalonSRX armMotorPrimary;
  private TalonSRX armMotorSecondary;
  private double encoderOffset = ArmConstants.kStageOne_AbsEncoderInitialOffset;
  private double setpointRad = encoderOffset;
  private double kCG[];
  private double kCB[];
  private double relativeCG[];
  private double relativeCB[];
  private double angle;

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
  //Config
  //Getters
  public void calculateStageData() {
    relativeCG = calculateRelativeCG();
    relativeCB = calculateRelativeCB();
  }
  public double[] calculateRelativeCB() {
    Rotation2d relativeCBAngle = new Rotation2d(new Rotation2d(kCB[0], kCB[1]).getRadians() + angle);
    double magnitude = Math.sqrt(Math.pow(kCB[0], 2) + Math.pow(kCB[1], 2));
    return new double[] {relativeCBAngle.getCos() * magnitude, relativeCBAngle.getSin() * magnitude, kCB[2] * (Math.hypot(kCB[4] - relativeCBAngle.getCos(), kCB[5] - relativeCBAngle.getSin()) - kCB[3]), new Rotation2d(kCB[4] - relativeCBAngle.getCos(), kCB[5] - relativeCBAngle.getSin()).getRadians() - relativeCBAngle.getRadians()};
  }
  public double[] calculateRelativeCG() {
    /*
    x, y -> angle
    angle + angle
    angle -> x, y
    x, y * magnitude
    */
    Rotation2d relativeCGAngle = new Rotation2d(new Rotation2d(kCG[0], kCG[1]).getRadians() + angle);
    double magnitude = Math.sqrt(Math.pow(kCG[0], 2) + Math.pow(kCG[1], 2));
    return new double[] {relativeCGAngle.getCos() * magnitude, relativeCGAngle.getSin() * magnitude, kCG[2]};
  }
  public double[] getRelativeCG() {
    return relativeCG;
  }
  public double[] getRelativeCB() {
    return relativeCB;
  }
  public double getEncoderRad() {
    return armMotorPrimary.getSelectedSensorPosition() * ArmConstants.kstageOne_encoderTicksToRadians;
  }
  //Setters
  public void limitSwitchPassed(){
    armMotorPrimary.setSelectedSensorPosition(ArmConstants.kStageOne_LimitSwitchAngleRad);
  }
  public void setArmPositionRad(double setpoint){
    armMotorPrimary.set(TalonSRXControlMode.Position, setpoint);
  }
  //Util

  @Override
  public void periodic() {
    calculateStageData();
    // This method will be called once per scheduler run
    //armMotorPrimary.set(TalonSRXControlMode.PercentOutput,pid.calculate(getAbsoluteEncoderRad())); //replace this with internal PID
  }
}
