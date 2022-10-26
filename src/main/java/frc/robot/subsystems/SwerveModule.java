// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  private final TalonFX driveMotor; //why can't i make these private (sobbing emoji)
  private final TalonFX turningMotor;

  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private final PIDController turningPidController;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
          int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    //absolute encoder
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);
    
    //motors
    driveMotor = new TalonFX(driveMotorId);
    turningMotor = new TalonFX(turningMotorId);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    //set conversion constants
    
    turningPidController = new  PIDController(ModuleConstants.kPTurning, 0, 0); //proportional control is enough
    turningPidController.enableContinuousInput(-Math.PI, Math.PI); //tells PID that system is circular

    resetEncoders(); //resets encoders when the robot boots up
  }

  public double getDrivePosition() {
    return driveMotor.getSelectedSensorPosition();
  }

  public double getTurningPosition() {
    return turningMotor.getSelectedSensorPosition();
  }

  public double getDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity();
  }

  public double getTurningVelocity() {
    return turningMotor.getSelectedSensorVelocity();
  }

  public double getAbsoluteEncoderRad() {
    //divides voltage reading by amount of voltage we are supplying it -> gives us how many percent of a full rotation it is reading
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI; //converts to radians
    angle -= absoluteEncoderOffsetRad; //subtracts the offset to get the actual wheel angles
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0); //multiply -1 if reversed
  }

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0); //reset drive motor encoder to 0
    turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad()); //resets turning motor encoder to absolute encoder value
    //makes it so the turning motor wheels are in line with the actual angle
  }

  //wpi lib requests info in form of swerve module state, so this method converts it
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    //prevents wheels from going to OG pos when joysticks are not moved
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle); //makes it so wheel never turns more than 90 deg

    //DO I USE VELOCITY OR PERCENT OUTPUT???
    driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //scales vel down using max speed
    turningMotor.set(TalonFXControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    //^^^calculates output for the angle setpoint and current pos
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString()); //debugging info
  }

  public void stop() {
    driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    turningMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
