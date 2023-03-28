// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Robot;

public class SwerveModule extends SubsystemBase {
  //motors
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;

  //abs encoder
  private final DutyCycleEncoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  //Values
	private double delta = 0;
	private double deltaConverted = 0;
	private double setAngle = 0;
  private double timestamp = Timer.getFPGATimestamp();

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    //absolute encoder
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
    absoluteEncoder.setDistancePerRotation(1);
    
    //motors
      //drive
    driveMotor = new WPI_TalonFX(driveMotorId);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 15, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 199, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 211, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 223, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 227, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 229, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 241, 1000);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 251, 1000);

    driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveMotor, 1000);
    driveMotor.setNeutralMode(NeutralMode.Coast);
    driveMotor.setInverted(driveMotorReversed);
      //debug output: driveMotor.config_kF(0, 0);
      //debug output: driveMotor.config_kP(0, 0);
      //turn
    turningMotor = new WPI_TalonFX(turningMotorId);    
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 15, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 199, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 211, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 223, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 227, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 229, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 241, 1000);
    turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 251, 1000);
    
    turningMotor.configAllSettings(Robot.ctreConfigs.swerveTurnMotor, 1000);
    turningMotor.setInverted(turningMotorReversed);
    while (Timer.getFPGATimestamp() < timestamp + 2) {}
    turningMotor.setNeutralMode(NeutralMode.Coast);
    //both
      enableVoltageCompensation(true);  
      //initialize encoders in thread so they don't timeout
      new Thread(() -> {
        try {
              Thread.sleep(1000);
              resetEncoders();
        } catch (Exception e) {}
      }).start();
		
		//Dashboard
		//Debug output: SmartDashboard.putNumber("kPT", ModuleConstants.kPTurning);
    //Debug output: SmartDashboard.putNumber("kDT", ModuleConstants.kDTurning);
    //Debug output: SmartDashboard.putNumber("kPDrive", ModuleConstants.kPDrive);
    //Debug output: SmartDashboard.putNumber("kAFFDrive", ModuleConstants.kAFFDrive);
  }

  //Configuration
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0); //reset drive motor encoder to 0
    turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad() * ModuleConstants.RADIANS_TO_TURNING); //resets turning motor encoder to absolute encoder value
  }
  public void enableVoltageCompensation(boolean onOff) {
    driveMotor.enableVoltageCompensation(onOff);
    turningMotor.enableVoltageCompensation(onOff);
  } 

  //Getters
  public Rotation2d getTurningPosition() {
    return new Rotation2d(turningMotor.getSelectedSensorPosition() / ModuleConstants.RADIANS_TO_TURNING);
  }
  public double getDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity() / ModuleConstants.METERS_TO_DRIVE_VELOCITY; //convert raw sensor units to m/s
  }
  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getAbsolutePosition(); //range 0-1
    angle *= ModuleConstants.ABS_TO_RADIANS; //converts to radians
    angle += absoluteEncoderOffsetRad; //subtracts the offset to get the actual wheel angles
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0); //multiply -1 if reversed
  }

  public SwerveModuleState getState() { //wpi lib requests info in form of swerve module state, so this method converts it
    return new SwerveModuleState(getDriveVelocity(), getTurningPosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      driveMotor.getSelectedSensorPosition() / ModuleConstants.METERS_TO_DRIVE,
      new Rotation2d(turningMotor.getSelectedSensorPosition() / ModuleConstants.RADIANS_TO_TURNING)
    );
  }

  //Setters
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) { //prevents wheels from going to OG pos when joysticks are not moved
      driveMotor.set(TalonFXControlMode.Velocity, 0);
      return;
    }
    //Debug output: SmartDashboard.putNumber("preOpRadians" + absoluteEncoder.getSourceChannel(), state.angle.getRadians());
    state = SwerveModuleState.optimize(state, getState().angle); //makes it so wheel never turns more than 90 deg
		delta = state.angle.getRadians() - getTurningPosition().getRadians(); //error
		deltaConverted = delta % Math.PI; //error converted to representative of the actual gap; error > pi indicates we aren't taking the shortest route to setpoint, but rather doing one or more 180* rotations.this is caused by the discontinuity of numbers(pi is the same location as -pi, yet -pi is less than pi)
		setAngle = Math.abs(deltaConverted) < (Math.PI / 2) ? getTurningPosition().getRadians() + deltaConverted : getTurningPosition().getRadians() - ((deltaConverted/Math.abs(deltaConverted)) * (Math.PI-Math.abs(deltaConverted))); //makes set angle +/- 1/2pi of our current position(capable of pointing all directions)

    //Debug intput: driveMotor.config_kP(0, SmartDashboard.getNumber("kPDrive", ModuleConstants.kPDrive));
    driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond * ModuleConstants.METERS_TO_DRIVE_VELOCITY, DemandType.ArbitraryFeedForward, (state.speedMetersPerSecond/Math.abs(state.speedMetersPerSecond)) * ModuleConstants.kAFFDrive); //velocity control
    //Debug output: driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond * ModuleConstants.METERS_TO_DRIVE_VELOCITY, DemandType.ArbitraryFeedForward, (state.speedMetersPerSecond/Math.abs(state.speedMetersPerSecond)) * SmartDashboard.getNumber("kAFFDrive", ModuleConstants.kAFFDrive)); //velocity control
    turningMotor.set(TalonFXControlMode.Position, setAngle * ModuleConstants.RADIANS_TO_TURNING); //Position Control
    
    //Debug output: SmartDashboard.putNumber("stateAngle" + absoluteEncoder.getSourceChannel(), getState().angle.getRadians());
    //Debug output: SmartDashboard.putString("Swerve[" + absoluteEncoder.getSourceChannel() + "] state", state.toString());
    //Debug output: SmartDashboard.putNumber("setRadians" + absoluteEncoder.getSourceChannel(), state.angle.getRadians());
    //Debug output: SmartDashboard.putNumber("setT" + absoluteEncoder.getSourceChannel(), turningMotor.getClosedLoopTarget());
    //Debug output: SmartDashboard.putNumber("voltageT" + absoluteEncoder.getSourceChannel(), turningMotor.getMotorOutputVoltage());
  }
  public void stop() { //sets both voltage outputs to 0
    driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    turningMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
		//Debug output: turningMotor.config_kP(0, SmartDashboard.getNumber("kPT", ModuleConstants.kPTurning));
    //Debug input: turningMotor.config_kD(0, SmartDashboard.getNumber("kDT", ModuleConstants.kDTurning));
		//Debug output: SmartDashboard.putBoolean("absPos"+this.turningMotor.getDeviceID(), absoluteEncoder.isConnected());
    //Debug output: SmartDashboard.putNumber("relRadians" + absoluteEncoder.getSourceChannel(), getTurningPosition() * ModuleConstants.RADIANS_TO_TURNING);
    //Debug output: SmartDashboard.putNumber("absRadians" + absoluteEncoder.getSourceChannel(), getAbsoluteEncoderRad());
    //Debug output: SmartDashboard.putNumber("abs0-1" + absoluteEncoder.getSourceChannel(), absoluteEncoder.getAbsolutePosition());
    //Debug output: SmartDashboard.putNumber(this.name+".sDrivePos",getDrivePosition());
    //Debug output: SmartDashboard.putNumber("errorT" + absoluteEncoder.getSourceChannel(), turningMotor.getClosedLoopError());
    //Debug output: SmartDashboard.putNumber("deltaC" + absoluteEncoder.getSourceChannel(), deltaConverted);
    //Debug output: SmartDashboard.putNumber("Voltd" + absoluteEncoder.getSourceChannel(), driveMotor.getMotorOutputVoltage());
    //Debug output: SmartDashboard.putNumber("Vd" + absoluteEncoder.getSourceChannel(), getDriveVelocity());
    //Debug output: SmartDashboard.putNumber("Sd" + absoluteEncoder.getSourceChannel(), driveMotor.getClosedLoopTarget() / ModuleConstants.METERS_TO_DRIVE_VELOCITY);
  }
}
