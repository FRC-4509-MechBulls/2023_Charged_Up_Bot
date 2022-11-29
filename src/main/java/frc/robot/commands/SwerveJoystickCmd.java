// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction; //whether user wants command to be field oriented
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private double rawMagnitudeTranslation = 0;
  private double scaledMagnitudeTranslation = 0;
  private double directionTranslation[] = {0, 0};
  private double rawMagnitudeRotation = 0;
  private double scaledMagnitudeRotation = 0;
  private double directionRotation = 0;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get()*-1;
    double turningSpeed = turningSpdFunction.get()*-1;

    // 2. Apply deadband
    //debug output: SmartDashboard.putNumber("inputX", xSpeed);
    //debug output: SmartDashboard.putNumber("inputY", ySpeed);
    //debug output: SmartDashboard.putNumber("inputT", turningSpeed);
    //raw inputs
    rawMagnitudeTranslation = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)); //magnitude of joystick input
    directionTranslation[0] = xSpeed/rawMagnitudeTranslation; //x component of raw input (THESE ARE NOT 1/-1)
    directionTranslation[1] = ySpeed/rawMagnitudeTranslation;
    //scaling
    scaledMagnitudeTranslation = (1/(1-OIConstants.kDeadband))*(rawMagnitudeTranslation-OIConstants.kDeadband); //converted to be representative of deadzone using point slope form (y-y1)=(m)(x-x1) -> (scaled-minimun raw input)=(maximum input/(1-deadzone))(raw input-deadzone) -> scaled=(maximum input/(1-deadzone))(raw input-deadzone)
    if (rawMagnitudeTranslation > OIConstants.kDeadband) {
      xSpeed = directionTranslation[0] * scaledMagnitudeTranslation; //original direction, scaled magnitude... this is kinda a misnomer because this x component itself is a magnitude, but it is representative of a direction of the raw input
      ySpeed = directionTranslation[1] * scaledMagnitudeTranslation;
    } else {
      xSpeed = 0.0;
      ySpeed = 0.0; //zero tiny inputs
    }
    //raw inputs
    rawMagnitudeRotation = Math.abs(turningSpeed); //magnitude of joystick input
    directionRotation = turningSpeed/rawMagnitudeRotation; //conveys polarity +/-
    SmartDashboard.putNumber("directionR", directionRotation);
    //scaling
    scaledMagnitudeRotation = (1/(1-OIConstants.kDeadband))*(rawMagnitudeRotation-OIConstants.kDeadband); //same algorithm as scaled magnitude above
    if (Math.abs(turningSpeed) > OIConstants.kDeadband) {
      turningSpeed = directionRotation * scaledMagnitudeRotation; //same as above for translation
    } else turningSpeed = 0.0; //zero tiny inputs

    // 2.5 square inputs
    xSpeed = xSpeed * Math.abs(xSpeed);
    ySpeed = ySpeed * Math.abs(ySpeed);
    turningSpeed = turningSpeed * Math.abs(turningSpeed);

    // 3. Make the driving smoother, no sudden acceleration from sudden inputs
    xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
    //debug output: SmartDashboard.putNumber("xspeed", xSpeed);
    //debug output: SmartDashboard.putNumber("turningspeed", turningSpeed);

    // 4. Construct desired chassis speeds (convert to appropriate reference frames)
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
