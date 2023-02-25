// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction; //whether user wants command to be field oriented
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private double TranslationMagnitude;
  private double TranslationMagnitudeScaled;
  private double rotationMagnitude = 0;
  private double scaledMagnitudeRotation = 0;
  private PIDController turningPID = new PIDController(DriveConstants.kPTurning, 0, DriveConstants.kDTurning);

  Rotation2d translationDirection;
  Rotation2d rotationDirection;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    this.yLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
    addRequirements(swerveSubsystem);

    //dashboard
    //Debug output: SmartDashboard.putNumber("kPTurning", DriveConstants.kPTurning);
    //Debug output: SmartDashboard.putNumber("kPFudge", DriveConstants.kPFudge);
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
      //debug output: SmartDashboard.putNumber("inputX", xSpeed);
      //debug output: SmartDashboard.putNumber("inputY", ySpeed);
      //debug output: SmartDashboard.putNumber("inputT", turningSpeed);
    
    //1.5 interpret joystick data
      //translation
        TranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)); //x and y vectors to polar magnitude
        translationDirection = new Rotation2d(ySpeed, xSpeed); //creates Rotation2D representing the direction of the input vectors using yspeed/xspeed as the cos/sin of the direction
      //Rotation
        rotationMagnitude = Math.abs(turningSpeed); //magnitude of joystick input
        rotationDirection = new Rotation2d(turningSpeed, 0); //conveys polarity +/-
        //Debug output: SmartDashboard.putNumber("directionR", rotationDirection.getCos());

    //1.55 scale magnitudes to reflect deadzone
      //Translation
        TranslationMagnitudeScaled = (1/(1-OIConstants.DEADBAND))*(TranslationMagnitude-OIConstants.DEADBAND); //converted to be representative of deadzone using point slope form (y-y1)=(m)(x-x1) -> (scaled-minimun raw input)=(maximum input/(1-deadzone))(raw input-deadzone) -> scaled=(maximum input/(1-deadzone))(raw input-deadzone)
      //Rotation
        scaledMagnitudeRotation = (1/(1-OIConstants.DEADBAND))*(rotationMagnitude-OIConstants.DEADBAND); //same algorithm as scaled magnitude translation

    //2.0 deadzone and construct outputs
      //Translation
        if (TranslationMagnitude > OIConstants.DEADBAND) {
          ySpeed = translationDirection.getCos() * TranslationMagnitudeScaled; //original direction, scaled magnitude... this is kinda a misnomer because this x component itself is a magnitude, but it is representative of a direction of the raw input
          xSpeed = translationDirection.getSin() * TranslationMagnitudeScaled;
        } else {
          xSpeed = 0.0;
          ySpeed = 0.0; //zero inputs < deadzone
        }
      //Rotation
        if (rotationMagnitude > OIConstants.DEADBAND) {
          turningSpeed = rotationDirection.getCos() * scaledMagnitudeRotation; //same as above for translation
        } else {
          turningSpeed = 0.0; //zero inputs < deadzone
        }

    /*
    // 2.5 square inputs //not needed for now, only needed it when there was a bug elsewhere
    xSpeed = xSpeed * Math.abs(xSpeed);
    ySpeed = ySpeed * Math.abs(ySpeed);
    turningSpeed = turningSpeed * Math.abs(turningSpeed);
    */

    // 3. Make the driving smoother, no sudden acceleration from sudden inputs
      xSpeed = xLimiter.calculate(xSpeed * DriveConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
      ySpeed = yLimiter.calculate(ySpeed * DriveConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
      turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
      //debug output: SmartDashboard.putNumber("xspeed", xSpeed);
      //debug output: SmartDashboard.putNumber("turningspeed", turningSpeed);

    //3.5. Fudge Factor to eliminate uncommanded change in direction when translating and rotating simultaneously
      ySpeed += turningSpeed * (-xSpeed) * DriveConstants.kPFudge;
      //debug output: ySpeed += turningSpeed * (-xSpeed) * SmartDashboard.getNumber("kPFudge", DriveConstants.kPFudge);
      xSpeed += turningSpeed * ySpeed * DriveConstants.kPFudge;
      //debug output: xSpeed += turningSpeed * ySpeed * SmartDashboard.getNumber("kPFudge", DriveConstants.kPFudge);

    // 3.55. P loops to create accurate outputs
      //turning
        //Debug intput: turningPID.setP(SmartDashboard.getNumber("kPTurning", DriveConstants.kPTurning));
        turningSpeed += turningPID.calculate(swerveSubsystem.getAngularVelocity(), turningSpeed);
      //drive
        
      
      
    // 4. Construct desired chassis speeds (convert to appropriate reference frames)
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      swerveSubsystem.fieldOriented(true);
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else {
      swerveSubsystem.fieldOriented(false);
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
