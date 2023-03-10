// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalance. */
  SwerveSubsystem swerveSubsystem;
  double startTime;
  double timeLimit;

  public AutoBalanceCommand(SwerveSubsystem swerveSubsystem, double timeLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    startTime = Timer.getFPGATimestamp();
    this.timeLimit = timeLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.driveAutoBalance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime < timeLimit;
  }
}
