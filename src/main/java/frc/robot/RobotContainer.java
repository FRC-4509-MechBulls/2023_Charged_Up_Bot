// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.lib.NavigationField;
import frc.robot.subsystems.GraphicalTelemetrySubsystem;
import frc.robot.subsystems.PathingTelemetrySub;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
  private final GraphicalTelemetrySubsystem pathingTelemSub = new PathingTelemetrySub();

  private final NavigationField navigationField = new NavigationField((PathingTelemetrySub) pathingTelemSub, swerveSubsystem);


  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Command rc_drive = new RunCommand(()-> swerveSubsystem.joystickDrive(driverController.getLeftY()*-1,driverController.getLeftX()*-1,driverController.getRightX()*-1), swerveSubsystem);
  private final Command swerve_toggleFieldOriented = new InstantCommand(swerveSubsystem::toggleFieldOriented);
  private final Command rc_goToTag = new RunCommand(()->swerveSubsystem.drive(visionSubsystem.getDesiredSpeeds()[0],visionSubsystem.getDesiredSpeeds()[1],visionSubsystem.getDesiredSpeeds()[2],true,false), swerveSubsystem);
  private final Command rc_goToPose = new RunCommand(()->swerveSubsystem.driveToPose(new Pose2d()), swerveSubsystem);
  private final Command rc_generateNavPoses = new InstantCommand(()->navigationField.setNavPoint(new Pose2d(2.5,0,new Rotation2d())));
  private final Command rc_navToPose = new RunCommand(()->swerveSubsystem.driveToPose(navigationField.getNextNavPoint()),swerveSubsystem);
  private final Command swerve_resetPose = new InstantCommand(swerveSubsystem::resetPose);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(rc_drive);
    //pathingTelemSub.setDefaultCommand(new RunCommand(()->pathingTelemSub.periodic(),pathingTelemSub));
  //  (PathingTelemetrySub)pathingTelemSub.set

    pathingTelemSub.init();
    // Configure the button bindings
    configureButtonBindings();

    //inputs
    SmartDashboard.putNumber("x1",0);
    SmartDashboard.putNumber("y1",0);
    SmartDashboard.putNumber("x2",0);
    SmartDashboard.putNumber("y2",0);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kBack.value).whenPressed(() -> swerveSubsystem.zeroHeading());
    new JoystickButton(driverController, XboxController.Button.kY.value).whenHeld(rc_goToTag);
    new JoystickButton(driverController, XboxController.Button.kB.value).whenHeld(rc_goToPose);
    new JoystickButton(driverController, XboxController.Button.kStart.value).whenPressed(swerve_toggleFieldOriented);
    new JoystickButton(driverController, XboxController.Button.kA.value).whenPressed(swerve_resetPose);

    new JoystickButton(driverController, XboxController.Button.kX.value).whenPressed(rc_generateNavPoses);
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).whenHeld(rc_navToPose);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
