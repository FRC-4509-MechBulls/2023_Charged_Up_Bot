// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.NavToPointCommand;
import frc.robot.subsystems.arm.Grabber;
import frc.robot.subsystems.arm.StageOneSub;
import frc.robot.subsystems.arm.StageTwoSub;
import frc.robot.subsystems.state.FMSGetter;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.nav.NavigationField;
import frc.robot.subsystems.arm.EFSub;
import frc.robot.subsystems.nav.GraphicalTelemetrySubsystem;
import frc.robot.subsystems.nav.PathingTelemetrySub;
import frc.robot.subsystems.nav.VisionSubsystem;
import frc.robot.subsystems.state.StateControllerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final FMSGetter fmsGetter = new FMSGetter();
  private final StateControllerSubsystem stateControllerSubsystem = new StateControllerSubsystem(fmsGetter);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(fmsGetter);
  private final GraphicalTelemetrySubsystem pathingTelemSub = new PathingTelemetrySub(stateControllerSubsystem);
  private final NavigationField navigationField = new NavigationField((PathingTelemetrySub) pathingTelemSub, swerveSubsystem, fmsGetter,stateControllerSubsystem);
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem, (PathingTelemetrySub) pathingTelemSub);
  private final EFSub endEffectorSubsystem = new EFSub();

  private final StageOneSub stageOneSub = new StageOneSub();
  private final StageTwoSub stageTwoSub = new StageTwoSub();
  private final EFSub efSub = new EFSub();
  private final Grabber grabberSubsystem = new Grabber(stageOneSub,stageTwoSub,efSub,stateControllerSubsystem);


  private final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  private final XboxController operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);
  private final Command rc_drive = new RunCommand(()-> swerveSubsystem.joystickDrive(driverController.getLeftY()*-1,driverController.getLeftX()*-1,driverController.getRightX()*-1), swerveSubsystem);
  private final Command stateController_processInputs = new RunCommand(()-> stateControllerSubsystem.processRawAxisValues(operatorController.getPOV(), operatorController.getRightTriggerAxis()),stateControllerSubsystem);
  private final Command swerve_toggleFieldOriented = new InstantCommand(swerveSubsystem::toggleFieldOriented);
  private final Command rc_goToTag = new RunCommand(()->swerveSubsystem.drive(visionSubsystem.getDesiredSpeeds()[0],visionSubsystem.getDesiredSpeeds()[1],visionSubsystem.getDesiredSpeeds()[2],true,false), swerveSubsystem);
  private final Command rc_goToPose = new RunCommand(()->swerveSubsystem.driveToPose(new Pose2d()), swerveSubsystem);
  private final Command rc_generateNavPoses = new InstantCommand(()->navigationField.setNavPoint(new Pose2d(2.5,0,new Rotation2d())));
  private final Command rc_navToPose = new RunCommand(()->swerveSubsystem.driveToPose(navigationField.getNextNavPoint()),swerveSubsystem);
  private final Command swerve_resetPose = new InstantCommand(swerveSubsystem::resetPose);





  SendableChooser<Command> autoChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(rc_drive);
    stateControllerSubsystem.setDefaultCommand(stateController_processInputs);

    //pathingTelemSub.setDefaultCommand(new RunCommand(()->pathingTelemSub.periodic(),pathingTelemSub));
  //  (PathingTelemetrySub)pathingTelemSub.set

    pathingTelemSub.init();
    // Configure the button bindings
    configureButtonBindings();

    //inputs
//    SmartDashboard.putNumber("x1",0);
//    SmartDashboard.putNumber("y1",0);
//    SmartDashboard.putNumber("x2",0);
//    SmartDashboard.putNumber("y2",0);

    NavToPointCommand nav1 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-6.36,2.93,Rotation2d.fromDegrees(180)),15);
    NavToPointCommand nav2 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-2.95,3.0,Rotation2d.fromDegrees(0)),15);
    NavToPointCommand nav3 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-6.36, 2.38,Rotation2d.fromDegrees(180)),15);
    NavToPointCommand nav4 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-2.95,2.0,Rotation2d.fromDegrees(0)),15);
    NavToPointCommand nav5 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-6.37,1.82,Rotation2d.fromDegrees(180)),15);
    NavToPointCommand nav6 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-4.51, 0.6,Rotation2d.fromDegrees(0)),15);

    Command twoConeBalanceSequence = nav1.andThen(nav2.andThen(nav3.andThen(nav4.andThen(nav5.andThen(nav6)))));


    autoChooser.setDefaultOption("twoConeBalanceSequence", twoConeBalanceSequence);
    SmartDashboard.putData("Auto Chooser",autoChooser);


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private POVButton[] dpad = new POVButton[]{new POVButton(operatorController,0),new POVButton(operatorController,90),new POVButton(operatorController,180),new POVButton(operatorController,270)};
  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kBack.value).whenPressed(() -> swerveSubsystem.zeroHeading());
  //  new JoystickButton(driverController, XboxController.Button.kB.value).whenHeld(rc_goToPose);
    new JoystickButton(driverController, XboxController.Button.kStart.value).whenPressed(swerve_toggleFieldOriented);
  //  new JoystickButton(driverController, XboxController.Button.kA.value).whenPressed(swerve_resetPose);

    //new JoystickButton(driverController, XboxController.Button.kX.value).whenPressed(rc_generateNavPoses);

    new JoystickButton(driverController, XboxController.Button.kRightBumper.value).whileTrue(rc_navToPose);
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(navigationField::engageNav));
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onFalse(new InstantCommand(navigationField::disengageNav));

    new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(stateControllerSubsystem::itemCubeButton));
    new JoystickButton(operatorController,XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(stateControllerSubsystem::itemConeUprightButton));

    new JoystickButton(operatorController,XboxController.Button.kA.value).onTrue(new InstantCommand(stateControllerSubsystem::setAgArmToIntake));
    new JoystickButton(operatorController,XboxController.Button.kB.value).onTrue(new InstantCommand(stateControllerSubsystem::setAgArmToHolding));

    new JoystickButton(driverController,XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(stateControllerSubsystem::setAgArmToPlacing));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
