package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.DirectToPointCommand;
import frc.robot.commands.NavToPointCommand;
import frc.robot.commands.SleepCommand;
import frc.robot.subsystems.arm.Grabber;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.nav.NavigationField;
import frc.robot.subsystems.state.StateControllerSubsystem;

import javax.swing.plaf.nimbus.State;
import java.awt.font.TransformAttribute;

public class MB_AutoCommandChooser {

    SendableChooser<Command> autoChooser = new SendableChooser<>();
    NavigationField navigationField;
    SwerveSubsystem swerveSubsystem;
    StateControllerSubsystem stateController;
    Grabber grabber;

    public MB_AutoCommandChooser(NavigationField navigationField, SwerveSubsystem swerveSubsystem, StateControllerSubsystem stateController, Grabber grabber){
        this.navigationField = navigationField;
        this.swerveSubsystem = swerveSubsystem;
        this.stateController = stateController;
        this.grabber = grabber;
        autoChooser = new SendableChooser<Command>();

        autoChooser.addOption("r_c_justBalance",redBalancerCenter());
        autoChooser.addOption("r_c_placeAndBalance",redCenter_scoreLeaveAndBalance());

        autoChooser.addOption("b_c_justBalance",blueBalancerCenter());



    }

    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }

    public Command exampleNavCommand(){
        NavToPointCommand nav1 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-6.36,2.93, Rotation2d.fromDegrees(180)),15);
        NavToPointCommand nav2 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-2.95,3.0,Rotation2d.fromDegrees(0)),15);
        NavToPointCommand nav3 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-6.36, 2.38,Rotation2d.fromDegrees(180)),15);
        NavToPointCommand nav4 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-2.95,2.0,Rotation2d.fromDegrees(0)),15);
        NavToPointCommand nav5 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-6.37,1.82,Rotation2d.fromDegrees(180)),15);
        NavToPointCommand nav6 = new NavToPointCommand(navigationField,swerveSubsystem,new Pose2d(-4.51, 0.6,Rotation2d.fromDegrees(0)),15);

        return nav1.andThen(nav2.andThen(nav3.andThen(nav4.andThen(nav5.andThen(nav6)))));

    }

    public Command redBalancerCenter(){
        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-6.36,1.33), Rotation2d.fromDegrees(180))));
        DirectToPointCommand nav1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-4.25,1.33,Rotation2d.fromDegrees(180)),6, Units.inchesToMeters(3),2,0.1,Constants.DriveConstants.turnPValue);
        SleepCommand sleepCommand = new SleepCommand(0.2);
        AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(swerveSubsystem,15);
        return setInitialPose.andThen(nav1.andThen(autoBalanceCommand));
    }
    public Command blueBalancerCenter(){
        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(6.36,1.33), Rotation2d.fromDegrees(0))));
        DirectToPointCommand nav1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(4.25,1.33,Rotation2d.fromDegrees(0)),6, Units.inchesToMeters(3),2,0.1,Constants.DriveConstants.turnPValue);
        SleepCommand sleepCommand = new SleepCommand(0.2);
        AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(swerveSubsystem,15);
        return setInitialPose.andThen(nav1.andThen(autoBalanceCommand));
    }

public Command redCenter_scoreLeaveAndBalance(){
    Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-6.21,1.25), Rotation2d.fromDegrees(180)))); //cube placing position
    DirectToPointCommand backAway = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.70,1.25,Rotation2d.fromDegrees(180)),3,Units.inchesToMeters(1),2,0.3, Constants.DriveConstants.turnPValue);
    Command setToPlacingCube = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CUBE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS2)));
    SleepCommand sleepCommand = new SleepCommand(1);
    DirectToPointCommand navToPlace = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.21,1.25,Rotation2d.fromDegrees(180)),4, Units.inchesToMeters(0.25),2,0.5,Constants.DriveConstants.turnPValue);
    SleepCommand sleepCommand1 = new SleepCommand(0);
    InstantCommand place = new InstantCommand(()->grabber.overrideDesiredEFWait());
    SleepCommand sleepCommand2 = new SleepCommand(1);
    DirectToPointCommand backAway1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.70,1.25,Rotation2d.fromDegrees(180)),3,Units.inchesToMeters(1),2,0.5,Constants.DriveConstants.turnPValue);
    SleepCommand sleepCommand3 = new SleepCommand(0);
    Command retractArm = new InstantCommand(()->stateController.setAgArmToHolding());
    DirectToPointCommand navToBalancer = new DirectToPointCommand(swerveSubsystem,new Pose2d(-4.25,1.25,Rotation2d.fromDegrees(180)),6, Units.inchesToMeters(3),2,0.5,Constants.DriveConstants.turnPValue);
    AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(swerveSubsystem,15);

    return setInitialPose.andThen(backAway.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(sleepCommand1.andThen(place.andThen(sleepCommand2.andThen(backAway1.andThen(sleepCommand3).andThen(retractArm.andThen(navToBalancer.andThen(autoBalanceCommand))))))))))); //kill me
}




}
