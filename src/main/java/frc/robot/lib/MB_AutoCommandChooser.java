package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.DirectToPointCommand;
import frc.robot.commands.NavToPointCommand;
import frc.robot.commands.SleepCommand;
import frc.robot.subsystems.arm.Grabber;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.nav.NavigationField;
import frc.robot.subsystems.state.StateControllerSubsystem;

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
     //   autoChooser.addOption("debug_red_goToStart",redRight_Debug_goToStartPos(false));
        autoChooser.addOption("do-nothing",doNothing());

      //  autoChooser.addOption("r_c_justBalance",redBalancerCenter(false));
        autoChooser.addOption("r_C_placeAndBalance",redCenter_scoreLeaveAndBalance(false));
      //  autoChooser.addOption("r_r_scoreLeaveIntakeScore_old", redRight_scoreLeaveIntakeScore_old(false));
        autoChooser.addOption("r_R_scoreLeaveAlign", redRight_scoreLeaveAlign(false));
        autoChooser.addOption("r_R_scoreLeaveIntakeScore_new", redRight_scoreLeaveIntakeScore_new(false));



        //autoChooser.addOption("b_c_justBalance",redBalancerCenter(true));
        autoChooser.addOption("b_C_placeAndBalance",redCenter_scoreLeaveAndBalance(true));
        //autoChooser.addOption("b_r_scoreLeaveIntakeScore_old", redRight_scoreLeaveIntakeScore_old(true));
        autoChooser.addOption("b_L_scoreLeaveAlign", redRight_scoreLeaveAlign(true));
        autoChooser.addOption("b_L_scoreLeaveIntakeScore_new", redRight_scoreLeaveIntakeScore_new(true));

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
    public Command doNothing(){
        return new SleepCommand(1);
    }

    public Command redBalancerCenter(boolean reverseForBlue){
        int reverseX = 1;
        double zeroAngle = 0;
        if(reverseForBlue){
            reverseX = -1;
            zeroAngle = 180;
        }
        int finalReverseX = reverseX;
        double finalZeroAngle = zeroAngle;
        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-6.36 * finalReverseX,1.33), Rotation2d.fromDegrees(180+ finalZeroAngle))));
        DirectToPointCommand nav1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-4.25* finalReverseX,1.33,Rotation2d.fromDegrees(180+finalZeroAngle)),6, Units.inchesToMeters(3),2,0.1,Constants.DriveConstants.turnPValue);
        SleepCommand sleepCommand = new SleepCommand(0.2);
        AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(swerveSubsystem,15);
        return setInitialPose.andThen(nav1.andThen(autoBalanceCommand));
    }

public Command redCenter_scoreLeaveAndBalance(boolean reverseForBlue){
    int reverseX = 1;
    double zeroAngle = 0;
    if(reverseForBlue){
        reverseX = -1;
        zeroAngle = 180;
    }
    int finalReverseX = reverseX;
    double finalZeroAngle = zeroAngle;
    double posP = 2;
    double extraCommunityTravelDist = Units.inchesToMeters(48);

    Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-6.21*finalReverseX,1.25), Rotation2d.fromDegrees(180+finalZeroAngle)))); //cube placing position
    DirectToPointCommand backAway = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.70*finalReverseX,1.25,Rotation2d.fromDegrees(180+finalZeroAngle)),3,Units.inchesToMeters(1),2,posP, Constants.DriveConstants.turnPValue);
    Command setToPlacingCube = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CUBE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3)));
    SleepCommand sleepCommand = new SleepCommand(1);
    DirectToPointCommand navToPlace = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.21*finalReverseX,1.25,Rotation2d.fromDegrees(180+finalZeroAngle)),3, Units.inchesToMeters(0.25),2,posP,Constants.DriveConstants.turnPValue);
    SleepCommand sleepCommand1 = new SleepCommand(0);
    InstantCommand place = new InstantCommand(()->grabber.overrideDesiredEFWait());
    SleepCommand sleepCommand2 = new SleepCommand(1);
    DirectToPointCommand backAway1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.70*finalReverseX,1.25,Rotation2d.fromDegrees(180+finalZeroAngle)),3,Units.inchesToMeters(1),2,posP,Constants.DriveConstants.turnPValue);
    SleepCommand sleepCommand3 = new SleepCommand(0);
    Command retractArm = new InstantCommand(()->stateController.setAgArmToHolding());
    DirectToPointCommand navToLeaveCommunity = new DirectToPointCommand(swerveSubsystem,new Pose2d((-3.45+ extraCommunityTravelDist)*finalReverseX ,1.25,Rotation2d.fromDegrees(180+finalZeroAngle)),3,Units.inchesToMeters(3),5,posP,Constants.DriveConstants.turnPValue);
    SleepCommand waitToReenter = new SleepCommand(1.5);
    DirectToPointCommand navToBalancer = new DirectToPointCommand(swerveSubsystem,new Pose2d(-4.25*finalReverseX,1.25,Rotation2d.fromDegrees(180+finalZeroAngle)),3, Units.inchesToMeters(3),2,posP,Constants.DriveConstants.turnPValue);

    AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(swerveSubsystem,15);

    return setInitialPose.andThen(backAway.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(sleepCommand1.andThen(place.andThen(sleepCommand2.andThen(backAway1.andThen(sleepCommand3).andThen(retractArm.andThen(navToLeaveCommunity.andThen(waitToReenter).andThen(navToBalancer.andThen(autoBalanceCommand)))))))))))); //kill me
}




    public Command redRight_scoreLeaveAlign(boolean reverseForBlue){
        int reverseX = 1;
        double zeroAngle = 0;
        double pickupAngle = 8.54;
        if(reverseForBlue){
            reverseX = -1;
            zeroAngle = 180;
            pickupAngle = 180-pickupAngle;
        }
        int finalReverseX = reverseX;
        double finalZeroAngle = zeroAngle;
        double finalPickupAngle = pickupAngle;
        double standardPosTolerance = Units.inchesToMeters(2);
        double posP = 3;

        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-5.362*finalReverseX,-1.047), Rotation2d.fromDegrees(180+finalZeroAngle)))); //-6.21*reverse, -0.43
        Command setToPlacingCube = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CUBE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3)));
        SleepCommand sleepCommand = new SleepCommand(1);
        DirectToPointCommand navToPlace = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.454*finalReverseX,-0.416,Rotation2d.fromDegrees(180+finalZeroAngle)),3, Units.inchesToMeters(0.5),2,0.3,Constants.DriveConstants.turnPValue);
        InstantCommand place = new InstantCommand(()->grabber.overrideDesiredEFWait());
        SleepCommand sleepCommand2 = new SleepCommand(2);
        DirectToPointCommand intermediate1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.247*finalReverseX,-0.762,Rotation2d.fromDegrees(180+finalZeroAngle)),3,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command retractArm = new InstantCommand(()->stateController.setAgArmToHolding());
        Command waitAfterRetract = new SleepCommand(0.5);
        DirectToPointCommand alignToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d(-2.554*finalReverseX,-0.762,Rotation2d.fromDegrees(finalPickupAngle)),4,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command setToIntakingCone = new InstantCommand(()->stateController.setAgArmToIntake()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE))));
        SleepCommand pauseForIntake = new SleepCommand(1.5);
        DirectToPointCommand navToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d(-1.775*finalReverseX,-0.645,Rotation2d.fromDegrees(finalPickupAngle)),4,Units.inchesToMeters(1),2,0.5,Constants.DriveConstants.turnPValue);
        Command waitAfterPickup = new SleepCommand(0.2);
        Command setToHoldCone = new InstantCommand(()->stateController.setAgArmToHolding());
        Command waitAfterHold = new SleepCommand(0.2);
        DirectToPointCommand intermediate2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-4.39*finalReverseX,-0.787,Rotation2d.fromDegrees(180+finalZeroAngle)),3,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command setToPlacingCone = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE))));
        SleepCommand sleepCommand3 = new SleepCommand(0.5);
        DirectToPointCommand navToPlace2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.454*finalReverseX,-0.975,Rotation2d.fromDegrees(180+finalZeroAngle)),3, Units.inchesToMeters(0.5),2,1,Constants.DriveConstants.turnPValue);
        InstantCommand place2 = new InstantCommand(()->grabber.overrideDesiredEFWait()); //6.454, -.975 ^

        //   return setInitialPose.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(place.andThen(sleepCommand2.andThen(intermediate1.andThen(retractArm.andThen(alignToPickup.andThen(setToIntakingCone.andThen(pauseForIntake.andThen(navToPickup.andThen(waitAfterPickup.andThen(setToHoldCone.andThen(intermediate2.andThen(setToPlacingCone.andThen(sleepCommand3.andThen(navToPlace2.andThen(place2))))))))))))))))));
        return setInitialPose.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(place.andThen(sleepCommand2.andThen(intermediate1.andThen(retractArm.andThen(waitAfterRetract).andThen(alignToPickup.andThen(setToIntakingCone.andThen(pauseForIntake.andThen(navToPickup.andThen(waitAfterPickup.andThen(setToHoldCone.andThen(waitAfterHold.andThen(intermediate2.andThen(setToPlacingCone.andThen(sleepCommand3.andThen(navToPlace2.andThen(place2)))))))))))))))))));
//
    }

    public Command redRight_scoreLeaveIntakeScore_new(boolean reverseForBlue){
        int reverseX = 1;
        double zeroAngle = 0;
        double pickupAngle = 8.54;
        if(reverseForBlue){
            reverseX = -1;
            zeroAngle = 180;
            pickupAngle = 180-pickupAngle;
        }
        int finalReverseX = reverseX;
        double finalZeroAngle = zeroAngle;
        double finalPickupAngle = pickupAngle;
        double standardPosTolerance = Units.inchesToMeters(2);
        double posP = 3;

        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-5.362*finalReverseX,-1.047), Rotation2d.fromDegrees(180+finalZeroAngle)))); //-6.21*reverse, -0.43
        Command setToPlacingCube = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CUBE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3)));
        SleepCommand sleepCommand = new SleepCommand(1);
        DirectToPointCommand navToPlace = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.454*finalReverseX,-0.416,Rotation2d.fromDegrees(180+finalZeroAngle)),3, Units.inchesToMeters(0.5),2,0.3,Constants.DriveConstants.turnPValue);
        InstantCommand place = new InstantCommand(()->grabber.overrideDesiredEFWait());
        SleepCommand sleepCommand2 = new SleepCommand(2);
        DirectToPointCommand intermediate1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.247*finalReverseX,-0.762,Rotation2d.fromDegrees(180+finalZeroAngle)),3,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command retractArm = new InstantCommand(()->stateController.setAgArmToHolding());
        Command waitAfterRetract = new SleepCommand(0.5);
        DirectToPointCommand alignToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d(-2.554*finalReverseX,-0.762,Rotation2d.fromDegrees(finalPickupAngle)),4,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command setToIntakingCone = new InstantCommand(()->stateController.setAgArmToIntake()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE))));
        SleepCommand pauseForIntake = new SleepCommand(1.5);
        DirectToPointCommand navToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d(-1.775*finalReverseX,-0.645,Rotation2d.fromDegrees(finalPickupAngle)),4,Units.inchesToMeters(1),2,0.5,Constants.DriveConstants.turnPValue);
        Command waitAfterPickup = new SleepCommand(0.2);
        Command setToHoldCone = new InstantCommand(()->stateController.setAgArmToHolding());
        Command waitAfterHold = new SleepCommand(0.2);
        DirectToPointCommand intermediate2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-4.39*finalReverseX,-0.787,Rotation2d.fromDegrees(180+finalZeroAngle)),3,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command setToPlacingCone = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE))));
        SleepCommand sleepCommand3 = new SleepCommand(0.5);
        DirectToPointCommand navToPlace2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.454*finalReverseX,-0.975,Rotation2d.fromDegrees(180+finalZeroAngle)),3, Units.inchesToMeters(0.5),2,1,Constants.DriveConstants.turnPValue);
        InstantCommand place2 = new InstantCommand(()->grabber.overrideDesiredEFWait()); //6.454, -.975 ^

        //   return setInitialPose.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(place.andThen(sleepCommand2.andThen(intermediate1.andThen(retractArm.andThen(alignToPickup.andThen(setToIntakingCone.andThen(pauseForIntake.andThen(navToPickup.andThen(waitAfterPickup.andThen(setToHoldCone.andThen(intermediate2.andThen(setToPlacingCone.andThen(sleepCommand3.andThen(navToPlace2.andThen(place2))))))))))))))))));
        return setInitialPose.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(place.andThen(sleepCommand2.andThen(intermediate1.andThen(retractArm.andThen(waitAfterRetract).andThen(alignToPickup.andThen(setToIntakingCone.andThen(pauseForIntake.andThen(navToPickup.andThen(waitAfterPickup.andThen(setToHoldCone.andThen(waitAfterHold.andThen(intermediate2.andThen(setToPlacingCone.andThen(sleepCommand3.andThen(navToPlace2.andThen(place2)))))))))))))))))));
//
    }

    public Command redRight_Debug_goToStartPos(boolean reverseForBlue){
        int reverseX = 1;
        double zeroAngle = 0;
        double pickupAngle = 30;
        if(reverseForBlue){
            reverseX = -1;
            zeroAngle = 180;
            pickupAngle = 180-30;
        }
        int finalReverseX = reverseX;
        double finalZeroAngle = zeroAngle;
        double standardPosTolerance = Units.inchesToMeters(0.1);
        double posP = 3;

        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-5.70*finalReverseX,-0.7), Rotation2d.fromDegrees(180+finalZeroAngle)))); //-6.21*reverse, -0.43
        DirectToPointCommand backAway = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.70*finalReverseX,-0.7,Rotation2d.fromDegrees(180+finalZeroAngle)),4,standardPosTolerance,2,posP, Constants.DriveConstants.turnPValue);
        return backAway;
    }




}
