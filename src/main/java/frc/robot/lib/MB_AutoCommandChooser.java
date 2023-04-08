package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.Grabber;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.nav.NavigationField;
import frc.robot.subsystems.state.StateControllerSubsystem;

import java.util.concurrent.atomic.AtomicReference;

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
      //  autoChooser.addOption("r_c_placeAndBalance",redCenter_scoreLeaveAndBalance(false));
      //  autoChooser.addOption("r_r_scoreLeaveIntakeScore_old", redRight_scoreLeaveIntakeScore_old(false));
      //  autoChooser.addOption("r_r_scoreLeaveIntakeScore_new",redRight_scoreLeaveIntakeScore_untested(false));

        autoChooser.addOption("r_L_doubleScore",redLeft_doubleScore(false));
        autoChooser.addOption("r_C_placeBalance",redCenter_placeBalance(false));



        //autoChooser.addOption("b_c_justBalance",redBalancerCenter(true));
       // autoChooser.addOption("b_c_placeAndBalance",redCenter_scoreLeaveAndBalance(true));
        //autoChooser.addOption("b_r_scoreLeaveIntakeScore_old", redRight_scoreLeaveIntakeScore_old(true));
       // autoChooser.addOption("b_L_scoreLeaveIntakeScore_new",redRight_scoreLeaveIntakeScore_untested(true));

        autoChooser.addOption("b_R_doubleScore",redLeft_doubleScore(true));
        autoChooser.addOption("b_C_placeBalance",redCenter_placeBalance(true));



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



    public Command redRight_scoreLeaveIntakeScore_old(boolean reverseForBlue){
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
        double standardPosTolerance = Units.inchesToMeters(4);
        double posP = 3;

        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-5.70*finalReverseX,-0.7), Rotation2d.fromDegrees(180+finalZeroAngle)))); //-6.21*reverse, -0.43
        DirectToPointCommand backAway = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.70*finalReverseX,-0.7,Rotation2d.fromDegrees(180+finalZeroAngle)),2,standardPosTolerance,2,posP, Constants.DriveConstants.turnPValue);
        Command setToPlacingCube = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CUBE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS2)));
        SleepCommand sleepCommand = new SleepCommand(2);
        DirectToPointCommand navToPlace = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.28*finalReverseX,-0.43,Rotation2d.fromDegrees(180+finalZeroAngle)),4, Units.inchesToMeters(0.5),2,0.3,Constants.DriveConstants.turnPValue);
        InstantCommand place = new InstantCommand(()->grabber.overrideDesiredEFWait());
        SleepCommand sleepCommand2 = new SleepCommand(2);
        DirectToPointCommand backAway1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.70*finalReverseX,-0.7,Rotation2d.fromDegrees(180+finalZeroAngle)),3,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command retractArm = new InstantCommand(()->stateController.setAgArmToHolding());
        DirectToPointCommand backAway2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-3.0*finalReverseX,-0.7,Rotation2d.fromDegrees(180+finalZeroAngle)),4,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        DirectToPointCommand alignToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d(-2.40*finalReverseX,-1.35,Rotation2d.fromDegrees(pickupAngle)),4,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command setToIntakingCone = new InstantCommand(()->stateController.setAgArmToIntake()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE))));
        SleepCommand pauseForIntake = new SleepCommand(2.5);
        DirectToPointCommand navToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d(-1.56*finalReverseX,-0.82,Rotation2d.fromDegrees(pickupAngle)),4,Units.inchesToMeters(1),2,0.5,Constants.DriveConstants.turnPValue);
        Command waitAfterPickup = new SleepCommand(1.5);
        Command setToHoldCone = new InstantCommand(()->stateController.setAgArmToHolding());
        DirectToPointCommand alignToPickup_reversed = new DirectToPointCommand(swerveSubsystem,new Pose2d(-2.40*finalReverseX,-1.35,Rotation2d.fromDegrees(180+finalZeroAngle)),4,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        DirectToPointCommand backAway2_again = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.70*finalReverseX,-0.7,Rotation2d.fromDegrees(180+finalZeroAngle)),4,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command setToPlacingCone = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS2)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE))));
        SleepCommand sleepCommand3 = new SleepCommand(1.5);
        DirectToPointCommand navToPlace2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.28*finalReverseX,-0.98,Rotation2d.fromDegrees(180+finalZeroAngle)),7, Units.inchesToMeters(0.5),2,1,Constants.DriveConstants.turnPValue);
        InstantCommand place2 = new InstantCommand(()->grabber.overrideDesiredEFWait()); //6.454, -.975 ^

        return setInitialPose.andThen(backAway.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(place.andThen(sleepCommand2.andThen(backAway1.andThen(retractArm.andThen(backAway2.andThen(alignToPickup))))))))));
    }


    public Command  redRight_scoreLeaveIntakeScore_untested(boolean reverseForBlue){
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
        SleepCommand sleepCommand = new SleepCommand(2);
        DirectToPointCommand navToPlace = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.454*finalReverseX,-0.416,Rotation2d.fromDegrees(180+finalZeroAngle)),3, Units.inchesToMeters(0.5),2,0.3,Constants.DriveConstants.turnPValue);
        InstantCommand place = new InstantCommand(()->grabber.overrideDesiredEFWait());
        SleepCommand sleepCommand2 = new SleepCommand(2);
        DirectToPointCommand intermediate1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.247*finalReverseX,-0.762,Rotation2d.fromDegrees(180+finalZeroAngle)),3,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command retractArm = new InstantCommand(()->stateController.setAgArmToHolding());
        Command waitAfterRetract = new SleepCommand(1);
        DirectToPointCommand alignToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d(-2.554*finalReverseX,-0.762,Rotation2d.fromDegrees(finalPickupAngle)),4,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command setToIntakingCone = new InstantCommand(()->stateController.setAgArmToIntake()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE))));
        SleepCommand pauseForIntake = new SleepCommand(2);
        DirectToPointCommand navToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d(-1.775*finalReverseX,-0.645,Rotation2d.fromDegrees(finalPickupAngle)),4,Units.inchesToMeters(1),2,0.5,Constants.DriveConstants.turnPValue);
        Command waitAfterPickup = new SleepCommand(1);
        Command setToHoldCone = new InstantCommand(()->stateController.setAgArmToHolding());
        Command waitAfterHold = new SleepCommand(2);
        DirectToPointCommand intermediate2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-4.39*finalReverseX,-0.787,Rotation2d.fromDegrees(180+finalZeroAngle)),3,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue);
        Command setToPlacingCone = new InstantCommand(()->stateController.setAgArmToPlacing()).andThen(new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE))).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE))));
        SleepCommand sleepCommand3 = new SleepCommand(1);
        DirectToPointCommand navToPlace2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.454*finalReverseX,-0.975,Rotation2d.fromDegrees(180+finalZeroAngle)),3, Units.inchesToMeters(0.5),2,1,Constants.DriveConstants.turnPValue);
        InstantCommand place2 = new InstantCommand(()->grabber.overrideDesiredEFWait()); //6.454, -.975 ^

        //   return setInitialPose.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(place.andThen(sleepCommand2.andThen(intermediate1.andThen(retractArm.andThen(alignToPickup.andThen(setToIntakingCone.andThen(pauseForIntake.andThen(navToPickup.andThen(waitAfterPickup.andThen(setToHoldCone.andThen(intermediate2.andThen(setToPlacingCone.andThen(sleepCommand3.andThen(navToPlace2.andThen(place2))))))))))))))))));
        return setInitialPose.andThen(setToPlacingCube.andThen(sleepCommand.andThen(navToPlace.andThen(place.andThen(sleepCommand2.andThen(intermediate1.andThen(retractArm.andThen(waitAfterRetract).andThen(alignToPickup.andThen(setToIntakingCone.andThen(pauseForIntake.andThen(navToPickup.andThen(waitAfterPickup.andThen(setToHoldCone.andThen(waitAfterHold.andThen(intermediate2)))))))))))))));
//
    }

    public Command redLeft_doubleScore(boolean reverseForBlue){
        int reverseX = 1;
        double zeroAngle = 0;
        double pickupAngle = 0;
        double postPickupAngle = 15+180;

        double intakeYValue = Units.inchesToMeters(121.61-17.25);

        double intermediate1Angle = -45 + zeroAngle;
        if(reverseForBlue){
            reverseX = -1;
            zeroAngle = 180;
            pickupAngle = 180-pickupAngle;
            postPickupAngle = 180-postPickupAngle;
            intermediate1Angle = 45 + zeroAngle;
            intakeYValue = 3.4 + Units.inchesToMeters(5);
        }
        int finalReverseX = reverseX;
        double finalZeroAngle = zeroAngle;
        double finalPickupAngle = pickupAngle;
        double standardPosTolerance = Units.inchesToMeters(2);
        double posP = 3;

        double fasterMaxSpeed = 0.65;
        double fasterMaxTurn = 0.45;

        double slowerMaxSpeed = 0.1;
        double secondPlaceMaxSpeed = 0.10;
        double slowerMaxTurn = 0.1;

        double inPlaceTurnSpeedMax = 0.30;
        double inPlaceTurnP = Constants.DriveConstants.turnPValue / 2.0;

        double intakingMaxSpeed = 0.15;



        //initial pose
        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-6.337*finalReverseX,2.428), Rotation2d.fromDegrees(180+finalZeroAngle))));
        //set type to cone, then level to L3, then set to placing\
        Command setToPlacingCone = new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE)).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3))).andThen(new InstantCommand(()->stateController.setAgArmToPlacing()));
        //maxOutputController
        Command maxOutputController = new StageTwoMaxOutputControllerCommand(grabber.getStageTwoSub(), -155, true);
        Command setToPlacingWithOutputController = Commands.deadline(setToPlacingCone,maxOutputController);
        //wait 4 seconds
        SleepCommand sleepCommand = new SleepCommand(3); //replace with the arm angle being crossed >:P
        //eject cone
        Command place = new InstantCommand(()->grabber.overrideDesiredEFWait());
        SleepCommand sleepCommand2 = new SleepCommand(0.5);
        //retract arm
        Command retractArm = new InstantCommand(()->stateController.setAgArmToHolding());
        //go to intermediate position
        DirectToPointCommand intermediate1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.844*finalReverseX,Units.inchesToMeters(128.1),Rotation2d.fromDegrees(intermediate1Angle)),4,standardPosTolerance,5,posP,Constants.DriveConstants.turnPValue,0.30,0.25);
        //intermediate two
        DirectToPointCommand intermediate2 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-3.00*finalReverseX,Units.inchesToMeters(128.1),Rotation2d.fromDegrees(finalZeroAngle)),3,standardPosTolerance,2,posP,Constants.DriveConstants.turnPValue,fasterMaxSpeed,slowerMaxTurn); // y from cad
        //set to intaking
        Command setToIntakingCone = new InstantCommand(()->stateController.setAgnosticGrabberMode(StateControllerSubsystem.AgnosticGrabberMode.INTAKING)).andThen(new InstantCommand(()->stateController.setItemFallen(StateControllerSubsystem.ItemFallen.FALLEN_CONE)));
        //align to pickup
    //    DirectToPointCommand navToAlignPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d((Units.inchesToMeters(-47.36) - 1.449 )*finalReverseX,Units.inchesToMeters(121.61) - 0.388,Rotation2d.fromDegrees(finalPickupAngle)),4,Units.inchesToMeters(1),2,3,Constants.DriveConstants.turnPValue, fasterMaxSpeed, fasterMaxTurn);
        //wait 1 second
     //   SleepCommand pauseForIntake = new SleepCommand(0.5);
      //  DirectToPointCommand pauseForIntake = new DirectToPointCommand(swerveSubsystem,new Pose2d((Units.inchesToMeters(-47.36) - 1.449 )*finalReverseX,Units.inchesToMeters(121.61) - 0.388,Rotation2d.fromDegrees(finalPickupAngle)),4,Units.inchesToMeters(1),2,0.5,Constants.DriveConstants.turnPValue, slowerMaxSpeed, slowerMaxTurn);

        //drive to pickup
        DirectToPointCommand navToPickup = new DirectToPointCommand(swerveSubsystem,new Pose2d((Units.inchesToMeters(-47.36 + 10))*finalReverseX,intakeYValue,Rotation2d.fromDegrees(finalPickupAngle)),2.5,Units.inchesToMeters(4),5,3,Constants.DriveConstants.turnPValue, intakingMaxSpeed, slowerMaxTurn); //y was Units.inchesToMeters(121.61)

        //wait 1 second
       // SleepCommand waitAfterPickup = new SleepCommand(0);
      //  DirectToPointCommand turnAfterPickup  = new DirectToPointCommand(swerveSubsystem,new Pose2d(Units.inchesToMeters(-47.36)*finalReverseX,Units.inchesToMeters(121.61),Rotation2d.fromDegrees(postPickupAngle)),4,Units.inchesToMeters(1),2,2,Constants.DriveConstants.turnPValue, slowerMaxSpeed, fasterMaxTurn);
        //set to holding
        Command setToHoldCone = new InstantCommand(()->stateController.setAgArmToHolding());
      //  DirectToPointCommand armToHoldingPause = new DirectToPointCommand(swerveSubsystem,new Pose2d((Units.inchesToMeters(-47.36 + 8))*finalReverseX,3.4 + Units.inchesToMeters(3),Rotation2d.fromDegrees(finalPickupAngle)),0.5,-1,-1,2,Constants.DriveConstants.turnPValue, slowerMaxSpeed, slowerMaxTurn); //y was Units.inchesToMeters(121.61)

        //back to intermediate 1
        DirectToPointCommand postPickupSpin = new DirectToPointCommand(swerveSubsystem,new Pose2d((Units.inchesToMeters(-47.36 + 10))*finalReverseX,intakeYValue,Rotation2d.fromDegrees(postPickupAngle)),1,Units.inchesToMeters(8),3,2,inPlaceTurnP, slowerMaxSpeed, inPlaceTurnSpeedMax); //y was Units.inchesToMeters(121.61)


        DirectToPointCommand intermediate3 = new DirectToPointCommand(swerveSubsystem,new Pose2d((-4.7 + 0.2)*finalReverseX,Units.inchesToMeters(128.1),Rotation2d.fromDegrees(180+ finalZeroAngle)),4,standardPosTolerance,5,posP,Constants.DriveConstants.turnPValue,fasterMaxSpeed,fasterMaxTurn);

        //place L3
        Command setToPlacingCone2 = new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE)).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3))).andThen(new InstantCommand(()->stateController.setAgArmToPlacing()));
        //align to place final cone
        DirectToPointCommand navToAlignPlace = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.337*finalReverseX,3.546,Rotation2d.fromDegrees(180+finalZeroAngle)),3,-1,-1,posP,Constants.DriveConstants.turnPValue,secondPlaceMaxSpeed,slowerMaxTurn);
        //sleep
       // SleepCommand sleepCommand3 = new SleepCommand(4);
     //   DirectToPointCommand sleepCommand3 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-6.337*finalReverseX,3.546,Rotation2d.fromDegrees(180+finalZeroAngle)),3,standardPosTolerance,2,0.5,Constants.DriveConstants.turnPValue,slowerMaxSpeed,slowerMaxTurn);

        //place
        Command place2 = new InstantCommand(()->grabber.overrideDesiredEFWait());


     //   return setInitialPose.andThen(setToPlacingCone.andThen(sleepCommand.andThen(place.andThen(sleepCommand2.andThen(retractArm.andThen(intermediate1.andThen(intermediate2.andThen(setToIntakingCone.andThen(navToAlignPickup.andThen(pauseForIntake)).andThen(navToPickup.andThen(waitAfterPickup.andThen(turnAfterPickup).andThen(setToHoldCone).andThen(intermediate3.andThen(setToPlacingCone2.andThen(navToAlignPlace.andThen(sleepCommand3.andThen(place2)))))))))))))));
        return setInitialPose.andThen(setToPlacingWithOutputController.andThen(sleepCommand.andThen(place.andThen(sleepCommand2.andThen(retractArm.andThen(intermediate1.andThen(intermediate2.andThen(setToIntakingCone.andThen(navToPickup.andThen(setToHoldCone.andThen(postPickupSpin.andThen(intermediate3.andThen(setToPlacingCone2.andThen(navToAlignPlace.andThen(place2)))))))))))))));
    }

    public Command redCenter_placeBalance(boolean reverseForBlue){ //red left center, blue right center
        int reverseX = 1;
        double zeroAngle = 180;
        double pickupAngle = 0;
        if(reverseForBlue){
            reverseX = -1;
            zeroAngle +=180;
            pickupAngle = 180-pickupAngle;
        }
        int finalReverseX = reverseX;
        double finalZeroAngle = zeroAngle;
        double standardPosTolerance = Units.inchesToMeters(2);
        double posP = 3;

        double typicalMaxSpeed = 0.30;
        double typicalMaxRotationSpeed = 0.10;


        //initial pose
        Command setInitialPose = new InstantCommand(()->swerveSubsystem.resetPose(new Pose2d(new Translation2d(-6.337*finalReverseX,1.869), Rotation2d.fromDegrees(finalZeroAngle))));
        //set type to cone, then level to L3, then set to placing
        Command setToPlacingCone = new InstantCommand(()->stateController.setItemType(StateControllerSubsystem.ItemType.CONE)).andThen(new InstantCommand(()->stateController.setPlacingLevel(StateControllerSubsystem.Level.POS3))).andThen(new InstantCommand(()->stateController.setAgArmToPlacing()));

        //output controller
        Command maxOutputController = new StageTwoMaxOutputControllerCommand(grabber.getStageTwoSub(), -155, true);
        Command setToPlacingWithOutputController = Commands.deadline(setToPlacingCone,maxOutputController);


        //wait 4 seconds
        SleepCommand sleepCommand = new SleepCommand(3); //replace with the arm angle being crossed >:P
        //eject cone
        Command place = new InstantCommand(()->grabber.overrideDesiredEFWait());
        SleepCommand sleepCommand2 = new SleepCommand(0.5);
        //retract arm
        Command retractArm = new InstantCommand(()->stateController.setAgArmToHolding());
        //center before charge station
        DirectToPointCommand intermediate1 = new DirectToPointCommand(swerveSubsystem,new Pose2d(-5.97*finalReverseX,1.31,Rotation2d.fromDegrees(finalZeroAngle)),1.5,standardPosTolerance,5,posP,Constants.DriveConstants.turnPValue,typicalMaxSpeed,typicalMaxRotationSpeed);
//-5.70*finalReverseX,1.25
        //go over charge station
        DirectToPointCommand goOverChargeStation = new DirectToPointCommand(swerveSubsystem,new Pose2d(-1.95*finalReverseX,1.31,Rotation2d.fromDegrees(finalZeroAngle)),5,Units.inchesToMeters(4),4,posP,Constants.DriveConstants.turnPValue,typicalMaxSpeed,typicalMaxRotationSpeed);

        //go over charge station
        DirectToPointCommand holdBehindChargeStation = new DirectToPointCommand(swerveSubsystem,new Pose2d(-1.95*finalReverseX,1.31,Rotation2d.fromDegrees(finalZeroAngle)),1,-1,-1,posP,Constants.DriveConstants.turnPValue,0.1,0.1);


        //go back
        DirectToPointCommand goOnChargeStation  = new DirectToPointCommand(swerveSubsystem,new Pose2d(-4.3*finalReverseX,1.31,Rotation2d.fromDegrees(finalZeroAngle)),3,Units.inchesToMeters(4),4,posP,Constants.DriveConstants.turnPValue,typicalMaxSpeed,typicalMaxRotationSpeed);

        //do the dew
        AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(swerveSubsystem,15);





        return setInitialPose.andThen(setToPlacingWithOutputController.andThen(sleepCommand.andThen(place.andThen(sleepCommand2.andThen(retractArm.andThen(intermediate1.andThen(goOverChargeStation.andThen(holdBehindChargeStation).andThen(goOnChargeStation.andThen(autoBalanceCommand)))))))));
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
