// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static final class SimulationConstants{
        public static final boolean simulationEnabled = true;
        public static final double speedMultiplier = 1.5;
        public static final double turningSpeedMultiplier = 50;

    }

    public static final class EndEffectorConstants {
        public static final int EF_MOTOR_TOP_ID = 14;
        public static final int EF_MOTOR_BOTTOM_ID = 13;

        public static final double INTAKE_CONE_TOP_OUTPUT = 0.8;
        public static final double INTAKE_CONE_BOTTOM_OUTPUT = 0.8;

        public static final double INTAKE_CUBE_TOP_OUTPUT = 0.0;
        public static final double INTAKE_CUBE_BOTTOM_OUTPUT = 0.8;
        public static final double HOLD_CONE_TOP_OUTPUT = 0.3;
        public static final double HOLD_CONE_BOTTOM_OUTPUT = -0.3;

        public static final double HOLD_CUBE_TOP_OUTPUT = 0;
        public static final double HOLD_CUBE_BOTTOM_OUTPUT = 0;
        public static final double PLACE_CONE_TOP_OUTPUT = -0.8;
        public static final double PLACE_CONE_BOTTOM_OUTPUT = -0.8;
        public static final double PLACE_CUBE_TOP_OUTPUT = -1.0;
        public static final double PLACE_CUBE_BOTTOM_OUTPUT = 0.6;
    }

    public static final class ArmConstants{
        public static final double stageOne_kP = 0.5; //undecided
        public static final double stageOne_kI = 0; //undecided
        public static final double stageOne_kD = 0; //undecided

        public static final double stageTwo_kP = 0.5; //undecided
        public static final double stageTwo_kI = 0; //undecided
        public static final double stageTwo_kD = 0; //undecided

        public static final double STAGE_ONE_ENCODER_TICKS_TO_RADIANS =  Math.PI * 2;
        public static final double STAGE_TWO_ENCODER_TICKS_TO_RADIANS = 1 * Math.PI * 2;


        public static final int STAGE_ONE_MOTOR_LEFT_ID = 11; //undecided
        public static final int STAGE_ONE_MOTOR_RIGHT_ID = 12; //undecided

        public static final double ABS_TO_RADIANS = 2.0 * Math.PI;

        public static final boolean STAGE_ONE_ABS_ENCODER_REVERSED = false; //undecided

        public static final double STAGE_ONE_ABS_ENCODER_INITIAL_OFFSET = 0; //undecided

        public static final double STAGE_ONE_LIMIT_SWITCH_ANGLE_RAD = 0; //undecided


        public static final int STAGE_TWO_MOTOR_LEFT_CHANNEL = 1; //undecided
        public static final int STAGE_TWO_MOTOR_RIGHT_CHANNEL = 2; //undecided

        public static final boolean STAGE_TWO_ABS_ENCODER_REVERSED = false; //undecided

        public static final double STAGE_TWO_ABS_ENCODER_INITIAL_OFFSET = 0; //undecided

        public static final double STAGE_TWO_LIMIT_SWITCH_ANGLE_RAD = 0; //undecided

        public static final double[] intakingConesUprightArmPos = {0,0}; //undecided
        public static final double[] intakingConesFallenArmPos = {0,0}; //undecided
        public static final double[] intakingCubesArmPos = {0,0}; //undecided
        public static final double[] holdingArmPos = {0,0}; //undecided
        public static final double[] aimingArmPos = {0,0}; //undecided
        public static final double[] placingArmPosOne = {0,0}; //undecided
        public static final double[] placingArmPosTwo = {0,0}; //undecided
        public static final double[] placingArmPosThree = {0,0}; //undecided

        public static final double[] stageOneCG = {6, 0, 10.64};
        public static final double[] stageOneTransmissionData = {1/((2.41/12)*(8.8507457916 )), 0.46, 2, 421.88}; //rateOfChangeTWithRespectToV, efficiency, numberOfMotors, gearRatio
        public static final double stageOneLength = 28.75;
        public static final double[] stageOnePivotCoordinate = {-4.864, 18.66};
        public static final boolean stageOneRedirected = true;
        public static final double[] stageOneSpringMountCoordinate = {9.636, 14};
        public static final double[] stageOneSpringRedirectCoordinate = {-0.7145, 8.125};
        public static final double stageOneSpringRestLength = 0; //undecided
        public static final double[] stageOneCBCoordinate = {12.14, 1.723};
        public static final double stageOneSpringConstant = 0; //undecided
        
        public static final double[] stageTwoCG = {10.9, 0, 2};
        public static final double[] stageTwoTransmissionData = {1/((3.28/12/12)*(8.8507457916 )), 0.46, 2, 145.45}; //rateOfChangeTWithRespectToV, efficiency, numberOfMotors, gearRatio
        public static final double stageTwoLength = 28.75;
        public static final double[] stageTwoPivotCoordinate = {stageOnePivotCoordinate[0] + stageOneLength, stageOnePivotCoordinate[1]};
        public static final boolean stageTwoRedirected = false;
        public static final double[] stageTwoSpringMountCoordinate = {-16.61, 1.723};
        public static final double[] stageTwoSpringRedirectCoordinate = {0, 0};
        public static final double stageTwoSpringRestLength = 0; //undecided
        public static final double[] stageTwoCBCoordinate = {-3.5, 0.3895};
        public static final double stageTwoSpringConstant = 0; //undecided

        public static final double[] endEffectorCG = {6.75, 0.75, 12.7};
        public static final double[] placingConeArmPosOne = {0,0}; //undecided
        public static final double[] placingConeArmPosTwo = {0,0}; //undecided
        public static final double[] placingConeArmPosThree = {0,0}; //undecided

        public static final double[] placingCubeArmPosOne = {0,0}; //undecided
        public static final double[] placingCubeArmPosTwo = {0,0}; //undecided
        public static final double[] placingCubeArmPosThree = {0,0}; //undecided


        public static final double angleToleranceToUpdateEF = 0.5;
    }


    public static final class ModuleConstants {
        //Physical
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.8);
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.4286;

        //Conversions
        public static final double FALCON_TICKS = 2048;
        public static final double RADIANS_TO_FALCON = FALCON_TICKS / (2 * Math.PI);
        public static final double RADIANS_TO_TURNING = RADIANS_TO_FALCON * kTurningMotorGearRatio;
        public static final double WHEEL_CIRCUMFERENCE = kWheelDiameterMeters * Math.PI;
        public static final double METERS_TO_ROTATIONS = (1 / WHEEL_CIRCUMFERENCE);
        public static final double METERS_TO_DRIVE = METERS_TO_ROTATIONS * kDriveMotorGearRatio * FALCON_TICKS;
        public static final double METERS_TO_DRIVE_VELOCITY = METERS_TO_DRIVE / 10;
        public static final double ABS_TO_RADIANS = 2.0 * Math.PI;

        //Gains
            //Turn
            public static final double kPTurning = 0.21; //0.21 //works from 0.1-0.3 but 0.21 seems to offer low chattering and pretty quick alignment
            //Drive
            public static final double kAFFDrive = 0.015; //0.0151 //0.015
            public static final double kFDrive = .045012; //0.04390375 //0.03751 //.045012
            public static final double kPDrive = 0.02; //0.08 //0.02

        public static final double NEUTRAL_DEADBAND = 0.01;
    }

    public static final class DriveConstants {

        public static final double TRACK_WIDTH = Units.inchesToMeters(23.625); //need to find
        // Distance between right and left wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(23.625); //need to find


        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        );

        //TBD
        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 3;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 5;
        public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 7;
        public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 9;

        //TBD
        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 4;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 6;
        public static final int BACK_LEFT_TURNING_MOTOR_PORT = 8;
        public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 10;

        public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = true;
        public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = true;

        public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = true;
        public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = true;
        public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = true;

        //TBD
        public static final int FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 0;
        public static final int FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 1;
        public static final int BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 2;
        public static final int BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 3;

        //sort of calculated
        public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -2.260 +Math.PI; //-2.29+Math.PI
        public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -6.087+Math.PI; //-6.06+Math.PI
        public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -5.420+Math.PI; //-2.25
        public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -3.925+Math.PI; //-0.75

        public static final boolean FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

        public static final double RADIUS = Units.inchesToMeters(32/2);

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.8; //13.3 adjusted, 16.4 free //empirical 4.8 meters when not on ground
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / RADIUS;

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = Units.feetToMeters(100); //10 //100 for testing
        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 2 * Math.PI; //idk
        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        public static final double DEGREES_TO_RADIANS = (2*Math.PI) / 360;

        public static final double kPTurning = 0.0015; //0.0015 low-no oscillation
        public static final double kDTurning = 0.0; //0.0 unnecissary

        public static final double kPFudge = 0.02; //0.2 seems pretty close

        public static final boolean USE_NAV_X_OVER_PIGEON = false;


        public static final double posTolerance = Units.inchesToMeters(0.5);
        public static final double rotationTolerance = 2;

        
        public static final double GYRO_Z_ERROR = 0.674; //.674
        public static final double GYRO_MOUNT_POSE_PITCH = 0;
        public static final double GYRO_MOUNT_POSE_YAW = 0;
        public static final double GYRO_MOUNT_POSE_ROLL = 0;

        public static final double drivePValue = 0.5; //% speed for every meter away from target
        public static final double turnPValue = 1/180.0; //% speed for every degree away from target

        public static final double maxPowerOut = 0.3;
        public static final double maxTurningPowerOut = 0.3;

    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final int DRIVER_Y_AXIS = XboxController.Axis.kLeftY.value;
        public static final int DRIVER_X_AXIS = XboxController.Axis.kLeftX.value;
        public static final int DRIVER_ROT_AXIS = XboxController.Axis.kRightX.value;
        public static final int DRIVER_FIELD_ORIENTED_BUTTON_IDX = XboxController.Button.kRightBumper.value;

        public static final double DEADBAND = 0.06; //0.0275-0.03 //0.06
    }

    public static final class RobotConstants {
        public static final double MAIN_LOOP_PERIOD = 0.02;
        public static final double ROBOT_NOMINAL_VOLTAGE = 12.0;
    }

    public static final class PathingConstants{
        public static final double ROBOT_LENGTH = Units.inchesToMeters(34);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(34);
        public static final double ROBOT_RADIUS =  Math.sqrt(Math.pow(ROBOT_LENGTH /2,2)+Math.pow(ROBOT_WIDTH /2,2));

        public static final double maxLineDist = 8.0;
        public static final double lineDistIterator = 1.5;
        public static final double moveAngles = 8;
        public static final int maxRecursionDepth = 2;
        public static final int innerLineTestCount = 12;

        public static final double reachedInBetweenPointThreshold = Units.inchesToMeters(2);

      //  public static final double maxCPUTime = 0.30; //max fraction of thread time to spend on pathing
        public static final int minPathingDelay = 10; //min time to take in ms

        public static final double recalcThreshold = Units.inchesToMeters(2); // max distance to travel before recalculating trajectory



    }

    public static final class VisionConstants {
        public static final double MAX_AMBIGUITY = 0.05;
        public static final double CAM_X_OFFSET = Units.inchesToMeters(15.5-21);
        public static final double CAM_Y_OFFSET = Units.inchesToMeters(15.5-6);
        public static final double camDirFromCenter = Math.atan2(CAM_Y_OFFSET, CAM_X_OFFSET);
        public static final double camDistFromCenter = Math.sqrt(Math.pow(CAM_X_OFFSET,2)+Math.pow(CAM_Y_OFFSET,2));

    }

    public static final class FieldConstants{
        public static final double width1 = 16.56;
        public static final double height1 = 8.05; // 8.176  //we might need to re-measure some things 😭

        public static final double leftWallPos = -width1/2;
        public static final double rightWallPos = width1/2;
        public static final double bottomWallPos = -height1/2;
        public static final double topWallPos = height1/2;

        public static final double nodesWidth = 1.55;
        public static final double nodesHeight = 5.497;

        public static final double barrierLength = 1.984;

        public static final double chargeStationX = 4.417;
        public static final double chargeStationY = 1.267;
        public static final double chargeStationWidth = 1.931;
        public static final double chargeStationHeight = 2.471;

        public static final double chargeStationFarX = chargeStationX + chargeStationWidth/2; //roughly 5.374
        public static final double chargeStationCloseX = chargeStationX - chargeStationWidth/2;
        public static final double chargeStationTopY = chargeStationY + chargeStationHeight/2;
        public static final double chargeStationBottomY = chargeStationY - chargeStationHeight/2;

        public static final double doubleSubstationDepth = 0.404;

        public static final double distFromFarEdgesToTags = 1.013143;

        public static final double distBetweenTags = 1.676400;

        public static final double centerTagY = 1.234789;

        public static final double aprilTagX = width1/2 - nodesWidth;
        public static final double lonesomeAprilTagY = -2.741613;
        public static final double lonesomeAprilTagX = width1/2 - doubleSubstationDepth;

        public static final double aprilTagOriginX = rightWallPos;
        public static final double aprilTagOriginY = topWallPos;
        public static final double[] aprilTagYDiffsFromOriginInches = {610.77,610.77,610.77,636.96,14.25,40.45,40.45,40.45}; //https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf
        public static final double[] aprilTagXDiffsFromOriginInches = {42.19,108.19,174.19,265.74,265.74,174.19,108.19,42.19};

        public static final double nodeX1 = 7.071332;
        public static final double nodeX2 = 7.426611 + Units.inchesToMeters(1);
        public static final double nodeX3 = nodeX2+0.432133;

        public static final double topNodeY = 3.495389;
        public static final double yDistBetweenNodes = 0.558800;

        public static final double chargeStationEdgeLength = Units.inchesToMeters(6);




    }

}
