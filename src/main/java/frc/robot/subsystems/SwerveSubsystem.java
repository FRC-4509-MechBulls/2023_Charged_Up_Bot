// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.FieldTag;
import frc.robot.lib.MathThings;

public class SwerveSubsystem extends SubsystemBase {
  //Modules
    private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, 
                                                            DriveConstants.kFrontLeftTurningMotorPort, 
                                                            DriveConstants.kFrontLeftDriveEncoderReversed, 
                                                            DriveConstants.kFrontLeftTurningEncoderReversed, 
                                                            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
                                                            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
                                                            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, 
                                                            DriveConstants.kFrontRightTurningMotorPort, 
                                                            DriveConstants.kFrontRightDriveEncoderReversed, 
                                                            DriveConstants.kFrontRightTurningEncoderReversed, 
                                                            DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
                                                            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
                                                            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    private final SwerveModule backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, 
                                                          DriveConstants.kBackLeftTurningMotorPort, 
                                                          DriveConstants.kBackLeftDriveEncoderReversed, 
                                                          DriveConstants.kBackLeftTurningEncoderReversed, 
                                                          DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
                                                          DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
                                                          DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, 
                                                            DriveConstants.kBackRightTurningMotorPort, 
                                                            DriveConstants.kBackRightDriveEncoderReversed, 
                                                            DriveConstants.kBackRightTurningEncoderReversed, 
                                                            DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
                                                            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
                                                            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
  //Gyro
    private WPI_Pigeon2 gyro = new WPI_Pigeon2(9);

  //Odometry
    private SwerveDrivePoseEstimator odometry;
    private Pose2d initialPose;
    private ChassisSpeeds chassisSpeeds;

  //Values
    static boolean fieldOriented;
  private double translationMagnitude;

  private double translationMagnitudeScaled;
  private double rotationMagnitude = 0;
  private double scaledMagnitudeRotation = 0;
  Rotation2d translationDirection;
  Rotation2d rotationDirection;


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    initialPose = new Pose2d();
    constructOdometry();
        //put in thread so it doesn't stop the rest of our code from running
        new Thread(() -> {
                try {
                        Thread.sleep(1000);
                        gyro.configFactoryDefault();
                        gyro.configMountPose(AxisDirection.NegativeY, AxisDirection.PositiveZ);
                        zeroHeading();
                        Thread.sleep(1000);
                        constructOdometry(); //custructs odometry with newly corrct gyro values
                } catch (Exception e) {
                }
        
        }).start();
        //allows gyro to calibrate for 1 sec before requesting to reset^^
  }

  SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
  PIDController turningPID = new PIDController(DriveConstants.kPTurning, 0, DriveConstants.kDTurning);


  public void drive(double xSpeed, double ySpeed, double turningSpeed, boolean limited, boolean fieldOriented){
   // SmartDashboard.putNumber("dr_xSpeed",xSpeed);
   // SmartDashboard.putNumber("dr_ySpeed",ySpeed);
   // SmartDashboard.putNumber("dr_rSpeed",turningSpeed);

    //  Make the driving smoother, no sudden acceleration from sudden inputs
    if(limited) {
      xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
      ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
      turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
    }
    // Construct desired chassis speeds (convert to appropriate reference frames)
    ChassisSpeeds chassisSpeeds;

    SmartDashboard.putBoolean("fieldOriented", fieldOriented);

    //3.5. Fudge Factor to eliminate uncommanded change in direction when translating and rotating simultaneously
    ySpeed += turningSpeed * (-xSpeed) * DriveConstants.kPFudge;
    //debug output: ySpeed += turningSpeed * (-xSpeed) * SmartDashboard.getNumber("kPFudge", DriveConstants.kPFudge);
    xSpeed += turningSpeed * ySpeed * DriveConstants.kPFudge;
    //debug output: xSpeed += turningSpeed * ySpeed * SmartDashboard.getNumber("kPFudge", DriveConstants.kPFudge);

    // 3.55. P loops to create accurate outputs
    //turning
    //Debug intput: turningPID.setP(SmartDashboard.getNumber("kPTurning", DriveConstants.kPTurning));
    turningSpeed += turningPID.calculate(getAngularVelocity(), turningSpeed);

//4 - Convert and send chasis speeds
    if(fieldOriented)
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, getRotation2d());
    else
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);
  }
  public void toggleFieldOriented(){fieldOriented = !fieldOriented;}
  public boolean getFieldOriented(){return fieldOriented;}
  public void setFieldOriented(boolean set){fieldOriented = set;}
  public void resetPose(){
    odometry.resetPosition(new Rotation2d(), getPositions(), new Pose2d());
  }

  public void joystickDrive(double xSpeed, double ySpeed, double turningSpeed){
    //1.5 interpret joystick data
    //translation
    translationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)); //x and y vectors to polar magnitude
    translationDirection = new Rotation2d(ySpeed, xSpeed); //creates Rotation2D representing the direction of the input vectors using yspeed/xspeed as the cos/sin of the direction
    //Rotation
    rotationMagnitude = Math.abs(turningSpeed); //magnitude of joystick input
    rotationDirection = new Rotation2d(turningSpeed, 0); //conveys polarity +/-
    //Debug output: SmartDashboard.putNumber("directionR", rotationDirection.getCos());

    //1.55 scale magnitudes to reflect deadzone
    //Translation
    translationMagnitudeScaled = (1/(1- Constants.OIConstants.kDeadband))*(translationMagnitude - Constants.OIConstants.kDeadband); //converted to be representative of deadzone using point slope form (y-y1)=(m)(x-x1) -> (scaled-minimun raw input)=(maximum input/(1-deadzone))(raw input-deadzone) -> scaled=(maximum input/(1-deadzone))(raw input-deadzone)
    //Rotation
    scaledMagnitudeRotation = (1/(1- Constants.OIConstants.kDeadband))*(rotationMagnitude- Constants.OIConstants.kDeadband); //same algorithm as scaled magnitude translation

    //2.0 deadzone and construct outputs
    //Translation
    if (translationMagnitude > Constants.OIConstants.kDeadband) {
      ySpeed = translationDirection.getCos() * translationMagnitudeScaled; //original direction, scaled magnitude... this is kinda a misnomer because this x component itself is a magnitude, but it is representative of a direction of the raw input
      xSpeed = translationDirection.getSin() * translationMagnitudeScaled;
    } else {
      xSpeed = 0.0;
      ySpeed = 0.0; //zero inputs < deadzone
    }
    //Rotation
    if (rotationMagnitude > Constants.OIConstants.kDeadband) {
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
    drive(xSpeed, ySpeed, turningSpeed, true, this.fieldOriented);
  }

  //Configuration
  public void zeroHeading() { //reset gyroscope to have it set the current direction as the forward direction of field when robot boots up
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }

  public void zeroHeading(double yaw) {
    gyro.zeroGyroBiasNow();
    gyro.setYaw(yaw);
  }

  // constructs odometry object, called in the SwerveSubsystem constructor
  public void constructOdometry() {
    odometry = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, 
    getRotation2d(), 
    getPositions(), 
    initialPose);
  }
  
  // Getters
  // A number equal to x - (y Q), where Q is the quotient of x / y rounded to the nearest integer
  // (if x / y falls halfway between two integers, the even integer is returned)
  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw(), 360); //clamps value between -/+ 180 deg where zero is forward
  }

  // module states
  // groups each individual module state into an array to be used in other functions
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
  }

  // module positions
  // groups each individual module position into an array to be used in other functions
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
  }

  // chassis speeds
  public ChassisSpeeds getChassisSpeeds() {
    chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
    return fieldOriented ? 
    ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, getRotation2d()) :
    chassisSpeeds;
  }

  // get rotational velocity for closed loop
  public double getAngularVelocity() {
    return -gyro.getRate() * DriveConstants.kDegreesToRadians;
  }

  //since wpilib often wants heading in format of Rotation2d
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  //Setters 
  public void fieldOriented(boolean isFieldOriented) {
    fieldOriented = isFieldOriented;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    // normalizes wheel speeds in case max speed reached^^
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  // stops robot movement
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void updateOdometry() {
    //odometry.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(Math.toRadians(getHeading())), getStates()); //make rotation difference work!!!
    odometry.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(Math.toRadians(getHeading())), getPositions());

    //debug output: SmartDashboard.putNumber("OdoH", odometry.getEstimatedPosition().getRotation().getDegrees());
    //debug output: SmartDashboard.putNumber("gyroH", getHeading());
    //debug output: SmartDashboard.putNumber("OdoY", Units.metersToFeet(odometry.getEstimatedPosition().getY()));
    //debug output: SmartDashboard.putNumber("OdoX", Units.metersToFeet(odometry.getEstimatedPosition().getX()));      
  }
    
  public void debugOutputs() {
    //debug output: SmartDashboard.putNumber("CSH", getChassisSpeeds().omegaRadiansPerSecond);
    //debug output: SmartDashboard.putNumber("CSY", getChassisSpeeds().vyMetersPerSecond);
    //debug output: SmartDashboard.putNumber("CSX", getChassisSpeeds().vxMetersPerSecond);
  }

  // Periodic
  @Override
  public void periodic() {
    //constantly updates the gyro angle
    var gyroAngle = gyro.getRotation2d();
    //update odometry
      
    updateOdometry();

    //SmartDashboard.putNumber("o_x",odometry.getEstimatedPosition().getX());
    //SmartDashboard.putNumber("o_y",odometry.getEstimatedPosition().getY());
    //SmartDashboard.putNumber("o_r",odometry.getEstimatedPosition().getRotation().getDegrees());

    //dashboard outputs
      debugOutputs();
    }

public Pose2d getEstimatedPosition(){
    return odometry.getEstimatedPosition();
}

    // Vision stuff

public void fieldTagSpotted(FieldTag fieldTag, Transform3d transform, double latency, double ambiguity){
  if(ambiguity>Constants.VisionConstants.kMaxAmbiguity) return;

  //1. calculate X and Y position of camera based on X and Y components of tag and create a pose from that
    Rotation2d newRotation = new Rotation2d( ( Math.IEEEremainder((-transform.getRotation().getZ() - fieldTag.getPose().getRotation().getRadians()+4*Math.PI),2*Math.PI)));
    double newY = 0-( transform.getY()* Math.cos(-newRotation.getRadians()) + transform.getX() * Math.cos(Math.PI/2 - newRotation.getRadians()) + fieldTag.getPose().getY() ) ;
    double newX = 0- ( transform.getY()*Math.sin(-newRotation.getRadians()) + transform.getX() * Math.sin(Math.PI/2 - newRotation.getRadians()) + fieldTag.getPose().getX() ) ;
    Pose2d newPose = new Pose2d(newX,newY, newRotation);

    SmartDashboard.putNumber("new_x", newX);
    SmartDashboard.putNumber("new_y", newY);

    //2. Pass vision measurement to odometry
    SmartDashboard.putNumber("new rotation",newRotation.getDegrees());
    odometry.addVisionMeasurement(new Pose2d(newX,newY,newRotation),Timer.getFPGATimestamp() - latency*(1.0/1000));
  //  zeroHeading(newRotation.getDegrees());

  }

  public double[] getDesiredSpeeds(Pose2d pose){
    //get desired X and Y speed to reach a given pose
    double[] out = new double[3];
    double rotationDiff = (pose.getRotation().getRadians() - odometry.getEstimatedPosition().getRotation().getRadians());
    double xDiff = (pose.getX() - odometry.getEstimatedPosition().getX());
    double yDiff = (pose.getY() - odometry.getEstimatedPosition().getY());
    double dist = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));

    double dirToPose = Math.atan2(yDiff,xDiff);

    out[0] = dist * Math.cos(dirToPose +rotationDiff) * 0.5;
    out[1] = dist * Math.sin(dirToPose+rotationDiff) * 0.5;
    out[2] = rotationDiff * 0.7;

    if(Math.abs(rotationDiff)<Math.toRadians(2)) out[2] = 0;
    if(Math.abs(out[0])<0.005) out[0] = 0;
    if(Math.abs(out[1])<0.005) out[1] = 0;

    return out;
  }

  public void driveToPose(Pose2d pose){
    double[] speeds = getDesiredSpeeds(pose);

    double ang = Math.atan2(speeds[1],speeds[0]);
    double mag = Math.sqrt(Math.pow(speeds[0],2)+Math.pow(speeds[1],2));
    if(mag>0.2){
      speeds[0] = Math.cos(ang)*0.2;
      speeds[1] = Math.sin(ang)*0.2;
    }

    //speeds[0] = MathThings.absMax(speeds[0],0.2);
    //speeds[1] = MathThings.absMax(speeds[1],0.2);
    speeds[2] = MathThings.absMax(speeds[2],0.2);

    drive(speeds[0],speeds[1],speeds[2],true,false);
  }



}