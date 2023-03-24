// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MB_Math;
import frc.robot.subsystems.nav.EFNavSystem;
import frc.robot.subsystems.nav.EFPathingTelemetrySub;
import frc.robot.subsystems.state.StateControllerSubsystem;

import static frc.robot.Constants.ArmConstants;
import static frc.robot.Constants.ArmConstants.stageOneInBetweenPlacingAngleRad;
import static frc.robot.Constants.ArmConstants.stageOneInBetweenPlacingThresholdRad;

import java.awt.geom.Point2D;


public class Grabber extends SubsystemBase {

  StateControllerSubsystem stateController;
  //???
  StageOneSub stageOneSub; //refactor this to armStageOneSubsystem >:(
  StageTwoSub stageTwoSub;
  EFSub endEffectorSubsystem;
  EFNavSystem eFNavSystem;

  private double stageOneAFF;
  private double stageTwoAFF;
  private double[] setpointXY = {1, 1};
  private double[] setpointThetaPhi;
  private double[] eFPosition;

  public enum ArmModes {INTAKING_CUBE, INTAKING_CONE_UPRIGHT, INTAKING_CONE_FALLEN, HOLDING, PLACING_CONE_LVL1, PLACING_CONE_LVL2, PLACING_CONE_LVL3,PLACING_CUBE_LVL1,PLACING_CUBE_LVL2,PLACING_CUBE_LVL3, POST_PLACING_CONE_LVL1, POST_PLACING_CONE_LVL2, POST_PLACING_CONE_LVL3, POST_PLACING_CUBE_LVL1, POST_PLACING_CUBE_LVL2, POST_PLACING_CUBE_LVL3}
  public enum EFModes {INTAKING_CONE, INTAKING_CUBE, HOLDING_CUBE, HOLDING_CONE, PLACING_CUBE_BOTTOM,PLACING_CUBE_TOP, PLACING_CONE, STOPPED}

  private EFModes efMode = EFModes.STOPPED;
  private ArmModes armMode = ArmModes.HOLDING;
  private EFModes desiredEFMode = EFModes.STOPPED;
  private ArmModes desiredArmMode = ArmModes.HOLDING;
  EFPathingTelemetrySub telemetrySub;

  /** Creates a new Grabber. */
  public Grabber(StageOneSub stageOneSub, StageTwoSub stageTwoSub, EFSub endEffectorSubsystem, StateControllerSubsystem stateController, EFNavSystem eFNavSystem, EFPathingTelemetrySub telemetrySub) {
    this.stageOneSub = stageOneSub;
    this.stageTwoSub = stageTwoSub;
    this.endEffectorSubsystem = endEffectorSubsystem;
    this.stateController = stateController;
    this.eFNavSystem = eFNavSystem;
    this.telemetrySub = telemetrySub;
    //SmartDashboard.putNumber("test_inX",1);
    //SmartDashboard.putNumber("test_inY",1);
  }
  //random???
  public void setArmPosition(ArmModes armMode){
    double[] armPositions = getArmPositions(armMode);
    if(armPositions.length>=2)
      setSetpointXY(armPositions);
    this.armMode = armMode;
  }
  public void setEndEffectorMode(EFModes effectorMode){
    switch(effectorMode){
      case INTAKING_CUBE: endEffectorSubsystem.intakeCube(); break;
      case INTAKING_CONE: endEffectorSubsystem.intakeCone(); break;
      case HOLDING_CONE: endEffectorSubsystem.holdCone(); break;
      case HOLDING_CUBE: endEffectorSubsystem.holdCube(); break;
      case PLACING_CONE: endEffectorSubsystem.placeCone(); break;
      case PLACING_CUBE_BOTTOM: endEffectorSubsystem.placeCubeBottom(); break;
      case PLACING_CUBE_TOP: endEffectorSubsystem.placeCubeTop(); break;
      case STOPPED: endEffectorSubsystem.stopMotors(); break;
    }
    efMode = effectorMode;
  }
  public double[] getArmPositions(ArmModes armMode){
    switch (armMode){
      case INTAKING_CUBE: return ArmConstants.intakingCubesArmPos;
      case INTAKING_CONE_UPRIGHT: return ArmConstants.intakingConesUprightArmPos;
      case INTAKING_CONE_FALLEN: return ArmConstants.intakingConesFallenArmPos;
      case HOLDING: return ArmConstants.holdingArmPos;

      case PLACING_CONE_LVL1: return ArmConstants.placingConeArmPosOne;
      case PLACING_CONE_LVL2: return ArmConstants.placingConeArmPosTwo;
      case PLACING_CONE_LVL3: return ArmConstants.placingConeArmPosThree;

      case PLACING_CUBE_LVL1: return ArmConstants.placingCubeArmPosOne;
      case PLACING_CUBE_LVL2: return ArmConstants.placingCubeArmPosTwo;
      case PLACING_CUBE_LVL3: return ArmConstants.placingCubeArmPosThree;

        case POST_PLACING_CONE_LVL1: return ArmConstants.postPlacingConeArmPosOne;
        case POST_PLACING_CONE_LVL2: return ArmConstants.postPlacingConeArmPosTwo;
        case POST_PLACING_CONE_LVL3: return ArmConstants.postPlacingConeArmPosThree;

        case POST_PLACING_CUBE_LVL1: return ArmConstants.postPlacingCubeArmPosOne;
        case POST_PLACING_CUBE_LVL2: return ArmConstants.postPlacingCubeArmPosTwo;
        case POST_PLACING_CUBE_LVL3: return ArmConstants.postPlacingCubeArmPosThree;
    }
    return new double[]{};
  }
  public void setDesiredEFMode(EFModes desiredEFMode){
    this.desiredEFMode = desiredEFMode;
  }
  public void setDesiredArmMode(ArmModes desiredArmMode){
    this.desiredArmMode = desiredArmMode;
  }
  public EFModes getDesiredEFMode(){
    return desiredEFMode;
  }
  public ArmModes getDesiredArmMode(){
    return desiredArmMode;
  }
  public EFModes getEFMode(){
    return efMode;
  }
  public ArmModes getArmMode(){
    return armMode;
  }
  public void setDesiredArmAndEFModes(ArmModes armMode, EFModes efMode){
    setDesiredEFMode(efMode);
    setDesiredArmMode(armMode);
  }
  //config
  //setters
  public void setSetpointXY(double[] coordinate){
    setpointXY = coordinate;
  }

  //getters
  //util
  private void calculateGrabberData() {
    double stageTwoArmAngle = stageTwoSub.getAngle();
    double stageOneArmAngle = stageOneSub.getAngle();
    double[] stageTwoDefaultSpringStartCoordinateRelativeToPivot = stageTwoSub.getDefaultSpringStartCoordinateRelativeToPivot();
    double[] stageOneDefaultSpringStartCoordinateRelativeToPivot = stageOneSub.getDefaultSpringStartCoordinateRelativeToPivot();
    double[] stageTwoDefaultSpringEndCoordinateRelativeToPivot = stageTwoSub.getDefaultSpringEndCoordinateRelativeToPivot();
    double[] stageOneDefaultSpringEndCoordinateRelativeToPivot = stageOneSub.getDefaultSpringEndCoordinateRelativeToPivot();
    double stageTwoSpringConstant = stageTwoSub.getSpringConstant();
    double stageOneSpringConstant = stageOneSub.getSpringConstant();
    double stageTwoRestingSpringLength = stageTwoSub.getRestingSpringLength();
    double stageOneRestingSpringLength = stageOneSub.getRestingSpringLength();
    double stageTwoVoltsPerTorque = stageTwoSub.getVoltsPerTorque();
    double stageOneVoltsPerTorque = stageOneSub.getVoltsPerTorque();

    double stageOneMass = stageOneSub.getMass();
    double stageTwoMass = stageTwoSub.getMass();
    double eFMass = endEffectorSubsystem.getMass();
    double stageTwoAndEFMass = eFMass + stageTwoMass;
    double stageOneAndStageTwoAndEFMass = stageTwoAndEFMass + stageOneMass;

    double stageTwoLength = stageTwoSub.getLength();
    double stageOneLength = stageOneSub.getLength();
    double stageTwoAngleRelativeToFloor = stageOneArmAngle + stageTwoArmAngle;

    double[] defaultStageOneCGCoordinateRelativeToStageOnePivot = stageOneSub.getDefaultCGCoordinateRelativeToPivot();
    double[] stageOneCGCoordinateRelativeToStageOnePivot = calculateRotateCoordinate(defaultStageOneCGCoordinateRelativeToStageOnePivot, stageOneArmAngle);
    double stageTwoPivotCoordinateRelativeToStageOnePivotY = new Rotation2d(stageOneArmAngle).getSin() * stageOneLength;
    double stageTwoPivotCoordinateRelativeToStageOnePivotX = new Rotation2d(stageOneArmAngle).getCos() * stageOneLength;
    double[] stageTwoPivotCoordinateRelativeToStageOnePivot = {stageTwoPivotCoordinateRelativeToStageOnePivotX, stageTwoPivotCoordinateRelativeToStageOnePivotY};
    double[] defaultStageTwoCGCoordinateRelativeToStageTwoPivot = stageTwoSub.getdefaultCGCoordinateRelativeToPivot();
    double[] stageTwoCGCoordinateRelativeToStageTwoPivot = calculateRotateCoordinate(defaultStageTwoCGCoordinateRelativeToStageTwoPivot, stageTwoAngleRelativeToFloor);
    double eFPivotCoordinateRelativeToStageTwoPivotCoordinateX = new Rotation2d(stageTwoAngleRelativeToFloor).getCos() * stageTwoLength;
    double eFPivotCoordinateRelativeToStageTwoPivotCoordinateY = new Rotation2d(stageTwoAngleRelativeToFloor).getSin() * stageTwoLength;
    double[] eFPivotCoordinateRelativeToStageTwoPivotCoordinate = {eFPivotCoordinateRelativeToStageTwoPivotCoordinateX, eFPivotCoordinateRelativeToStageTwoPivotCoordinateY};
    double[] eFCGCoordinateRelativeToEFPivot = endEffectorSubsystem.getCGCoordinateRelativeToPivot();
    double[] eFCGCoordinateRelativeToStageTwoPivot = calculateCoordinateSum(eFCGCoordinateRelativeToEFPivot, eFPivotCoordinateRelativeToStageTwoPivotCoordinate);
    double[] stageTwoAndEFCGCoordinateRelativeToStageTwoPivot = calculateCoordinateWeightedAverage(eFCGCoordinateRelativeToStageTwoPivot, eFMass, stageTwoCGCoordinateRelativeToStageTwoPivot, stageTwoMass);
    double[] stageTwoAndEFCGCoordinateRelativeToStageOnePivot = calculateCoordinateSum(stageTwoAndEFCGCoordinateRelativeToStageTwoPivot, stageTwoPivotCoordinateRelativeToStageOnePivot);
    double[] stageOneAndStageTwoAndEFCGCoordinateRelativeToStageOnePivot = calculateCoordinateWeightedAverage(stageTwoAndEFCGCoordinateRelativeToStageOnePivot, stageTwoAndEFMass, stageOneCGCoordinateRelativeToStageOnePivot, stageOneMass);

    stageTwoAFF = calculateAFF(stageTwoArmAngle, 
                              stageTwoDefaultSpringStartCoordinateRelativeToPivot, 
                              stageTwoDefaultSpringEndCoordinateRelativeToPivot, 
                              stageTwoSpringConstant, 
                              stageTwoRestingSpringLength, 
                              stageTwoAndEFMass, 
                              stageTwoAndEFCGCoordinateRelativeToStageTwoPivot, 
                              stageTwoVoltsPerTorque);
    stageOneAFF = calculateAFF(stageOneArmAngle, 
                              stageOneDefaultSpringStartCoordinateRelativeToPivot, 
                              stageOneDefaultSpringEndCoordinateRelativeToPivot, 
                              stageOneSpringConstant, 
                              stageOneRestingSpringLength, 
                              stageOneAndStageTwoAndEFMass, 
                              stageOneAndStageTwoAndEFCGCoordinateRelativeToStageOnePivot, 
                              stageOneVoltsPerTorque);

    updateSetpointThetaPhiButMisleading(setpointXY);

    double[] stageOnePivotCoordinateRelativeToOrigin = ArmConstants.stageOnePivotCoordinate;
    double[] stageTwoPivotCoordinateRelativeToOrigin = calculateCoordinateSum(stageOnePivotCoordinateRelativeToOrigin, stageTwoPivotCoordinateRelativeToStageOnePivot);
    double[] eFPivotCoordinateRelativeToOrigin = calculateCoordinateSum(stageTwoPivotCoordinateRelativeToOrigin, eFPivotCoordinateRelativeToStageTwoPivotCoordinate);

    eFPosition = eFPivotCoordinateRelativeToOrigin;
  }

  private void updateSetpointThetaPhiButMisleading(double[] setpointXY){
    if(stateController.getAgnosticGrabberMode() != StateControllerSubsystem.AgnosticGrabberMode.PLACING){
      firstStageHit93 = false;
      setpointThetaPhi = convertGrabberXYToThetaPhi(setpointXY);
      return;
    }
    //at this point you know that you are placing
    if(lastArmMode !=stateController.getArmMode()) firstStageHit93 = false;
    if(!firstStageHit93){
      setpointThetaPhi = new double[]{stageOneInBetweenPlacingAngleRad,stageTwoSub.getAngle()};
      if(Math.abs(stageOneSub.getAngle() - stageOneInBetweenPlacingAngleRad)< stageOneInBetweenPlacingThresholdRad)
        firstStageHit93 = true;
    }else{
      setpointThetaPhi = convertGrabberXYToThetaPhi(setpointXY);
    }
    lastArmMode = stateController.getArmMode();
  }

  private boolean firstStageHit93 = false;
  private ArmModes lastArmMode = ArmModes.HOLDING;


  private double calculateAFF(double armAngle, double[] defaultSpringStartCoordinateRelativeToPivot, double[] defaultSpringEndCoordinateRelativeToPivot, double springConstant, double restingSpringLength, double armMass, double[] cGCoordinateRelativeToPivot, double voltsPerTorque) {
    double passiveTorque = calculateArmTorque(armAngle, defaultSpringStartCoordinateRelativeToPivot, defaultSpringEndCoordinateRelativeToPivot, springConstant, restingSpringLength, armMass, cGCoordinateRelativeToPivot);
    double demandedTorque = -passiveTorque;

    double voltage = demandedTorque * voltsPerTorque;
    return voltage;
  }
  private double calculateArmTorque(double armAngle, double[] defaultSpringStartCoordinateRelativeToPivot, double[] defaultSpringEndCoordinateRelativeToPivot, double springConstant, double restingSpringLength, double armMass, double[] cGCoordinateRelativeToPivot) {
    double counterBalanceTorque = calculateCounterBalanceTorque(armAngle, defaultSpringStartCoordinateRelativeToPivot, defaultSpringEndCoordinateRelativeToPivot, springConstant, restingSpringLength);
    double gravityTorque = calculateGravityTorque(armMass, cGCoordinateRelativeToPivot);

    return (counterBalanceTorque - gravityTorque);
  }
  private double calculateCounterBalanceTorque(double armAngle, double[] defaultSpringStartCoordinateRelativeToPivot, double[] defaultSpringEndCoordinateRelativeToPivot, double springConstant, double restingSpringLength) {    
    double springEndDistanceFromPivot = calculateMagnitude(defaultSpringEndCoordinateRelativeToPivot[0], defaultSpringEndCoordinateRelativeToPivot[1]);
    double[] springStartCoordinate = defaultSpringStartCoordinateRelativeToPivot;
    double[] springEndCoordinate = calculateRotateCoordinate(defaultSpringEndCoordinateRelativeToPivot, armAngle);
    double[] currentSpringVector = subtractCoordinates(springEndCoordinate, springStartCoordinate);

    double springCurrentAngle = MathUtil.angleModulus(new Rotation2d(currentSpringVector[0], currentSpringVector[1]).getRadians());
    double springEndAngle = MathUtil.angleModulus(new Rotation2d(springEndCoordinate[0], springEndCoordinate[1]).getRadians());
    double approachAngle = springEndAngle - springCurrentAngle;

    double currentSpringLength = calculateMagnitude(currentSpringVector[0], currentSpringVector[1]);
    double displacement = currentSpringLength - restingSpringLength;
    if (displacement < 0) {
      return 0;
    }
    double grossForce = displacement * springConstant;
    double realForce = grossForce * Math.sin(approachAngle);
    double cBTorque = realForce * springEndDistanceFromPivot;

    return cBTorque;
  }
  private double calculateGravityTorque(double armMass, double[] cGCoordinateRelativeToPivot) {
    double cGDistance = calculateMagnitude(cGCoordinateRelativeToPivot[0], cGCoordinateRelativeToPivot[1]);
    double cGAngleRelativeToFloor = MathUtil.angleModulus(new Rotation2d(cGCoordinateRelativeToPivot[0], cGCoordinateRelativeToPivot[1]).getRadians());
    double realForce = armMass * Math.cos(cGAngleRelativeToFloor);
    double gravityTorque = realForce * cGDistance;

    return gravityTorque;
  }
  private void updateGrabberData() {
    stageOneSub.setAFF(stageOneAFF);
    stageTwoSub.setAFF(stageTwoAFF);
    stageOneSub.setSetpoint(setpointThetaPhi[0]);
    stageTwoSub.setSetpoint(setpointThetaPhi[1]);

    double eFPositionMetricX = Units.inchesToMeters(eFPosition[0]);
    double eFPositionMetricY = Units.inchesToMeters(eFPosition[1]);

    eFNavSystem.updatePivotPoint(new Point2D.Double(eFPositionMetricX, eFPositionMetricY));
  }
  private double[] calculateCoordinateSum(double[] coordinateOne, double[] coordinateTwo) {
    double newCoordinateX = coordinateOne[0] + coordinateTwo[0];
    double newCoordinateY = coordinateOne[1] + coordinateTwo[1];

    double[] newCoordinate = {newCoordinateX, newCoordinateY};

    return newCoordinate;
  }
  private double[] calculateCoordinateWeightedAverage(double[] coordinateOne, double weightOne, double[] coordinateTwo, double weightTwo) {
    double magnitude = weightOne + weightTwo;

    double newCoordinateX = (weightOne * coordinateOne[0] + weightTwo * coordinateTwo[0])/magnitude;
    double newCoordinateY = (weightOne * coordinateOne[1] + weightTwo * coordinateTwo[1])/magnitude;

    double[] newCoordinate = {newCoordinateX, newCoordinateY};

    return newCoordinate;
  }
  private double calculateMagnitude(double valueOne, double valueTwo) {
    return Math.sqrt(Math.pow(valueOne, 2) + Math.pow(valueTwo, 2));
  }
  private double[] calculateRotateCoordinate(double[] coordinate, double angle) {
    double magnitude = calculateMagnitude(coordinate[0], coordinate[1]);
    double originalDirection = MathUtil.angleModulus(new Rotation2d(coordinate[0], coordinate[1]).getRadians());
    Rotation2d newDirection = new Rotation2d(originalDirection + angle);
    double rotatedCoordinateX = newDirection.getCos() * magnitude;
    double rotatedCoordinateY = newDirection.getSin() * magnitude;
    double[] rotatedCoordinate = {rotatedCoordinateX, rotatedCoordinateY};
    return rotatedCoordinate;
  }
  private double[] subtractCoordinates(double[] coordinate, double[] origin) { //difference between two coordinates as a coordinate
    double coordinateX = coordinate[0];
    double coordinateY = coordinate[1];

    double originX = origin[0];
    double originY = origin[1];

    double x = coordinateX-originX;
    double y = coordinateY-originY;

    return new double[] {x, y};
  }
  private double calculateCoordinateMagnitude(double[] coordinate) {
    double x = coordinate[0];
    double y = coordinate[1];

    double magnitude = Math.hypot(x, y);

    return magnitude;
  }
  private double[] convertGrabberXYToThetaPhi(double[] coordinate) {
    double[] rawCoordinate = coordinate;

    //scale rawCoordinate back so the arm doesn't go crazy go stupid
    if(MB_Math.dist(ArmConstants.stageOnePivotCoordinate[0],ArmConstants.stageOnePivotCoordinate[1],rawCoordinate[0],rawCoordinate[1])>=ArmConstants.maxExtension){
      double angleToXY = Math.atan2(coordinate[1] - ArmConstants.stageOnePivotCoordinate[1],coordinate[0]- ArmConstants.stageOnePivotCoordinate[0]);
      double totalArmLength = ArmConstants.maxExtension;
      double x = ArmConstants.stageOnePivotCoordinate[0] + Math.cos(angleToXY) * totalArmLength;
      double y = ArmConstants.stageOnePivotCoordinate[1] + Math.sin(angleToXY) * totalArmLength;
      rawCoordinate = new double[]{x,y};
    }

    double[] pivotOne = stageOneSub.getPivotCoordinate();

    double[] coordinateRelativeToPivotOne = subtractCoordinates(rawCoordinate, pivotOne);

    double x = coordinateRelativeToPivotOne[0]; //coordinates relative to pivot
    double y = coordinateRelativeToPivotOne[1];

    double referenceAngle = new Rotation2d(x, y).getRadians(); //overall angle
    double stageOneLength = stageOneSub.getLength();
    double stageTwoLength = stageTwoSub.getLength();
    double totalLength = calculateCoordinateMagnitude(coordinateRelativeToPivotOne); //overall length

    double theta = referenceAngle + calculateLawOfCosines(stageOneLength, stageTwoLength, totalLength);
    double phi = -(Units.degreesToRadians(180) - calculateLawOfCosines(stageTwoLength, totalLength, stageOneLength));

    return new double[] {theta, phi};
  }
  private double calculateLawOfCosines(double clockwiseAdjacentLength, double oppositeLength, double counterclockwiseAdjacentLength) {
    double a = clockwiseAdjacentLength;
    double b = oppositeLength;
    double c = counterclockwiseAdjacentLength;
    return Math.acos((Math.pow(a, 2) + Math.pow(c, 2) - Math.pow(b, 2)) / (2*a*c));
  }
  public void overrideDesiredEFWait(){
    setEndEffectorMode(desiredEFMode);
  }
  double placeBeforePostTimestamp = -1;

  @Override
  public void periodic() {

    Pose2d nextNavPoint = eFNavSystem.getNextNavPoint();
    double[] navPointInInches = new double[]{Units.metersToInches(nextNavPoint.getX()), Units.metersToInches(nextNavPoint.getY())};
    setSetpointXY(navPointInInches);
    //SmartDashboard.putNumber("nextNav_x",nextNavPoint.getX());
    //SmartDashboard.putNumber("nextNav_y",nextNavPoint.getY());

    calculateGrabberData();
    updateGrabberData();

    double[] eFPositionButInMeters = new double[]{Units.inchesToMeters(eFPosition[0]),Units.inchesToMeters(eFPosition[1])};
    eFNavSystem.updatePivotPoint(eFPositionButInMeters);
    //SmartDashboard.putNumber("efPositionMeters_x",eFPositionButInMeters[0]);
   // SmartDashboard.putNumber("efPositionMeters_y",eFPositionButInMeters[1]);
    eFNavSystem.updateDesiredPose(getArmPositions(stateController.getArmMode()));

    setDesiredEFMode(stateController.getEFMode());

    if(getEFMode()!= getDesiredEFMode()){
      double distFromDest = MB_Math.dist(eFPositionButInMeters[0],eFPositionButInMeters[1],eFNavSystem.getDesiredPose().getX(),eFNavSystem.getDesiredPose().getY());
      SmartDashboard.putNumber("distFromDest",distFromDest);
      if(desiredEFMode!= EFModes.PLACING_CONE && desiredEFMode!=EFModes.PLACING_CUBE_TOP && desiredEFMode!=EFModes.PLACING_CUBE_BOTTOM) setEndEffectorMode(desiredEFMode);
      //if(distFromDest<Units.inchesToMeters(2))
        //setEndEffectorMode(desiredEFMode);

    }
    SmartDashboard.putBoolean("EFMatches", getEFMode()==getDesiredEFMode());
    SmartDashboard.putString("EFMode",getEFMode().toString());

    //when the arm is in place cone L2 or L3 and the EF matches the desired EF, move the arm to the appropriate post-placing position
    if(getEFMode()==getDesiredEFMode() && (stateController.getArmMode()==ArmModes.PLACING_CONE_LVL2 || stateController.getArmMode()==ArmModes.PLACING_CONE_LVL3)){
        if(placeBeforePostTimestamp==-1)
          placeBeforePostTimestamp = Timer.getFPGATimestamp();

        if(Timer.getFPGATimestamp() - placeBeforePostTimestamp > ArmConstants.timeBeforePostPlacing){
          stateController.setAgArmToPostPlacing();
          placeBeforePostTimestamp = -1;
        }

    }

    
   // double inX = SmartDashboard.getNumber("test_inX",1);
   // double inY = SmartDashboard.getNumber("test_inY",1);

    //double[] thetaPhi = convertGrabberXYToThetaPhi(new double[]{inX,inY});
   // SmartDashboard.putNumber("test_outX",Units.radiansToDegrees(thetaPhi[0]));
   // SmartDashboard.putNumber("test_outY",Units.radiansToDegrees(thetaPhi[1]));

    telemetrySub.updateStageOneAngle(stageOneSub.getAngle());
    telemetrySub.updateStageTwoAngle(stageTwoSub.getAngle());

    //setEndEffectorMode(stateController.getEFMode()); //???

    //double[] testingThetaPhi = convertGrabberXYToThetaPhi(eFPosition);
    //SmartDashboard.putNumber("theta", Units.radiansToDegrees(testingThetaPhi[0]));
    //SmartDashboard.putNumber("phi", Units.radiansToDegrees(testingThetaPhi[1]));


   // setDesiredArmAndEFModes(stateController.getArmMode(), ); //???
    //???
    /*


    stageOneSub.setAFF(getStageOneAFF());
    stageTwoSub.setAFF(getStageOneAFF());
    setArmPosition(getArmMode());


    if(getEFMode() != getDesiredEFMode()){ //wait for the arm to be within the distance to update the EF mode if intaking or placing
      boolean armOneAligned = MB_Math.isWithinDistanceOf(stageOneSub.getEncoderRad(), getArmPositions(getDesiredArmMode())[0],ArmConstants.angleToleranceToUpdateEF);
      boolean armTwoAligned = MB_Math.isWithinDistanceOf(stageTwoSub.getEncoderRad(), getArmPositions(getDesiredArmMode())[1],ArmConstants.angleToleranceToUpdateEF);
      boolean desiredEFIsHolding = getDesiredEFMode()==EFModes.HOLDING_CONE || getDesiredEFMode()==EFModes.HOLDING_CUBE;  //don't wait if you're switching to a holding mode
      if((armOneAligned && armTwoAligned) || (desiredEFIsHolding)){
        setEndEffectorMode(desiredEFMode);
      }
    }


    if(getArmMode()!=getDesiredArmMode()){ //instantly set arm position if it doesn't match the desired position
      armMode = getDesiredArmMode();
    }
*/

  }
}
