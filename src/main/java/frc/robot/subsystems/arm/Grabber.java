// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MB_Math;
import frc.robot.subsystems.state.StateControllerSubsystem;

import static frc.robot.Constants.ArmConstants;


public class Grabber extends SubsystemBase {

  StateControllerSubsystem stateController;
  StageOneSub stageOneSub; //refactor this to armStageOneSubsystem >:(
  StageTwoSub stageTwoSub;
  EFSub endEffectorSubsystem;


  private double stageOneAFF;
  private double stageTwoAFF;
  private double stageTwoGravityAFF;
  private double stageOneGravityAFF;

  /** Creates a new Grabber. */
  public Grabber(StageOneSub stageOneSub, StageTwoSub stageTwoSub, EFSub endEffectorSubsystem, StateControllerSubsystem stateController) {
    this.stageOneSub = stageOneSub;
    this.stageTwoSub = stageTwoSub;
    this.endEffectorSubsystem = endEffectorSubsystem;
    this.stateController = stateController;
  }
  public enum ArmModes {INTAKING_CUBE, INTAKING_CONE_UPRIGHT, INTAKING_CONE_FALLEN, HOLDING, PLACING_CONE_LVL1, PLACING_CONE_LVL2, PLACING_CONE_LVL3,PLACING_CUBE_LVL1,PLACING_CUBE_LVL2,PLACING_CUBE_LVL3}
  public enum EFModes {INTAKING_CONE, INTAKING_CUBE, HOLDING_CUBE, HOLDING_CONE, PLACING_CUBE, PLACING_CONE, STOPPED}

  public void setArmPosition(ArmModes armMode){
    double[] armPositions = getArmPositions(armMode);
    if(armPositions.length>=2)
      setArmPosition(armPositions);
    this.armMode = armMode;
  }

  public void setEndEffectorMode(EFModes effectorMode){
    switch(effectorMode){
      case INTAKING_CUBE: endEffectorSubsystem.intakeCube(); break;
      case INTAKING_CONE: endEffectorSubsystem.intakeCone(); break;
      case HOLDING_CONE: endEffectorSubsystem.holdCone(); break;
      case HOLDING_CUBE: endEffectorSubsystem.holdCube(); break;
      case PLACING_CONE: endEffectorSubsystem.placeCone(); break;
      case PLACING_CUBE: endEffectorSubsystem.placeCube(); break;
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
    }
    return new double[]{};
  }

  private EFModes efMode = EFModes.STOPPED;
  private ArmModes armMode = ArmModes.HOLDING;
  private EFModes desiredEFMode = EFModes.STOPPED;
  private ArmModes desiredArmMode = ArmModes.HOLDING;

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
  public ArmModes getArmMode(){return armMode;}




  public void setArmPosition(double[] armPosition){
    stageOneSub.setArmPositionRad(armPosition[0]);
    stageTwoSub.setArmPositionRad(armPosition[1]);
  }

  public void setDesiredArmAndEFModes(ArmModes armMode, EFModes efMode){
    setDesiredEFMode(efMode);
    setDesiredArmMode(armMode);
  }


  public void calculateArmData() {
    double stageTwoArmAngle = stageTwoSub.getAngle();
    boolean stageTwoRedirected = stageTwoSub.getRedirected();
    double[] stageTwoSpringMountCoordinate = stageTwoSub.getSpringMountCoordinate();
    double[] stageTwoSpringRedirectCoordinate = stageTwoSub.getSpringRedirectCoordinate();
    double stageTwoSpringRestLength = stageTwoSub.getSpringRestLength();
    double[] stageTwoKcBCoordinate = stageTwoSub.getkCBCoordinate();
    double stageTwoSpringConstant = stageTwoSub.getSpringConstant();

    double stageOneArmAngle = stageOneSub.getAngle();
    boolean stageOneRedirected = stageOneSub.getRedirected();
    double[] stageOneSpringMountCoordinate = stageOneSub.getSpringMountCoordinate();
    double[] stageOneSpringRedirectCoordinate = stageOneSub.getSpringRedirectCoordinate();
    double stageOneSpringRestLength = stageOneSub.getSpringRestLength();
    double[] stageOneKcBCoordinate = stageOneSub.getkCBCoordinate();
    double stageOneSpringConstant = stageOneSub.getSpringConstant();

    double[] eFCG = endEffectorSubsystem.getCG();
    double[] stageTwoCG = stageTwoSub.getCG();
    double[] stageOneCG = stageOneSub.getCG();

    double[] eFCGRelativeToStageTwo = sumCGCoordinates(stageTwoCG, eFCG);
    double[] eFStageTwoFusedCG = calculateFusedCG(eFCGRelativeToStageTwo, stageTwoCG);

    double[] eFStageTwoFusedCGRelativeToStageOne = sumCGCoordinates(stageOneCG, eFStageTwoFusedCG);
    double[] eFStageTwoStageOneFusedCG = calculateFusedCG(eFStageTwoFusedCGRelativeToStageOne, stageOneCG);

    double[] stageOneTransmissionData = stageOneSub.getTransmissionData();
    double[] stageTwoTransmissionData = stageTwoSub.getTransmissionData();

    stageTwoAFF = calculateAFF(eFStageTwoFusedCG, stageTwoArmAngle, stageTwoRedirected, stageTwoSpringMountCoordinate, stageTwoSpringRedirectCoordinate, stageTwoSpringRestLength, stageTwoKcBCoordinate, stageTwoSpringConstant, stageOneTransmissionData);
    stageOneAFF = calculateAFF(eFStageTwoStageOneFusedCG, stageOneArmAngle, stageOneRedirected, stageOneSpringMountCoordinate, stageOneSpringRedirectCoordinate, stageOneSpringRestLength, stageOneKcBCoordinate, stageOneSpringConstant, stageTwoTransmissionData);
  }


  public double calculateCounterBalanceTorque(double armAngle, boolean redirected, double[] springMountCoordinate, double[] springRedirectCoordinate, double springRestLength, double[] kcBCoordinate, double springConstant) {
    boolean springRedirected = redirected;
    double referenceAngle = armAngle;
    double kSpringConstant = springConstant;
    double[] kCBCoordinate = kcBCoordinate; //coordinate relative to pivot point and arm where spring attaches to arm
    double kSpringRestLength = springRestLength;
    double[] kSpringRedirectCoordinate = springRedirectCoordinate;
    double[] kSpringMountCoordinate = springMountCoordinate;

    double kCBX = kCBCoordinate[0];
    double kCBY = kCBCoordinate[1];
    double kCBAngle = new Rotation2d(kCBCoordinate[0], kCBCoordinate[1]).getRadians();

    double cBAngle = referenceAngle + kCBAngle;
    double cBRadius = calculateMagnitude(kCBX, kCBY);

    double cBX = Math.cos(cBAngle) * cBRadius;
    double cBY = Math.sin(cBAngle) * cBRadius;

    double[] cBCoordinate = {cBX, cBY}; //coordinate relative to pivot point and ground where spring attaches to arm

    double springCurrentLength;
    double springAngle;
    if (springRedirected) {
      double[] springMountRedirectVector = subtractCoordinates(kSpringRedirectCoordinate, kSpringMountCoordinate);
      double springMountRedirectX = springMountRedirectVector[0];
      double springMountRedirectY = springMountRedirectVector[1];
      double springMountRedirectDistance = calculateMagnitude(springMountRedirectX, springMountRedirectY);
      double[] springRedirectCBVector = subtractCoordinates(cBCoordinate, kSpringRedirectCoordinate);
      double springRedirectCBX = springRedirectCBVector[0];
      double springRedirectCBY = springRedirectCBVector[1];
      double springRedirectCBDistance = calculateMagnitude(springRedirectCBX, springRedirectCBY);
      springCurrentLength = springMountRedirectDistance + springRedirectCBDistance;
      springAngle = new Rotation2d(springRedirectCBX, springRedirectCBY).getRadians();
    } else {
      double[] springMountCBVector = subtractCoordinates(cBCoordinate, kSpringMountCoordinate);
      double springMountCBX = springMountCBVector[0];
      double springMountCBY = springMountCBVector[1];
      double springMountCBDistance = calculateMagnitude(springMountCBX, springMountCBY);
      springCurrentLength = springMountCBDistance;
      springAngle = new Rotation2d(springMountCBX, springMountCBY).getRadians();
    }

    double springApproachAngle = Math.abs(MathUtil.angleModulus(cBAngle - springAngle));

    double springDisplacement = springCurrentLength - kSpringRestLength;

    double springGrossForce = springDisplacement * kSpringConstant;
    double cBRealForce = Math.cos(springApproachAngle) * springGrossForce;

    //double[] cB = {cBX, cBY, kSpringConstant * (Math.hypot(kCB[4] - cBAngle.getCos(), kCB[5] - cBAngle.getSin()) - kCB[3]), new Rotation2d(kCB[4] - cBAngle.getCos(), kCB[5] - cBAngle.getSin()).getRadians() - cBAngle.getRadians()};

    double cBTorque = cBRealForce * cBRadius;

    return cBTorque;
  }

  public void updateArmData() {
    stageOneSub.setAFF(stageOneAFF);
    stageTwoSub.setAFF(stageTwoAFF);
  }

  public double calculateAFF(double[] armCG, double kArmAngle, boolean kRedirected, double[] kSpringMountCoordinate, double[] kSpringRedirectCoordinate, double kSpringRestLength, double[] kCBCoordinate, double kSpringConstant, double[] transmissionData) {
    double[] cG = armCG;
    double armAngle = kArmAngle;
    boolean redirected = kRedirected;
    double[] springMountCoordinate = kSpringMountCoordinate;
    double[] springRedirectCoordinate = kSpringRedirectCoordinate;
    double springRestLength = kSpringRestLength;
    double[] kcBCoordinate = kCBCoordinate;
    double springConstant = kSpringConstant;

    double torque = calculateArmTorque(cG, armAngle, redirected, springMountCoordinate, springRedirectCoordinate, springRestLength, kcBCoordinate, springConstant);
    double torqueCoefficient = calculateTorqueCoefficient(transmissionData);

    double voltage = torque * torqueCoefficient;

    return voltage;
  }
  public double calculateTorqueCoefficient(double[] transmissionData) {
    double rateOfChangeTWithRespectToV = transmissionData[0];
    double efficiency = transmissionData[1];
    double numberOfMotors = transmissionData[2];
    double gearRatio = transmissionData[3];

    return (1/rateOfChangeTWithRespectToV) * (1/efficiency) * (1/numberOfMotors) * (1/gearRatio);
  }
  public double calculateArmTorque(double[] cG, double kArmAngle, boolean kRedirected, double[] kSpringMountCoordinate, double[] kSpringRedirectCoordinate, double kSpringRestLength, double[] kCBCoordinate, double kSpringConstant) {
    double cGX = cG[0];
    double cGY = cG[1];
    double cGMass = cG[2];

    double armAngle = kArmAngle;
    boolean redirected = kRedirected;
    double[] springMountCoordinate = kSpringMountCoordinate;
    double[] springRedirectCoordinate = kSpringRedirectCoordinate;
    double springRestLength = kSpringRestLength;
    double[] kcBCoordinate = kCBCoordinate;
    double springConstant = kSpringConstant;

    double cGAngle = calculateCGAngleRad(cG);
    double cGDist = Math.hypot(cGX, cGY);

    double gravityTorque = calculateGravityTorque(cGAngle, cGMass, cGDist);
    double counterBalanceTorque = calculateCounterBalanceTorque(armAngle, redirected, springMountCoordinate, springRedirectCoordinate, springRestLength, kcBCoordinate, springConstant);

    return gravityTorque - counterBalanceTorque;
  }
  public double calculateCGAngleRad(double[] cG) {
    return new Rotation2d(cG[0], cG[1]).getRadians();
  }
  public double[] sumCGCoordinates (double[] origin, double[] point) {
    return new double[] {point[0] + origin[0], point[1] + origin[1], point[2]};
  }
  public double[] calculateFusedCG(double[] cGOne, double[] cGTwo) {
    double magnitude = calculateMagnitude(cGOne[2], cGTwo[2]);
    return new double[] {((cGOne[2]/magnitude) * cGOne[0]) + ((cGTwo[2]/magnitude) * cGTwo[0]),
            ((cGOne[2]/magnitude) * cGOne[1]) + ((cGTwo[2]/magnitude) * cGTwo[1]),
            cGOne[2] + cGTwo[2]};
  }
  public double calculateMagnitude(double valueOne, double valueTwo) {
    return Math.sqrt(Math.pow(valueOne, 2) + Math.pow(valueTwo, 2));
  }
  public double calculateGravityTorque(double cGAngleRad, double massLb, double cGDistanceIn) {
    return Math.cos(cGAngleRad) * massLb * cGDistanceIn;
  }

  public double[] subtractCoordinates(double[] coordinate, double[] origin) { //difference between two coordinates as a coordinate
    double coordinateX = coordinate[0];
    double coordinateY = coordinate[1];

    double originX = origin[0];
    double originY = origin[1];

    double x = coordinateX-originX;
    double y = coordinateY-originY;

    return new double[] {x, y};
  }

  public double[] sumCoordinates(double[] coordinate, double[] origin) { //difference between two coordinates as a coordinate
    double coordinateX = coordinate[0];
    double coordinateY = coordinate[1];

    double originX = origin[0];
    double originY = origin[1];

    double x = coordinateX + originX;
    double y = coordinateY + originY;

    return new double[] {x, y};
  }

  public double calculateCoordinateMagnitude(double[] coordinate) {
    double x = coordinate[0];
    double y = coordinate[1];

    double magnitude = Math.hypot(x, y);

    return magnitude;
  }

  public double[] convertGrabberXYToThetaOmega(double[] coordinate) {
    double[] rawCoordinate = coordinate;

    double[] pivotOne = stageOneSub.getPivotCoordinate();

    double[] coordinateRelativeToPivotOne = subtractCoordinates(rawCoordinate, pivotOne);

    double x = coordinateRelativeToPivotOne[0]; //coordinates relative to pivot
    double y = coordinateRelativeToPivotOne[1];

    double referenceAngle = new Rotation2d(x, y).getRadians(); //overall angle
    double stageOneLength = stageOneSub.getLength();
    double stageTwoLength = stageTwoSub.getLength();
    double totalLength = calculateCoordinateMagnitude(coordinateRelativeToPivotOne); //overall length

    double theta = referenceAngle + calculateLawOfCosines(stageOneLength, stageTwoLength, totalLength);
    double omega = referenceAngle + calculateLawOfCosines(stageTwoLength, totalLength, stageOneLength);

    return new double[] {theta, omega};
  }

  public double calculateLawOfCosines(double clockwiseAdjacentLength, double oppositeLength, double counterclockwiseAdjacentLength) {
    double a = clockwiseAdjacentLength;
    double b = oppositeLength;
    double c = counterclockwiseAdjacentLength;
    return Math.acos((Math.pow(a, 2) + Math.pow(c, 2) - Math.pow(b, 2)) / (2*a*c));
  }

  public double getStageOneAFF() {
    return stageOneAFF;
  }
  public double getStageTwoAFF() {
    return stageTwoAFF;
  }


  @Override
  public void periodic() {
    calculateArmData();
    updateArmData();
    // This method will be called once per scheduler run
   // setDesiredArmAndEFModes(stateController.getArmMode(), );
    setEndEffectorMode(stateController.getEFMode());
 //   SmartDashboard.putString("EFMode",stateController.getEFMode().toString());
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
