// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Grabber extends SubsystemBase {

  private ArmStageOne armStageOne; //refactor this to armStageOneSubsystem >:( //can we name the variable "stageOne" cus I dont feel like typing arm for no reason
  private ArmStageTwo armStageTwo;
  private EndEffectorSubsystem endEffectorSubsystem;

  private double stageOneAFF;
  private double stageTwoAFF;
  private double stageTwoGravityAFF;
  private double stageOneGravityAFF;
  /** Creates a new Grabber. */
  public Grabber(ArmStageOne armStageOne, ArmStageTwo armStageTwo, EndEffectorSubsystem endEffectorSubsystem) {
    this.armStageOne = armStageOne;
    this.armStageTwo = armStageTwo;
    this.endEffectorSubsystem = endEffectorSubsystem;
  }
  public enum ArmModes {INTAKING_CUBE,INTAKING_CONE_UPRIGHT,INTAKING_CONE_FALLEN,HOLDING_CONE,HOLDING_CUBE,PLACINGLVL1,PLACINGLVL2,PLACINGLVL3}

  public void setArmPosition(ArmModes armMode){
    switch(armMode){
      case INTAKING_CUBE: setArmPosition(ArmConstants.intakingCubesArmPos); break;
      case INTAKING_CONE_UPRIGHT: setArmPosition(ArmConstants.intakingConesUprightArmPos); break;
      case INTAKING_CONE_FALLEN: setArmPosition(ArmConstants.intakingConesFallenArmPos); break;
      case HOLDING_CONE:
        case HOLDING_CUBE: setArmPosition(ArmConstants.holdingArmPos); break;
      case PLACINGLVL1: setArmPosition(ArmConstants.placingArmPosOne); break;
      case PLACINGLVL2: setArmPosition(ArmConstants.placingArmPosTwo); break;
      case PLACINGLVL3: setArmPosition(ArmConstants.placingArmPosThree); break;
    }
  }

  public void setEndEffectorMode(ArmModes armMode){
    switch(armMode){
      case INTAKING_CUBE: endEffectorSubsystem.intakeCube(); break;
      case INTAKING_CONE_UPRIGHT:
        case INTAKING_CONE_FALLEN: endEffectorSubsystem.intakeCone(); break;
      case HOLDING_CONE: endEffectorSubsystem.holdCone();
      case HOLDING_CUBE: endEffectorSubsystem.holdCube();
      case PLACINGLVL1: case PLACINGLVL2: case PLACINGLVL3: endEffectorSubsystem.placeCone();
    }
  }

  public void setArmPosition(double[] armPosition){
    if(armPosition.length<2) return;
    armStageOne.setArmPositionRad(armPosition[0]);
    armStageOne.setArmPositionRad(armPosition[1]);
  }


  public void setGripperMode(ArmModes armMode){
    setEndEffectorMode(armMode);
    setArmPosition(armMode);
  }

   
  public void calculateArmData() {
    double stageTwoArmAngle = armStageTwo.getAngle();
    boolean stageTwoRedirected = armStageTwo.getRedirected();
    double[] stageTwoSpringMountCoordinate = armStageTwo.getSpringMountCoordinate();
    double[] stageTwoSpringRedirectCoordinate = armStageTwo.getSpringRedirectCoordinate();
    double stageTwoSpringRestLength = armStageTwo.getSpringRestLength();
    double[] stageTwoKcBCoordinate = armStageTwo.getkCBCoordinate();
    double stageTwoSpringConstant = armStageTwo.getSpringConstant();

    double stageOneArmAngle = armStageOne.getAngle();
    boolean stageOneRedirected = armStageOne.getRedirected();
    double[] stageOneSpringMountCoordinate = armStageOne.getSpringMountCoordinate();
    double[] stageOneSpringRedirectCoordinate = armStageOne.getSpringRedirectCoordinate();
    double stageOneSpringRestLength = armStageOne.getSpringRestLength();
    double[] stageOneKcBCoordinate = armStageOne.getkCBCoordinate();
    double stageOneSpringConstant = armStageOne.getSpringConstant();

    double[] eFCG = endEffectorSubsystem.getCG();
    double[] stageTwoCG = armStageTwo.getCG();
    double[] stageOneCG = armStageOne.getCG();

    double[] eFCGRelativeToStageTwo = sumCGCoordinates(stageTwoCG, eFCG);
    double[] eFStageTwoFusedCG = calculateFusedCG(eFCGRelativeToStageTwo, stageTwoCG);

    double[] eFStageTwoFusedCGRelativeToStageOne = sumCGCoordinates(stageOneCG, eFStageTwoFusedCG);
    double[] eFStageTwoStageOneFusedCG = calculateFusedCG(eFStageTwoFusedCGRelativeToStageOne, stageOneCG);

    double[] stageOneTransmissionData = armStageOne.getTransmissionData();
    double[] stageTwoTransmissionData = armStageTwo.getTransmissionData();

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
    armStageOne.setAFF(stageOneAFF);
    armStageTwo.setAFF(stageTwoAFF);
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

    double[] pivotOne = armStageOne.getPivotCoordinate();

    double[] coordinateRelativeToPivotOne = subtractCoordinates(rawCoordinate, pivotOne);

    double x = coordinateRelativeToPivotOne[0]; //coordinates relative to pivot
    double y = coordinateRelativeToPivotOne[1];

    double referenceAngle = new Rotation2d(x, y).getRadians(); //overall angle
    double stageOneLength = armStageOne.getLength();
    double stageTwoLength = armStageTwo.getLength();
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
  }
}