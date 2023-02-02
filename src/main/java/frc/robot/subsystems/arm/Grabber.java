// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants;


public class Grabber extends SubsystemBase {

  ArmStageOne armStageOne; //refactor this to armStageOneSubsystem >:( //can we name the variable "stageOne" cus I dont feel like typing arm for no reason
  ArmStageTwo armStageTwo;
  EndEffectorSubsystem endEffectorSubsystem;

  double stageOneAFF;
  double stageTwoAFF;
  double stageTwoGravityAFF;
  double stageOneGravityAFF;
  double[] stageTwoCGRelativeToOrigin;
  double[] eFCGRelativeToOrigin;
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
    stageTwoCGRelativeToOrigin = calculateRelativeCG(armStageOne.getRelativeCG(), armStageTwo.getRelativeCG());
    eFCGRelativeToOrigin = calculateRelativeCG(stageTwoCGRelativeToOrigin, endEffectorSubsystem.getRelativeCG());
    stageOneAFF = calculateAFF(calculateFusedCG(calculateFusedCG(armStageOne.getRelativeCG(), stageTwoCGRelativeToOrigin), eFCGRelativeToOrigin), armStageOne.getRelativeCB());
    stageTwoAFF = calculateAFF(calculateFusedCG(armStageTwo.getRelativeCG(), calculateRelativeCG(armStageTwo.getRelativeCG(), endEffectorSubsystem.getRelativeCG())), armStageTwo.getRelativeCB());
  }
  public double calculateAFF(double[] cG, double[] cB) {
    return calculateGravityTorque(new Rotation2d(cG[0], cG[1]).getRadians(), cG[2], calculateMagnitude(cG[0], cG[1])) - calculateCounterBalanceTorque(cB[3], cB[2], cB[0], cB[1]);
  }
  public double[] calculateRelativeCG (double[] origin, double[] point) {
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
  public double calculateCounterBalanceTorque(double cBAngleRad, double springForceLB, double cBx, double cBy) {
    return Math.sin(cBAngleRad) * springForceLB *  Math.hypot(cBx, cBy);
  }

  @Override
  public void periodic() {
    calculateArmData();
    // This method will be called once per scheduler run
  }
}