// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MathThings;
import frc.robot.subsystems.arm.ArmStageOne;
import frc.robot.subsystems.arm.ArmStageTwo;

import javax.swing.plaf.nimbus.State;

import static frc.robot.Constants.ArmConstants;


public class Grabber extends SubsystemBase {

  StateControllerSubsystem stateController;
  ArmStageOne armStageOne; //refactor this to armStageOneSubsystem >:(
  ArmStageTwo armStageTwo;
  EndEffectorSubsystem endEffectorSubsystem;
  /** Creates a new Grabber. */
  public Grabber(ArmStageOne armStageOne, ArmStageTwo armStageTwo, EndEffectorSubsystem endEffectorSubsystem, StateControllerSubsystem stateController) {
    this.armStageOne = armStageOne;
    this.armStageTwo = armStageTwo;
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
    armStageOne.setArmPositionRad(armPosition[0]);
    armStageTwo.setArmPositionRad(armPosition[1]);
  }

  public void setDesiredArmAndEFModes(ArmModes armMode, EFModes efMode){
    setDesiredEFMode(efMode);
    setDesiredArmMode(armMode);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDesiredArmAndEFModes(stateController.getArmMode(), stateController.getEFMode());


    if(getEFMode() != getDesiredEFMode()){ //wait for the arm to be within the tolerance to update the EF mode if intaking or placing
      boolean armOneAligned = MathThings.isWithinRangeOf(armStageOne.getEncoderRad(), getArmPositions(getDesiredArmMode())[0],ArmConstants.angleToleranceToUpdateEF);
      boolean armTwoAligned = MathThings.isWithinRangeOf(armStageTwo.getEncoderRad(), getArmPositions(getDesiredArmMode())[1],ArmConstants.angleToleranceToUpdateEF);
      boolean desiredEFIsHolding = getDesiredEFMode()==EFModes.HOLDING_CONE || getDesiredEFMode()==EFModes.HOLDING_CUBE;  //don't wait if you're switching to a holding mode
      if((armOneAligned && armTwoAligned) || (desiredEFIsHolding)){
        setEndEffectorMode(desiredEFMode);
      }
    }

    if(getArmMode()!=getDesiredArmMode()){ //instantly set arm position if it doesn't match the desired position
      armMode = getDesiredArmMode();
    }


  }
}