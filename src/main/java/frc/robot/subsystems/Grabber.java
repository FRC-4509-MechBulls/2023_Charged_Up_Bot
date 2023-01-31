// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmStageOne;
import frc.robot.subsystems.arm.ArmStageTwo;
import static frc.robot.Constants.ArmConstants;


public class Grabber extends SubsystemBase {

  ArmStageOne armStageOne; //refactor this to armStageOneSubsystem >:(
  ArmStageTwo armStageTwo;
  EndEffectorSubsystem endEffectorSubsystem;
  /** Creates a new Grabber. */
  public Grabber(ArmStageOne armStageOne, ArmStageTwo armStageTwo, EndEffectorSubsystem endEffectorSubsystem) {
    this.armStageOne = armStageOne;
    this.armStageTwo = armStageTwo;
    this.endEffectorSubsystem = endEffectorSubsystem;
  }
  public enum ArmModes {INTAKING_CUBE, INTAKING_CONE_UPRIGHT, INTAKING_CONE_FALLEN, HOLDING, PLACING_CONE_LVL1, PLACING_CONE_LVL2, PLACING_CONE_LVL3,PLACING_CUBE_LVL1,PLACING_CUBE_LVL2,PLACING_CUBE_LVL3}
  public enum EFModes {INTAKING_CONE, INTAKING_CUBE, HOLDING_CUBE, HOLDING_CONE, PLACING_CUBE, PLACING_CONE}

  public void setArmPosition(ArmModes armMode){
    switch(armMode){
      case INTAKING_CUBE: setArmPosition(ArmConstants.intakingCubesArmPos); break;
      case INTAKING_CONE_UPRIGHT: setArmPosition(ArmConstants.intakingConesUprightArmPos); break;
      case INTAKING_CONE_FALLEN: setArmPosition(ArmConstants.intakingConesFallenArmPos); break;
      case HOLDING: setArmPosition(ArmConstants.holdingArmPos); break;

      case PLACING_CONE_LVL1: setArmPosition(ArmConstants.placingConeArmPosOne); break;
      case PLACING_CONE_LVL2: setArmPosition(ArmConstants.placingConeArmPosTwo); break;
      case PLACING_CONE_LVL3: setArmPosition(ArmConstants.placingConeArmPosThree); break;

      case PLACING_CUBE_LVL1: setArmPosition(ArmConstants.placingCubeArmPosOne); break;
      case PLACING_CUBE_LVL2: setArmPosition(ArmConstants.placingCubeArmPosTwo); break;
      case PLACING_CUBE_LVL3: setArmPosition(ArmConstants.placingCubeArmPosThree); break;
    }
  }

  public void setEndEffectorMode(EFModes effectorMode){
    switch(effectorMode){
      case INTAKING_CUBE: endEffectorSubsystem.intakeCube(); break;
      case INTAKING_CONE: endEffectorSubsystem.intakeCone(); break;
      case HOLDING_CONE: endEffectorSubsystem.holdCone(); break;
      case HOLDING_CUBE: endEffectorSubsystem.holdCube(); break;
      case PLACING_CONE: endEffectorSubsystem.placeCone(); break;
      case PLACING_CUBE: endEffectorSubsystem.placeCube(); break;
    }


    int i = 0;
    switch(i){
      case 0: break;
      case 1:
    }
  }

  public void setArmPosition(double[] armPosition){
    if(armPosition.length<2) return;
    armStageOne.setArmPositionRad(armPosition[0]);
    armStageOne.setArmPositionRad(armPosition[1]);
  }

  public void setArmAndEFModes(ArmModes armMode, EFModes efMode){
   
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
