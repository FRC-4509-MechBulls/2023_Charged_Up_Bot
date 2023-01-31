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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
