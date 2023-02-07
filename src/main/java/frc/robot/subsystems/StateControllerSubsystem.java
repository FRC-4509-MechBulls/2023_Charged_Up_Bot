// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StateControllerSubsystem extends SubsystemBase {
  /** Creates a new StateControllerSubsystem. */
  public StateControllerSubsystem() {
    
  }

  int setPointIndex = 0;
  AgnosticGrabberMode agnosticGrabberMode = AgnosticGrabberMode.HOLDING;
  ItemType itemType = ItemType.NONE;
  ItemFallen itemFallen = ItemFallen.NOT_FALLEN;
  Level placingLevel = Level.POS1;

public int getSetpointIndex(){
    return setPointIndex;
}
public void setSetpointIndex(int setPointIndex){this.setPointIndex = setPointIndex;}
  public void iterateSetPoint(){
    setPointIndex++;
  }
  public void decimateSetPoint(){
    setPointIndex--;
  }

  public enum ItemType{CUBE,CONE,NONE}
  public enum ItemFallen{FALLEN_CONE,NOT_FALLEN}
  public enum AgnosticGrabberMode{INTAKING, HOLDING, PLACING} //ignores item held and depth
  public enum Level{POS1, POS2, POS3} //placing mode - pos1 is ground, pos3 is highest

  public Grabber.ArmModes getArmMode(){
    if(agnosticGrabberMode == AgnosticGrabberMode.INTAKING){
      if(itemType == ItemType.CONE){
        if(itemFallen == ItemFallen.FALLEN_CONE)
          return Grabber.ArmModes.INTAKING_CONE_FALLEN;
        return Grabber.ArmModes.INTAKING_CONE_UPRIGHT;
      }
      if(itemType == ItemType.CUBE)
        return Grabber.ArmModes.INTAKING_CUBE;
    }
    if(agnosticGrabberMode == AgnosticGrabberMode.PLACING){
      if(itemType == ItemType.CONE){
        switch (placingLevel){
          case POS1: return Grabber.ArmModes.PLACING_CONE_LVL1;
          case POS2: return Grabber.ArmModes.PLACING_CONE_LVL2;
          case POS3: return Grabber.ArmModes.PLACING_CONE_LVL3;
        }
      }
      if(itemType == ItemType.CUBE){
        switch(placingLevel){
          case POS1: return Grabber.ArmModes.PLACING_CUBE_LVL1;
          case POS2: return Grabber.ArmModes.PLACING_CUBE_LVL2;
          case POS3: return Grabber.ArmModes.PLACING_CUBE_LVL3;
        }
      }
    }

    return Grabber.ArmModes.HOLDING;
  }
  public Grabber.EFModes getEFMode(){
    if(agnosticGrabberMode == AgnosticGrabberMode.INTAKING){
      switch (itemType){
        case CONE: return Grabber.EFModes.INTAKING_CONE;
        case CUBE: return Grabber.EFModes.INTAKING_CUBE;
      }
    }
    if(agnosticGrabberMode == AgnosticGrabberMode.HOLDING){
      switch (itemType){
        case CONE: return Grabber.EFModes.HOLDING_CONE;
        case CUBE: return Grabber.EFModes.HOLDING_CUBE;
      }
    }
    if(agnosticGrabberMode == AgnosticGrabberMode.PLACING){
      switch (itemType){
        case CONE: return Grabber.EFModes.PLACING_CONE;
        case CUBE: return Grabber.EFModes.PLACING_CUBE;
      }
    }
  return Grabber.EFModes.STOPPED;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
