// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.FMSGetter;
import frc.robot.subsystems.arm.Grabber;

public class StateControllerSubsystem extends SubsystemBase {
  /** Creates a new StateControllerSubsystem. */
  FMSGetter fmsGetter;
  public StateControllerSubsystem(FMSGetter fmsGetter) {
    this.fmsGetter = fmsGetter;
  }

  int setPointIndex = 0;
  int placingLevelIndex = 0;
  private AgnosticGrabberMode agnosticGrabberMode = AgnosticGrabberMode.HOLDING;
  private ItemType itemType = ItemType.NONE;
  private ItemFallen itemFallen = ItemFallen.NOT_FALLEN;
  private Level placingLevel = Level.POS1;
  private Level[] placingLevels = {Level.POS1,Level.POS2,Level.POS3};

  public void setAgnosticGrabberMode(AgnosticGrabberMode agnosticGrabberMode){this.agnosticGrabberMode = agnosticGrabberMode;}
  public void setItemType(ItemType itemType){this.itemType = itemType;}
  public void setItemFallen(ItemFallen itemFallen){this.itemFallen = itemFallen;}
  public void setPlacingLevel(Level placingLevel){this.placingLevel = placingLevel;}

  public AgnosticGrabberMode getAgnosticGrabberMode(){return agnosticGrabberMode;}
  public ItemType getItemType(){return itemType;}
  public ItemFallen getItemFallen(){return itemFallen;}
  public Level getPlacingLevel(){return placingLevel;}

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

  public void iteratePlacingLevel(){placingLevelIndex++; updatePlacingLevelFromIndex();}
  public void decimatePlacingLevel(){placingLevelIndex--; updatePlacingLevelFromIndex();}

int lastPlacingPOV = -1;
  double lastFallenConeTriggerVal = 0;
  void placingPOVPressed(int pov){
    switch (pov){
      case 270: if(fmsGetter.isRedAlliance()) decimateSetPoint(); else iterateSetPoint(); break;
      case 90: if(fmsGetter.isRedAlliance()) iterateSetPoint(); else decimateSetPoint(); break;
      case 180: iteratePlacingLevel(); break;
      case 0: decimatePlacingLevel(); break;
    }
  }
  public void processRawAxisValues(int placingPOV, double fallenConeTriggerVal){
    if(lastPlacingPOV == -1 && placingPOV!=-1)
      placingPOVPressed(placingPOV);
    if(fallenConeTriggerVal>0.8)
      itemConeFallenButton();
lastPlacingPOV = placingPOV;
  }
  public void updatePlacingLevelFromIndex(){
    if(placingLevelIndex<0) placingLevelIndex = 0;
    if(placingLevelIndex>placingLevels.length-1) placingLevelIndex = placingLevels.length-1;
    placingLevel = placingLevels[placingLevelIndex];
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


  public void itemCubeButton(){itemType = ItemType.CUBE; itemFallen = ItemFallen.NOT_FALLEN;}
  public void itemConeFallenButton(){itemType = ItemType.CONE; itemFallen = ItemFallen.FALLEN_CONE;}
  public void itemConeUprightButton(){itemType = ItemType.CONE; itemFallen = ItemFallen.NOT_FALLEN;}

  public void setAgArmToIntake(){agnosticGrabberMode = AgnosticGrabberMode.INTAKING;}
  public void setAgArmToHolding(){agnosticGrabberMode = AgnosticGrabberMode.HOLDING;}

  public void setAgArmToPlacing(){agnosticGrabberMode = AgnosticGrabberMode.PLACING;}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
