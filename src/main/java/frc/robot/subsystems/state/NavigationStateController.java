// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.state;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.state.StateControllerSubsystem.AgnosticGrabberMode;
import frc.robot.subsystems.state.StateControllerSubsystem.ItemFallen;
import frc.robot.subsystems.state.StateControllerSubsystem.ItemType;
import frc.robot.subsystems.state.StateControllerSubsystem.Level;

public class NavigationStateController extends SubsystemBase {
  StateControllerSubsystem stateController;
  int desiredState;
  //states
    //intaking
    public static final int stateIntakingFallenCone = 0;
    public static final int stateIntakingCube = 1;
    public static final int stateIntakingStandingCone = 2;
    //centering
    public static final int stateCenteringCone = 10;
    //holding
    public static final int stateHoldingCone = 20;
    public static final int stateHoldingCube = 21;
    //staging
    public static final int stateStagingCubeL1 = 30;
    public static final int stateStagingConeL1 = 31;
    public static final int stateStagingCubeL2 = 32;
    public static final int stateStagingConeL2 = 33;
    public static final int stateStagingCubeL3 = 34;
    public static final int stateStagingConeL3 = 35;
    //placing
    public static final int statePlacingCubeL1 = 40;
    public static final int statePlacingConeL1 = 41;
    public static final int statePlacingCubeL2 = 42;
    public static final int statePlacingConeL2 = 43;
    public static final int statePlacingCubeL3 = 44;
    public static final int statePlacingConeL3 = 45;

  /** Creates a new NavigationStateController. */
  public NavigationStateController(StateControllerSubsystem rc_stateController) {
    stateController = rc_stateController;
  }

  //util
  private void retrieveState() {
    AgnosticGrabberMode agnosticGrabberMode = stateController.getAgnosticGrabberMode();
    ItemType itemType = stateController.getItemType();
    ItemFallen itemFallen = stateController.getItemFallen();
    Level level = stateController.getPlacingLevel();

    //intaking
    if (agnosticGrabberMode.equals(AgnosticGrabberMode.INTAKING)) {
      //cone
      if (itemType.equals(ItemType.CONE)) {
        //fallen
        if (itemFallen.equals(ItemFallen.FALLEN_CONE)) {desiredState = stateIntakingFallenCone; return;}
        //standing
        if (itemFallen.equals(ItemFallen.NOT_FALLEN)) {desiredState = stateIntakingStandingCone; return;}
      }
      //cube
      if (itemType.equals(ItemType.CUBE)) {desiredState = stateIntakingCube; return;}
      //none
      if(itemType.equals(ItemType.NONE)) {desiredState = stateHoldingCube; return;}
    }
    //
  }

  @Override
  public void periodic() {
    retrieveState();
    // This method will be called once per scheduler run
  }
}
