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

public int getSetpointIndex(){
    return setPointIndex;
}
  public void iterateSetPoint(){
    setPointIndex++;
  }
  public void decimateSetPoint(){
    setPointIndex--;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
