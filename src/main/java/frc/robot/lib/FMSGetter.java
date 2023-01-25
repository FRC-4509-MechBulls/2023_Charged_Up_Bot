// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FMSGetter extends SubsystemBase {
  private NetworkTable FMSTable;

  /** Creates a new FMSGetter. */
  public FMSGetter() {
    // get the default NetworkTable instance
    NetworkTableInstance defaultInst = NetworkTableInstance.getDefault();
    FMSTable = defaultInst.getTable("FMSInfo");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
}

  public boolean isRedAlliance(){
    return FMSTable.getEntry("IsRedAlliance").getBoolean(true);
  }

  public int getStationNumber(){
    return (int)FMSTable.getEntry("StationNumber").getInteger(1);
  }

}
