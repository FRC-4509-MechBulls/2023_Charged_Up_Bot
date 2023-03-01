// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.FieldObjects;

import frc.robot.subsystems.state.StateControllerSubsystem;

/** Add your docs here. */
public class PreplacedItem {

    private double x;
    private double y;
    private StateControllerSubsystem.ItemType itemType;
    //private PreplacedItems itemIndex;
    
    //public enum PreplacedItems{B1, B2, B3, B4, R1, R2, R3, R4}
    
    public PreplacedItem(double x, double y, StateControllerSubsystem.ItemType itemType/*, PreplacedItems itemIndex*/) {
        this.x = x;
        this.y = y;
        this.itemType = itemType;
        //this.itemIndex = itemIndex;
    }

    public double getX(){return x;}
    public double getY(){return y;}
    public StateControllerSubsystem.ItemType getItemType(){return itemType;}

    public void setItemType(/*PreplacedItems itemIndex,*/ StateControllerSubsystem.ItemType itemType) {
        
    }

}
