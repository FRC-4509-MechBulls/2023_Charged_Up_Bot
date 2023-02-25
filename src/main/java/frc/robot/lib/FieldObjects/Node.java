package frc.robot.lib.FieldObjects;

import frc.robot.subsystems.state.StateControllerSubsystem;

public class Node {

    public enum NodeType{CUBE,CONE,HYBRID}

    private double x;
    private double y;
    private NodeType nodeType;
    private StateControllerSubsystem.Level level;

    public Node(double x, double y, NodeType nodeType, StateControllerSubsystem.Level level){
        this.x = x;
        this.y = y;
        this.nodeType = nodeType;
        this.level = level;
    }
    public double getX(){return x;}
    public double getY(){return y;}
    public NodeType getNodeType(){return nodeType;}
    public StateControllerSubsystem.Level getLevel(){return level;}

}
