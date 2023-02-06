package frc.robot.lib;

public class Node {
    public enum Level{POS1, POS2, POS3} //pos1 is ground, pos3 is highest
    public enum NodeType{CUBE,CONE,HYBRID}

    private double x;
    private double y;
    private NodeType nodeType;
    private Level level;

    public Node(double x, double y, NodeType nodeType, Level level){
        this.x = x;
        this.y = y;
        this.nodeType = nodeType;
        this.level = level;
    }
    public double getX(){return x;}
    public double getY(){return y;}
    public NodeType getNodeType(){return nodeType;}
    public Level getLevel(){return level;}

}
