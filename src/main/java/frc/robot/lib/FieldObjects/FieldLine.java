package frc.robot.lib.FieldObjects;

import org.opencv.core.Scalar;

import java.awt.geom.Line2D;

public class FieldLine {
private Line2D.Double line;
private Scalar color = new Scalar(255,255,255);
private boolean isABarrier = true;

    public FieldLine(Line2D.Double line){
        this.line = line;
    }
    public FieldLine(Line2D.Double line, boolean isABarrier){
        this(line);
        this.isABarrier = isABarrier;
    }
    public FieldLine(Line2D.Double line,boolean isABarrier, Scalar color){
        this(line,isABarrier);
        this.color = color;
    }

    public void setLine(Line2D.Double line){
        this.line.setLine(line.getP1(),line.getP2());
    }

    public Line2D.Double getLine(){
        return line;
    }
    public Scalar getColor(){
        return color;
    }

    public boolean isABarrier(){return isABarrier;}
}
