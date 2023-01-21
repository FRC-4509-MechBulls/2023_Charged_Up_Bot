package frc.robot.lib;

import javax.sound.sampled.Line;
import java.awt.geom.Line2D;

public class Barrier {
private Line2D.Double line;
    public Barrier(Line2D.Double line){
        this.line = line;
    }
    public Line2D.Double getLine(){
        return line;
    }
}
