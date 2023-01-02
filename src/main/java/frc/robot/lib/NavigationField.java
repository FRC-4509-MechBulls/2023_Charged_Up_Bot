package frc.robot.lib;

import java.awt.geom.Line2D;
import java.util.ArrayList;

public class NavigationField {
ArrayList<Line2D.Double> barriers = new ArrayList<Line2D.Double>();
public NavigationField(){
    barriers.add(new Line2D.Double(1.55,0.8,2.47,0.8));
    barriers.add(new Line2D.Double(2.47,0.8,2.47,1.86));
    barriers.add(new Line2D.Double(2.47,1.86,1.55,1.86));
    barriers.add(new Line2D.Double(1.55,1.86,1.55,0.8));
}

}
