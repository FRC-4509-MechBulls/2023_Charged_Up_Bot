package frc.robot.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.PathingTelemetrySub;
import frc.robot.subsystems.SwerveSubsystem;

import java.awt.geom.Line2D;
import java.util.ArrayList;

public class NavigationField extends SubsystemBase {

ArrayList<Line2D.Double> barriers = new ArrayList<Line2D.Double>();
PathingTelemetrySub pTelemetrySub;
SwerveSubsystem swerveSubsystem;
public NavigationField(PathingTelemetrySub telemetrySub, SwerveSubsystem swerveSubsystem){
    this.pTelemetrySub = telemetrySub;
    this.swerveSubsystem = swerveSubsystem;

    barriers.add(new Line2D.Double(1,0.5,1,-0.5));
    this.pTelemetrySub.updateBarriers(barriers);
}

public void periodic(){
pTelemetrySub.updateRobotPose(swerveSubsystem.getEstimatedPosition());
}

public boolean clearPathOnLine(Line2D.Double line){
    double lineDir = Math.atan2(line.getY2() - line.getY1() , line.getX2() - line.getX1());
    double lineDist = Math.sqrt(Math.pow(line.getX1() - line.getX2(),2) + Math.pow(line.getY1() - line.getY2(),2));

    double edgePtX1 = line.getX1() + Math.cos(lineDir - Math.PI) * Constants.PathingConstants.kRobotRadius;
    double edgePtY1 = line.getX1() + Math.sin(lineDir - Math.PI) * Constants.PathingConstants.kRobotRadius;

    double edgePtX2 = line.getX1() + Math.cos(lineDir + Math.PI) * Constants.PathingConstants.kRobotRadius;
    double edgePtY2 = line.getX1() + Math.sin(lineDir + Math.PI) * Constants.PathingConstants.kRobotRadius;

    double destEdgePtX1 = edgePtX1 + (line.getX2() - line.getX1());

return false;
}

}
