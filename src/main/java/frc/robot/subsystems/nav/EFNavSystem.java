package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.FieldObjects.FieldLine;
import frc.robot.lib.FieldObjects.Node;
import frc.robot.lib.LineIntersection;
import frc.robot.lib.MB_Math;
import frc.robot.subsystems.arm.Grabber;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.state.FMSGetter;
import frc.robot.subsystems.state.StateControllerSubsystem;
import org.opencv.core.Scalar;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;


import static frc.robot.Constants.EFPathingConstants.*;


public class EFNavSystem extends SubsystemBase {
Point2D.Double pivotPoint = new Point2D.Double(0,0);
EFPathingTelemetrySub telemetrySub;
    ArrayList<FieldLine> fieldLines = new ArrayList<FieldLine>();
    public EFNavSystem(EFPathingTelemetrySub telemetrySub){
        this.telemetrySub = telemetrySub;
        createBarriers();
        telemetrySub.updateBarriers(fieldLines);
    }

    public void periodic(){

    }

    private void createBarriers(){
        fieldLines.clear();
        fieldLines.add(new FieldLine(new Line2D.Double(-15,0,15,0),true,new Scalar(100,100,100))); //ground
        fieldLines.add(new FieldLine(new Line2D.Double(-BUMPER_X_FROM_ORIGIN,BUMPER_Y_FROM_ORIGIN,BUMPER_X_FROM_ORIGIN,BUMPER_Y_FROM_ORIGIN))); //bumper top
        fieldLines.add(new FieldLine(new Line2D.Double(-BUMPER_X_FROM_ORIGIN,BUMPER_Y_FROM_ORIGIN,-BUMPER_X_FROM_ORIGIN,0))); //bumper left
        fieldLines.add(new FieldLine(new Line2D.Double(BUMPER_X_FROM_ORIGIN,BUMPER_Y_FROM_ORIGIN,BUMPER_X_FROM_ORIGIN,0))); //bumper right
        fieldLines.add(new FieldLine(new Line2D.Double(-BUMPER_X_FROM_ORIGIN,0,BUMPER_X_FROM_ORIGIN,0))); //bumper bottom
    }

}
