package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.nav.NavigationField;
import frc.robot.subsystems.drive.SwerveSubsystem;


public class DirectToPointCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    Pose2d desiredPose;
    double distToFinished;
    double timeout;
    double posTolerance = Constants.DriveConstants.posTolerance;
    double rotationTolerance = Constants.DriveConstants.rotationTolerance;

    public DirectToPointCommand(SwerveSubsystem swerveSubsystem,Pose2d desiredPose, double timeout) {
        this.swerveSubsystem = swerveSubsystem;
        this.desiredPose = desiredPose;
        this.timeout = timeout;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);

    }

    double posP = 0.2;
    double rotP = 0.2;

    public DirectToPointCommand(SwerveSubsystem swerveSubsystem, Pose2d desiredPose, double timeout, double posTolerance, double rotationTolerance, double posP, double rotP){
        this(swerveSubsystem,desiredPose,timeout);
        this.posTolerance = posTolerance;
        this.rotationTolerance = rotationTolerance;
        this.posP = posP;
        this.rotP = rotP;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */

    double startTime;
    public void initialize() {
        firstExecuteDone = false;
    }

    public void start(){
        this.startTime = Timer.getFPGATimestamp();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    boolean firstExecuteDone = false;

    @Override
    public void execute() {
        if(!firstExecuteDone){
            start();
            firstExecuteDone = true;
        }
        swerveSubsystem.driveToPose(desiredPose,posP,rotP);
    }


    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        boolean timedOut = Timer.getFPGATimestamp() - startTime>timeout;
        boolean reachedGoal = Math.sqrt(Math.pow(desiredPose.getX() - swerveSubsystem.getEstimatedPosition().getX(),2)+Math.pow(desiredPose.getY() - swerveSubsystem.getEstimatedPosition().getY(),2)) < posTolerance;
        boolean reachedRotation = Math.abs(desiredPose.getRotation().getDegrees() - swerveSubsystem.getEstimatedPosition().getRotation().getDegrees()) < rotationTolerance;
        return timedOut || (reachedGoal && reachedRotation);
    }


    @Override
    public void end(boolean interrupted) {

    }
}
