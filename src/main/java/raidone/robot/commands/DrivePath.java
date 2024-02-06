package raidone.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Swerve;

public class DrivePath extends Command {
    
    private Swerve swerve;
    private PathPlannerPath path;
    private boolean isFirstPath;

    /**
     * Creates a new DrivePath
     * 
     * @param swerve Swerve subsystem
     * @param path PathPlanner path
     * @param isFirstPath First path of the autonomous sequence
     */
    public DrivePath(Swerve swerve, PathPlannerPath path, boolean isFirstPath) {
        this.swerve = swerve;
        this.path = path;
        this.isFirstPath = isFirstPath;

        addRequirements(swerve);
    }

    /**
     * Creates a new DrivePath
     * 
     * @param swerveSwerve subsystem
     * @param path PathPlanner path
     */
    public DrivePath(Swerve swerve, PathPlannerPath path) {
        this(swerve, path, false);
    }

    @Override
    public void initialize() {    
        PathPoint initialPose = path.getPoint(0);
        
        if (isFirstPath) {
            swerve.setPose(
                new Pose2d(
                    initialPose.position,
                    initialPose.rotationTarget.getTarget()
                )
            );
        }
    }

    @Override
    public void execute() {
        driveTrajectory(path);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(
            new Translation2d(0, 0),
            0,
            false,
            false
        );
    }

    @Override
    public boolean isFinished() {
        Translation2d finalPose = path.getPoint(path.numPoints() - 1).position;

        if (Math.abs(swerve.getPose().getX() - finalPose.getX()) < 0.1 && Math.abs(swerve.getPose().getY() - finalPose.getY()) < 0.1) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Moves the swerve subsystem along a desired trajectory
     * 
     * @param swerve Swerve subsystem
     * @param trajectory PathPlanner path
     */
    private void driveTrajectory(PathPlannerPath path) {
        AutoBuilder.followPath(path);
    }

}
