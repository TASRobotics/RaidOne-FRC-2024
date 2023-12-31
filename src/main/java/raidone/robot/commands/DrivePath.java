package raidone.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import raidone.robot.Constants.SwerveConstants;
import raidone.robot.subsystems.Swerve;

public class DrivePath extends CommandBase {

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
                    initialPose.holonomicRotation
                )
            );
        }
    }

    @Override
    public void execute() {
        driveTrajectory(swerve, path);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false);
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
    private CommandBase driveTrajectory(Swerve swerve, PathPlannerPath path) {
        return Commands.runOnce( () -> new FollowPathWithEvents(
            new FollowPathHolonomic(
                path,
                swerve::getPose,
                swerve::getRelativeSpeeds,
                swerve::driveRelative,
                new HolonomicPathFollowerConfig(
                    new PIDConstants(
                        SwerveConstants.kPathing_kP,
                        SwerveConstants.kPathing_kI,
                        SwerveConstants.kPathing_kD
                    ),
                    new PIDConstants(
                        SwerveConstants.kRotor_kP,
                        SwerveConstants.kRotor_kI,
                        SwerveConstants.kRotor_kD
                    ),
                    4.5, // Assuming MK4i module is using L2 ratio with NEO V1.0/1.1
                    0.4,
                    new ReplanningConfig()
                ),
                swerve
            ),
            path,
            swerve::getPose
        ));
    }

}
