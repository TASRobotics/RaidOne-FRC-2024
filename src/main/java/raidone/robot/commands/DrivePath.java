package raidone.robot.commands;

import static raidone.robot.Constants.Swerve.TRACK_WIDTH;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        driveTrajectory();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true, true);
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
    private Command driveTrajectory() {
        return Commands.runOnce( () -> AutoBuilder.followPath(path));
        // return Commands.runOnce( () -> new FollowPathHolonomic(
        //     path,
        //     swerve::getPose,
        //     swerve::getRelativeSpeeds,
        //     swerve::driveRelative,
        //     new HolonomicPathFollowerConfig(
        //         new PIDConstants(
        //             0.005,
        //             0.0,
        //             0.0
        //         ),
        //         new PIDConstants(
        //             0.37,
        //             0.0,
        //             0.0
        //         ),
        //         2.5,
        //         (TRACK_WIDTH / 2.0),
        //         new ReplanningConfig()
        //     ),
        //     () -> {
        //         var alliance = DriverStation.getAlliance();
        //         if (alliance.isPresent()) {
        //             return alliance.get() != DriverStation.Alliance.Red;
        //         }
        //         return true;
        //     },
        //     swerve
        // ));
    }

}