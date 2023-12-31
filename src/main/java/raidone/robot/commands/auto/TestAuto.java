package raidone.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import raidone.robot.RobotContainer;
import raidone.robot.commands.DrivePath;

public class TestAuto extends CommandBase {

    private final PathPlannerPath path1 = PathPlannerPath.fromPathFile("Path 1");

    public TestAuto() {}

    @Override
    public void initialize() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new DrivePath(RobotContainer.getSwerve(), path1, true)
        );

        commandGroup.schedule();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
