package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Climb;

public class ClimbHome extends Command {
    private Climb climb;

    public ClimbHome() {
        this.climb = Climb.system();
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.climbHome();
    }

    @Override
    public boolean isFinished() {
        return climb.getClimbLimit();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimbMotor();
    }
}