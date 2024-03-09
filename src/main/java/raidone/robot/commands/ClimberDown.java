package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Climb;

// Manual home command
public class ClimberDown extends Command {
    private Climb climb;

    public ClimberDown() {
        this.climb = Climb.system();
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.climbHome();
        climb.followHome();
    }

    @Override
    public boolean isFinished() {
        return climb.getClimbLimit() && climb.getFollowLimit();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimbMotor();
        climb.stopFollowMotor();
    }
}