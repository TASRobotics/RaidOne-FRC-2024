package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Climb;

// Manual home command
public class ClimberHome extends Command {
    private Climb climb;
    private double speed;

    public ClimberHome(double speed) {
        this.climb = Climb.system();
        this.speed = speed;
        
        addRequirements(this.climb);
    }

    @Override
    public void execute() {
        if (!climb.getClimbLimit()) {
            climb.runClimb(-1 * speed);
        }
        if (!climb.getFollowLimit()) {
            climb.runFollow(-1 * speed);
        }
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
