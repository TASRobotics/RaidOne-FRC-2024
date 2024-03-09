package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Climb;

// PID up command
public class ClimberUp extends Command {
    private Climb climb;
    private double climbSetpoint, followSetpoint;

    public ClimberUp(double climbSetpoint, double followSetpoint) {
        this.climb = Climb.system();
        this.climbSetpoint = climbSetpoint;
        this.followSetpoint = followSetpoint;

        addRequirements(climb);
    }

    @Override
    public void execute() {
        if (!climb.climbAtPos()) {
            climb.runClimb(climbSetpoint);
        } else {
            climb.stopClimbMotor();
        }
        if (!climb.followAtPos()) {
            climb.runFollow(followSetpoint);
        } else {
            climb.stopFollowMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return (climb.climbAtPos() && climb.followAtPos()) || (climb.getClimbLimit() && climb.getFollowLimit());
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimbMotor();
        climb.stopFollowMotor();
    }
}
