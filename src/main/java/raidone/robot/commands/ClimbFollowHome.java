package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Climb;

public class ClimbFollowHome extends Command {
    private Climb climb;
    private double speed;

    public ClimbFollowHome( double speed) {
        this.climb = Climb.system();
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.runFollow(-1 * speed);
    }

    @Override
    public boolean isFinished() {
        return climb.getFollowLimit();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopFollowMotor();
    }
} 