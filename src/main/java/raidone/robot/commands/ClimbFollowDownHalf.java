package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.Constants;
import raidone.robot.subsystems.Climb;

public class ClimbFollowDownHalf extends Command {
    private Climb climb;
    private double speed;

    public ClimbFollowDownHalf(double speed) {
        this.climb = Climb.system();
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.runFollow(-1 * speed);
    }

    @Override
    public boolean isFinished() {
        return climb.getFollowEncoderPos() <= Constants.Climb.HALFWAY_POS_ROT || climb.getFollowLimit();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopFollowMotor();
    }
}