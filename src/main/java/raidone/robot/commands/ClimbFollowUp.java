package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.Constants;
import raidone.robot.subsystems.Climb;

public class ClimbFollowUp extends Command {
    private Climb climb;
    private double speed;

    public ClimbFollowUp(double speed) {
        this.climb = Climb.system();
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.runFollow(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climb.getFollowEncoderPos()) >= Constants.Climb.TOP_POS_ROT;
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopFollowMotor();
    }
} 