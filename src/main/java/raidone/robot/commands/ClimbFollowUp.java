package raidone.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.Constants;
import raidone.robot.subsystems.Climb;

public class ClimbFollowUp extends Command {
    private Climb climb;
    private double position;

    public ClimbFollowUp(double position) {
        this.climb = Climb.system();
        this.position = position;
        // addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.runFollow(position);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climb.getFollowEncoderPos() - position) < 0.2 || position <= 0 ? false : climb.getFollowLimit();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopFollowMotor();
    }
} 