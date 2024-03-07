package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.Constants;
import raidone.robot.subsystems.Climb;

public class ClimbUp extends Command {
    private Climb climb;
    private double position;

    public ClimbUp(double position) {
        this.climb = Climb.system();
        this.position = position;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.runClimb(position);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climb.getClimbEncoderPos() - position) < 0.2 || position >= 0 ? false : climb.getClimbLimit();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimbMotor();
    }
}