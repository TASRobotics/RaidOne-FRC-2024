package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.Constants;
import raidone.robot.subsystems.Climb;

public class ClimbDownHalf extends Command {
    private Climb climb;
    private double speed;

    public ClimbDownHalf(double speed) {
        this.climb = Climb.system();
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.runClimb(-1 * speed);
    }

    @Override
    public boolean isFinished() {
        return climb.getClimbEncoderPos() <= Constants.Climb.HALFWAY_POS_ROT || climb.getClimbLimit();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimbMotor();
    }
}