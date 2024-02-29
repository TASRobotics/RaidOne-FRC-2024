package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.Constants;
import raidone.robot.subsystems.Climb;

public class ClimbUp extends Command {
    private Climb climb;
    private double speed;

    public ClimbUp(double speed) {
        this.climb = Climb.system();
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.run(speed);
    }

    @Override
    public boolean isFinished() {
        return climb.getEncoderPos() >= Constants.Climb.TOP_POS_ROT;
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopMotors();
    }
} 