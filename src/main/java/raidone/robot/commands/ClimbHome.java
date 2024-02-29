package raidone.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.Constants;
import raidone.robot.subsystems.Climb;

public class ClimbHome extends Command {
    private Climb climb;
    private double speed;

    public ClimbHome( double speed) {
        this.climb = Climb.system();
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.run(-1 * speed);
    }

    @Override
    public boolean isFinished() {
        return climb.getEncoderPos() <= Constants.Climb.BOTTOM_POS_ROT;
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopMotors();
    }
} 