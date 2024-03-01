package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Climb;

public class ClimbHome extends Command {
    private Climb climb;
    private double speed;

    public ClimbHome(double speed) {
        this.climb = Climb.system();
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.runClimb(-1 * speed);
    }

    @Override
    public boolean isFinished() {
        return climb.getClimbLimit();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimbMotor();
    }
}