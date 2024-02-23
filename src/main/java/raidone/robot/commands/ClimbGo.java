package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Climb;
import raidone.robot.subsystems.Arm;

public class ClimbGo extends Command {
    private Climb climb;
    private double speed;

    public ClimbGo(Climb climb, double speed) {
        this.climb = climb;
        this.speed = speed;
        addRequirements(this.climb);
    }

    @Override
    public void execute() {
        climb.run(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopMotors();
    }
}
