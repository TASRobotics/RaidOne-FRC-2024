package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Wrist;

public class WristPID extends Command{
    private Wrist wrist = Wrist.system();
    private double target;

    public WristPID(double target){
        this.target = target;

        addRequirements(wrist);

    }
    @Override
    public void execute() {
        wrist.moveTo(this.target);
    }
    @Override
    public void end(boolean interrupted) {
        wrist.stopMotors();
    }
}
