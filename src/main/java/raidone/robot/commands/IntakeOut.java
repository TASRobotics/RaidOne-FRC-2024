package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Intake;

public class IntakeOut extends Command {
    private Intake intake;
    private double percent;

    public IntakeOut(double p) {
        intake = Intake.system();
        percent = p;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.run(percent);

    }

    @Override
    public boolean isFinished() {
        // return intake.getLimit();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
