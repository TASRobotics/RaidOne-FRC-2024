package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Intake;

public class IntakeEject extends Command {
    private Intake intake;

    public IntakeEject() {
        intake = Intake.system();

        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.run(-1 * 1.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
