package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Intake;

public class IntakeRetract extends Command {
    private Intake intake;

    public IntakeRetract(Intake in) {
        intake = in;
    }

    @Override
    public void initialize() {
        intake.resetEncoder();
    }

    @Override
    public void execute() {
        intake.run(-0.2);
    }

    @Override
    public boolean isFinished() {
        return intake.isRetracted();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
