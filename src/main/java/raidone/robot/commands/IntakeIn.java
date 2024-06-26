package raidone.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import raidone.robot.Robot;
import raidone.robot.RobotContainer;
import raidone.robot.subsystems.Intake;

public class IntakeIn extends Command {
    private Intake intake;
    private double percent;

    public IntakeIn(double p) {
        intake = Intake.system();
        percent = p;
        
        addRequirements(this.intake);
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
        return intake.getLimit();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
