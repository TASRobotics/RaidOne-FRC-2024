package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Wrist;

public class WristStop extends Command {
    private Wrist wrist;

    public WristStop() {
        this.wrist = Wrist.system();
        
        addRequirements(this.wrist);
    }

    @Override
    public void execute() {
       wrist.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
