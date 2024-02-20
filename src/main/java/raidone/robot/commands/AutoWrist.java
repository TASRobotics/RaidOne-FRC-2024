package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Wrist;

public class AutoWrist extends Command {
    private static Wrist wrist;
    
    public AutoWrist(Wrist wrist) {
        this.wrist = wrist;
        addRequirements(this.wrist);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
