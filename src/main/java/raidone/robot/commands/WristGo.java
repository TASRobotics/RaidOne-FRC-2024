package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Wrist;

public class WristGo extends Command {
    private static Wrist wrist;
    private double setpoint;
    
    public WristGo(Wrist wrist, double setpoint) {
        this.wrist = wrist;
        this.setpoint = setpoint;
        addRequirements(this.wrist);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        wrist.setPos(setpoint);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(wrist.getEncoder().getPosition() - setpoint) <= 0.2;
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
