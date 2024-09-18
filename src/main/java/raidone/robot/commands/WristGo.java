package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Wrist;

public class WristGo extends Command {
    private Wrist wrist;
    private double speed;

    public WristGo(double speed) {
        this.wrist = Wrist.system();
        this.speed = speed;
        
        addRequirements(this.wrist);
    }

    @Override
    public void execute() {
        //wrist.setPos(setpoint);
        wrist.percentOut(speed);
    }

    // @Override
    // public boolean isFinished() {
    //     //return Math.abs(wrist.getEncoder().getPosition() - setpoint) <= 0.2;
    //     return true;
    // }

    @Override
    public void end(boolean interrupted) {
        //super.end(interrupted);
        wrist.stopMotors();
    }
}
