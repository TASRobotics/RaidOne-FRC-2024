package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Wrist;

public class WristHome extends Command {
    private Wrist wrist;
    private double setpoint;
    public WristHome(double setpoint) {
        this.wrist = Wrist.system();
        this.setpoint = setpoint;
        addRequirements(this.wrist);
    }

    @Override
    public void execute() {
        //wrist.setPos(setpoint);
        wrist.home();
    }

    //leo add
    @Override
    public boolean isFinished() {
        return wrist.isHomed();
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stopMotors();
    }
}