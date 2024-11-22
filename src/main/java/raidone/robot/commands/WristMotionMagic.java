package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.MotorConfigConstants;
import raidone.robot.subsystems.Wrist;

public class WristMotionMagic extends Command {
    private Wrist wrist;
    private double setpoint;
    public WristMotionMagic(double setpoint) {
        this.wrist = Wrist.system();
        this.setpoint = setpoint;
        addRequirements(this.wrist);
    }

    @Override
    public void execute() {
        wrist.setPos(setpoint);
    }

    //leo q: return hv conditions or no cuz we needed it to be false to minimize bounce 
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stopMotors();
    }
}
