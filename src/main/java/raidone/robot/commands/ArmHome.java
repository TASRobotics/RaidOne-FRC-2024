package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Arm;

public class ArmHome extends Command {
    private Arm arm;
    private double setpoint;
    public ArmHome(double setpoint) {
        this.arm = Arm.system();
        this.setpoint = setpoint;
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        
        arm.setPos(setpoint);
    }

    @Override
    public boolean isFinished() {
        return arm.isHomed();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotors();
    }
}
