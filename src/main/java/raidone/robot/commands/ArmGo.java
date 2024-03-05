package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Arm;

public class ArmGo extends Command {
    private Arm arm;
    private double setpoint;

    public ArmGo(double setpoint) {
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
        return Math.abs(arm.getEncoder().getPosition() - setpoint) <= 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotors();
    }   
}