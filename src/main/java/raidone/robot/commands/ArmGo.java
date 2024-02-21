package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Arm;

public class ArmGo extends Command {    
    private static Arm arm;
    private double setpoint;

    public ArmGo(Arm arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(this.arm);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        arm.setPos(setpoint);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getEncoder().getPosition() - setpoint) <= 0.01;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotors();
    }
}