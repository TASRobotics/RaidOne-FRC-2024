package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Arm;

public class AutoArm extends Command {    
    private static Arm arm;

    public AutoArm(Arm arm) {
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        arm.setPos();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotors();
    }
}