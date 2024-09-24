package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Arm;

public class ArmGo extends Command {
    private Arm arm;
    private double speed;
    //private double setpoint;

    public ArmGo(double speed) {
        this.arm = Arm.system();
        this.speed = speed;
        
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        //arm.setPos(setpoint);
        arm.percentOut(speed);
    }

    //@Override
    //public boolean isFinished() {
        //return Math.abs(arm.getEncoder().getPosition() - setpoint) <= 0.2;
    //    return true;
    //}

    @Override
    public void end(boolean interrupted) {
        arm.stopMotors();
    }   
}