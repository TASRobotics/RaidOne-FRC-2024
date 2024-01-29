package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.IntakeArm;

public class ArmHome extends Command{
    private IntakeArm arm;

    public ArmHome(IntakeArm arm){
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        arm.armGoHome();
        arm.updateForLimSwitch();

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return arm.getArmForLim();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        arm.stopArm();
    }
}