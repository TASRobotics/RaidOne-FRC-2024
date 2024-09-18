package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Arm;

public class ArmStop extends Command {
    private Arm arm;

    public ArmStop() {
        this.arm = Arm.system();
        
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
       arm.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
