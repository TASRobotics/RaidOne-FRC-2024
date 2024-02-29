package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Arm;

public class ArmHome extends Command {
    private Arm arm;

    public ArmHome() {
        this.arm = Arm.system();
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        arm.home();
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
