package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Wrist;

public class WristHome extends Command {
    private Wrist wrist;

    public WristHome() {
        this.wrist = Wrist.wristSys;
        addRequirements(this.wrist);
    }

    @Override
    public void execute() {
        wrist.home();
    }

    @Override
    public boolean isFinished() {
        return wrist.isHomed();
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stopMotors();
    }
}
