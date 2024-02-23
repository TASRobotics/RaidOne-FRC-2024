package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Wrist;

public class WristHome extends Command {    
    private static Wrist wrist;

    public WristHome(Wrist wrist) {
        this.wrist = wrist;
        addRequirements(this.wrist);
    }

    @Override
    public void initialize(){
        
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
