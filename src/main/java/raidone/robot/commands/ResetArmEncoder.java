package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Arm;

public class ResetArmEncoder extends Command{
    private Arm arm;
    
    public ResetArmEncoder(){
        this.arm = Arm.system();
        addRequirements(this.arm);
    }

    public void execute(){
        if(arm.getLimit()) {
            arm.getEncoder().setPosition(0);
        }
    }
}