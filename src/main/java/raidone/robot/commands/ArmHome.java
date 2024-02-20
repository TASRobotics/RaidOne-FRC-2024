package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Arm;
import raidone.robot.subsystems.Wrist;

public class ArmHome extends Command {    
    private static Arm arm;
    private static Wrist wrist;

    public ArmHome(Arm arm, Wrist wrist) {
        this.arm = arm;
        this.wrist = wrist;
        addRequirements(this.arm);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        if(!wrist.isHomed()){
            wrist.home();
        }else{
            arm.home();
        }
        
    }

    @Override
    public boolean isFinished() {
        return (arm.isHomed() && wrist.isHomed());
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotors();
        wrist.stopMotors();
    }
}
