package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Arm;

public class ArmMotionProfile extends Command {
    private Arm arm;
    private double speed;
    private double setpoint;

    public ArmMotionProfile(double setpoint) {
        this.arm = Arm.system();
        this.setpoint = setpoint;
        
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        //leo added
        arm.setPos(setpoint);
        
    }

    //@Override
    public boolean isFinished() {
        return false;
    //    return true;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotors();
    }  
}
