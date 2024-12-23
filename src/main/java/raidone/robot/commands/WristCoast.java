
package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Wrist;

public class WristCoast extends Command {
    private Wrist wrist;
    private boolean coast;

    public WristCoast(boolean coast) {
        this.wrist = Wrist.system();
        this.coast = coast;
        
        addRequirements(this.wrist);
    }

    @Override
    public void execute() {
        //wrist.setPos(setpoint);
        if(coast){
            wrist.enableCoast();
        } else {
            wrist.enableBrake();
        }
    }

    @Override
    public boolean isFinished() {
        //return Math.abs(wrist.getEncoder().getPosition() - setpoint) <= 0.2;
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        //wrist.stopMotors();
    }
}

