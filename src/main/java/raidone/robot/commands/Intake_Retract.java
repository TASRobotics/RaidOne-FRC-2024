package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.RobotContainer;
import raidone.robot.subsystems.Intake;

import raidone.robot.RobotContainer;


public class Intake_Retract extends Command{
    private Intake intake;
    private double percent;
    public Intake_Retract(Intake in) {
        intake = in;
    }
    @Override
    public void initialize(){
        intake.resetEncoder();
    }

    @Override
    public void execute() {
        intake.run(-0.2);
        
    }

    @Override
    public boolean isFinished() {
        return intake.isRetracted();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        RobotContainer.noteStatus = true;
    }  


}
