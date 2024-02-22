package raidone.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Wrist;
import raidone.robot.subsystems.Arm;


public class Home extends Command{
    private static Wrist wrist;
    private static Arm arm;

    public Home(Arm arm, Wrist wrist) {
        this.wrist = wrist;
        this.arm = arm;
        addRequirements(this.wrist);
        addRequirements(this.arm);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute() {
        wrist.home();
        arm.home();
    }

    @Override
    public boolean isFinished() {
        return (wrist.isHomed() && arm.isHomed());
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stopMotors();
        arm.stopMotors();
    }
}
