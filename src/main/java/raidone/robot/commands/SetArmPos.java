package raidone.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.Constants.ArmConstants;
import raidone.robot.subsystems.IntakeArm;


public class SetArmPos extends Command{
    private IntakeArm arm;
    private double setpoint;
    private final Joystick driver = new Joystick(0);

    /**
     * subsystem, and target
     * @param arm
     */
    public SetArmPos(IntakeArm arm, double setpoint){
        this.arm = arm;
        addRequirements(this.arm);

        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        arm.posArm(setpoint);
        if(driver.getRawButtonPressed(XboxController.Button.kA.value)) setpoint = ArmConstants.SCORINGPOS_ARM_SETPOINT;
        if(driver.getRawButtonPressed(XboxController.Button.kB.value)) setpoint = ArmConstants.INTAKEPOS_ARM_SETPOINT;
        // arm.updateForLimSwitch();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        arm.stopIntake();
    }
}