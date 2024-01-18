package raidone.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.controller.PIDController;

import raidone.robot.Constants;
import raidone.robot.Constants.ArmConstants;

public class IntakeArm {
    private TalonFX armMaster;
    private TalonFX armSlave;
    private TalonFX intakeJoint;

    private StatusSignal<Boolean> intakeRevLim;

    private PIDController intakePID;

    public IntakeArm(int armMasterID, int armSlaveID, int intakeJointID){//arm1 and arm2 are set to 1 and 3 respectively on the proto bot
        armMaster = new TalonFX(armMasterID);
        armSlave = new TalonFX(armSlaveID);

        intakeJoint = new TalonFX(intakeJointID);

        intakeRevLim = intakeJoint.getFault_ReverseHardLimit();

        intakePID = new PIDController(ArmConstants.kIntakeJoint_kP, ArmConstants.kIntakeJoint_kI, ArmConstants.kIntakeJoint_kD);

        armSlave.setControl(new Follower(armMaster.getDeviceID(), true));
    }

    public int goHome(){
        while(intakeRevLim.getValue()){
            intakeJoint.set(-0.1);
        }
        intakeJoint.stopMotor();
        return 1;
    }
}
