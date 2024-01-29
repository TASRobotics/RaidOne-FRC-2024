package raidone.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidone.robot.Constants.ArmConstants;

public class IntakeArm extends SubsystemBase {
    private TalonFX armMaster;
    private TalonFX armSlave;
    private TalonFX intakeJoint;

    private MotionMagicVoltage m_mmReq;

    private StatusSignal<Boolean> intakeForLim;
    private boolean armForLim;

    private PIDController intakePID;
    private TalonFXConfiguration cfg;
    private MotionMagicConfigs mm;

    public IntakeArm(int armMasterID, int armSlaveID, int intakeJointID){//arm1 and arm2 are set to 1 and 3 respectively on the proto bot
        armMaster = new TalonFX(armMasterID);
        armSlave = new TalonFX(armSlaveID);

        m_mmReq = new MotionMagicVoltage(0);
        cfg = new TalonFXConfiguration();
        /**
         *Enable For Lim
         Enable For Lim Pos Res
         Inv motor
         */
        mm = cfg.MotionMagic;

        cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;

        intakeJoint = new TalonFX(intakeJointID);

        intakeForLim = intakeJoint.getFault_ForwardHardLimit();
        armForLim = armMaster.getFault_ForwardHardLimit().getValue();

        intakePID = new PIDController(ArmConstants.kIntakeJoint_kP, ArmConstants.kIntakeJoint_kI, ArmConstants.kIntakeJoint_kD);

        armSlave.setControl(new Follower(armMaster.getDeviceID(), true));

        SmartDashboard.putBoolean("Arm Stopped", false);

        SmartDashboard.putNumber("armCruise", 2); // 5 rotations per second cruise
        SmartDashboard.putNumber("armkAccel", 7); // Take approximately 0.5 seconds to reach max vel
        SmartDashboard.putNumber("armJerk", 0);

        mm.MotionMagicCruiseVelocity = SmartDashboard.getNumber("armCruise", 0); // 5 rotations per second cruise
        mm.MotionMagicAcceleration = SmartDashboard.getNumber("armkAccel", 0); // Take approximately 0.5 seconds to reach max vel
        mm.MotionMagicJerk = SmartDashboard.getNumber("armJerk", 0);

        SmartDashboard.putNumber("armkS", 0.25);
        SmartDashboard.putNumber("armkV", 0.12);
        SmartDashboard.putNumber("armkA", 0.01);
        SmartDashboard.putNumber("armkP", 1);
        SmartDashboard.putNumber("armkI", 0);
        SmartDashboard.putNumber("armkD", 0);

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = SmartDashboard.getNumber("armkS", 0.25);
        slot0.kV = SmartDashboard.getNumber("armkV", 0.12);
        slot0.kA = SmartDashboard.getNumber("armkA", 0.01);
        slot0.kP = SmartDashboard.getNumber("armkP", 1);
        slot0.kI = SmartDashboard.getNumber("armkI", 0);
        slot0.kD = SmartDashboard.getNumber("armkD", 0.1);


        FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
        status = armMaster.getConfigurator().apply(cfg);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    public void intakeGoHome(){
        intakeJoint.set(0.1);
    }

    public void armGoHome(){
        armMaster.set(0.1);
    }

    public void updateForLimSwitch(){
        intakeForLim = intakeJoint.getFault_ForwardHardLimit();
        armForLim = armMaster.getFault_ForwardHardLimit().getValue();

        SmartDashboard.putBoolean("Arm Limit Switch Status", armForLim);
    }

    public boolean getIntakeForLim(){
        return intakeJoint.getFault_ForwardHardLimit().getValue();
    }

    public boolean getArmForLim(){
        return armMaster.getFault_ForwardHardLimit().getValue();
    }

    public void stopIntake(){
        intakeJoint.stopMotor();
    }

    public void stopArm(){
        armMaster.stopMotor();
    }

    public void posArm(double setpoint){
        cfg.Slot0.kS = SmartDashboard.getNumber("armkS", 0.25);
        cfg.Slot0.kV = SmartDashboard.getNumber("armkV", 0.12);
        cfg.Slot0.kA = SmartDashboard.getNumber("armkA", 0.01);
        cfg.Slot0.kP = SmartDashboard.getNumber("armkP", 1);
        cfg.Slot0.kI = SmartDashboard.getNumber("armkI", 0);
        cfg.Slot0.kD = SmartDashboard.getNumber("armkD", 0.1);

        mm.MotionMagicCruiseVelocity = SmartDashboard.getNumber("armCruise", 0); // 5 rotations per second cruise
        mm.MotionMagicAcceleration = SmartDashboard.getNumber("armkAccel", 0); // Take approximately 0.5 seconds to reach max vel
        mm.MotionMagicJerk = SmartDashboard.getNumber("armJerk", 0);

        MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
        armMaster.setControl(armRequest.withPosition(setpoint));
        SmartDashboard.putNumber("Arm Position", armMaster.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Slave Position", armSlave.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Setpoint", setpoint);
    }

    public void posIntake(double setpoint){
        MotionMagicVoltage intakeRequest = new MotionMagicVoltage(0);
        intakeJoint.setControl(intakeRequest.withPosition(setpoint));
        SmartDashboard.putNumber("Intake Position", intakeJoint.getPosition().getValueAsDouble());
    }
}
