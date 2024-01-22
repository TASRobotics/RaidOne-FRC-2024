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
import raidone.robot.Constants.ArmConstants;

public class IntakeArm {
    private TalonFX armMaster;
    private TalonFX armSlave;
    private TalonFX intakeJoint;

    private MotionMagicVoltage m_mmReq;

    private StatusSignal<Boolean> intakeForLim;

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

        intakeJoint = new TalonFX(intakeJointID);

        intakeForLim = intakeJoint.getFault_ForwardHardLimit();

        intakePID = new PIDController(ArmConstants.kIntakeJoint_kP, ArmConstants.kIntakeJoint_kI, ArmConstants.kIntakeJoint_kD);

        armSlave.setControl(new Follower(armMaster.getDeviceID(), true));

        mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel 
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kV = 0.12;
        slot0.kS = 0.25;

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

    public void goHome(){
        intakeJoint.set(0.1);
    }

    public void updateForLimSwitch(){
        intakeForLim = intakeJoint.getFault_ForwardHardLimit();
    }

    public boolean getIntakeForLim(){
        return intakeJoint.getFault_ForwardHardLimit().getValue();
    }

    public void stopArm(){
        intakeJoint.stopMotor();
    }
}
