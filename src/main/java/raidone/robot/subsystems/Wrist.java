package raidone.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static raidone.robot.Constants.Arm.*;

public class Wrist extends SubsystemBase{
    private static Wrist wrist;
    private CANSparkMax m_wrist;
    private boolean isHomed;
    private SparkPIDController m_pid;
    private RelativeEncoder m_encoder;

    public Wrist() {
        isHomed = false;
        m_wrist = new CANSparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
        
        m_wrist.restoreFactoryDefaults();

        m_pid = m_wrist.getPIDController();
        m_encoder = m_wrist.getEncoder();

        m_pid.setP(0.0);
        m_pid.setI(0.0);
        m_pid.setD(0.0);
        m_pid.setIZone(0.0);
        m_pid.setFF(0.0);
        m_pid.setOutputRange(0.0, 0.0);

        m_pid.setSmartMotionMaxVelocity(0.0, 0);
        m_pid.setSmartMotionMinOutputVelocity(0.0, 0);
        m_pid.setSmartMotionMaxAccel(0.0, 0);
        m_pid.setSmartMotionAllowedClosedLoopError(0.0, 0);
    }

    public void stopMotors() {
        m_wrist.stopMotor();
    }

    public void setPos(double setpoint) {
        m_pid.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        double processVariable = m_encoder.getPosition();
    }

    public boolean isHomed() {
        // isHomed = SparkMax.ReadLimit
        return isHomed;
    }

    @Override
    public void periodic() {
    
    }
}
