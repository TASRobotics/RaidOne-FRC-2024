package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkMaxLimitSwitch.Direction;

import static raidone.robot.Constants.Arm.*;

public class Arm extends SubsystemBase{
    private static Arm arm;
    private CANSparkMax m_arm;
    private boolean isHomed;
    private SparkPIDController m_pid;
    private RelativeEncoder m_encoder;
    private SparkLimitSwitch s_limit1;
    private SparkLimitSwitch s_limit2;
    private Joystick driver = new Joystick(0);

    public Arm(){
        isHomed = false;
        m_arm = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);

        m_arm.restoreFactoryDefaults();

        m_pid = m_arm.getPIDController();
        m_encoder = m_arm.getEncoder();
        s_limit1 = m_arm.getForwardLimitSwitch(Type.kNormallyOpen);

        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setIZone(kIz);
        m_pid.setFF(kFF);
        m_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_pid.setSmartMotionMaxVelocity(maxVel, 0);
        m_pid.setSmartMotionMinOutputVelocity(minVel, 0);
        m_pid.setSmartMotionMaxAccel(maxAcc, 0);
        m_pid.setSmartMotionAllowedClosedLoopError(allowedErr, 0);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber("Set Position", 0);
    }

    public void stopMotors(){
        m_arm.stopMotor();
    }

    public boolean getLimit(){
        boolean limitStatus = s_limit1.isPressed()
        return limitStatus;
    }

    public void setPos(){
        double setpoint = 0;
        if(driver.getRawButton(XboxController.Button.kA.value)){
            setpoint = SCORINGPOS;
        }else if(driver.getRawButton(XboxController.Button.kB.value)){
            setpoint = INTAKEPOS;
        }
        m_pid.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("processVariable", m_encoder.getPosition());
    }

    public void home(){
        m_arm.set(-0.1);
    }

    public boolean isHomed(){
        isHomed = s_limit1.isPressed();
        return isHomed;
    }

    @Override
    public void periodic(){
        m_pid.setP(SmartDashboard.getNumber("P Gain", 0));
        m_pid.setI(SmartDashboard.getNumber("I Gain", 0));
        m_pid.setD(SmartDashboard.getNumber("D Gain", 0));
        m_pid.setIZone(SmartDashboard.getNumber("I Zone", 0));
        m_pid.setFF(SmartDashboard.getNumber("Feed Forward", 0));

        m_pid.setOutputRange(
            SmartDashboard.getNumber("Max Output", 0),
            SmartDashboard.getNumber("Min Output", 0));

        m_pid.setSmartMotionMaxVelocity(SmartDashboard.getNumber("Max Velocity", 0), 0);
        m_pid.setSmartMotionMinOutputVelocity(SmartDashboard.getNumber("Min Velocity", 0), 0);
        m_pid.setSmartMotionMaxAccel(SmartDashboard.getNumber("Max Acceleration", 0), 0);
        m_pid.setSmartMotionAllowedClosedLoopError(SmartDashboard.getNumber("Allowed Closed Loop Error", 0),0);        
    }
}