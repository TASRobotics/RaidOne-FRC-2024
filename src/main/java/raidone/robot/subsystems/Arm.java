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
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkMaxLimitSwitch.Direction;

import static raidone.robot.Constants.Arm.*;

public class Arm extends SubsystemBase{
    private static Arm arm;
    private CANSparkMax m_arm;
    private CANSparkMax m_follow;
    private boolean isHomed;
    private SparkPIDController m_pid;
    private RelativeEncoder m_encoder;
    private SparkLimitSwitch s_limit1;
    private SparkLimitSwitch s_limit2;
    private Joystick driver = new Joystick(0);
    private double setpoint = 0;

    public Arm(){
        System.out.println("Arm init");
        isHomed = false;
        m_arm = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        m_follow = new CANSparkMax(ARM_FOLLOW_ID, MotorType.kBrushless);

        m_arm.restoreFactoryDefaults();
        m_follow.restoreFactoryDefaults();

        m_arm.restoreFactoryDefaults();

        m_pid = m_arm.getPIDController();
        m_encoder = m_arm.getEncoder();
        s_limit1 = m_arm.getForwardLimitSwitch(Type.kNormallyOpen);
        s_limit2 = m_follow.getForwardLimitSwitch(Type.kNormallyOpen);
        
        s_limit1.enableLimitSwitch(true);
        s_limit2.enableLimitSwitch(true);

        m_arm.setSoftLimit(SoftLimitDirection.kReverse, -28);
        m_arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_follow.follow(m_arm, true);

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

        SmartDashboard.putNumber("Arm P Gain", kP);
        SmartDashboard.putNumber("Arm I Gain", kI);
        SmartDashboard.putNumber("Arm D Gain", kD);
        SmartDashboard.putNumber("Arm I Zone", kIz);
        SmartDashboard.putNumber("Arm Feed Forward", kFF);
        SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
        SmartDashboard.putNumber("Arm Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Arm Max Velocity", maxVel);
        SmartDashboard.putNumber("Arm Min Velocity", minVel);
        SmartDashboard.putNumber("Arm Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Arm Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber("Arm Set Position", setpoint);
    }

    public void stopMotors(){
        m_arm.stopMotor();
    }

    public boolean getLimit(){
        boolean limitStatus = s_limit1.isPressed() || s_limit2.isPressed();
        return limitStatus;
    }

    public void run(double speed){
        m_arm.set(speed);
    }

    public void setPos(){
        
        if(driver.getRawButton(XboxController.Button.kA.value)){
            setpoint = SCORINGPOS;
        }else if(driver.getRawButton(XboxController.Button.kB.value)){
            setpoint = INTAKEPOS;
        }
        m_pid.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("processVariable", m_encoder.getPosition());
    }

    public void home(){
        m_arm.set(0.1);
    }

    public boolean isHomed(){
        if(s_limit1.isPressed() || s_limit2.isPressed()){
            isHomed = true;
            m_encoder.setPosition(0);
        }else{
            isHomed = false;
        }
        return isHomed;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("arm position", m_encoder.getPosition());
        // m_pid.setP(SmartDashboard.getNumber("Arm P Gain", 0));
        // m_pid.setI(SmartDashboard.getNumber("Arm I Gain", 0));
        // m_pid.setD(SmartDashboard.getNumber("Arm D Gain", 0));
        // m_pid.setIZone(SmartDashboard.getNumber("Arm I Zone", 0));
        // m_pid.setFF(SmartDashboard.getNumber("Arm Feed Forward", 0));

        //if((getLimit() || m_encoder.getPosition()<0.1)){
        //    stopMotors();
        //}

        // m_pid.setOutputRange(
        //     SmartDashboard.getNumber("Arm Max Output", 0),
        //     SmartDashboard.getNumber("Arm Min Output", 0));

        // m_pid.setSmartMotionMaxVelocity(SmartDashboard.getNumber("Arm Max Velocity", 0), 0);
        // m_pid.setSmartMotionMinOutputVelocity(SmartDashboard.getNumber("Arm Min Velocity", 0), 0);
        // m_pid.setSmartMotionMaxAccel(SmartDashboard.getNumber("Arm Max Acceleration", 0), 0);
        // m_pid.setSmartMotionAllowedClosedLoopError(SmartDashboard.getNumber("Arm Allowed Closed Loop Error", 0),0);        
    }
}