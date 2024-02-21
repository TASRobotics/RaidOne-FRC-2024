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
import com.revrobotics.CANSparkBase.IdleMode;


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

        m_arm.setIdleMode(IdleMode.kBrake);
        m_follow.setIdleMode(IdleMode.kBrake);

        m_pid = m_arm.getPIDController();
        m_encoder = m_arm.getEncoder();
        s_limit1 = m_arm.getForwardLimitSwitch(Type.kNormallyOpen);
        s_limit2 = m_follow.getForwardLimitSwitch(Type.kNormallyOpen);
        
        s_limit1.enableLimitSwitch(true);
        s_limit2.enableLimitSwitch(true);

        m_arm.setSoftLimit(SoftLimitDirection.kReverse, -28);
        m_arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // m_arm.setInverted(true);
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

    public void setPos(double setpoint){
        m_pid.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("processVariable", m_encoder.getPosition());

    }

    public void home(){
        m_arm.set(0.2);
    }

    public RelativeEncoder getEncoder() {
        return m_encoder;
    }

    public boolean isHomed(){
        return isHomed;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("arm position", m_encoder.getPosition()); 
        SmartDashboard.putString("Arm Command", this.getCurrentCommand() != null ? this.getCurrentCommand().getName():""); 

        if(s_limit1.isPressed() || s_limit2.isPressed()){
            isHomed = true;
            m_encoder.setPosition(0);
        }else{
            isHomed = false;
        }
    }
}