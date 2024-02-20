package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static raidone.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
    private CANSparkMax m_roller;
    private SparkLimitSwitch s_beam;
    public SparkPIDController m_pid;
    //private XboxController testcontrol;
    
    public Intake(){
        m_roller = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        m_roller.setIdleMode(IdleMode.kBrake);
        //testcontrol = new XboxController(0);
        s_beam = m_roller.getForwardLimitSwitch(Type.kNormallyOpen);
        m_pid = m_roller.getPIDController();
        
    }
    public boolean getLimit(){
        boolean limitStatus = s_beam.isPressed();
        return limitStatus;
    }
    public void run(double s){
        
            m_roller.set(s);
 
    }

    public void stop(){
        m_roller.stopMotor();
    }


    public boolean getStatus(){
        return true;
    }

    public void resetEncoder(){
        m_roller.getEncoder().setPosition(0);
    }

    public boolean isRetracted(){
        return Math.abs(m_roller.getEncoder().getPosition()) > 8;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("pos", m_roller.getEncoder().getPosition());
    }
}
