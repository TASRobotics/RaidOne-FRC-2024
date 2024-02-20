package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static raidone.robot.Constants.Intake.*;

public class Intake {
    private CANSparkMax m_roller;
    private Trigger s_beam;
    private SparkPIDController m_pid;

    public Intake(){
        System.out.println("Intake init");
        m_roller = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

        s_beam = new Trigger(s_beam);
        m_pid = m_roller.getPIDController();
    }

    public void run(int dir){
        if(dir == kForward){

        }else{
            
        }
    }


    public boolean getStatus(){
        return true;
    }
}
