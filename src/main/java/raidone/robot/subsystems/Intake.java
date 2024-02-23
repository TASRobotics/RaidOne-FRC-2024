package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static raidone.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
    private CANSparkMax roller;
    private SparkLimitSwitch beam;
    public SparkPIDController pid;
    //private XboxController testcontrol;
    
    public Intake(){
        roller = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        roller.setIdleMode(IdleMode.kBrake);
        //testcontrol = new XboxController(0);
        beam = roller.getForwardLimitSwitch(Type.kNormallyOpen);
        pid = roller.getPIDController();
        
    }
    public boolean getLimit(){
        boolean limitStatus = beam.isPressed();
        return limitStatus;
    }
    public void run(double s){
        
            roller.set(s);
 
    }

    public void stop(){
        roller.stopMotor();
    }

    public void resetEncoder(){
        roller.getEncoder().setPosition(0);
    }

    public boolean isRetracted(){
        return Math.abs(roller.getEncoder().getPosition()) > 8;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("pos", roller.getEncoder().getPosition());
    }
}
