package raidone.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static raidone.robot.Constants.Wrist.*;

public class Wrist extends SubsystemBase{
    private CANSparkMax wrist;
    private CANSparkMax follower;
    private boolean isHomed;
    private SparkPIDController pid;
    private RelativeEncoder encoder;
    private SparkLimitSwitch limit;
    // private double setpoint = 0;

    public Wrist() {
        System.out.println("Wrist init");
        isHomed = false;
        wrist = new CANSparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
        follower = new CANSparkMax(WRIST_FOLLOW_ID, MotorType.kBrushless);
        wrist.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();
        

        pid = wrist.getPIDController();
        encoder = wrist.getEncoder();
        limit = wrist.getForwardLimitSwitch(Type.kNormallyOpen);

        wrist.setIdleMode(IdleMode.kBrake);
        follower.setIdleMode(IdleMode.kBrake);

        limit.enableLimitSwitch(true);

        follower.follow(wrist, true);

        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

        pid.setSmartMotionMaxVelocity(MAX_VEL, 0);
        pid.setSmartMotionMinOutputVelocity(MIN_VEL, 0);
        pid.setSmartMotionMaxAccel(MAX_ACCEL, 0);
        pid.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, 0);

        // SmartDashboard.putNumber("Wrist Set Position", setpoint);
        
        
    }

    public void stopMotors() {
        wrist.stopMotor();
    }

    public void setPos(double setpoint) {
        pid.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("processVariable", encoder.getPosition());
    }

    public void home(){
        wrist.set(-0.3);
    }

    public boolean isHomed(){
        return isHomed;
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    @Override
    public void periodic(){
        if(limit.isPressed()){
            isHomed = true;
            encoder.setPosition(0);
        }else{
            isHomed = false;
        }
        SmartDashboard.putNumber("Wrist Position", encoder.getPosition());
        // pid.setP(SmartDashboard.getNumber("P Gain", 0));
        // pid.setI(SmartDashboard.getNumber("I Gain", 0));
        // pid.setD(SmartDashboard.getNumber("D Gain", 0));
        // pid.setIZone(SmartDashboard.getNumber("I Zone", 0));
        // pid.setFF(SmartDashboard.getNumber("Feed Forward", 0));

        // pid.setOutputRange(
        //     SmartDashboard.getNumber("Max Output", 0),
        //     SmartDashboard.getNumber("Min Output", 0));

        // pid.setSmartMotionMaxVelocity(SmartDashboard.getNumber("Max Velocity", 0), 0);
        // pid.setSmartMotionMinOutputVelocity(SmartDashboard.getNumber("Min Velocity", 0), 0);
        // pid.setSmartMotionMaxAccel(SmartDashboard.getNumber("Max Acceleration", 0), 0);
        // pid.setSmartMotionAllowedClosedLoopError(SmartDashboard.getNumber("Allowed Closed Loop Error", 0),0); 
    }
}
