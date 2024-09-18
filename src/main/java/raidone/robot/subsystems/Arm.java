package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.Constants;
import raidone.robot.RobotContainer;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static raidone.robot.Constants.Arm.*;

public class Arm extends SubsystemBase{

    private static Arm armSys = new Arm();

    private TalonFX m_arm;
    private TalonFX m_follower;
    private boolean isHomed;
    private static CANifier limitCanifier;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private boolean reverseLimit = false;
  
    public Arm(){
        limitCanifier = RobotContainer.getCANifier();
        System.out.println("Arm init");
        isHomed = false;

        
        m_arm = new TalonFX(Constants.Arm.ARM_MOTOR_ID, "rio");
        m_follower = new TalonFX(Constants.Arm.ARM_FOLLOW_ID, "rio");

        var currentConfigs = new MotorOutputConfigs();

         // The left motor is CCW+
         currentConfigs.withInverted(Constants.Arm.inversion);
         currentConfigs.withNeutralMode(Constants.Arm.neutralMode);
         m_arm.getConfigurator().apply(currentConfigs);

        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_arm.getDeviceID(),false));
       
        
    }

    public void stopMotors(){
        //m_arm.stopMotor();
    }

    public boolean getLimit(){
        //boolean limitStatus = s_limit1.isPressed() || s_limit2.isPressed();
      //  return limitStatus;
      return true;
    }

    public void run(double speed){
     //   m_arm.set(speed);
    }

    public void setPos(){
        
        // if(driver.getRawButton(XboxController.Button.kA.value)){
        //     //setpoint = SCORINGPOS;
        // }else if(driver.getRawButton(XboxController.Button.kB.value)){
        //     //setpoint = INTAKEPOS;
        // }
        // m_pid.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        // SmartDashboard.putNumber("processVariable", m_encoder.getPosition());
    }

    public void home(){
      //  m_arm.set(0.1);
    }

    public boolean isHomed(){
        // if(s_limit1.isPressed() || s_limit2.isPressed()){
        //     isHomed = true;
        //     m_encoder.setPosition(0);
        // }else{
        //     isHomed = false;
        // }
        //return isHomed;
        return true;
    }

    public static Arm system(){
        return armSys;
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("arm position", m_encoder.getPosition());
        CANifier.PinValues values = new CANifier.PinValues();
        limitCanifier.getGeneralInputs(values);
        SmartDashboard.putBoolean("Arm_Left",values.LIMF);
        SmartDashboard.putBoolean("Arm_Right",values.QUAD_A);
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