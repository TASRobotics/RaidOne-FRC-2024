package raidone.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.Constants;
import raidone.robot.RobotContainer;

import static raidone.robot.Constants.Wrist.*;


public class Wrist extends SubsystemBase{
    private static Wrist wrist = new Wrist();
    private TalonFX m_wrist;
    private TalonFX m_follower;
    private boolean isHomed;
    private static CANifier limitCanifier;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private boolean reverseLimit = false;

    public Wrist() {
        limitCanifier = RobotContainer.getCANifier();
        System.out.println("Wrist init");
        isHomed = false;

        m_wrist = new TalonFX(Constants.Wrist.WRIST_MOTOR_ID, "rio");
        m_follower = new TalonFX(Constants.Wrist.WRIST_FOLLOW_ID, "rio");

        var currentConfigs = new MotorOutputConfigs();

         // The left motor is CW+
         //currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
         currentConfigs.withInverted(Constants.Wrist.inversion);
         currentConfigs.withNeutralMode(Constants.Wrist.neutralMode);
         m_wrist.getConfigurator().apply(currentConfigs);

        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_wrist.getDeviceID(), true));
       
        //m_wrist.setNeutralMode(NeutralModeValue.Coast);
        //m_follower.setNeutralMode(NeutralModeValue.Coast);

        //m_pid = m_wrist.getPIDController();
        //m_encoder = m_wrist.getEncoder();
        //s_limit = m_wrist.getForwardLimitSwitch(Type.kNormallyOpen);

        //m_follower.follow(m_wrist, true);

        // m_pid.setP(kP);
        // m_pid.setI(kI);
        // m_pid.setD(kD);
        // m_pid.setIZone(kIz);
        // m_pid.setFF(kFF);
        // m_pid.setOutputRange(kMinOutput, kMaxOutput);

        // m_pid.setSmartMotionMaxVelocity(maxVel, 0);
        // m_pid.setSmartMotionMinOutputVelocity(minVel, 0);
        // m_pid.setSmartMotionMaxAccel(maxAcc, 0);
        // m_pid.setSmartMotionAllowedClosedLoopError(allowedErr, 0);

        // SmartDashboard.putNumber("P Gain", kP);
        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Zone", kIz);
        // SmartDashboard.putNumber("Feed Forward", kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);

        // // display Smart Motion coefficients
        // SmartDashboard.putNumber("Max Velocity", maxVel);
        // SmartDashboard.putNumber("Min Velocity", minVel);
        // SmartDashboard.putNumber("Max Acceleration", maxAcc);
        // SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
        // SmartDashboard.putNumber("Set Position", setpoint);
    }

    public void percentOut(double speed){
        dutyCycle.Output = speed;
        m_wrist.setControl(dutyCycle.withLimitReverseMotion(reverseLimit));

    }

    public void stopMotors() {
        m_wrist.stopMotor();
    }

    public void setPos() {
        
        // if(driver.getRawButton(XboxController.Button.kA.value)){
        //     //setpoint = SCORINGPOS;
        // }else if(driver.getRawButton(XboxController.Button.kB.value)){
        //     //setpoint = INTAKEPOS;
        // }
        // m_pid.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        // SmartDashboard.putNumber("processVariable", m_encoder.getPosition());
    }

    public void home(){
        //m_wrist.set(-0.1);
        dutyCycle.Output = Constants.Wrist.homeSpeed;
        m_wrist.setControl(dutyCycle.withLimitReverseMotion(reverseLimit));
    }

    public boolean isHomed(){
         if(reverseLimit){
            isHomed = true;
            m_wrist.setPosition(0);
        }else{
            isHomed = false;
        }
         return isHomed;
    }

    @Override
    public void periodic(){
        getCANifierValues();
        SmartDashboard.putNumber("wrist encoder", m_wrist.getPosition().getValueAsDouble());
        if(reverseLimit){
            m_wrist.setPosition(0);
        }
        // m_pid.setP(SmartDashboard.getNumber("P Gain", 0));
        // m_pid.setI(SmartDashboard.getNumber("I Gain", 0));
        // m_pid.setD(SmartDashboard.getNumber("D Gain", 0));
        // m_pid.setIZone(SmartDashboard.getNumber("I Zone", 0));
        // m_pid.setFF(SmartDashboard.getNumber("Feed Forward", 0));

        // m_pid.setOutputRange(
        //     SmartDashboard.getNumber("Max Output", 0),
        //     SmartDashboard.getNumber("Min Output", 0));

        // m_pid.setSmartMotionMaxVelocity(SmartDashboard.getNumber("Max Velocity", 0), 0);
        // m_pid.setSmartMotionMinOutputVelocity(SmartDashboard.getNumber("Min Velocity", 0), 0);
        // m_pid.setSmartMotionMaxAccel(SmartDashboard.getNumber("Max Acceleration", 0), 0);
        // m_pid.setSmartMotionAllowedClosedLoopError(SmartDashboard.getNumber("Allowed Closed Loop Error", 0),0); 
    }

    public void getCANifierValues(){
        CANifier.PinValues values = new CANifier.PinValues();
        limitCanifier.getGeneralInputs(values);
        boolean reverseLeftLimit = values.QUAD_B;
        boolean reverseRightLimit = values.LIMR;
        reverseLimit = !reverseLeftLimit || !reverseRightLimit;
        SmartDashboard.putBoolean("Wrist_Right",reverseRightLimit);
        SmartDashboard.putBoolean("Wrist_Left",reverseLeftLimit);
    }

    public static Wrist system() {
        return wrist;
    }
}
