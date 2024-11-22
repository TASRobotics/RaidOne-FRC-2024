package raidone.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.MotorConfigConstants;
import raidone.robot.Constants;
import raidone.robot.RobotContainer;

//import static raidone.robot.Constants.Wrist.*;


public class Wrist extends SubsystemBase{
    // private static WristState wrSt = new enum thingy
    private static Wrist wrist = new Wrist();
    private TalonFX m_wrist;
    private TalonFX m_follower;
    
    private boolean isHomed;
    private static CANifier limitCanifier;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private boolean reverseLimit = false;

    private boolean keepReseting = false;
    private int count = 0;
    private int countsToReset = 5;

    enum WristStateEnum {
        AT_INTAKE_POS,
        AT_SCORE_POS,
        AT_HOME_POS,
        MOVING
    }
    private static WristStateEnum wristState = WristStateEnum.AT_HOME_POS;

    public Wrist() {
        limitCanifier = RobotContainer.getCANifier();
        System.out.println("Wrist init");
        isHomed = false;

        m_wrist = new TalonFX(Constants.Wrist.WRIST_MOTOR_ID, Constants.Wrist.wristCANbus);
        m_follower = new TalonFX(Constants.Wrist.WRIST_FOLLOW_ID, Constants.Wrist.wristCANbus);

        
         TalonFXConfiguration config = getDefaultConfig();
         m_wrist.getConfigurator().apply(config);
         m_follower.getConfigurator().apply(config);
        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_wrist.getDeviceID(), true));

    }

    public void percentOut(double speed){
        dutyCycle.Output = speed;
        m_wrist.setControl(dutyCycle.withLimitReverseMotion(reverseLimit));

    }

    public void stopMotors() {
        m_wrist.stopMotor();
    }

    public void setPos( double setpoint) {
        //leo added
        MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(setpoint);
        m_request = m_request.withLimitReverseMotion(reverseLimit);
        m_wrist.setControl(m_request.withPosition(setpoint));
    }

    public void home(){
        dutyCycle.Output = Constants.Wrist.homeSpeed;
        m_wrist.setControl(dutyCycle.withLimitReverseMotion(reverseLimit));
    }

    public boolean isHomed(){
         if(reverseLimit){
            isHomed = true;
            m_wrist.setPosition(0);
            count = 0;
            keepReseting = true;
        }else{
            isHomed = false;
        }
         return isHomed;
    }

    @Override
    public void periodic(){
        
        getCANifierValues();
        SmartDashboard.putNumber("wrist encoder", m_wrist.getPosition().getValueAsDouble());
        if(keepReseting){
            if(reverseLimit){
                m_wrist.setPosition(0);
            }
            count++;
            if(count >= countsToReset){
                count = 0;
                keepReseting = false;
            }
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

    public void enableBrake(){
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.withInverted(MotorConfigConstants.Wrist.inversion);
         currentConfigs.withNeutralMode(NeutralModeValue.Brake);
         m_wrist.getConfigurator().apply(currentConfigs);
         m_follower.getConfigurator().apply(currentConfigs);
        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_wrist.getDeviceID(), true));
    }
    public void enableCoast(){
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.withInverted(MotorConfigConstants.Wrist.inversion);
         currentConfigs.withNeutralMode(NeutralModeValue.Coast);
         m_wrist.getConfigurator().apply(currentConfigs);
         m_follower.getConfigurator().apply(currentConfigs);
        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_wrist.getDeviceID(), true));
    }

    public WristStateEnum getState(){
        return wristState;
    }
    

    private TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        var motorOutputConfig = new MotorOutputConfigs();
        m_wrist.getConfigurator().apply(motorOutputConfig);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(MotorConfigConstants.Wrist.sensorToMechanismRatio);
        config.withFeedback(feedbackConfigs);

        // The left motor is CW+
        //currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorOutputConfig.withInverted(MotorConfigConstants.Wrist.inversion);
        motorOutputConfig.withNeutralMode(MotorConfigConstants.Wrist.neutralMode);
        config.withMotorOutput(motorOutputConfig);
        
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(MotorConfigConstants.Wrist.supplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(MotorConfigConstants.Wrist.supplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(MotorConfigConstants.Wrist.supplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(MotorConfigConstants.Wrist.supplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);

         // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKV(MotorConfigConstants.Wrist.kV);
        slot0Configs.withKS(MotorConfigConstants.Wrist.kS);
        slot0Configs.withKP(MotorConfigConstants.Wrist.kP);
        slot0Configs.withKI(MotorConfigConstants.Wrist.kI);
        slot0Configs.withKD(MotorConfigConstants.Wrist.kD);
        config.withSlot0(slot0Configs);

        // Motion Magic Configuration
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicExpo_kV(MotorConfigConstants.Wrist.motionMagicExpoVelocity);
        motionMagicConfigs.withMotionMagicExpo_kA(MotorConfigConstants.Wrist.motionMagicExpoAccel);
        config.withMotionMagic(motionMagicConfigs);

        // Software Limit Switch Configuration 
        config.withSoftwareLimitSwitch(MotorConfigConstants.Wrist.normalSoftLimits);

        // Hardware Limit Switch Configuration
        HardwareLimitSwitchConfigs hardwareLimitConfigs = new HardwareLimitSwitchConfigs();
        hardwareLimitConfigs.withReverseLimitSource(MotorConfigConstants.Wrist.reverseLimitSource);
        hardwareLimitConfigs.withReverseLimitType(MotorConfigConstants.Wrist.reverseLimitType);
        hardwareLimitConfigs.withReverseLimitEnable(MotorConfigConstants.Wrist.reverseLimitEnabled);
        hardwareLimitConfigs.withReverseLimitAutosetPositionEnable(MotorConfigConstants.Wrist.reverseLimitAutosetPositionEnabled);
        hardwareLimitConfigs.withReverseLimitAutosetPositionValue(MotorConfigConstants.Wrist.reverseLimitAutosetPositionValue);

        hardwareLimitConfigs.withForwardLimitSource(MotorConfigConstants.Wrist.forwardLimitSource);
        hardwareLimitConfigs.withForwardLimitType(MotorConfigConstants.Wrist.forwardLimitType);
        hardwareLimitConfigs.withForwardLimitEnable(MotorConfigConstants.Wrist.forwardLimitEnabled);
        hardwareLimitConfigs.withForwardLimitAutosetPositionEnable(MotorConfigConstants.Wrist.forwardLimitAutosetPositionEnabled);
        hardwareLimitConfigs.withForwardLimitAutosetPositionValue(MotorConfigConstants.Wrist.forwardLimitAutosetPositionValue);
        config.withHardwareLimitSwitch(hardwareLimitConfigs);

        return config;
     
    }
}
