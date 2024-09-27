package raidone.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.Constants;
import raidone.robot.Robot;
import raidone.robot.RobotContainer;

//import static raidone.robot.Constants.Wrist.*;


public class Wrist extends SubsystemBase{
    private static Wrist wrist = new Wrist();
    private TalonFX m_wrist;
    private TalonFX m_follower;
    private boolean isHomed;
    private static CANifier limitCanifier;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    private boolean reverseLimit = false;

    private boolean keepReseting = false;
    private int count = 0;
    private int countsToReset = 5;

    public Wrist() {
        limitCanifier = RobotContainer.getCANifier();
        System.out.println("Wrist init");
        isHomed = false;

        m_wrist = new TalonFX(Constants.Wrist.WRIST_MOTOR_ID, Constants.Wrist.wristCANbus);
        m_follower = new TalonFX(Constants.Wrist.WRIST_FOLLOW_ID, Constants.Wrist.wristCANbus);

        //var currentConfigs = new MotorOutputConfigs();
        //m_wrist.getConfigurator().apply(currentConfigs);

         // The left motor is CW+
         //currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
         //currentConfigs.withInverted(Constants.Wrist.inversion);
         //currentConfigs.withNeutralMode(Constants.Wrist.neutralMode);
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

    public void moveTo(double target) {
        m_wrist.setControl(m_request.withPosition(target).withLimitReverseMotion(reverseLimit));
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
        //SmartDashboard.putBoolean("wrist coast", m_wrist
        //if(reverseLimit && !isHomed()){
        //    m_wrist.setPosition(0);
        //}
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
        currentConfigs.withInverted(Constants.Wrist.inversion);
         currentConfigs.withNeutralMode(NeutralModeValue.Brake);
         m_wrist.getConfigurator().apply(currentConfigs);
         m_follower.getConfigurator().apply(currentConfigs);
        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_wrist.getDeviceID(), true));
    }
    public void enableCoast(){
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.withInverted(Constants.Wrist.inversion);
         currentConfigs.withNeutralMode(NeutralModeValue.Coast);
         m_wrist.getConfigurator().apply(currentConfigs);
         m_follower.getConfigurator().apply(currentConfigs);
        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_wrist.getDeviceID(), true));
    }

    private TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        var motorOutputConfig = new MotorOutputConfigs();
        m_wrist.getConfigurator().apply(motorOutputConfig);

        // The left motor is CW+
        //currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorOutputConfig.withInverted(Constants.Wrist.inversion);
        motorOutputConfig.withNeutralMode(Constants.Wrist.neutralMode);
        config.withMotorOutput(motorOutputConfig);
        
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(Constants.Wrist.supplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(Constants.Wrist.supplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(Constants.Wrist.supplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(Constants.Wrist.supplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);

         // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.withKV(Constants.Wrist.kV);
        slot0Configs.withKP(Constants.Wrist.kP);
        slot0Configs.withKI(Constants.Wrist.kI);
        slot0Configs.withKD(Constants.Wrist.kD);
        config.withSlot0(slot0Configs);

        // Motion Magic Configuration
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(Constants.Wrist.motionMagicVelocity);
        motionMagicConfigs.withMotionMagicAcceleration(Constants.Wrist.motionMagicAccel);
        motionMagicConfigs.withMotionMagicJerk(Constants.Wrist.motionMagicJerk);
        config.withMotionMagic(motionMagicConfigs);

        // Software Limit Switch Configuration 
        config.withSoftwareLimitSwitch(Constants.Wrist.normalSoftLimits);

        // Hardware Limit Switch Configuration
        HardwareLimitSwitchConfigs hardwareLimitConfigs = new HardwareLimitSwitchConfigs();
        hardwareLimitConfigs.withReverseLimitSource(Constants.Wrist.reverseLimitSource);
        hardwareLimitConfigs.withReverseLimitType(Constants.Wrist.reverseLimitType);
        hardwareLimitConfigs.withReverseLimitEnable(Constants.Wrist.reverseLimitEnabled);
        hardwareLimitConfigs.withReverseLimitAutosetPositionEnable(Constants.Wrist.reverseLimitAutosetPositionEnabled);
        hardwareLimitConfigs.withReverseLimitAutosetPositionValue(Constants.Wrist.reverseLimitAutosetPositionValue);

        hardwareLimitConfigs.withForwardLimitSource(Constants.Wrist.forwardLimitSource);
        hardwareLimitConfigs.withForwardLimitType(Constants.Wrist.forwardLimitType);
        hardwareLimitConfigs.withForwardLimitEnable(Constants.Wrist.forwardLimitEnabled);
        hardwareLimitConfigs.withForwardLimitAutosetPositionEnable(Constants.Wrist.forwardLimitAutosetPositionEnabled);
        hardwareLimitConfigs.withForwardLimitAutosetPositionValue(Constants.Wrist.forwardLimitAutosetPositionValue);
        config.withHardwareLimitSwitch(hardwareLimitConfigs);

        return config;
     
    }
}
