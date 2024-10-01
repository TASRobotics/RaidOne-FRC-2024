package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.Constants;
import raidone.robot.RobotContainer;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class Arm extends SubsystemBase{

    private static Arm armSys = new Arm();

    private TalonFX m_arm;
    private TalonFX m_follower;
    private boolean isHomed;
    private static CANifier limitCanifier;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private boolean reverseLimit = false;

    private boolean keepReseting = false;
    private int count = 0;
    private int countsToReset = 20;

    private boolean moving = false;

    

    public enum ArmStateEnum {
        AT_SCORE_POS,
        AT_HOME_POS,
        MOVING
    }
    private static ArmStateEnum armState = ArmStateEnum.AT_HOME_POS;
  
    public Arm(){
        limitCanifier = RobotContainer.getCANifier();
        isHomed = false;

        m_arm = new TalonFX(Constants.Arm.ARM_MOTOR_ID, Constants.Arm.armCANbus);
        m_follower = new TalonFX(Constants.Arm.ARM_FOLLOW_ID, Constants.Arm.armCANbus);

        TalonFXConfiguration config = getDefaultConfig();
        m_arm.getConfigurator().apply(config);
        m_follower.getConfigurator().apply(config);
        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_arm.getDeviceID(),true));

        System.out.println("Arm init");
        
    }

    public void stopMotors(){
        m_arm.stopMotor();
    }


    public void percentOut(double speed){
        //dutyCycle.Output = speed;
        //m_arm.setControl(dutyCycle.withOutput(speed));
        m_arm.setControl(dutyCycle.withOutput(speed).withLimitReverseMotion(reverseLimit));

    }

    public boolean isMoving(){
        if(m_arm.getVelocity().getValueAsDouble() > 0.1 || m_arm.getVelocity().getValueAsDouble() < 0.1){
            moving = true;
        } else {
            moving = false;
        }
        return moving;
    }

    public void setPos(){
        
        // if(driver.getRawButton(XboxController.Button.kA.value)){
        //     //setpoint = SCORINGPOS;
        // }else if(driver.getRawButton(XboxController.Button.kB.value)){
        //     //setpoint = INTAKEPOS;
        // }
    }

    public void home(){
      //  m_arm.set(0.1);
      percentOut(Constants.Arm.homeSpeed);
    }

    public boolean isHomed(){
        if(reverseLimit){
            isHomed = true;
            m_arm.setPosition(0);
            count = 0;
            keepReseting = true;
        }else{
            isHomed = false;
        }
         return isHomed;
    }

    public static Arm system(){
        return armSys;
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("arm position", m_encoder.getPosition());
        //CANifier.PinValues values = new CANifier.PinValues();
        getCANifierValues();
        SmartDashboard.putNumber("arm encoder", m_arm.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("reseting",keepReseting);
        if(keepReseting){
            if(reverseLimit){
                m_arm.setPosition(0);
            }
            count++;
            if(count >= countsToReset){
                count = 0;
                keepReseting = false;
            }
        }

        if(isMoving()){
            armState = ArmStateEnum.MOVING;
        } else if (isHomed){
            armState = ArmStateEnum.AT_HOME_POS;
        } else if (m_arm.getPosition().getValueAsDouble() < Constants.Arm.scoringPosition + Constants.Arm.positionTolerance &&
                   m_arm.getPosition().getValueAsDouble() > Constants.Arm.scoringPosition - Constants.Arm.positionTolerance){
            armState = ArmStateEnum.AT_SCORE_POS;            
        }

        
     
    }

    public void getCANifierValues(){
        CANifier.PinValues values = new CANifier.PinValues();
        limitCanifier.getGeneralInputs(values);
        boolean reverseLeftLimit = values.LIMF;
        boolean reverseRightLimit = values.QUAD_A;
        reverseLimit = !reverseLeftLimit || !reverseRightLimit;
        SmartDashboard.putBoolean("Arm_Left",reverseLeftLimit);
        SmartDashboard.putBoolean("Arm_Right",reverseRightLimit);
        SmartDashboard.putBoolean("Arm_Limits",reverseLimit);
    }

    public ArmStateEnum getState(){
        return armState;
    }

    private TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        var motorOutputConfig = new MotorOutputConfigs();
        m_arm.getConfigurator().apply(motorOutputConfig);

        // The left motor is CW+
        //currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorOutputConfig.withInverted(Constants.Arm.MotorOutput.inversion);
        motorOutputConfig.withNeutralMode(Constants.Arm.MotorOutput.neutralMode);
        config.withMotorOutput(motorOutputConfig);
        
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(Constants.Arm.CurrentLimits.supplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(Constants.Arm.CurrentLimits.supplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(Constants.Arm.CurrentLimits.supplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(Constants.Arm.CurrentLimits.supplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);

         // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.withKV(Constants.Arm.kV);
        slot0Configs.withKP(Constants.Arm.PositionPID.kP);
        slot0Configs.withKI(Constants.Arm.PositionPID.kI);
        slot0Configs.withKD(Constants.Arm.PositionPID.kD);
        config.withSlot0(slot0Configs);

        // Motion Magic Configuration
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(Constants.Arm.MotionMagic.motionMagicVelocity);
        motionMagicConfigs.withMotionMagicAcceleration(Constants.Arm.MotionMagic.motionMagicAccel);
        motionMagicConfigs.withMotionMagicJerk(Constants.Arm.MotionMagic.motionMagicJerk);
        config.withMotionMagic(motionMagicConfigs);

        // Software Limit Switch Configuration 
        config.withSoftwareLimitSwitch(Constants.Arm.SoftwareLimits.normalSoftLimits);

        // Hardware Limit Switch Configuration
        HardwareLimitSwitchConfigs hardwareLimitConfigs = new HardwareLimitSwitchConfigs();
        hardwareLimitConfigs.withReverseLimitSource(Constants.Arm.HardWareLimits.reverseLimitSource);
        hardwareLimitConfigs.withReverseLimitType(Constants.Arm.HardWareLimits.reverseLimitType);
        hardwareLimitConfigs.withReverseLimitEnable(Constants.Arm.HardWareLimits.reverseLimitEnabled);
        hardwareLimitConfigs.withReverseLimitAutosetPositionEnable(Constants.Arm.HardWareLimits.reverseLimitAutosetPositionEnabled);
        hardwareLimitConfigs.withReverseLimitAutosetPositionValue(Constants.Arm.HardWareLimits.reverseLimitAutosetPositionValue);

        hardwareLimitConfigs.withForwardLimitSource(Constants.Arm.HardWareLimits.forwardLimitSource);
        hardwareLimitConfigs.withForwardLimitType(Constants.Arm.HardWareLimits.forwardLimitType);
        hardwareLimitConfigs.withForwardLimitEnable(Constants.Arm.HardWareLimits.forwardLimitEnabled);
        hardwareLimitConfigs.withForwardLimitAutosetPositionEnable(Constants.Arm.HardWareLimits.forwardLimitAutosetPositionEnabled);
        hardwareLimitConfigs.withForwardLimitAutosetPositionValue(Constants.Arm.HardWareLimits.forwardLimitAutosetPositionValue);
        config.withHardwareLimitSwitch(hardwareLimitConfigs);

        return config;
     
    }
}