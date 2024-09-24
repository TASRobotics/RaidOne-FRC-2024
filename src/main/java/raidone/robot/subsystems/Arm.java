package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.Constants;
import raidone.robot.RobotContainer;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import static raidone.robot.Constants.Arm.*;

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
  
    public Arm(){
        limitCanifier = RobotContainer.getCANifier();
        System.out.println("Arm init");
        isHomed = false;

        
        m_arm = new TalonFX(Constants.Arm.ARM_MOTOR_ID, Constants.Arm.armCANbus);
        m_follower = new TalonFX(Constants.Arm.ARM_FOLLOW_ID, Constants.Arm.armCANbus);

        var currentConfigs = new MotorOutputConfigs();
        m_arm.getConfigurator().apply(currentConfigs); //apply default config to factory reset
        m_follower.getConfigurator().apply(currentConfigs);

        // The left motor is CCW+
        currentConfigs.withInverted(Constants.Arm.inversion);
        currentConfigs.withNeutralMode(Constants.Arm.neutralMode);

        m_arm.getConfigurator().apply(currentConfigs);
        m_follower.getConfigurator().apply(currentConfigs);
         
        HardwareLimitSwitchConfigs hardwareLimitConfigs = new HardwareLimitSwitchConfigs();
        hardwareLimitConfigs.withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin);
        hardwareLimitConfigs.withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);
        hardwareLimitConfigs.withReverseLimitEnable(false);
        hardwareLimitConfigs.withReverseLimitAutosetPositionEnable(false);
        hardwareLimitConfigs.withReverseLimitAutosetPositionValue(0);

        
        m_arm.getConfigurator().apply(hardwareLimitConfigs);
        m_follower.getConfigurator().apply(hardwareLimitConfigs);
        //

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        softwareLimitSwitchConfigs.withReverseSoftLimitEnable(false);
        softwareLimitSwitchConfigs.withReverseSoftLimitThreshold(-1);
        softwareLimitSwitchConfigs.withForwardSoftLimitEnable(true);
        softwareLimitSwitchConfigs.withForwardSoftLimitThreshold(36);

        m_arm.getConfigurator().apply(softwareLimitSwitchConfigs);
        m_follower.getConfigurator().apply(softwareLimitSwitchConfigs);
        
         // Ensure our followers are following their respective leader
         m_follower.setControl(new Follower(m_arm.getDeviceID(),true));
       
        
    }

    public void stopMotors(){
        m_arm.stopMotor();
    }


    public void percentOut(double speed){
        //dutyCycle.Output = speed;
        m_arm.setControl(dutyCycle.withOutput(speed));
        //m_arm.setControl(dutyCycle.withOutput(speed).withLimitReverseMotion(reverseLimit));

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
      percentOut(-0.3);
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


}