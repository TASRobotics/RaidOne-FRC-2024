package raidone.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidone.robot.Constants;

public class Climb extends SubsystemBase {
    private TalonFX climbFX;
    private HardwareLimitSwitchConfigs climbLimitSwitchConfigs;

    private TalonFX followFX;
    private Follower followerConfig;
    private HardwareLimitSwitchConfigs followerLimitSwitchConfigs;

    private static Climb climbSys = new Climb();

    private Climb() {
        climbFX = new TalonFX(Constants.Climb.CLIMB_MOTOR_ID);
        climbLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        climbLimitSwitchConfigs.withReverseLimitAutosetPositionEnable(true);
        climbLimitSwitchConfigs.ReverseLimitAutosetPositionValue = 0.0;
        climbFX.getConfigurator().apply(climbLimitSwitchConfigs);

        followFX = new TalonFX(Constants.Climb.CLIMB_FOLLOW_ID);
        followerConfig = new Follower(Constants.Climb.CLIMB_MOTOR_ID, true);
        followFX.setControl(followerConfig);

        followerLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        followerLimitSwitchConfigs.withForwardLimitAutosetPositionEnable(true);
        followerLimitSwitchConfigs.ForwardLimitAutosetPositionValue = 0.0;
        followFX.getConfigurator().apply(followerLimitSwitchConfigs);
    }

    public double getEncoderPos(){
        return climbFX.getPosition().getValueAsDouble();
    }

    public void run(double speed){
        climbFX.set(speed);
    }

    public void stopMotors() {
        climbFX.stopMotor();
    }

    public static Climb system(){
        return climbSys;
    }
}