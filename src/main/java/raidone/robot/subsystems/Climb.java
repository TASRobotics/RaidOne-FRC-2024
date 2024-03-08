package raidone.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidone.robot.Constants;

public class Climb extends SubsystemBase {
    private TalonFX climbFX;
    private HardwareLimitSwitchConfigs climbLimitSwitchConfigs;

    private TalonFX followFX;
    // private Follower followerConfig;
    private HardwareLimitSwitchConfigs followerLimitSwitchConfigs;

    private static Climb climbSys = new Climb();

    private Climb() {
        System.out.println("Climb Subsystem Init");

        climbLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        climbLimitSwitchConfigs.withReverseLimitAutosetPositionEnable(true);
        climbLimitSwitchConfigs.ReverseLimitAutosetPositionValue = 0.0;

        climbFX = new TalonFX(Constants.Climb.CLIMB_MOTOR_ID);
        climbFX.getConfigurator().apply(new TalonFXConfiguration());
        climbFX.getConfigurator().apply(climbLimitSwitchConfigs);

        followerLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        followerLimitSwitchConfigs.withForwardLimitAutosetPositionEnable(true);
        followerLimitSwitchConfigs.ForwardLimitAutosetPositionValue = 0.0;

        followFX = new TalonFX(Constants.Climb.CLIMB_FOLLOW_ID);
        followFX.getConfigurator().apply(new TalonFXConfiguration());
        followFX.setInverted(true);
        followFX.getConfigurator().apply(followerLimitSwitchConfigs);

    }

    public double getClimbEncoderPos() {
        return climbFX.getPosition().getValueAsDouble();
    }

    public double getFollowEncoderPos() {
        return followFX.getPosition().getValueAsDouble();
    }

    public void runClimb(double speed) {
        climbFX.set(speed);
    }

    public void runFollow(double speed) {
        followFX.set(-1 * speed);
    }

    public void stopClimbMotor() {
        climbFX.stopMotor();
    }

    public void stopFollowMotor() {
        followFX.stopMotor();
    }

    public boolean getClimbLimit() {
        return climbFX.getReverseLimit().getValue().value == 0;
    }

    public boolean getFollowLimit() {
        return followFX.getForwardLimit().getValue().value == 0;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Follow Encoder Pos", getFollowEncoderPos());
    }

    public static Climb system() {
        return climbSys;
    }
}