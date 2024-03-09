package raidone.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidone.robot.Constants;

public class Climb extends SubsystemBase {
    private TalonFX climbFX;
    private HardwareLimitSwitchConfigs climbLimitSwitchConfigs;

    private TalonFX followFX;
    private HardwareLimitSwitchConfigs followerLimitSwitchConfigs;

    private Orchestra orchestra;

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

        orchestra = new Orchestra();
        orchestra.addInstrument(climbFX);
        orchestra.addInstrument(followFX);
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

    public void familyMart(){
        orchestra.loadMusic("family_mart.chrp");
        orchestra.play();
    }

    public static Climb system() {
        return climbSys;
    }
}