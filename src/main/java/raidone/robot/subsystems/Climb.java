package raidone.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidone.robot.Constants;

public class Climb extends SubsystemBase {
    private TalonFX climbFX;
    private HardwareLimitSwitchConfigs climbLimitSwitchConfigs;
    private Slot0Configs climbSlot0Configs;
    private MotionMagicConfigs climbMotionMagicConfigs;
    private MotionMagicVoltage climbRequest;

    private TalonFX followFX;
    private HardwareLimitSwitchConfigs followerLimitSwitchConfigs;
    private Slot0Configs followSlot0Configs;
    private MotionMagicConfigs followMotionMagicConfigs;
    private MotionMagicVoltage followRequest;

    private static Climb climbSys = new Climb();

    private Climb() {
        System.out.println("Climb Subsystem Init");

        climbLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        climbLimitSwitchConfigs.withReverseLimitAutosetPositionEnable(true);
        climbLimitSwitchConfigs.ReverseLimitAutosetPositionValue = 0.0;

        //TODO: ADD CURRENT LIMIT (35 Amp)

        climbSlot0Configs = new Slot0Configs();
        climbSlot0Configs.kS = Constants.Climb.kS;
        climbSlot0Configs.kV = Constants.Climb.kV;
        climbSlot0Configs.kA = Constants.Climb.kA;
        climbSlot0Configs.kP = Constants.Climb.kP;
        climbSlot0Configs.kI = Constants.Climb.kI;
        climbSlot0Configs.kD = Constants.Climb.kD;

        climbMotionMagicConfigs = new MotionMagicConfigs();
        climbMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.Climb.MOTION_MAGIC_MAX_VELOCITY_RPS;
        climbMotionMagicConfigs.MotionMagicAcceleration = Constants.Climb.MOTION_MAGIC_ACCELERAION_RPS2;
        climbMotionMagicConfigs.MotionMagicJerk = Constants.Climb.MOTION_MAGIC_JERK_RPS3;

        climbFX = new TalonFX(Constants.Climb.MOTOR_ID);
        climbFX.getConfigurator().apply(new TalonFXConfiguration());
        climbFX.getConfigurator().apply(climbLimitSwitchConfigs);
        climbFX.getConfigurator().apply(climbSlot0Configs);
        climbFX.getConfigurator().apply(climbMotionMagicConfigs);



        followerLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        followerLimitSwitchConfigs.withForwardLimitAutosetPositionEnable(true);
        followerLimitSwitchConfigs.ForwardLimitAutosetPositionValue = 0.0;

        followSlot0Configs = new Slot0Configs();
        followSlot0Configs.kS = Constants.Climb.kS;
        followSlot0Configs.kV = Constants.Climb.kV;
        followSlot0Configs.kA = Constants.Climb.kA;

        followSlot0Configs.kP = Constants.Climb.kP;
        followSlot0Configs.kI = Constants.Climb.kI;
        followSlot0Configs.kD = Constants.Climb.kD;

        followMotionMagicConfigs = new MotionMagicConfigs();
        followMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.Climb.MOTION_MAGIC_MAX_VELOCITY_RPS;
        followMotionMagicConfigs.MotionMagicAcceleration = Constants.Climb.MOTION_MAGIC_ACCELERAION_RPS2;
        followMotionMagicConfigs.MotionMagicJerk = Constants.Climb.MOTION_MAGIC_JERK_RPS3;

        followFX = new TalonFX(Constants.Climb.FOLLOW_ID);
        followFX.getConfigurator().apply(new TalonFXConfiguration());
        followFX.setInverted(true);
        followFX.getConfigurator().apply(followerLimitSwitchConfigs);
        followFX.getConfigurator().apply(followSlot0Configs);
        followFX.getConfigurator().apply(followMotionMagicConfigs);

    }

    public double getClimbEncoderPos() {
        return climbFX.getPosition().getValueAsDouble();
    }

    public double getFollowEncoderPos() {
        return followFX.getPosition().getValueAsDouble();
    }

    public void runClimb(double position) {
        climbRequest = new MotionMagicVoltage(position);
        climbFX.setControl(climbRequest.withPosition(position));
    }

    public void climbHome(){
        climbFX.set(-1 * 0.2);
    }

    public void runFollow(double position) {
        followRequest = new MotionMagicVoltage(position);
        followFX.setControl(followRequest.withPosition(position));
    }

    public void followHome(){
        followFX.set(0.2);
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

    public boolean climbAtPos(){
        return climbFX.getClosedLoopError().getValueAsDouble() <= Constants.Climb.ALLOWED_ERROR_ROT;
    }
    public boolean followAtPos(){
        return followFX.getClosedLoopError().getValueAsDouble() <= Constants.Climb.ALLOWED_ERROR_ROT;
    }

    public static Climb system() {
        return climbSys;
    }
}