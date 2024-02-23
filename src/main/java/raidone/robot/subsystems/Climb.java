package raidone.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidone.robot.Constants;

public class Climb extends SubsystemBase {
    private TalonFX climbFX;
    private Slot0Configs climbSlot0Config;

    private TalonFX followFX;
    private Follower followerConfig;

    public Climb() {
        climbFX = new TalonFX(Constants.Climb.CLIMB_MOTOR_ID);
        climbSlot0Config = new Slot0Configs();
        climbSlot0Config.kP = Constants.Climb.CLIMB_KP;
        climbSlot0Config.kI = Constants.Climb.CLIMB_KI;
        climbSlot0Config.kD = Constants.Climb.CLIMB_KD;
        climbFX.getConfigurator().apply(climbSlot0Config);

        followFX = new TalonFX(Constants.Climb.CLIMB_FOLLOW_ID);
        followerConfig = new Follower(Constants.Climb.CLIMB_MOTOR_ID, true);
        followFX.setControl(followerConfig);
    }

    public void run(double position){
        // climbFX.set(speed);
        PositionVoltage request = new PositionVoltage(position).withSlot(0);
        climbFX.setControl(request);
    }

    public void stopMotors() {
        climbFX.stopMotor();
    }
}
