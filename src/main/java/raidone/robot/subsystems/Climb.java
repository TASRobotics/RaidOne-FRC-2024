package raidone.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidone.robot.Constants;

public class Climb extends SubsystemBase{
    private TalonFX climb;
    private TalonFXConfiguration climbConfig;

    private TalonFX follow;
    private TalonFXConfiguration followConfig;
    private Follower followFollower;

    public Climb(){
        climb = new TalonFX(Constants.Climb.CLIMB_MOTOR_ID);

        follow = new TalonFX(Constants.Climb.CLIMB_FOLLOW_ID);
        followFollower = new Follower(Constants.Climb.CLIMB_MOTOR_ID, true);
        follow.setControl(followFollower);
    }

    public void run(double speed){
        climb.set(speed);
    }

    public void stopMotors(){
        climb.stopMotor();
    }
}
