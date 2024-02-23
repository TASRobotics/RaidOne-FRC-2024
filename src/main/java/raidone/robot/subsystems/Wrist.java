package raidone.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static raidone.robot.Constants.Wrist.*;

public class Wrist extends SubsystemBase {
    private CANSparkMax wrist;
    private CANSparkMax follower;
    private boolean isHomed;
    private SparkPIDController pid;
    private RelativeEncoder encoder;
    private SparkLimitSwitch limit;

    public Wrist() {
        System.out.println("Wrist init");
        isHomed = false;
        wrist = new CANSparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
        follower = new CANSparkMax(WRIST_FOLLOW_ID, MotorType.kBrushless);
        wrist.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();

        wrist.setInverted(true);

        pid = wrist.getPIDController();
        encoder = wrist.getEncoder();
        limit = wrist.getForwardLimitSwitch(Type.kNormallyOpen);

        wrist.setIdleMode(IdleMode.kBrake);
        follower.setIdleMode(IdleMode.kBrake);

        limit.enableLimitSwitch(true);

        follower.follow(wrist, true);

        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
    }

    public void stopMotors() {
        wrist.stopMotor();
    }

    public void setPos(double setpoint) {
        pid.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("processVariable", encoder.getPosition());
    }

    public void home() {
        wrist.set(0.3);
    }

    public boolean isHomed() {
        return isHomed;
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    @Override
    public void periodic() {
        // TODO: Remove this in favor of only chekcking when we need to (isfinished in a
        // command most probably)
        if (limit.isPressed()) {
            isHomed = true;
            encoder.setPosition(0);
        } else {
            isHomed = false;
        }
    }
}
