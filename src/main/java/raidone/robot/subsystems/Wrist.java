package raidone.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static raidone.robot.Constants.Wrist.*;

public class Wrist extends SubsystemBase {
    private CANSparkMax wrist, follower;
    private SparkPIDController pid;
    private RelativeEncoder encoder;
    private SparkLimitSwitch limit;
    private boolean isHomed;

    public static Wrist wristSys = new Wrist();

    private Wrist() {
        System.out.println("Wrist Subsystem Init");

        wrist = new CANSparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
        wrist.restoreFactoryDefaults();
        wrist.setInverted(true);
        wrist.setIdleMode(IdleMode.kBrake);

        follower = new CANSparkMax(WRIST_FOLLOW_ID, MotorType.kBrushless);
        follower.restoreFactoryDefaults();
        follower.setIdleMode(IdleMode.kBrake);
        follower.follow(wrist, true);

        pid = wrist.getPIDController();
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);

        encoder = wrist.getEncoder();

        limit = wrist.getForwardLimitSwitch(Type.kNormallyOpen);
        limit.enableLimitSwitch(true);

        isHomed = false;
    }

    public void trapezoidToPID(State output) {
        pid.setReference(output.position, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Wrist Trapazoid setpoint", output.position);
    }

    public State currentState() {
        return new State(wrist.getEncoder().getPosition(), wrist.getEncoder().getVelocity());
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

    public static Wrist system() {
        return wristSys;
    }
}
