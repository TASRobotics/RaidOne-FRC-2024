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

    public static Wrist wristSys = new Wrist();

    private Wrist() {
        System.out.println("Wrist Subsystem Init");

        wrist = new CANSparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
        wrist.restoreFactoryDefaults();
        wrist.setInverted(true);
        wrist.setIdleMode(IdleMode.kBrake);
        wrist.setSmartCurrentLimit(CURRENT_LIMIT);

        follower = new CANSparkMax(WRIST_FOLLOW_ID, MotorType.kBrushless);
        follower.restoreFactoryDefaults();
        follower.setIdleMode(IdleMode.kBrake);
        follower.follow(wrist, true);

        pid = wrist.getPIDController();
        pid.setP(kP, 0);
        pid.setI(kI, 0);
        pid.setD(kD, 0);
        pid.setIZone(kIz, 0);
        pid.setFF(kFF, 0);

        encoder = wrist.getEncoder();

        limit = wrist.getForwardLimitSwitch(Type.kNormallyOpen);
        limit.enableLimitSwitch(true);
    }

    public void trapezoidToPID(State output) {
        pid.setReference(output.position, CANSparkMax.ControlType.kPosition);// 0, FEED_FORWARD.calculate(output.position, output.velocity));
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
        wrist.set(0.5);
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public boolean getLimit(){
        if(limit.isPressed())
            encoder.setPosition(0);
        return limit.isPressed();
    }

    @Override
    public void periodic() {}

    public static Wrist system() {
        return wristSys;
    }
}
