package raidone.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkBase.IdleMode;

import static raidone.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
    private CANSparkMax arm;
    private CANSparkMax follow;

    private RelativeEncoder encoder;
    private SparkPIDController pid;
    private SparkLimitSwitch limit1, limit2;

    private boolean isHomed;

    private static Arm armSys = new Arm();

    private Arm() {
        System.out.println("Arm Subsystem init");

        arm = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        arm.restoreFactoryDefaults();
        arm.setIdleMode(IdleMode.kBrake);
        arm.setSoftLimit(SoftLimitDirection.kReverse, -28);
        arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        arm.setSmartCurrentLimit(20);

        follow = new CANSparkMax(ARM_FOLLOW_ID, MotorType.kBrushless);
        follow.restoreFactoryDefaults();
        follow.setIdleMode(IdleMode.kBrake);
        follow.follow(arm, true);

        pid = arm.getPIDController();
        pid.setP(kP, 0);
        pid.setI(kI, 0);
        pid.setD(kD, 0);
        pid.setIZone(kIz, 0);
        pid.setFF(kFF, 0);
        pid.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);


        encoder = arm.getEncoder();

        limit1 = arm.getForwardLimitSwitch(Type.kNormallyOpen);
        limit1.enableLimitSwitch(true);

        limit2 = follow.getForwardLimitSwitch(Type.kNormallyOpen);
        limit2.enableLimitSwitch(true);

        isHomed = false;
    }

    public void trapezoidToPID(State output) {
        pid.setReference(output.position, CANSparkMax.ControlType.kPosition);// 0, FEED_FORWARD.calculate(output.position, output.velocity));
        SmartDashboard.putNumber("Arm Trapazoid setpoint", output.position);
    }

    public State currentState() {
        return new State(arm.getEncoder().getPosition(), arm.getEncoder().getVelocity());
    }

    public void stopMotors() {
        arm.stopMotor();
    }

    public boolean getLimit() {
        return limit1.isPressed() || limit2.isPressed();
    }

    public void setPos(double setpoint) {
        pid.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("processVariable", encoder.getPosition());
    }

    public void home() {
        arm.set(0.2);
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public boolean isHomed() {
        return isHomed;
    }

    @Override
    public void periodic() {
        // TODO: Remove this in favor of only chekcking when we need to (isfinished in a
        // command most probably)
        if (limit1.isPressed() || limit2.isPressed()) {
            isHomed = true;
            encoder.setPosition(0);
        } else {
            isHomed = false;
        }
    }

    public static Arm system() {
        return armSys;
    }
}