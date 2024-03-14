package raidone.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import static raidone.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
    private CANSparkMax arm;
    private CANSparkMax follow;

    private RelativeEncoder encoder;
    private SparkPIDController pid;
    private SparkLimitSwitch limit1, limit2;

    private static Arm armSys = new Arm();

    private Arm() {
        System.out.println("Arm Subsystem init");

        arm = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        arm.restoreFactoryDefaults();
        arm.setIdleMode(IdleMode.kBrake);
        arm.setSoftLimit(SoftLimitDirection.kReverse, SOFTLIMIT);
        arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        arm.setSmartCurrentLimit(CURRENT_LIMIT);

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
    }

    public void trapezoidToPID(State output) {
        pid.setReference(output.position, CANSparkMax.ControlType.kPosition);// 0,
                                                                             // FEED_FORWARD.calculate(output.position,
                                                                // output.velocity));
        // SmartDashboard.putNumber("Arm Trapazoid setpoint", output.position);
    }

    public State currentState() {
        return new State(arm.getEncoder().getPosition(), arm.getEncoder().getVelocity());
    }

    public void stopMotors() {
        arm.stopMotor();
    }

    public void setPos(double setpoint) {
        pid.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        // SmartDashboard.putNumber("processVariable", encoder.getPosition());
    }

    public void home() {
        arm.set(0.4);
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public boolean getLimit() {
        if (limit1.isPressed() || limit2.isPressed())
            encoder.setPosition(0);
        return limit1.isPressed() || limit2.isPressed();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm encoder pos", arm.getEncoder().getPosition());
    }

    public static Arm system() {
        return armSys;
    }
}