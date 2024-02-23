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

import java.util.function.Consumer;

public class Arm extends SubsystemBase {
    private CANSparkMax arm;
    private CANSparkMax follow;
    private boolean isHomed;
    private SparkPIDController pid;
    private RelativeEncoder encoder;
    private SparkLimitSwitch limit1;
    private SparkLimitSwitch limit2;
    private double setpoint = 0;

    public Arm() {
        System.out.println("Arm init");
        isHomed = false;
        arm = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        follow = new CANSparkMax(ARM_FOLLOW_ID, MotorType.kBrushless);

        arm.restoreFactoryDefaults();
        follow.restoreFactoryDefaults();

        arm.setIdleMode(IdleMode.kBrake);
        follow.setIdleMode(IdleMode.kBrake);

        pid = arm.getPIDController();
        encoder = arm.getEncoder();
        limit1 = arm.getForwardLimitSwitch(Type.kNormallyOpen);
        limit2 = follow.getForwardLimitSwitch(Type.kNormallyOpen);

        limit1.enableLimitSwitch(true);
        limit2.enableLimitSwitch(true);

        arm.setSoftLimit(SoftLimitDirection.kReverse, -28);
        arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // arm.setInverted(true);
        follow.follow(arm, true);

        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

        SmartDashboard.putNumber("Arm Set Position", setpoint);
    }

    public void trapezoidToPID(State output) {
        pid.setReference(output.position, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Arm Trapazoid setpoint", output.position);
    }

    public State currentState() {
        return new State(arm.getEncoder().getPosition(), arm.getEncoder().getVelocity());
    }

    public void stopMotors() {
        arm.stopMotor();
    }

    public boolean getLimit() {
        boolean limitStatus = limit1.isPressed() || limit2.isPressed();
        return limitStatus;
    }

    public void run(double speed) {
        arm.set(speed);
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
        SmartDashboard.putNumber("arm position", encoder.getPosition());
        SmartDashboard.putString("Arm Command",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "");

        if (limit1.isPressed() || limit2.isPressed()) {
            isHomed = true;
            encoder.setPosition(0);
        } else {
            isHomed = false;
        }
    }
}