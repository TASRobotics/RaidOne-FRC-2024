package raidone.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static raidone.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
    private CANSparkMax roller;
    private SparkLimitSwitch beam;

    private static Intake intakeSys = new Intake();

    private Intake() {
        System.out.println("Intake Subsystem Init");

        roller = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        roller.setIdleMode(IdleMode.kBrake);

        beam = roller.getForwardLimitSwitch(Type.kNormallyOpen);
    }

    public boolean getLimit() {
        boolean limitStatus = beam.isPressed();
        return limitStatus;
    }

    public void run(double s) {
        roller.set(s);
    }

    public void stop() {
        roller.stopMotor();
    }

    public void resetEncoder() {
        roller.getEncoder().setPosition(0);
    }

    public boolean isRetracted() {
        return Math.abs(roller.getEncoder().getPosition()) > 2;
    }

    @Override
    public void periodic() {
    }

    public static Intake system() {
        return intakeSys;
    }
}
