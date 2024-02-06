package raidone.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import raidone.lib.math.Conversions;

public class SwerveModule {
    private CANSparkMax rotor;
    private CANSparkMax throttle;
    private CANcoder CANCoder;
    private RelativeEncoder throttleEncoder;
    public CANcoderConfiguration CANCoderConfig;

    private PIDController rotorPID;
    private SparkPIDController throttleVelController;
    private SimpleMotorFeedforward throttleFF;
    public SwerveModule(int throttleID, int rotorID, int canCoderID, double moduleAngleOffset, boolean throttleInversion) {
        throttle = new CANSparkMax(throttleID, MotorType.kBrushless);
        throttleEncoder = throttle.getEncoder();

        rotor = new CANSparkMax(rotorID, MotorType.kBrushless);

        CANCoder = new CANcoder(canCoderID);

        throttle.restoreFactoryDefaults();
        rotor.restoreFactoryDefaults();

        CANCoderConfig = new CANcoderConfiguration();

        rotor.setInverted(true);
        rotor.setIdleMode(IdleMode.kBrake);

        CANCoderConfig.withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withMagnetOffset(moduleAngleOffset));

        CANCoder.getConfigurator().apply(CANCoderConfig);

        rotorPID = new PIDController(
            Constants.Swerve.ROTOR_KP,
            Constants.Swerve.ROTOR_KI,
            Constants.Swerve.ROTOR_KD
        );

        rotorPID.enableContinuousInput(-180, 180);

        throttle.setIdleMode(IdleMode.kBrake);
        throttle.setInverted(throttleInversion);

        throttleEncoder.setVelocityConversionFactor(
                Constants.Swerve.THROTTLE_VEL_CONVERSION_FACTOR);
        throttleEncoder.setPositionConversionFactor(
                Constants.Swerve.THROTTLE_POS_CONVERSTION_FACTOR);
        throttleVelController = throttle.getPIDController();

        throttleVelController.setP(Constants.Swerve.THROTTLE_KP, 0);
        throttleVelController.setI(Constants.Swerve.THROTTLE_KI, 0);
        throttleVelController.setD(Constants.Swerve.THROTTLE_KD, 0);

        throttleFF = new SimpleMotorFeedforward(
            Constants.Swerve.THROTTLE_KS,
            Constants.Swerve.THROTTLE_KV,
            Constants.Swerve.THROTTLE_KA
        );
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        rotor.set(rotorPID.calculate(getState().angle.getDegrees(), desiredState.angle.getDegrees()));
        // rotorPID.setReference(desiredState.angle.getRotations(), ControlType.kPosition);
        
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            // driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            throttle.set(desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);
        } else {
            throttleVelController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0,
                throttleFF.calculate(desiredState.speedMetersPerSecond)
            );
        }
    }

    public void throttleGo(){
        throttle.set(0.25);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                throttleEncoder.getVelocity(),
                Rotation2d.fromRotations(CANCoder.getAbsolutePosition().getValue()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                throttleEncoder.getPosition(),
                Rotation2d.fromRotations(CANCoder.getPosition().getValue()));
    }

    public void resetToAbsolute() {
        rotor.getEncoder()
                .setPosition(Rotation2d.fromRotations(CANCoder.getAbsolutePosition().getValue()).getRotations());
    }
}