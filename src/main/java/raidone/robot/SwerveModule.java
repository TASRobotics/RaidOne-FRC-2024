package raidone.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private CANSparkMax rotor;
    private CANSparkMax throttle;
    private CANcoder rotorEncoder;
    private RelativeEncoder throttleEncoder;
    public CANcoderConfiguration CANCoderConfig;

    private PIDController rotorPID;

    public SwerveModule(int throttleID, int rotorID, int canCoderID, double moduleAngleOffset) {
        throttle = new CANSparkMax(throttleID, MotorType.kBrushless);
        throttleEncoder = throttle.getEncoder();

        rotor = new CANSparkMax(rotorID, MotorType.kBrushless);

        rotorEncoder = new CANcoder(canCoderID);

        throttle.restoreFactoryDefaults();
        rotor.restoreFactoryDefaults();

        CANCoderConfig = new CANcoderConfiguration();

        rotor.setInverted(false);
        rotor.setIdleMode(IdleMode.kBrake);

        CANCoderConfig.withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(moduleAngleOffset));

        rotorEncoder.getConfigurator().apply(CANCoderConfig);

        rotorPID = new PIDController(
                Constants.Swerve.ROTOR_KP,
                Constants.Swerve.ROTOR_KI,
                Constants.Swerve.ROTOR_KD);

        rotorPID.enableContinuousInput(-180, 180);

        throttle.setIdleMode(IdleMode.kBrake);

        throttleEncoder.setVelocityConversionFactor(
                Constants.Swerve.THROTTLE_VEL_CONVERSION_FACTOR);
        throttleEncoder.setPositionConversionFactor(
                Constants.Swerve.THROTTLE_POS_CONVERSTION_FACTOR);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        rotor.set(rotorPID.calculate(getState().angle.getDegrees(), desiredState.angle.getDegrees()));
        throttle.set(desiredState.speedMetersPerSecond);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                throttleEncoder.getVelocity(),
                Rotation2d.fromDegrees(throttleEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                throttleEncoder.getPosition(),
                Rotation2d.fromRotations(rotorEncoder.getPosition().getValue()));
    }
}