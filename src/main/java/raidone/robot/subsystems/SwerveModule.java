package raidone.robot.subsystems;

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
import raidone.robot.Constants;
import raidone.robot.Constants.SwerveConstants;

public class SwerveModule {

    private CANSparkMax rotor;
    private CANSparkMax throttle;

    private CANcoder rotorEncoder;
    private RelativeEncoder throttleEncoder;

    private PIDController rotorPID;

    /**
     * Constructs SwerveModule object
     * 
     * @param rotorID CAN ID of rotor motor
     * @param throttleID CAN ID of throttle motor
     * @param rotorEncoderID CAN ID of rotor encoder
     * @param rotorOffsetAngle Offset angle of rotor encoder
     */
    public SwerveModule(int rotorID, int throttleID, int rotorEncoderID, double rotorOffsetAngle) {

        rotor = new CANSparkMax(rotorID, MotorType.kBrushless);
        throttle = new CANSparkMax(throttleID, MotorType.kBrushless);

        rotorEncoder = new CANcoder(rotorEncoderID);
        throttleEncoder = throttle.getEncoder();

        rotor.restoreFactoryDefaults();
        throttle.restoreFactoryDefaults();
        rotorEncoder.getConfigurator().apply(new CANcoderConfiguration());

        rotor.setInverted(SwerveConstants.kRotorMotorInversion);
        rotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        rotor.setIdleMode(IdleMode.kBrake);

        CANcoderConfiguration rotorEncoderConfigs = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withMagnetOffset(rotorOffsetAngle)
                .withSensorDirection(SwerveConstants.kRotorEncoderDirection)
            );
        
        rotorEncoder.getConfigurator().apply(rotorEncoderConfigs);


        rotorPID = new PIDController(
            SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD
        );

        rotorPID.enableContinuousInput(-180, 180);

        throttle.enableVoltageCompensation(Constants.kVoltageCompensation);
        
        throttleEncoder.setVelocityConversionFactor(
            SwerveConstants.kThrottleVelocityConversionFactor
        );

        throttleEncoder.setPositionConversionFactor(
            SwerveConstants.kThrottlePositionConversionFactor
        );
    }

    /**
     * Gets current state of module
     * 
     * @return Current state of module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            throttleEncoder.getVelocity(),
            Rotation2d.fromDegrees(rotorEncoder.getAbsolutePosition().getValue())
        );
    }

    /**
     * Gets current position of module
     * 
     * @return Current position of module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            throttleEncoder.getPosition(),
            Rotation2d.fromDegrees(rotorEncoder.getAbsolutePosition().getValue())
        );
    }

    /**
     * Set module state
     * 
     * @param state Module state
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        
        double rotorOutput = rotorPID.calculate(
            getState().angle.getDegrees(),
            optimizedState.angle.getDegrees()
        );

        rotor.set(rotorOutput);
        throttle.set(optimizedState.speedMetersPerSecond);
    }

}
