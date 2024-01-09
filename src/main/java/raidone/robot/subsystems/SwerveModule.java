package raidone.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import raidone.robot.Constants;
import raidone.robot.Constants.SwerveConstants;

public class SwerveModule {

    private TalonFX rotor;
    private TalonFX throttle;

    private CANcoder rotorEncoder;
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

        rotor = new TalonFX(rotorID);
        throttle = new TalonFX(throttleID);

        rotorEncoder = new CANcoder(rotorEncoderID);

        rotor.getConfigurator().apply(new TalonFXConfiguration());
        throttle.getConfigurator().apply(new TalonFXConfiguration());
        rotorEncoder.getConfigurator().apply(new CANcoderConfiguration());

        rotor.setInverted(SwerveConstants.kRotorMotorInversion);
        rotor.setNeutralMode(NeutralModeValue.Brake);

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

        throttle.getConfigurator().apply(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        );
        throttle.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Gets current state of module
     * 
     * @return Current state of module
     */
    public SwerveModuleState getState() {
        double throttleVelocity = throttle.getVelocity().getValue() * SwerveConstants.kThrottleVelocityConversionFactor;
        return new SwerveModuleState(
            throttleVelocity,
            Rotation2d.fromDegrees(rotorEncoder.getAbsolutePosition().getValue())
        );
    }

    /**
     * Gets current position of module
     * 
     * @return Current position of module
     */
    public SwerveModulePosition getPosition() {
        double throttlePosition = throttle.getPosition().getValue() * SwerveConstants.kThrottlePositionConversionFactor;
        return new SwerveModulePosition(
            throttlePosition, 
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
        
        double rotorOutput = rotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        rotor.set(rotorOutput);
        throttle.set(optimizedState.speedMetersPerSecond);
    }

}
