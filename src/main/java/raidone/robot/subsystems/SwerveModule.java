package raidone.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private SparkPIDController throttleVelController;
    private SimpleMotorFeedforward throttleFF;

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

        rotor.setInverted(SwerveConstants.ROTOR_INVERSION);
        rotor.enableVoltageCompensation(Constants.VOLTAGE_COMPENSATION);
        rotor.setIdleMode(IdleMode.kBrake);

        CANcoderConfiguration rotorEncoderConfigs = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withMagnetOffset(rotorOffsetAngle)
                .withSensorDirection(SwerveConstants.ROTOR_ENCODER_DIRECTION)
            );
        
        rotorEncoder.getConfigurator().apply(rotorEncoderConfigs);


        rotorPID = new PIDController(
            SwerveConstants.ROTOR_KP,
            SwerveConstants.ROTOR_KI,
            SwerveConstants.ROTOR_KD
        );

        throttleVelController = throttle.getPIDController();

        throttleVelController.setP(SwerveConstants.THROTTLE_KP, 0);
        throttleVelController.setI(SwerveConstants.THROTTLE_KI, 0);
        throttleVelController.setD(SwerveConstants.THROTTLE_KD, 0);

        throttleFF = new SimpleMotorFeedforward(
            SwerveConstants.THROTTLE_KS,
            SwerveConstants.THROTTLE_KV,
            SwerveConstants.THROTTLE_KA
        );

        rotorPID.enableContinuousInput(-180, 180);

        throttle.enableVoltageCompensation(Constants.VOLTAGE_COMPENSATION);
        
        throttleEncoder.setVelocityConversionFactor(
            SwerveConstants.THROTTLE_VEL_COMPENSATION_FACTOR
        );

        throttleEncoder.setPositionConversionFactor(
            SwerveConstants.THROTTLE_POS_COMPENSATION_FACTOR
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
        throttleVelController.setReference(
            optimizedState.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            throttleFF.calculate(optimizedState.speedMetersPerSecond)
        );
    }

}
