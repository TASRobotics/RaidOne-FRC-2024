package raidone.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import raidone.robot.Constants;
import raidone.robot.Constants.SwerveConstants;

public class SwerveModule {

    private WPI_TalonFX rotor;
    private WPI_TalonFX throttle;

    private WPI_CANCoder rotorEncoder;
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

        rotor = new WPI_TalonFX(rotorID);
        throttle = new WPI_TalonFX(throttleID);

        rotorEncoder = new WPI_CANCoder(rotorEncoderID);
        //throttleEncoder = throttle.getEncoder();

        rotor.configFactoryDefault();
        throttle.configFactoryDefault();
        rotorEncoder.configFactoryDefault();

        rotor.setInverted(SwerveConstants.kRotorMotorInversion);
        rotor.configVoltageCompSaturation(Constants.kVoltageCompensation);
        rotor.enableVoltageCompensation(true);
        rotor.setNeutralMode(NeutralMode.Brake);

        rotorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        rotorEncoder.configMagnetOffset(rotorOffsetAngle);
        rotorEncoder.configSensorDirection(SwerveConstants.kRotorEncoderDirection);
        rotorEncoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );


        rotorPID = new PIDController(
            SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD
        );

        rotorPID.enableContinuousInput(-180, 180);

        throttle.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        throttle.configVoltageCompSaturation(Constants.kVoltageCompensation);
        throttle.enableVoltageCompensation(true);
        throttle.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Gets current state of module
     * 
     * @return Current state of module
     */
    public SwerveModuleState getState() {
        double throttleVelocity = throttle.getSelectedSensorVelocity() * SwerveConstants.kThrottleVelocityConversionFactor; 
        return new SwerveModuleState(
            throttleVelocity,
            Rotation2d.fromDegrees(rotorEncoder.getAbsolutePosition())
        );
    }

    /**
     * Gets current position of module
     * 
     * @return Current position of module
     */
    public SwerveModulePosition getPosition() {
        double throttlePosition = throttle.getSelectedSensorPosition() * SwerveConstants.kThrottlePositionConversionFactor;
        return new SwerveModulePosition(
            throttlePosition, 
            Rotation2d.fromDegrees(rotorEncoder.getAbsolutePosition())
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
