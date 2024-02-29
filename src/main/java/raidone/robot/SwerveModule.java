package raidone.robot;

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

public class SwerveModule {

    private CANSparkMax throttle;
    private CANSparkMax rotor;
    private CANcoder CANCoder;
    private RelativeEncoder throttleEncoder;
    public CANcoderConfiguration CANCoderConfig;

    private PIDController rotorPID;
    private SparkPIDController throttleVelController;
    private SimpleMotorFeedforward throttleFF;

    /**
     * Constructs a new SwerveModule
     * 
     * @param throttleID CAN ID of throttle motor
     * @param rotorID CAN ID of rotor motor
     * @param canCoderID CAN ID of CANcoder
     * @param moduleAngleOffset CANcoder offset
     * @param throttleInversion Invert throttle
     */
    public SwerveModule(int throttleID, int rotorID, int canCoderID, double moduleAngleOffset, boolean throttleInversion) {

        throttle = new CANSparkMax(throttleID, MotorType.kBrushless);
        throttleEncoder = throttle.getEncoder();

        rotor = new CANSparkMax(rotorID, MotorType.kBrushless);

        CANCoder = new CANcoder(canCoderID);

        throttle.restoreFactoryDefaults();
        rotor.restoreFactoryDefaults();

        throttle.setIdleMode(IdleMode.kBrake);
        throttle.setInverted(throttleInversion);

        throttleEncoder.setVelocityConversionFactor(
                Constants.Swerve.THROTTLE_VEL_CONVERSION_FACTOR);
        throttleEncoder.setPositionConversionFactor(
                Constants.Swerve.THROTTLE_POS_CONVERSTION_FACTOR);


        rotor.setInverted(true);
        rotor.setIdleMode(IdleMode.kBrake);


        CANCoder.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withMagnetOffset(moduleAngleOffset)
            )
        );


        rotorPID = new PIDController(
            Constants.Swerve.ROTOR_KP,
            Constants.Swerve.ROTOR_KI,
            Constants.Swerve.ROTOR_KD
        );

        rotorPID.enableContinuousInput(-180, 180);


        throttleVelController = throttle.getPIDController();

        throttleVelController.setP(Constants.Swerve.THROTTLE_KP, 0);
        throttleVelController.setI(Constants.Swerve.THROTTLE_KI, 0);
        throttleVelController.setD(Constants.Swerve.THROTTLE_KD, 0);
        throttleVelController.setFF(Constants.Swerve.THROTTLE_KF, 0);

        throttleVelController.setFeedbackDevice(throttleEncoder);

        throttleFF = new SimpleMotorFeedforward(
            Constants.Swerve.THROTTLE_KS,
            Constants.Swerve.THROTTLE_KV,
            Constants.Swerve.THROTTLE_KA
        );

    }

    /**
     * Sets desired state of swervemodule
     * 
     * @param desiredState Desired state
     * @param isOpenLoop Open loop
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        rotor.set(rotorPID.calculate(getState().angle.getDegrees(), desiredState.angle.getDegrees()));
        
        if (isOpenLoop) {
            throttle.set(desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED_MPS);
        } else {
            // double accel = 
            throttleVelController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0,
                throttleFF.calculate(desiredState.speedMetersPerSecond)
            );
            // throttle.set(throttleFF.calculate(desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED_MPS, getState().speedMetersPerSecond - prevVel));
            // throttle.set(throttleFF.calculate(desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED_MPS));
            // prevVel = getState().speedMetersPerSecond;
        }
    }

    /**
     * Gets swervemodule state
     * 
     * @return {@link SwerveModuleState} object
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                throttleEncoder.getVelocity(),
                Rotation2d.fromRotations(CANCoder.getAbsolutePosition().getValue()));
    }

    /**
     * Gets swervemodule position
     * 
     * @return {@link SwerveModulePosition} object
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                throttleEncoder.getPosition(),
                Rotation2d.fromRotations(CANCoder.getPosition().getValue()));
    }

    public void setCoast(){
        throttle.setIdleMode(IdleMode.kCoast);
        rotor.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake(){
        throttle.setIdleMode(IdleMode.kBrake);
        rotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Resets swervemodule to absolute position
     */
    public void resetToAbsolute() {
        rotor.getEncoder()
                .setPosition(Rotation2d.fromRotations(CANCoder.getAbsolutePosition().getValue()).getRotations());
    }
}