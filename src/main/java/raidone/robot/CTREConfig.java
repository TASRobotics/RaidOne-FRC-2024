package raidone.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public final class CTREConfig {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfig(double moduleAngleOffset){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.CAN_CODER_INVERT;
        swerveCANcoderConfig.withMagnetSensor(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(moduleAngleOffset)
        );

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.ROTOR_INVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.ROTOR_NEUTRAL_MODE;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.ROTOR_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.ROTOR_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.ROTOR_CURRENT_THRESHOLD;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.ROTOR_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.ROTOR_KP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.ROTOR_KI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.ROTOR_KD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.THROTTLE_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.THROTTLE_NEUTRAL_MODE;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.THROTTLE_GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.THROTTLE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.THROTTLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.THROTTLE_CURRENT_THRESHOLD;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.THROTTLE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.THROTTLE_KP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.THROTTLE_KI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.THROTTLE_KD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
    }
}