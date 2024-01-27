package raidone.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public final class CTREConfig {
    public TalonFXConfiguration rotorFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration throttleFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration CANCoderConfig = new CANcoderConfiguration();

    public CTREConfig(double moduleAngleOffset){
        /** Swerve CANCoder Configuration */
        CANCoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.CAN_CODER_INVERT;
        CANCoderConfig.withMagnetSensor(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(moduleAngleOffset)
        );

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        rotorFXConfig.MotorOutput.Inverted = Constants.Swerve.ROTOR_INVERT;
        rotorFXConfig.MotorOutput.NeutralMode = Constants.Swerve.ROTOR_NEUTRAL_MODE;

        /* Gear Ratio and Wrapping Config */
        rotorFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.ROTOR_GEAR_RATIO;
        rotorFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        rotorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.ANGLE_ENABLE_CURRENT_LIMIT;
        rotorFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.ROTOR_CURRENT_LIMIT;
        rotorFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.ROTOR_CURRENT_THRESHOLD;
        rotorFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.ROTOR_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        rotorFXConfig.Slot0.kP = Constants.Swerve.ROTOR_KP;
        rotorFXConfig.Slot0.kI = Constants.Swerve.ROTOR_KI;
        rotorFXConfig.Slot0.kD = Constants.Swerve.ROTOR_KD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        throttleFXConfig.MotorOutput.Inverted = Constants.Swerve.THROTTLE_INVERT;
        throttleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.THROTTLE_NEUTRAL_MODE;

        /* Gear Ratio Config */
        throttleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.THROTTLE_GEAR_RATIO;

        /* Current Limiting */
        throttleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.THROTTLE_ENABLE_CURRENT_LIMIT;
        throttleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.THROTTLE_CURRENT_LIMIT;
        throttleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.THROTTLE_CURRENT_THRESHOLD;
        throttleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.THROTTLE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        throttleFXConfig.Slot0.kP = Constants.Swerve.THROTTLE_KP;
        throttleFXConfig.Slot0.kI = Constants.Swerve.THROTTLE_KI;
        throttleFXConfig.Slot0.kD = Constants.Swerve.THROTTLE_KD;

        /* Open and Closed Loop Ramping */
        throttleFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        throttleFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;

        throttleFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
        throttleFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
    }
}