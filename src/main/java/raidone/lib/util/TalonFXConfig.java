package raidone.lib.util;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class TalonFXConfig {

    public static class SwerveModule {

        /**
         * 
         * @param rotorID CAN ID of rotor motor
         * @return Configured rotor motor (TalonFX object)
         */
        public static TalonFX configNewRotor(int rotorID) {
            MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
    
            Slot0Configs slot0Configs = new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0);
    
            ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
            closedLoopGeneralConfigs.ContinuousWrap = true;
    
            CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(25)
            .withSupplyCurrentThreshold(40)
            .withSupplyTimeThreshold(0.1);
    
            TalonFXConfiguration configs = new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withSlot0(slot0Configs)
                .withClosedLoopGeneral(closedLoopGeneralConfigs)
                .withCurrentLimits(currentLimitsConfigs);
    
            TalonFX rotor = new TalonFX(rotorID, "seCANdary");
            rotor.getConfigurator().apply(configs);
    
            return rotor;
        }
    
        /**
         * 
         * @param throttleID CAN ID of throttle motor
         * @return Configured throttle motor (TalonFX object)
         */
        public static TalonFX configNewThrottle(int throttleID) {
            MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

            FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio((6.75 / 1.0));

            CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(35)
                .withSupplyCurrentThreshold(60)
                .withSupplyTimeThreshold(0.1);

            Slot0Configs slot0Configs = new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0);

            OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.25)
                .withVoltageOpenLoopRampPeriod(0.25);
            
            ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(0.0)
                .withVoltageClosedLoopRampPeriod(0.0);

            TalonFXConfiguration configs = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs)
                .withCurrentLimits(currentLimitsConfigs)
                .withSlot0(slot0Configs)
                .withOpenLoopRamps(openLoopRampsConfigs)
                .withClosedLoopRamps(closedLoopRampsConfigs);

            TalonFX throttle = new TalonFX(throttleID, "seCANdary");
            throttle.getConfigurator().apply(configs);
            throttle.getConfigurator().setPosition(0.0);
    
            return throttle;
        }
    
        /**
         * 
         * @param CANCoderID CAN ID of CANcoder
         * @param angleOffset Angle offset of CANcoder
         * @return Configured CANcoder object
         */
        public static CANcoder configNewCANcoder(int CANCoderID, double angleOffset) {
            MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(angleOffset);

            CANcoder cancoder = new CANcoder(CANCoderID, "seCANdary");
            cancoder.getConfigurator().apply(magnetSensorConfigs);
            
            return cancoder;
        }

    }

}
