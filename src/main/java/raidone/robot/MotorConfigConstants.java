package raidone.robot;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import raidone.robot.Constants.Arm.Feedback;



public final class MotorConfigConstants {
    public static final class Wrist {
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue inversion = InvertedValue.Clockwise_Positive;
        
        // Current Limit Constants
        public static final double supplyCurrentLimit = 40.0;
        public static final boolean supplyCurrentEnable = true;
        public static final double supplyCurrentThreshold = 50.0;
        public static final double supplyTimeThreshold = 0.2;

         // Feedback Constants
        public static final double sensorToMechanismRatio = 100; // rotor rotations to wrist rotations

        // Position PID Constants
        public static final int positionPIDSlot = 0;
        public static final double kV = 12.0 / (6000.0 / sensorToMechanismRatio / 60.0); // 12.0 V / max speed rps
        public static final double kS = 0.18;
        public static final double kP = 70.0;
        public static final double kI = 0.0;
        public static final double kD = 2.5;
        public static final double kPIDUpdateHz = 1000;

        public static final double kTolerance = 2.0 / 360.0; // rotations

        // Motion Magic Constants
        public static final double theoreticalMaxSpeedRPS = 6000.0 / sensorToMechanismRatio / 60.0;
        // public static final double kTheoreticalMaxSpeedRPS = 100.0;
        public static final double motionMagicExpoVelocity = 12.0 / theoreticalMaxSpeedRPS * 1.0;
        public static final double motionMagicExpoAccel = 1.5;
        // public static final double motionMagicJerk = theoreticalMaxSpeedRPS * 30.0;

        // Software Limit Switch Constants
        public static SoftwareLimitSwitchConfigs normalSoftLimits = new SoftwareLimitSwitchConfigs();
        static {
            normalSoftLimits.ForwardSoftLimitEnable = false;
            normalSoftLimits.ForwardSoftLimitThreshold = 1000; //280.0 / 360.0; // rotations
            normalSoftLimits.ReverseSoftLimitEnable = false;
            normalSoftLimits.ReverseSoftLimitThreshold = 1000; // 0.0;
        }

        // Hardware Limit Switch Constants
        public static final ReverseLimitSourceValue reverseLimitSource = ReverseLimitSourceValue.Disabled;
        public static final ReverseLimitTypeValue reverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        public static final boolean reverseLimitEnabled = true; // check
        public static final boolean reverseLimitAutosetPositionEnabled = false; // check
        public static final double reverseLimitAutosetPositionValue = 0.0;
        public static final ForwardLimitSourceValue forwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        public static final ForwardLimitTypeValue forwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        public static final boolean forwardLimitEnabled = false; // check
        public static final boolean forwardLimitAutosetPositionEnabled = false; // check
        public static final double forwardLimitAutosetPositionValue = 0.0;
    }

    public static final class Arm{

         // Feedback Constants
        public static final double sensorToMechanismRatio = 100; // rotor rotations to wrist rotations

        // Position PID Constants
        public static final int positionPIDSlot = 0;
        public static final double kV = 11; // 12.0 V / max speed rps
        public static final double kS = 0.3;
        public static final double kP = 20.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        

        // Motion Magic Constants
        public static final double theoreticalMaxSpeedRPS = 6000.0 / sensorToMechanismRatio / 60.0;
        
        // public static final double kTheoreticalMaxSpeedRPS = 100.0;
        public static final double motionMagicVelocity = 0.9; //1.8
        public static final double motionMagicAccel = 1.8;
        // public static final double motionMagicJerk = theoreticalMaxSpeedRPS * 30.0;

        // public static final class UpMotionMagicConfigs {
        //     // Motion Magic Constants
        //     public static final double theoreticalMaxSpeedRPS = 6000.0 / sensorToMechanismRatio / 60.0;
        //     // public static final double kTheoreticalMaxSpeedRPS = 100.0;
        //     public static final double motionMagicExpoVelocity = 12.0 / theoreticalMaxSpeedRPS * 1.0;
        //     public static final double motionMagicExpoAccel = 1.5;
        //     // public static final double motionMagicJerk = theoreticalMaxSpeedRPS * 30.0;

        // }

        // // leo added if necessary cuz R0 has the same thang
        // public static final class DownMotionMagicConfigs {
        //     // Motion Magic Constants
        //     public static final double theoreticalMaxSpeedRPS = 6000.0 / sensorToMechanismRatio / 60.0;
        //     // public static final double kTheoreticalMaxSpeedRPS = 100.0;
        //     public static final double motionMagicExpoVelocity = 12.0 / theoreticalMaxSpeedRPS * 1.0;
        //     public static final double motionMagicExpoAccel = 1.5;
        //     // public static final double motionMagicJerk = theoreticalMaxSpeedRPS * 30.0;

        // }
    }
}

