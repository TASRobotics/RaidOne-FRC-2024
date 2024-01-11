package raidone.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    public static final class SwerveConstants {

        // SwerveModule IDs & offset angle
        public static final int kIIRotorID = 1;
        public static final int kIIThrottleID = 2;
        public static final int kIICANCoderID = 3;
        public static final double kIIRotorOffsetAngle = 0.0;

        public static final int kIIIRotorID = 4;
        public static final int kIIIThrottleID = 5;
        public static final int kIIICANCoderID = 6;
        public static final double kIIIRotorOffsetAngle = 0.0;

        public static final int kIRotorID = 7;
        public static final int kIThrottleID = 8;
        public static final int kICANCoderID = 9;
        public static final double kIRotorOffsetAngle = 0.0;

        public static final int kIVRotorID = 10;
        public static final int kIVThrottleID = 11;
        public static final int kIVCANCoderID = 12;
        public static final double kIVRotorOffsetAngle = 0.0;

        // Swerve kinematics (Order: leftFront, leftRear, rightFront, rightRear)
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 0.0)
        );

        // IMU ID
        public static final int kImuID = 13;

        // Rotor encoder & motor inversion
        public static final boolean kRotorMotorInversion = false;
        public static final SensorDirectionValue kRotorEncoderDirection = SensorDirectionValue.Clockwise_Positive;

        // Rotor PID Constants
        public static final double kRotor_kP = 0.0;
        public static final double kRotor_kI = 0.0;
        public static final double kRotor_kD = 0.0;

        // Velocity & acceleration of swerve
        public static final double kMaxVelocityMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecond = 3.0;

        // Wheel diameter
        public static final double kWheelDiameterMeters = 0.0;

        // Throttle gear ratio
        public static final double kThrottleGearRatio = 0.0;

        // Throttle conversion factors
        public static final double kThrottleVelocityConversionFactor =
            (1 / kThrottleGearRatio / 60) * kWheelDiameterMeters * Math.PI;

        public static final double kThrottlePositionConversionFactor =
            (1 / kThrottleGearRatio) * kWheelDiameterMeters * Math.PI;

        // Pathing PID constants 
        public static final double kPathing_kP = 0.0;
        public static final double kPathing_kI = 0.0;
        public static final double kPathing_kD = 0.0;

    }

    public static final class TeleopConstants {

        public static final double kDriveDeadband = 0.05;

    }

    public static double kVoltageCompensation = 12.0;

}
