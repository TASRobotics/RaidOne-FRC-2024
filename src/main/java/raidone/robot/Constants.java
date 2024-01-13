package raidone.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class SwerveConstants {

        // Swerve module ids and 
        public static final int kIThrottleID = 1;
        public static final int kIRotorID = 2;
        public static final int kICANCoderID = 1;
        public static final double kIRotorOffsetAngle = 0.0; 

        public static final int kIIThrottleID = 3;
        public static final int kIIRotorID = 4;
        public static final int kIICANCoderID = 2;
        public static final double kIIRotorOffsetAngle = 0.0; 

        public static final int kIIIThrottleID = 5;
        public static final int kIIIRotorID = 6;
        public static final int kIIICANCoderID = 3;
        public static final double kIIIRotorOffsetAngle = 0.0; 

        public static final int kIVThrottleID = 7;
        public static final int kIVRotorID = 8;
        public static final int kIVCANCoderID = 4;
        public static final double kIVRotorOffsetAngle = 0.0; 

        public static final double ROBOT_WIDTH_INCHES = 28.0;
        public static final double ROBOT_HALF_WIDTH_METERS = Units.inchesToMeters(ROBOT_WIDTH_INCHES) / 2.0;
        // public static final double ROBOT_RADIUS_INCHES = FastMath.hypot(ROBOT_WIDTH_INCHES, ROBOT_WIDTH_INCHES) / 2.0;
        // public static final double ROBOT_RADIUS_METERS = Units.inchesToMeters(ROBOT_RADIUS_INCHES);
        public static final double WHEEL_DIAMETER_INCHES = 4.0;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);

        public static final Translation2d MODULE_I_POSITION = new Translation2d(ROBOT_HALF_WIDTH_METERS, -ROBOT_HALF_WIDTH_METERS);
        public static final Translation2d MODULE_II_POSITION = new Translation2d(ROBOT_HALF_WIDTH_METERS, ROBOT_HALF_WIDTH_METERS);
        public static final Translation2d MODULE_III_POSITION = new Translation2d(-ROBOT_HALF_WIDTH_METERS, ROBOT_HALF_WIDTH_METERS);
        public static final Translation2d MODULE_IV_POSITION = new Translation2d(-ROBOT_HALF_WIDTH_METERS, -ROBOT_HALF_WIDTH_METERS);

        // Swerve kinematics (Order: leftFront, leftRear, rightFront, rightRear)
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            MODULE_I_POSITION,
            MODULE_II_POSITION,
            MODULE_III_POSITION,
            MODULE_IV_POSITION
        );

        // IMU ID
        public static final int kImuID = 1;

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
