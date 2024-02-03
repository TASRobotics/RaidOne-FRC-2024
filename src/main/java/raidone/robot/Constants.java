package raidone.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    public static final class SwerveConstants {

        // SwerveModule IDs & offset angle
        public static final int FRONT_LEFT_THROTTLE_ID = 1;
        public static final int FRONT_LEFT_ROTOR_ID = 2;
        public static final int FRONT_LEFT_CANCODER_ID = 1;
        public static final double FRONT_LEFT_ANGLE_OFFSET = 0.0;

        public static final int REAR_LEFT_THROTTLE_ID = 3;
        public static final int REAR_LEFT_ROTOR_ID = 4;
        public static final int REAR_LEFT_CANCODER_ID = 2;
        public static final double REAR_LEFT_ANGLE_OFFSET = 0.0;

        public static final int FRONT_RIGHT_THROTTLE_ID = 5;
        public static final int FRONT_RIGHT_ROTOR_ID = 6;
        public static final int FRONT_RIGHT_CANCODER_ID = 3;
        public static final double FRONT_RIGHT_ANGLE_OFFSET = 0.0;

        public static final int REAR_RIGHT_THROTTLE_ID = 7;
        public static final int REAR_RIGHT_ROTOR_ID = 8;
        public static final int REAR_RIGHT_CANCODER_ID = 4;
        public static final double REAR_RIGHT_ANGLE_OFFSET = 0.0;

        // Swerve kinematics (Order: leftFront, leftRear, rightFront, rightRear)
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 0.0)
        );

        // IMU ID
        public static final int IMU_ID = 0;

        // Rotor encoder & motor inversion
        public static final boolean ROTOR_INVERSION = false;
        public static final SensorDirectionValue ROTOR_ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;

        // Rotor PID Constants
        public static final double ROTOR_KP = 0.0;
        public static final double ROTOR_KI = 0.0;
        public static final double ROTOR_KD = 0.0;

        // Throttle PID constants
        public static final double THROTTLE_KP = 0.0;
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.0;

        // Throttle feedforward constants
        public static final double THROTTLE_KS = 0.0;
        public static final double THROTTLE_KA = 0.0;
        public static final double THROTTLE_KV = 0.0;

        // Velocity & acceleration of swerve
        public static final double MAX_VEL_MPS = 3.0;
        public static final double MAX_ACCEL_MPS2 = 3.0;

        // Wheel diameter
        public static final double WHEEL_DIAMETER_M = 0.0;

        // Throttle gear ratio
        public static final double THROTTLE_GEAR_RATIO = 0.0;

        // Throttle conversion factors
        public static final double THROTTLE_VEL_COMPENSATION_FACTOR =
            (1 / THROTTLE_GEAR_RATIO / 60) * WHEEL_DIAMETER_M * Math.PI;

        public static final double THROTTLE_POS_COMPENSATION_FACTOR =
            (1 / THROTTLE_GEAR_RATIO) * WHEEL_DIAMETER_M * Math.PI;

        // Pathing PID constants 
        public static final double PATHING_KP = 0.0;
        public static final double PATHING_KI = 0.0;
        public static final double PATHING_KD = 0.0;

    }

    public static final class TeleopConstants {

        public static final double DRIVE_DEADBAND = 0.05;

    }

    public static double VOLTAGE_COMPENSATION = 12.0;

}
