package raidone.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double STICK_DEADBAND = 0.03;

    public static final class Swerve {
        public static final int PIGEON_ID = 1;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.0);
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double THROTTLE_GEAR_RATIO = (6.75 / 1.0);
        public static final double ROTOR_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

        /* Motor Inverts */
        public static final InvertedValue THROTTLE_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue ROTOR_INVERT = InvertedValue.Clockwise_Positive;

        /* CanCoder Encoder Invert */
        public static final SensorDirectionValue CAN_CODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int ROTOR_CURRENT_LIMIT = 25;
        public static final int ROTOR_CURRENT_THRESHOLD = 40;
        public static final double ROTOR_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int THROTTLE_CURRENT_LIMIT = 35;
        public static final int THROTTLE_CURRENT_THRESHOLD = 60;
        public static final double THROTTLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean THROTTLE_ENABLE_CURRENT_LIMIT = true;

        public static final double VOLTAGE_COMPENSATION = 12.0;

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

        public static final double THROTTLE_VEL_CONVERSION_FACTOR = (1 / THROTTLE_GEAR_RATIO / 60)
                * WHEEL_DIAMETER_METERS * Math.PI;

        public static final double THROTTLE_POS_CONVERSTION_FACTOR = (1 / THROTTLE_GEAR_RATIO) * WHEEL_DIAMETER_METERS
                * Math.PI;

        /* Rotor PID Values */
        public static final double ROTOR_KP = 0.008;
        public static final double ROTOR_KI = 0.0;
        public static final double ROTOR_KD = 0.0001;

        /* Throttle PID Values */
        public static final double THROTTLE_KP = 2.0;
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.0;
        public static final double THROTTLE_KF = 0.0;

        /* Throttle Characterization Values */
        public static final double THROTTLE_KS = 0.32;
        public static final double THROTTLE_KV = 1.51;
        public static final double THROTTLE_KA = 0.27;

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5;
        public static final double MAX_ANGULAR_VELOCITY_RPS = 10.0;

        /* IDs */
        public static final int THROTTLE_I_ID = 1;
        public static final int ROTOR_I_ID = 2;
        public static final int CAN_CODER_I_ID = 1;
        public static final double MODULE_I_OFFSET = -0.772949;

        public static final int THROTTLE_II_ID = 3;
        public static final int ROTOR_II_ID = 4;
        public static final int CAN_CODER_II_ID = 2;
        public static final double MODULE_II_OFFSET = -0.897949;

        public static final int THROTTLE_III_ID = 5;
        public static final int ROTOR_III_ID = 6;
        public static final int CAN_CODER_III_ID = 3;
        public static final double MODULE_III_OFFSET = -0.532959;

        public static final int THROTTLE_IV_ID = 7;
        public static final int ROTOR_IV_ID = 8;
        public static final int CAN_CODER_IV_ID = 4;
        public static final double MODULE_IV_OFFSET = -0.104004;
    }

    public static final class Arm {
        public static final int ARM_MOTOR_ID = 9;
        public static final int ARM_FOLLOW_ID = 10;

        public static final State SCORINGPOS = new State(-24.5, 0);
        public static final State INTAKEPOS = new State(0.0, 0);

        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.007;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;

        public static final double MAX_OUTPUT = 1.0;
        public static final double MIN_OUTPUT = -0.6;
        public static final double MAX_VEL_RPS = 24.5 / 2;
        public static final double MAX_ACCEL_RPSS = MAX_VEL_RPS;
        public static final double ALLOWED_ERROR = 0.0;

        public static final Constraints ARM_CONSTRAINTS = new Constraints(Constants.Arm.MAX_VEL_RPS,
            Constants.Arm.MAX_ACCEL_RPSS);
        public static TrapezoidProfile ARM_Profile = new TrapezoidProfile(ARM_CONSTRAINTS);
    }

    public static final class Wrist {
        public static final int WRIST_MOTOR_ID = 11;
        public static final int WRIST_FOLLOW_ID = 12;

        
        public static final State SCORINGPOS = new State(-15, 0);
        public static final State INTAKEPOS = new State(-33.0, 0);
        public static final State HOMEPOS = new State(0.0, 0);
        
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.002;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;
        
        public static final double MAX_OUTPUT = 1.0;
        public static final double MIN_OUTPUT = -1.0;
        public static final double MAX_VEL_RPS = 33.0 / 2;
        public static final double MAX_ACCEL_RPSS = MAX_VEL_RPS;
        public static final double ALLOWED_ERROR = 2.0;
        
        public static final Constraints WRIST_CONSTRAINTS = new Constraints(Constants.Wrist.MAX_VEL_RPS,
            Constants.Wrist.MAX_ACCEL_RPSS);
        public static TrapezoidProfile WRIST_Profile = new TrapezoidProfile(WRIST_CONSTRAINTS);
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR_ID = 13;
        public static final double PERCENT = 0.9;
    }
}