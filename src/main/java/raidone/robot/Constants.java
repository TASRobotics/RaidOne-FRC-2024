package raidone.robot;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static final String CANIVORE_NAME = "seCANdary";

    public static final class Swerve {

        public static final int PIGEON_ID = 0;

        // Swerve dimensions & conversions
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.0);
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
        public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0);

        public static final double THROTTLE_GEAR_RATIO = (6.2 / 1.0);
        public static final double ROTOR_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

        public static final double THROTTLE_VEL_CONVERSION_FACTOR = (1 / THROTTLE_GEAR_RATIO / 60) * WHEEL_DIAMETER_M
                * Math.PI;

        public static final double THROTTLE_POS_CONVERSTION_FACTOR = (1 / THROTTLE_GEAR_RATIO) * WHEEL_DIAMETER_M
                * Math.PI;

        // Swerve Kinematics
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        // Swerve Current Limiting
        public static final int ROTOR_CURRENT_LIMIT = 25;
        public static final int ROTOR_CURRENT_THRESHOLD = 40;
        public static final double ROTOR_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int THROTTLE_CURRENT_LIMIT = 40;
        public static final int THROTTLE_CURRENT_THRESHOLD = 60;
        public static final double THROTTLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean THROTTLE_ENABLE_CURRENT_LIMIT = true;

        public static final double VOLTAGE_COMPENSATION = 12.0;

        public static final double OPEN_LOOP_RAMP = 0.25;

        /* Angle Motor PID Values */
        public static final double ROTOR_KP = 0.011;
        public static final double ROTOR_KI = 0.0;
        public static final double ROTOR_KD = 0.0;

        // Throttle PID constants
        public static final double THROTTLE_KP = 0.2;
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.0;// 0.035;
        public static final double THROTTLE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double THROTTLE_KS = 0.00;
        public static final double THROTTLE_KV = 2.7;
        public static final double THROTTLE_KA = 0.0;

        // Translation pathing PID constants
        public static final double TRANSLATION_KP = 0.5; // 0.12
        public static final double TRANSLATION_KI = 0.025;
        public static final double TRANSLATION_KD = 0.0;

        // Rotation pathing PID constants
        public static final double ROTATION_KP = 2.2; // 0.12
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.0;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 5.0 * 0.8;

        /* Neutral Modes */
        public static final NeutralModeValue ROTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue THROTTLE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final int THROTTLE_FL_ID = 1;
        public static final int ROTOR_FL_ID = 2;
        public static final int CAN_CODER_FL_ID = 1;
        public static final double MODULE_FL_OFFSET = 0.281738;

        public static final int THROTTLE_BL_ID = 3;
        public static final int ROTOR_BL_ID = 4;
        public static final int CAN_CODER_BL_ID = 2;
        public static final double MODULE_BL_OFFSET = 0.101562;

        public static final int THROTTLE_BR_ID = 5;
        public static final int ROTOR_BR_ID = 6;
        public static final int CAN_CODER_BR_ID = 3;
        public static final double MODULE_BR_OFFSET = 0.474854;

        public static final int THROTTLE_FR_ID = 7;
        public static final int ROTOR_FR_ID = 8;
        public static final int CAN_CODER_FR_ID = 4;
        public static final double MODULE_FR_OFFSET = -0.111328;

        public static final double STICK_DEADBAND = 0.1;
    }

    public static final class Arm {
        public static final int ARM_MOTOR_ID = 9;
        public static final int ARM_FOLLOW_ID = 10;

        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue inversion = InvertedValue.CounterClockwise_Positive;
        //public static final InvertedValue inversion = InvertedValue.Clockwise_Positive;
        public static final String armCANbus = "rio";

        public static final State SCORINGPOS = new State(-33, 0);
        public static final double SOFTLIMIT = SCORINGPOS.position - 2;
        public static final State INTAKEPOS = new State(0.0, 0);

        public static final State CONSTRAINTPOS = new State(-13, 0);

        public static final double homeSpeed = -0.3;

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
        public static final double kP = 80.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        public static final double kTolerance = 2.0 / 360.0; // rotations

        // Motion Magic Constants
        public static final double theoreticalMaxSpeedRPS = 6000.0 / sensorToMechanismRatio / 60.0 * 10;
        // public static final double kTheoreticalMaxSpeedRPS = 100.0;
        public static final double motionMagicVelocity = theoreticalMaxSpeedRPS * 0.30;
        public static final double motionMagicAccel = theoreticalMaxSpeedRPS * 3.0;
        public static final double motionMagicJerk = theoreticalMaxSpeedRPS * 30.0;

        // Software Limit Switch Constants
        public static SoftwareLimitSwitchConfigs normalSoftLimits = new SoftwareLimitSwitchConfigs();
        static {
            normalSoftLimits.ForwardSoftLimitEnable = true;
            normalSoftLimits.ForwardSoftLimitThreshold = 36; //108.0/360.0; // rotations
            normalSoftLimits.ReverseSoftLimitEnable = false;
            //normalSoftLimits.ReverseSoftLimitThreshold = -1000;
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

    public static final class Wrist {
        public static final int WRIST_MOTOR_ID = 11;
        public static final int WRIST_FOLLOW_ID = 12;

        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue inversion = InvertedValue.Clockwise_Positive;
        public static final String wristCANbus = "rio";

        public static final State SCORINGPOS = new State(-23.0, 0);
        public static final State INTAKEPOS = new State(-47.0, 0);
        public static final State HOMEPOS = new State(0.0, 0);

        public static final double homeSpeed = -0.4;

        
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
        public static final double kP = 80.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        public static final double kTolerance = 2.0 / 360.0; // rotations

        // Motion Magic Constants
        public static final double theoreticalMaxSpeedRPS = 6000.0 / sensorToMechanismRatio / 60.0 * 10;
        // public static final double kTheoreticalMaxSpeedRPS = 100.0;
        public static final double motionMagicVelocity = theoreticalMaxSpeedRPS * 0.30;
        public static final double motionMagicAccel = theoreticalMaxSpeedRPS * 3.0;
        public static final double motionMagicJerk = theoreticalMaxSpeedRPS * 30.0;

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

    

    public static final class Intake {
        public static final int INTAKE_MOTOR_ID = 13;
        public static final double IntakePercent = 1.0;
        public static final int CURRENT_LIMIT = 20;
        public static final int distanceThreshold = 100;
    }

    public static final class Climb {
        public static final int CLIMB_MOTOR_ID = 15;
        public static final int CLIMB_FOLLOW_ID = 14;

        public static final double UP_SPEED_PCT = 1.0;
        public static final double DOWN_SPEED_PCT = 0.7;

        public static final double BOTTOM_POS_ROT = 0.0;
        public static final double HALFWAY_POS_ROT = 45.0;
        public static final double TOP_POS_ROT = 90.0;
    }
}