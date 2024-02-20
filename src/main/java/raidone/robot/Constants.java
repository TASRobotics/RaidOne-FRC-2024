package raidone.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double STICK_DEADBAND = 0.03;

    public static final class Swerve {
        public static final int PIGEON_ID = 1;

        // public static final COTSTalonFXSwerveConstants chosenModule =  
        //     COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.0);
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

            // new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            // new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            // new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            // new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0));

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

        public static final double THROTTLE_VEL_CONVERSION_FACTOR = 
        (1/THROTTLE_GEAR_RATIO/60)*WHEEL_DIAMETER_METERS*Math.PI;

        public static final double THROTTLE_POS_CONVERSTION_FACTOR = 
        (1/THROTTLE_GEAR_RATIO)*WHEEL_DIAMETER_METERS*Math.PI;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ROTOR_KP = 0.008;
        public static final double ROTOR_KI = 0.0;
        public static final double ROTOR_KD = 0.0001;

        /* Drive Motor PID Values */
        public static final double THROTTLE_KP = 0.12; 
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.0;
        public static final double THROTTLE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double THROTTLE_KS = 0.32; 
        public static final double THROTTLE_KV = 1.51;
        public static final double THROTTLE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; 
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; 

        /* Neutral Modes */
        public static final NeutralModeValue ROTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue THROTTLE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final int THROTTLE_I_ID = 1;
        public static final int ROTOR_I_ID = 2;
        public static final int CAN_CODER_I_ID = 1;
        public static final double MODULE_I_OFFSET = -0.772949; // -0.772949

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

    public static final class Arm{
        public static final int ARM_MOTOR_ID = 9;
        public static final int ARM_FOLLOW_ID = 10;

        public static final double SCORINGPOS = -20.0;
        public static final double INTAKEPOS = 0.0;
        
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;
        public static final double kMaxOutput = 0.0;
        public static final double kMinOutput = 0.0;
        public static final double maxRPM = 0.0;
        public static final double maxVel = 0.0;
        public static final double minVel = 0.0;
        public static final double maxAcc = 0.0;
        public static final double allowedErr = 0.0;
    }

    public static final class Wrist{
        public static final int WRIST_MOTOR_ID = 11;
        public static final int WRIST_FOLLOW_ID = 12;

        public static final double SCORINGPOS = 0.0;
        public static final double INTAKEPOS = 0.0;
        
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;
        public static final double kMaxOutput = 0.0;
        public static final double kMinOutput = 0.0;
        public static final double maxRPM = 0.0;
        public static final double maxVel = 0.0;
        public static final double minVel = 0.0;
        public static final double maxAcc = 0.0;
        public static final double allowedErr = 0.0;
    }

    public static final class Intake{
        public static final int INTAKE_MOTOR_ID = 13;
        public static final int kForward = 1;
        public static final int kBackward = 0;
    }
}