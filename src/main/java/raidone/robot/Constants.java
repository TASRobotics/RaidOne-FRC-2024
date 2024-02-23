package raidone.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class Swerve {
        
        public static final int PIGEON_ID = 1;
        
        // Swerve dimensions & conversions
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.0);
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
        public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0);

        public static final double THROTTLE_GEAR_RATIO = (6.75 / 1.0);
        public static final double ROTOR_GEAR_RATIO = ((150.0 / 7.0) / 1.0);
        
        public static final double THROTTLE_VEL_CONVERSION_FACTOR = 
        (1 / THROTTLE_GEAR_RATIO / 60) * WHEEL_DIAMETER_M * Math.PI;
        
        public static final double THROTTLE_POS_CONVERSTION_FACTOR = 
        (1 / THROTTLE_GEAR_RATIO) * WHEEL_DIAMETER_M * Math.PI;

        
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
        
        public static final int THROTTLE_CURRENT_LIMIT = 35;
        public static final int THROTTLE_CURRENT_THRESHOLD = 60;
        public static final double THROTTLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean THROTTLE_ENABLE_CURRENT_LIMIT = true;
        
        public static final double VOLTAGE_COMPENSATION = 12.0;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;
        
        // Rotor PID constants 
        public static final double ROTOR_KP = 0.008;
        public static final double ROTOR_KI = 0.0;
        public static final double ROTOR_KD = 0.0001;
        
        // Throttle PID constants 
        public static final double THROTTLE_KP = 0.0; // 0.12 
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.0;
        public static final double THROTTLE_KF = 1.0 / 6000.0;
        
        // Throttle feedforward constants
        public static final double THROTTLE_KS = 0.00; // 0.32
        public static final double THROTTLE_KV = 2.7; // 1.51
        public static final double THROTTLE_KA = 10.0; // 0.27

        // Pathing consants
        public static final double TRANSLATION_KP = 0.000001;
        public static final double TRANSLATION_KI = 0.0;
        public static final double TRANSLATION_KD = 0.0;

        public static final double ROTATION_KP = 0.01;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;
        
        // Swerve Profiling Values 
        //* Meters per Second 
        public static final double MAX_SPEED_MPS = 4.5; 
        //* Radians per Second 
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        // SwerveModule ID & Offsets
        public static final int THROTTLE_LF_ID = 1;
        public static final int ROTOR_LF_ID = 2;
        public static final int CAN_CODER_LF_ID = 1;
        public static final double MODULE_LF_OFFSET = -0.772949;
        
        public static final int THROTTLE_LB_ID = 3;
        public static final int ROTOR_LB_ID = 4;
        public static final int CAN_CODER_LB_ID = 2;
        public static final double MODULE_LB_OFFSET = -0.897949;
        
        public static final int THROTTLE_RB_ID = 5;
        public static final int ROTOR_RB_ID = 6;
        public static final int CAN_CODER_RB_ID = 3;
        public static final double MODULE_RB_OFFSET = -0.532959;
        
        public static final int THROTTLE_RF_ID = 7;
        public static final int ROTOR_RF_ID = 8;
        public static final int CAN_CODER_RF_ID = 4;
        public static final double MODULE_RF_OFFSET = -0.104004;
    }

    public static final double STICK_DEADBAND = 0.03;

    public static final class Arm{
        public static final int ARM_MOTOR_ID = 9;
        public static final int ARM_FOLLOW_ID = 10;

        public static final double SCORINGPOS = -24.5;
        public static final double INTAKEPOS = 0.0;
        
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.007;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;
        public static final double MAX_OUTPUT = 1.0;
        public static final double MIN_OUTPUT = -0.6;
        public static final double MAX_VEL = 0.0;
        public static final double MIN_VEL = 0.0;
        public static final double MAX_ACCEL = 0.0;
        public static final double ALLOWED_ERROR = 0.0;
    }

    public static final class Wrist{
        public static final int WRIST_MOTOR_ID = 11;
        public static final int WRIST_FOLLOW_ID = 12;

        public static final double SCORINGPOS = -12;
        public static final double INTAKEPOS = -33.0;
        
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.002;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;
        public static final double MAX_OUTPUT = 1.0;
        public static final double MIN_OUTPUT = -1.0;
        public static final double MAX_VEL = 2000.0;
        public static final double MIN_VEL = 250.0;
        public static final double MAX_ACCEL = 2000.0;
        public static final double ALLOWED_ERROR = 2.0;
    }

    public static final class Intake{
        public static final int INTAKE_MOTOR_ID = 13;
        public static final int kForward = 1;
        public static final int kBackward = 0;
        public static final double percent = 0.9;
    }
}