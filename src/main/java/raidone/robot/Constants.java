package raidone.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import raidone.lib.util.COTSTalonFXSwerveConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  
            COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.0);
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.WHEEL_CIRCUMFERENCE;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double THROTTLE_GEAR_RATIO = chosenModule.THROTTLE_GEAR_RATIO;
        public static final double ROTOR_GEAR_RATIO = chosenModule.ROTOR_GEAR_RATIO;

        /* Motor Inverts */
        public static final InvertedValue ROTOR_INVERT = chosenModule.ROTOR_INVERT;
        public static final InvertedValue THROTTLE_INVERT = chosenModule.THROTTLE_INVERT;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CAN_CODER_INVERT = chosenModule.CAN_CODER_INVERT;

        /* Swerve Current Limiting */
        public static final int ROTOR_CURRENT_LIMIT = 25;
        public static final int ROTOR_CURRENT_THRESHOLD = 40;
        public static final double ROTOR_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int THROTTLE_CURRENT_LIMIT = 35;
        public static final int THROTTLE_CURRENT_THRESHOLD = 60;
        public static final double THROTTLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean THROTTLE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ROTOR_KP = chosenModule.ROTOR_KP;
        public static final double ROTOR_KI = chosenModule.ROTOR_KI;
        public static final double ROTOR_KD = chosenModule.ROTOR_KD;

        /* Drive Motor PID Values */
        public static final double THROTTLE_KP = 0.12; //TODO: This must be tuned to specific robot
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.0;
        public static final double THROTTLE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double THROTTLE_KS = 0.32; //TODO: This must be tuned to specific robot
        public static final double THROTTLE_KV = 1.51;
        public static final double THROTTLE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue ROTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue THROTTLE_NEUTRAL_MODE = NeutralModeValue.Brake;

        // angle offset in rotations
        // public static final SwerveModule.SwerveModuleConstants[] MODULE_CONSTANTS = new SwerveModule.SwerveModuleConstants[] {
        //     new SwerveModule.SwerveModuleConstants(1, 1, 2, 1, -0.114258),
        //     new SwerveModule.SwerveModuleConstants(2, 3, 4, 2, -0.27193),
        //     new SwerveModule.SwerveModuleConstants(3, 5, 6, 3, -0.096680),
        //     new SwerveModule.SwerveModuleConstants(4, 7, 8, 4, -0.121582),
        // };

        public static final int THROTTLE_I_ID = 1;
        public static final int ROTOR_I_ID = 2;
        public static final int CAN_CODER_I_ID = 1;
        public static final double MODULE_I_OFFSET = -0.114258;

        public static final int THROTTLE_II_ID = 3;
        public static final int ROTOR_II_ID = 4;
        public static final int CAN_CODER_II_ID = 2;
        public static final double MODULE_II_OFFSET = -0.27193;

        public static final int THROTTLE_III_ID = 5;
        public static final int ROTOR_III_ID = 6;
        public static final int CAN_CODER_III_ID = 3;
        public static final double MODULE_III_OFFSET = -0.096680;

        public static final int THROTTLE_IV_ID = 7;
        public static final int ROTOR_IV_ID = 8;
        public static final int CAN_CODER_IV_ID = 4;
        public static final double MODULE_IV_OFFSET = -0.121582;
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }
}