package raidone.robot.subsystems;

import raidone.robot.SwerveModule;
import raidone.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static raidone.robot.Constants.Swerve.*;

import javax.lang.model.element.ModuleElement;

import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public Pigeon2 imu;

    // private Field2d field = new Field2d();

    public Swerve() {
        imu = new Pigeon2(Constants.Swerve.pigeonID, "seCANdary");
        imu.getConfigurator().apply(new Pigeon2Configuration());
        imu.setYaw(0);

        swerveModules = new SwerveModule[] {
                new SwerveModule(THROTTLE_I_ID, ROTOR_I_ID, CAN_CODER_I_ID, MODULE_I_OFFSET),
                new SwerveModule(THROTTLE_II_ID, ROTOR_II_ID, CAN_CODER_II_ID, MODULE_II_OFFSET),
                new SwerveModule(THROTTLE_III_ID, ROTOR_III_ID, CAN_CODER_III_ID, MODULE_III_OFFSET),
                new SwerveModule(THROTTLE_IV_ID, ROTOR_IV_ID, CAN_CODER_IV_ID, MODULE_IV_OFFSET)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_DRIVE_KINEMATICS, getRotation(),
                getModulePositions());

        // SmartDashboard.putData("field", field);

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.37, 0.0, 0.0), // Rotation PID constants
                        2.5, // Max module speed, in m/s
                        TRACK_WIDTH / 2, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        // for (SwerveModule mod : swerveModules) {
        // mod.setDesiredState(swerveModuleStates[mod.getModuleConstants().MODULE_NUMBER
        // - 1], isOpenLoop);
        // }

        swerveModules[3].setDesiredState(swerveModuleStates[3], isOpenLoop);
        swerveModules[0].setDesiredState(swerveModuleStates[0], isOpenLoop);
        swerveModules[1].setDesiredState(swerveModuleStates[1], isOpenLoop);
        swerveModules[2].setDesiredState(swerveModuleStates[2], isOpenLoop);

    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        // for(SwerveModule mod : swerveModules){
        // mod.setDesiredState(desiredStates[mod.getModuleConstants().MODULE_NUMBER -1],
        // false);

        swerveModules[3].setDesiredState(desiredStates[3], false);
        swerveModules[0].setDesiredState(desiredStates[0], false);
        swerveModules[1].setDesiredState(desiredStates[1], false);
        swerveModules[2].setDesiredState(desiredStates[2], false);

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        // for (SwerveModule mod : swerveModules) {
        // states[mod.getModuleConstants().MODULE_NUMBER - 1] = mod.getState();
        // }

        states[3] = swerveModules[3].getState();
        states[0] = swerveModules[0].getState();
        states[1] = swerveModules[1].getState();
        states[2] = swerveModules[2].getState();

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        // for (SwerveModule mod : swerveModules) {
        // positions[mod.getModuleConstants().MODULE_NUMBER - 1] = mod.getPosition();
        // }

        positions[3] = swerveModules[3].getPosition();
        positions[0] = swerveModules[0].getPosition();
        positions[1] = swerveModules[1].getPosition();
        positions[2] = swerveModules[2].getPosition();

        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    // public Rotation2d getRotation() {
    // return Rotation2d.fromDegrees(imu.getYaw().getValue());
    // }

    public Rotation2d getRotation() {
        return imu.getRotation2d();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRelativeSpeeds() {
        return SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public void driveRelative(ChassisSpeeds speed) {
        setModuleStates(SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speed));
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getRotation(), getModulePositions());

        SmartDashboard.putNumber("Velocity (m/s)", getModuleStates()[0].speedMetersPerSecond);

        SmartDashboard.putNumber("Rotation", imu.getAngle());
        SmartDashboard.putNumber("Y", getPose().getY());
        SmartDashboard.putNumber("X", getPose().getX());

        // field.setRobotPose(this.getPose());

        // for (SwerveModule mod : swerveModules) {
        // SmartDashboard.putNumber("Mod " + mod.getModuleConstants().MODULE_NUMBER + "
        // CANcoder",
        // mod.getCANcoder().getDegrees());
        // SmartDashboard.putNumber("Mod " + mod.getModuleConstants().MODULE_NUMBER + "
        // Angle",
        // mod.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Mod " + mod.getModuleConstants().MODULE_NUMBER + "
        // Velocity",
        // mod.getState().speedMetersPerSecond);
        // }
    }
}