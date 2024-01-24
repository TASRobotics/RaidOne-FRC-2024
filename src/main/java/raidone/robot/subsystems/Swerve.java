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
    public Pigeon2 imu;

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backRightModule;
    private SwerveModule backLeftModule;

    // private Field2d field = new Field2d();

    public Swerve() {
        imu = new Pigeon2(Constants.Swerve.pigeonID, "seCANdary");
        imu.getConfigurator().apply(new Pigeon2Configuration());
        imu.setYaw(0);

        frontLeftModule = new SwerveModule(THROTTLE_FL_ID, ROTOR_FL_ID, CAN_CODER_FL_ID, MODULE_FL_OFFSET);
        backLeftModule = new SwerveModule(THROTTLE_BL_ID, ROTOR_BL_ID, CAN_CODER_BL_ID, MODULE_BL_OFFSET);
        backRightModule = new SwerveModule(THROTTLE_BR_ID, ROTOR_BR_ID, CAN_CODER_BR_ID, MODULE_BR_OFFSET);
        frontRightModule = new SwerveModule(THROTTLE_FR_ID, ROTOR_FR_ID, CAN_CODER_FR_ID, MODULE_FR_OFFSET);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_DRIVE_KINEMATICS, getRotation(),
                getModulePositions());

        // SmartDashboard.putData("field", field);

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getRelativeSpeeds,
            this::driveRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(0.005, 0.0, 0.0),
                new PIDConstants(0.37, 0.0, 0.0),
                2.5,
                TRACK_WIDTH / 2,
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
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

        backLeftModule.setDesiredState(swerveModuleStates[0], isOpenLoop);
        backRightModule.setDesiredState(swerveModuleStates[1], isOpenLoop);
        frontRightModule.setDesiredState(swerveModuleStates[2], isOpenLoop);
        frontLeftModule.setDesiredState(swerveModuleStates[3], isOpenLoop);

    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        backLeftModule.setDesiredState(desiredStates[0], false);
        backRightModule.setDesiredState(desiredStates[1], false);
        frontRightModule.setDesiredState(desiredStates[2], false);
        frontLeftModule.setDesiredState(desiredStates[3], false);

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = backLeftModule.getState();
        states[1] = backRightModule.getState();
        states[2] = frontRightModule.getState();
        states[3] = frontLeftModule.getState();

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        positions[0] = backLeftModule.getPosition();
        positions[1] = backRightModule.getPosition();
        positions[2] = frontRightModule.getPosition();
        positions[3] = frontLeftModule.getPosition();

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

    public Rotation2d getRotation() {
        return imu.getRotation2d();
    }

    public void resetModulesToAbsolute() {
        backLeftModule.resetToAbsolute();
        backRightModule.resetToAbsolute();
        frontRightModule.resetToAbsolute();
        frontLeftModule.resetToAbsolute();
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