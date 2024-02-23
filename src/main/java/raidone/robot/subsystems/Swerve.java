package raidone.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static raidone.robot.Constants.Swerve.*;

import raidone.robot.Constants;
import raidone.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    
    private Pigeon2 imu;
    public SwerveModule moduleFL, moduleBL, moduleBR, moduleFR;

    public SwerveDriveOdometry swerveOdometry;

    private Field2d field = new Field2d();

    public Swerve() {

        moduleFL = new SwerveModule(
            THROTTLE_LF_ID,
            ROTOR_LF_ID,
            CAN_CODER_LF_ID,
            MODULE_LF_OFFSET,
            false
        );

        moduleBL = new SwerveModule(
            THROTTLE_LB_ID,
            ROTOR_LB_ID,
            CAN_CODER_LB_ID,
            MODULE_LB_OFFSET,
            false
        );

        moduleBR = new SwerveModule(
            THROTTLE_RB_ID,
            ROTOR_RB_ID,
            CAN_CODER_RB_ID,
            MODULE_RB_OFFSET,
            true
        );

        moduleFR = new SwerveModule(
            THROTTLE_RF_ID,
            ROTOR_RF_ID,
            CAN_CODER_RF_ID,
            MODULE_RF_OFFSET,
            true
        );


        imu = new Pigeon2(PIGEON_ID);
        imu.getConfigurator().apply(new Pigeon2Configuration());
        imu.setYaw(0);

        swerveOdometry = new SwerveDriveOdometry(
            SWERVE_DRIVE_KINEMATICS,
            getRotation(),
            getModulePositions()
        );

        configureAutoBuilder();
    }

    /**
     * Drives the swerve
     * 
     * @param translation XY velocities
     * @param rotation Rotation velocity
     * @param fieldRelative Field relative driving
     * @param isOpenLoop Open loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(),
                                        translation.getY(),
                                        rotation,
                             getHeading())
            : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED_MPS);

        moduleFL.setDesiredState(swerveModuleStates[0], isOpenLoop);
        moduleFR.setDesiredState(swerveModuleStates[1], isOpenLoop);
        moduleBL.setDesiredState(swerveModuleStates[2], isOpenLoop);
        moduleBR.setDesiredState(swerveModuleStates[3], isOpenLoop);

    }

    /**
     * Sets swervemodule states
     * 
     * @param desiredStates Desired swerve module states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED_MPS);

        SmartDashboard.putNumber("input vel mps", desiredStates[0].speedMetersPerSecond);

        moduleFL.setDesiredState(desiredStates[0], false);
        moduleFR.setDesiredState(desiredStates[1], false);
        moduleBL.setDesiredState(desiredStates[2], false);
        moduleBR.setDesiredState(desiredStates[3], false);

    }

    public Command setX() {
        return runOnce( () -> {
            moduleFL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
		    moduleBL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
		    moduleBR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
		    moduleFR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
        });
	}

    /**
     * Gets swervemodule states
     * 
     * @return Module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = moduleFL.getState();
        states[1] = moduleFR.getState();
        states[2] = moduleBL.getState();
        states[3] = moduleBR.getState();

        return states;
    }

    /**
     * Gets swervemodule positions
     * 
     * @return Module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        positions[0] = moduleFL.getPosition();
        positions[1] = moduleFR.getPosition();
        positions[2] = moduleBL.getPosition();
        positions[3] = moduleBR.getPosition();

        return positions;
    }

    /**
     * Gets robot pose
     * 
     * @return Robot pose
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Sets robot pose
     * 
     * @param pose Desired pose
     */
    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    /**
     * Gets robot heading as a {@link Rotation2d} object
     * 
     * @return Robot heading as {@link Rotation2d} object
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Sets robot heading
     * 
     * @param heading {@link Rotation2d} heading
     */
    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    /**
     * Zeros robot heading
     */
    public void zeroHeading() {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * Gets IMU rotation as a {@link Rotation2d} object
     * 
     * @return IMU rotation as {@link Rotation2d} object
     */
    public Rotation2d getRotation() {
        return imu.getRotation2d();
    }

    /**
     * Reset swervemodules to absolute position
     */
    public void resetModulesToAbsolute() {
        moduleFL.resetToAbsolute();
        moduleBL.resetToAbsolute();
        moduleBR.resetToAbsolute();
        moduleFR.resetToAbsolute();
    }

    /**
     * Gets robot relative speeds
     * 
     * @return Robot speeds as {@link ChassisSpeeds} object
     */
    public ChassisSpeeds getRelativeSpeeds() {
        return SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public Pigeon2 getImu() {
        return imu;
    }

    /**
     * <ul>
     * <li>Robot relative driving</li>
     * <li>Used for PathPlanner holonomic driving</li>
     * </ul>
     * 
     * @param speed Speed as {@link ChassisSpeeds} object
     */
    public void driveRelative(ChassisSpeeds speed) {
        setModuleStates(SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speed));
    }

    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getRelativeSpeeds,
            this::driveRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(
                    Constants.Swerve.TRANSLATION_KP,
                    Constants.Swerve.TRANSLATION_KI,
                    Constants.Swerve.TRANSLATION_KD
                ),
                new PIDConstants(
                    Constants.Swerve.ROTATION_KP,
                    Constants.Swerve.ROTATION_KI,
                    Constants.Swerve.ROTATION_KD
                ),
                Constants.Swerve.MAX_SPEED_MPS,
                Constants.Swerve.TRACK_WIDTH / 2.0,
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

    @Override
    public void periodic() {
        swerveOdometry.update(getRotation(), getModulePositions());

        SmartDashboard.putNumber("Velocity (m/s)", getModuleStates()[0].speedMetersPerSecond);

        SmartDashboard.putNumber("Rotation", imu.getAngle());
        SmartDashboard.putNumber("Y", getPose().getY());
        SmartDashboard.putNumber("X", getPose().getX());

        SmartDashboard.putNumber("actual vel mps", getModuleStates()[0].speedMetersPerSecond);

        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field); 
    }
}