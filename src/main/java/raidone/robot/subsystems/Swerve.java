package raidone.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import raidone.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {

    private final Pigeon2 imu = new Pigeon2(SwerveConstants.kImuID, "seCANdary");

    private final SwerveModule moduleI, moduleIV, moduleII, moduleIII;
    private final SwerveDriveOdometry odometry;
    
    public Swerve() {

        moduleI = new SwerveModule(
            SwerveConstants.kIThrottleID, 
            SwerveConstants.kIRotorID, 
            SwerveConstants.kICANCoderID, 
            SwerveConstants.kIRotorOffsetAngle
        );

        moduleII = new SwerveModule(
            SwerveConstants.kIIThrottleID, 
            SwerveConstants.kIIRotorID, 
            SwerveConstants.kIICANCoderID, 
            SwerveConstants.kIIRotorOffsetAngle
        );

        moduleIII = new SwerveModule(
            SwerveConstants.kIIIThrottleID, 
            SwerveConstants.kIIIRotorID, 
            SwerveConstants.kIIICANCoderID, 
            SwerveConstants.kIIIRotorOffsetAngle
        );

        moduleIV = new SwerveModule(
            SwerveConstants.kIVThrottleID, 
            SwerveConstants.kIVRotorID, 
            SwerveConstants.kIVCANCoderID, 
            SwerveConstants.kIVRotorOffsetAngle
        );

        odometry = new SwerveDriveOdometry(
            SwerveConstants.kSwerveKinematics,
            imu.getRotation2d(),
            getModulePositions()
        );

    }

    @Override
    public void periodic(){
        // Update odometry with current module state
        // odometry.update(
        //     imu.getRotation2d(),
        //     getModulePositions()
        // );

        Logger.recordOutput("Swerve states", this.getModuleStates());
    }

    /**
     * Gets current chassis speeds relative to itself
     * 
     * @return Chassis speeds
     */
    public ChassisSpeeds getRelativeSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Drives the chassis at a certain speed relative to itself
     * 
     * @param speed Desired chassis speed
     */
    public void driveRelative(ChassisSpeeds speed) {
        setModuleStates(SwerveConstants.kSwerveKinematics.toSwerveModuleStates(speed));
    }

    /**
     * Drives the swerve (Input range: [-1, 1] )
     * 
     * @param xSpeed Percent power for X-axis
     * @param ySpeed Percent power for Y-axis
     * @param zSpeed Percent percent power for rotation
     * @param fieldOriented Drive orientation (true = field oriented, false = robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
        SwerveModuleState[] states = null;
        if (fieldOriented) {
            states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, imu.getRotation2d())
            );
        } else {
            states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
            );
        }
        setModuleStates(states);
    }

    /**
     * Get swerve module states
     * @return Swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            moduleI.getState(),
            moduleIV.getState(),
            moduleII.getState(),
            moduleIII.getState()
        };
    }

    /**
     * Get swerve module positions
     * @return Swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            moduleI.getPosition(),
            moduleIV.getPosition(),
            moduleII.getPosition(),
            moduleIII.getPosition()
        };
    }

    /**
     * Set swerve module states
     * @param desiredStates Array of desired states (Order: leftFront, leftRear, rightFront, rightRear)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        moduleI.setState(desiredStates[0]);
        moduleIV.setState(desiredStates[1]);
        moduleII.setState(desiredStates[2]);
        moduleIII.setState(desiredStates[3]);
    }

    /**
	 * Sets the wheels into an X formation to prevent movement.
     * 
     * @return 
	 */
	public Command setX() {
        return runOnce( () -> {
            moduleI.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		    moduleIV.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		    moduleII.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		    moduleIII.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
	}

    /**
     * Get predicted pose
     * @return pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Set robot pose
     * @param pose robot pose
     */
    public void setPose(Pose2d pose) {
        odometry.resetPosition(imu.getRotation2d(), getModulePositions(), pose);
    }

}
