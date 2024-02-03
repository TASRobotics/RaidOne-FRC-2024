package raidone.robot.subsystems;

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
import edu.wpi.first.math.geometry.Translation2d;
import raidone.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {

    private final Pigeon2 imu = new Pigeon2(SwerveConstants.IMU_ID);//, SwerveConstants.CAN_BUS_NAME);

    private final SwerveModule leftFrontModule, rightFrontModule, leftRearModule, rightRearModule;
    private final SwerveDriveOdometry odometry;
    
    public Swerve() {

        leftFrontModule = new SwerveModule(
            SwerveConstants.FRONT_LEFT_THROTTLE_ID, 
            SwerveConstants.FRONT_LEFT_ROTOR_ID, 
            SwerveConstants.FRONT_LEFT_CANCODER_ID, 
            SwerveConstants.FRONT_LEFT_ANGLE_OFFSET
        );

        rightFrontModule = new SwerveModule(
            SwerveConstants.FRONT_RIGHT_THROTTLE_ID, 
            SwerveConstants.FRONT_RIGHT_ROTOR_ID, 
            SwerveConstants.FRONT_RIGHT_CANCODER_ID, 
            SwerveConstants.FRONT_RIGHT_ANGLE_OFFSET
        );

        leftRearModule = new SwerveModule(
            SwerveConstants.REAR_LEFT_THROTTLE_ID, 
            SwerveConstants.REAR_LEFT_ROTOR_ID, 
            SwerveConstants.REAR_LEFT_CANCODER_ID, 
            SwerveConstants.REAR_LEFT_ANGLE_OFFSET
        );

        rightRearModule = new SwerveModule(
            SwerveConstants.REAR_RIGHT_THROTTLE_ID, 
            SwerveConstants.REAR_RIGHT_ROTOR_ID, 
            SwerveConstants.REAR_RIGHT_CANCODER_ID, 
            SwerveConstants.REAR_RIGHT_ANGLE_OFFSET
        );

        odometry = new SwerveDriveOdometry(
            SwerveConstants.SWERVE_DRIVE_KINEMATICS,
            imu.getRotation2d(),
            getModulePositions()
        );

    }

    public double getHeading(){
        return Math.IEEEremainder(imu.getAngle(), 360);
    }

    public void zeroHeading() {
        odometry.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getRotation() {
        return imu.getRotation2d();
    }

    @Override
    public void periodic(){
        // Update odometry with current module state
        odometry.update(
            imu.getRotation2d(),
            getModulePositions()
        );
    }

    /**
     * Gets current chassis speeds relative to itself
     * 
     * @return Chassis speeds
     */
    public ChassisSpeeds getRelativeSpeeds() {
        return SwerveConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Drives the chassis at a certain speed relative to itself
     * 
     * @param speed Desired chassis speed
     */
    public void driveRelative(ChassisSpeeds speed) {
        setModuleStates(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speed));
    }

    /**
     * Drives the swerve (Input range: [-1, 1] )
     * 
     * @param xSpeed Percent power for X-axis
     * @param ySpeed Percent power for Y-axis
     * @param rotSpeed Percent percent power for rotation
     * @param fieldOriented Drive orientation (true = field oriented, false = robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldOriented) {
        SwerveModuleState[] states = null;
        if (fieldOriented) {
            states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, imu.getRotation2d())
            );
        } else {
            states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
            );
        }
        setModuleStates(states);
    }

    
    // public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    //     SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
    //             fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
    //                     translation.getX(),
    //                     translation.getY(),
    //                     rotation,
    //                     getHeading())
    //                     : new ChassisSpeeds(
    //                             translation.getX(),
    //                             translation.getY(),
    //                             rotation));
    //     SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_VEL_MPS);

    //     // for (SwerveModule mod : swerveModules) {
    //     // mod.setDesiredState(swerveModuleStates[mod.getModuleConstants().MODULE_NUMBER
    //     // - 1], isOpenLoop);
    //     // }

    //     leftFrontModule.setState(swerveModuleStates[0]);
    //     leftRearModule.setState(swerveModuleStates[1]);
    //     rightRearModule.setState(swerveModuleStates[2]);
    //     rightFrontModule.setState(swerveModuleStates[3]);

    // }

    /**
     * Get swerve module states
     * @return Swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            leftFrontModule.getState(),
            leftRearModule.getState(),
            rightRearModule.getState(),
            rightFrontModule.getState()
        };
    }

    /**
     * Get swerve module positions
     * @return Swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            leftFrontModule.getPosition(),
            leftRearModule.getPosition(),
            rightRearModule.getPosition(),
            rightFrontModule.getPosition()
        };
    }

    /**
     * Set swerve module states
     * @param desiredStates Array of desired states (Order: leftFront, leftRear, rightFront, rightRear)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        leftFrontModule.setState(desiredStates[0]);
        leftRearModule.setState(desiredStates[1]);
        rightRearModule.setState(desiredStates[2]);
        rightFrontModule.setState(desiredStates[3]);
    }

    /**
	 * Sets the wheels into an X formation to prevent movement.
     * 
     * @return 
	 */
	public Command setX() {
        return runOnce( () -> {
            leftFrontModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		    leftRearModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		    rightRearModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		    rightFrontModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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
