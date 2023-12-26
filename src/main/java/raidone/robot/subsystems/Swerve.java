package raidone.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import raidone.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {

    private final WPI_PigeonIMU imu = new WPI_PigeonIMU(SwerveConstants.kImuID);

    private final SwerveModule leftFrontModule, rightFrontModule, leftRearModule, rightRearModule;
    private final SwerveDriveOdometry odometry;
    
    public Swerve() {

        leftFrontModule = new SwerveModule(
            SwerveConstants.kLeftFrontThrottleID, 
            SwerveConstants.kLeftFrontRotorID, 
            SwerveConstants.kLeftFrontCANCoderID, 
            SwerveConstants.kLeftFrontRotorOffsetAngle
        );

        rightFrontModule = new SwerveModule(
            SwerveConstants.kRightFrontThrottleID, 
            SwerveConstants.kRightFrontRotorID, 
            SwerveConstants.kRightFrontCANCoderID, 
            SwerveConstants.kRightFrontRotorOffsetAngle
        );

        leftRearModule = new SwerveModule(
            SwerveConstants.kLeftRearThrottleID, 
            SwerveConstants.kLeftRearRotorID, 
            SwerveConstants.kLeftRearCANCoderID, 
            SwerveConstants.kLeftRearRotorOffsetAngle
        );

        rightRearModule = new SwerveModule(
            SwerveConstants.kRightRearThrottleID, 
            SwerveConstants.kRightRearRotorID, 
            SwerveConstants.kRightRearCANCoderID, 
            SwerveConstants.kRightRearRotorOffsetAngle
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
        odometry.update(
            imu.getRotation2d(),
            getModulePositions()
        );
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
            leftFrontModule.getState(),
            rightFrontModule.getState(),
            leftRearModule.getState(),
            rightRearModule.getState()
        };
    }

    /**
     * Get swerve module positions
     * @return Swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            leftFrontModule.getPosition(),
            rightFrontModule.getPosition(),
            leftRearModule.getPosition(),
            rightRearModule.getPosition()
        };
    }

    /**
     * Set swerve module states
     * @param desiredStates Array of desired states (Order: leftFront, leftRear, rightFront, rightRear)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        leftFrontModule.setState(desiredStates[0]);
        rightFrontModule.setState(desiredStates[1]);
        leftRearModule.setState(desiredStates[2]);
        rightRearModule.setState(desiredStates[3]);
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
