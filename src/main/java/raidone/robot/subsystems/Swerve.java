package raidone.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static raidone.robot.Constants.Swerve.*;
import raidone.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    private Pigeon2 imu;
    public SwerveModule FLModule, BLModule, BRModule, FRModule;
    public SwerveDriveOdometry swerveOdometry;
    private Field2d field = new Field2d();

    public Swerve() {
        imu = new Pigeon2(PIGEON_ID);
        imu.getConfigurator().apply(new Pigeon2Configuration());
        imu.setYaw(0);

        
        FLModule = new SwerveModule(THROTTLE_I_ID, ROTOR_I_ID, CAN_CODER_I_ID, MODULE_I_OFFSET);
        BLModule = new SwerveModule(THROTTLE_II_ID, ROTOR_II_ID, CAN_CODER_II_ID, MODULE_II_OFFSET);
        BRModule = new SwerveModule(THROTTLE_III_ID, ROTOR_III_ID, CAN_CODER_III_ID, MODULE_III_OFFSET);
        FRModule = new SwerveModule(THROTTLE_IV_ID, ROTOR_IV_ID, CAN_CODER_IV_ID, MODULE_IV_OFFSET);

        swerveOdometry = new SwerveDriveOdometry(SWERVE_DRIVE_KINEMATICS, getRotation(), getModulePositions());
    }

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
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        FLModule.setDesiredState(swerveModuleStates[0], isOpenLoop); //3
        BLModule.setDesiredState(swerveModuleStates[2], isOpenLoop); //0
        BRModule.setDesiredState(swerveModuleStates[3], isOpenLoop); //1
        FRModule.setDesiredState(swerveModuleStates[1], isOpenLoop); //2

    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        FLModule.setDesiredState(desiredStates[0], false); //3
        BLModule.setDesiredState(desiredStates[2], false); //0
        BRModule.setDesiredState(desiredStates[3], false); //1
        FRModule.setDesiredState(desiredStates[1], false); //2

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = FLModule.getState();
        states[2] = BLModule.getState();
        states[3] = BRModule.getState();
        states[1] = FRModule.getState();

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        positions[0] = FLModule.getPosition();
        positions[2] = BLModule.getPosition();
        positions[3] = BRModule.getPosition();
        positions[1] = FRModule.getPosition();

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
        FLModule.resetToAbsolute();
        BLModule.resetToAbsolute();
        BRModule.resetToAbsolute();
        FRModule.resetToAbsolute();
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

        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field); 
    }
}