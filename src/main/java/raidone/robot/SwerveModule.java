package raidone.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import raidone.lib.math.Conversions;

public class SwerveModule {
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private CTREConfig ctreConfig;
    private SwerveModuleConstants moduleConstants;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(SwerveModuleConstants moduleConstants){

        this.moduleConstants = moduleConstants;        
        ctreConfig = new CTREConfig(moduleConstants.ANGLE_OFFSET_ROTATIONS);
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.CAN_CODER_ID, "seCANdary");
        angleEncoder.getConfigurator().apply(ctreConfig.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.ROTOR_ID, "seCANdary");
        mAngleMotor.getConfigurator().apply(ctreConfig.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.THROTTLE_ID, "seCANdary");
        mDriveMotor.getConfigurator().apply(ctreConfig.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - this.moduleConstants.ANGLE_OFFSET_ROTATIONS;
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModuleConstants getModuleConstants() {
        return this.moduleConstants;
    }
    public static class SwerveModuleConstants {
        public final int MODULE_NUMBER;
        public final int THROTTLE_ID;
        public final int ROTOR_ID;
        public final int CAN_CODER_ID;
        public final double ANGLE_OFFSET_ROTATIONS;
    
        /***
         * 
         * @param moduleNum
         * @param throttleID
         * @param rotorID
         * @param canCoderID
         * @param angleOffset in TalonFX rotations
         */
        public SwerveModuleConstants(int moduleNum, int throttleID, int rotorID, int canCoderID, double angleOffset) {
            this.MODULE_NUMBER = moduleNum;
            this.THROTTLE_ID = throttleID;
            this.ROTOR_ID = rotorID;
            this.CAN_CODER_ID = canCoderID;
            this.ANGLE_OFFSET_ROTATIONS = angleOffset;
        }
    }
}