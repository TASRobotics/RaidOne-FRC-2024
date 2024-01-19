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
    private double moduleAngleOffset;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.THROTTLE_KS, Constants.Swerve.THROTTLE_KV, Constants.Swerve.THROTTLE_KA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int throttleID, int rotorID, int canCoderID, double moduleAngleOffset){

        ctreConfig = new CTREConfig(moduleAngleOffset);
        this.moduleAngleOffset = moduleAngleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(canCoderID, "seCANdary");
        angleEncoder.getConfigurator().apply(ctreConfig.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(rotorID, "seCANdary");
        mAngleMotor.getConfigurator().apply(ctreConfig.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(throttleID, "seCANdary");
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
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - this.moduleAngleOffset;
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    // public SwerveModuleConstants getModuleConstants() {
    //     return this.moduleConstants;
    // }
    // public class SwerveModuleConstants {
    //     public final int MODULE_NUMBER;
    //     public final int THROTTLE_ID;
    //     public final int ROTOR_ID;
    //     public final int CAN_CODER_ID;
    //     public final double ANGLE_OFFSET_ROTATIONS;
    
    //     /***
    //      * 
    //      * @param moduleNum
    //      * @param throttleID
    //      * @param rotorID
    //      * @param canCoderID
    //      * @param angleOffset in TalonFX rotations
    //      */
    //     public SwerveModuleConstants(int moduleNum, int throttleID, int rotorID, int canCoderID, double angleOffset) {
    //         this.MODULE_NUMBER = moduleNum;
    //         this.THROTTLE_ID = throttleID;
    //         this.ROTOR_ID = rotorID;
    //         this.CAN_CODER_ID = canCoderID;
    //         this.ANGLE_OFFSET_ROTATIONS = angleOffset;
    //     }
    // }
}