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
import raidone.lib.util.TalonFXConfig;

public class SwerveModule {
    private TalonFX rotor;
    private TalonFX throttle;
    private CANcoder rotorEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.THROTTLE_KS, Constants.Swerve.THROTTLE_KV, Constants.Swerve.THROTTLE_KA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int throttleID, int rotorID, int canCoderID, double moduleAngleOffset){
        /* Angle Encoder Config */
        rotorEncoder = TalonFXConfig.SwerveModule.configNewCANcoder(canCoderID, moduleAngleOffset);

        /* Angle Motor Config */
        rotor = TalonFXConfig.SwerveModule.configNewRotor(rotorID);
        resetToAbsolute();

        /* Drive Motor Config */
        throttle = TalonFXConfig.SwerveModule.configNewThrottle(throttleID);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        rotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            throttle.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            throttle.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(rotorEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        // double absolutePosition = getCANcoder().getRotations() - this.moduleAngleOffset;
        rotor.setPosition(getCANcoder().getRotations());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(throttle.getVelocity().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(rotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(throttle.getPosition().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(rotor.getPosition().getValue())
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