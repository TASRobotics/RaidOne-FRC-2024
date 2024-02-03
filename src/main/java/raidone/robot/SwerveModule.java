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
    private TalonFX rotor;
    private TalonFX throttle;
    private CANcoder CANCoder;
    private CTREConfig ctreConfig;

    private final SimpleMotorFeedforward throttleFeedForward = new SimpleMotorFeedforward(Constants.Swerve.THROTTLE_KS, Constants.Swerve.THROTTLE_KV, Constants.Swerve.THROTTLE_KA);

    /* drive motor control requests */
    private final DutyCycleOut throttleDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage throttleVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage rotorPosition = new PositionVoltage(0);

    public SwerveModule(int throttleID, int rotorID, int canCoderID, double moduleAngleOffset){

        ctreConfig = new CTREConfig(moduleAngleOffset);

        /* Angle Encoder Config */
        CANCoder = new CANcoder(canCoderID, "seCANdary");
        CANCoder.getConfigurator().apply(ctreConfig.CANCoderConfig);

        /* Angle Motor Config */
        rotor = new TalonFX(rotorID, "seCANdary");
        rotor.getConfigurator().apply(ctreConfig.rotorFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        throttle = new TalonFX(throttleID, "seCANdary");
        throttle.getConfigurator().apply(ctreConfig.throttleFXConfig);
        throttle.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        rotor.setControl(rotorPosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            throttleDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            throttle.setControl(throttleDutyCycle);
        }
        else {
            throttleVelocity
    .Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
            throttleVelocity
    .FeedForward = throttleFeedForward.calculate(desiredState.speedMetersPerSecond);
            throttle.setControl(throttleVelocity
    );
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(CANCoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
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

    public void stopMotors(){
        rotor.stopMotor();
        throttle.stopMotor();
    }
}