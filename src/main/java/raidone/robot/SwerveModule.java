package raidone.robot;

import javax.sound.sampled.Control;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import raidone.lib.math.Conversions;

public class SwerveModule {
    private CANSparkMax rotor;
    private CANSparkMax throttle;
    private CANcoder rotorEncoder;
    private RelativeEncoder throttleEncoder;
    public CANcoderConfiguration CANCoderConfig;

    private PIDController rotorPID;

    public SwerveModule(int throttleID, int rotorID, int canCoderID, double moduleAngleOffset){
        throttle = new CANSparkMax(throttleID, MotorType.kBrushless);
        throttleEncoder = throttle.getEncoder();

        rotor = new CANSparkMax(rotorID, MotorType.kBrushless);

        rotorEncoder = new CANcoder(canCoderID);

        throttle.restoreFactoryDefaults();
        rotor.restoreFactoryDefaults();

        CANCoderConfig = new CANcoderConfiguration();

        rotor.setInverted(false);
        rotor.setIdleMode(IdleMode.kBrake);

        CANCoderConfig.withMagnetSensor(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(moduleAngleOffset)
        );

        rotorEncoder.getConfigurator().apply(CANCoderConfig);

        rotorPID = new PIDController(
            Constants.Swerve.ROTOR_KP,
            Constants.Swerve.ROTOR_KI,
            Constants.Swerve.ROTOR_KD
        );

        rotorPID.enableContinuousInput(-180, 180);

        // 根據之前的常數配置 
        throttle.setIdleMode(IdleMode.kBrake);

        // 給與 throttle encoder 轉換係數以便它以米每秒而不是 RPM 為單位讀取速度
        throttleEncoder.setVelocityConversionFactor(
            Constants.Swerve.
        );
        throttleEncoder.setPositionConversionFactor(
            Constants.Swerve.
        );
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.o ptimize(desiredState, getState().angle); 
        rotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){ 
             driveDutyC ycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            throttle.setControl(driveDutyCycle);
        }
        e     driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
        
                       driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            throttle.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotat ions(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        // double absolutePositio n = getCANcoder().getRotations() - this.moduleAngleOffset;

        //        rotor.setPosition(getCANcoder().getRotations());
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