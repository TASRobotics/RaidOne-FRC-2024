package raidone.robot;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
        // rotorEncoder = TalonFXConfig.SwerveModule.configNewCANcoder(canCoderID, moduleAngleOffset);

        rotorEncoder = new CANcoder(canCoderID, "seCANdary");
        rotorEncoder.getConfigurator().apply(
            new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(moduleAngleOffset)
        );

        /* Angle Motor Config */
        // rotor = TalonFXConfig.SwerveModule.configNewRotor(rotorID);
        // resetToAbsolute();

        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        closedLoopGeneralConfigs.ContinuousWrap = true;

        rotor = new TalonFX(rotorID, "seCANdary");
        rotor.getConfigurator().apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
                )
                .withSlot0(
                    new Slot0Configs()
                        .withKP(0.0)
                        .withKI(0.0)
                        .withKD(0.0)
                        .withKS(0.0)
                        .withKA(0.0)
                )
                .withClosedLoopGeneral(closedLoopGeneralConfigs)
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(25)
                    .withSupplyCurrentThreshold(40)
                    .withSupplyTimeThreshold(0.1)
                )
        );

        resetToAbsolute();

        /* Drive Motor Config */
        // throttle = TalonFXConfig.SwerveModule.configNewThrottle(throttleID);

        throttle = new TalonFX(throttleID, "seCANdary");
        throttle.getConfigurator().apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio((6.75 / 1.0))
                )
                .withSlot0(
                    new Slot0Configs()
                        .withKP(0.0)
                        .withKI(0.0)
                        .withKD(0.0)
                        .withKS(0.0)
                        .withKA(0.0)
                )
                .withClosedLoopGeneral(closedLoopGeneralConfigs)
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(0.25)
                    .withVoltageOpenLoopRampPeriod(0.25)
                )
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withDutyCycleClosedLoopRampPeriod(0.0)
                    .withVoltageClosedLoopRampPeriod(0.0)
                )
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(35)
                    .withSupplyCurrentThreshold(60)
                    .withSupplyTimeThreshold(0.1)
                )
        );
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