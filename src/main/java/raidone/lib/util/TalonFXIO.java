package raidone.lib.util;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
//TODO: replace all parameters of these types with primitives or Strings
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * TalonFXIO class to create a TalonFX instance with configurations
 */
public class TalonFXIO {
    // private TalonFX instance only created when build is called
    private TalonFX talonFX;
    // TalonFX configuration objects
    private CurrentLimitsConfigs currentLimitsConfigs;
    private FeedbackConfigs feedbackConfigs;
    private HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs;
    private MotorOutputConfigs motorOutputConfigs;
    private TalonFXConfiguration talonFXConfig;

    // TalonFXIO default constructor
    public TalonFXIO() {
        this.talonFXConfig = new TalonFXConfiguration();
    }

    // get talonFX instance
    public TalonFX getTalonFX() {
        return this.talonFX;
    }

    /**
     * Build the talonFX instance with the CAN_ID and busName and apply any
     * configurations that were set.
     * Call this method at the end of the configs method chain.
     * 
     * @param CAN_ID  the CAN ID of the TalonFX
     * @param busName the bus name of the TalonFX
     * @throws TalonFXConfigException if the motorOutputConfigs is null
     */
    public TalonFXIO build(int CAN_ID, String busName) {
        if (this.talonFX == null) {
            this.talonFX = new TalonFX(CAN_ID, busName);
        }
        if (this.motorOutputConfigs == null) {
            DriverStation.reportError(
                    "MotorOutputConfigs must be set for the TalonFX ID: " + CAN_ID + " on bus: " + busName, true);
        }
        if (this.feedbackConfigs == null) {
            DriverStation.reportError(
                    "FeedbackConfigs must be set for the TalonFX ID: " + CAN_ID + " on bus: " + busName, true);
        }
        // apply the configurations
        if (this.motorOutputConfigs != null) {
            this.talonFXConfig.withMotorOutput(this.motorOutputConfigs);
        }
        if (this.feedbackConfigs != null) {
            this.talonFXConfig.withFeedback(this.feedbackConfigs);
        }
        if (this.currentLimitsConfigs != null) {
            this.talonFXConfig.withCurrentLimits(this.currentLimitsConfigs);
        }
        if (this.hardwareLimitSwitchConfigs != null) {
            this.talonFXConfig.withHardwareLimitSwitch(this.hardwareLimitSwitchConfigs);
        }
        this.talonFX.getConfigurator().apply(this.talonFXConfig);
        return this;
    }

    /**
     * Chainable withStatorCurrentLimits method
     * 
     * @param statorLimit_AMPS current limit in AMPS to restrict the torque output
     *                         of the motor.
     *                         Default limit is 120A and commonly range 80-160 but
     *                         lower for something hitting a hardstop
     * @param enable           enable or disable the current limit
     */
    public TalonFXIO withStatorCurrentLimits(double statorLimit_AMPS, boolean enable) {
        if (this.currentLimitsConfigs == null) {
            this.currentLimitsConfigs = new CurrentLimitsConfigs();
        }
        this.currentLimitsConfigs.StatorCurrentLimit = statorLimit_AMPS;
        this.currentLimitsConfigs.StatorCurrentLimitEnable = enable;
        return this;
    }

    /**
     * Chainable withSupplyCurrentLimits method
     * 
     * @param supplyLimit_AMPS current limit in AMPS to restrict the torque output
     *                         of the motor.
     *                         Reasonable range is 70A with a lower limit of 40A
     *                         after 1s. Commonly 20-80A
     * @param enable           enable or disable the current limit
     */
    public TalonFXIO withSupplyCurrentLimits(double supplyLimit_AMPS, boolean enable) {
        if (this.currentLimitsConfigs == null) {
            this.currentLimitsConfigs = new CurrentLimitsConfigs();
        }
        this.currentLimitsConfigs.SupplyCurrentLimit = supplyLimit_AMPS;
        this.currentLimitsConfigs.SupplyCurrentLimitEnable = enable;
        return this;
    }

    /**
     * Threshold current to exceed for a specified time before the supply limit is
     * enforced
     * 
     * @param supplyCurrentThreshold_AMPS the current to exceed before enforcing the
     *                                    supply limit
     * @param supplyTimeThreshold_SEC     time in seconds the threshold current must
     *                                    be exceeded before the supply limit is
     *                                    enforced
     */
    public TalonFXIO withSupplyCurrentThreshold(double supplyCurrentThreshold_AMPS, double supplyTimeThreshold_SEC) {
        if (this.currentLimitsConfigs == null) {
            this.currentLimitsConfigs = new CurrentLimitsConfigs();
        }
        this.currentLimitsConfigs.SupplyCurrentThreshold = supplyCurrentThreshold_AMPS;
        this.currentLimitsConfigs.SupplyTimeThreshold = supplyTimeThreshold_SEC;
        return this;
    }

    /**
     * Remote Sensor Configuration for the TalonFX
     * 
     * @param remoteSensorID     the remote sensor CAN_ID
     * @param remoteSensorSource the remote sensor source
     */
    public TalonFXIO withRemoteSensor(int remoteSensorID, FeedbackSensorSourceValue remoteSensorSource) {
        if (this.feedbackConfigs == null) {
            this.feedbackConfigs = new FeedbackConfigs();
        }
        this.feedbackConfigs.FeedbackRemoteSensorID = remoteSensorID;
        this.feedbackConfigs.FeedbackSensorSource = remoteSensorSource;
        return this;
    }

    /**
     * Sensor ratio configuration for the TalonFX
     * 
     * @param rotorToSensorRatio the ratio of the motor rotor to the sensor
     * @param sensorToMechRatio  the ratio of the sensor to the mechanism
     */
    public TalonFXIO withSensorRatios(double rotorToSensorRatio, double sensorToMechRatio) {
        if (this.feedbackConfigs == null) {
            this.feedbackConfigs = new FeedbackConfigs();
        }
        this.feedbackConfigs.RotorToSensorRatio = rotorToSensorRatio;
        this.feedbackConfigs.SensorToMechanismRatio = sensorToMechRatio;
        return this;
    }

    /**
     * Enable fwd limit with autoset position
     * 
     * @param fwdLimitEnable       enable forward limit switch
     * @param fwdLimitSetPosEnable enable autoset position when fwd limit triggered
     * @param fwdLimitSetPos       position to autoset when fwd limit triggered
     */
    public TalonFXIO withFwdLimitSwitch(boolean fwdLimitEnable, boolean fwdLimitSetPosEnable, double fwdLimitSetPos) {
        if (this.hardwareLimitSwitchConfigs == null) {
            this.hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        }
        this.hardwareLimitSwitchConfigs.ForwardLimitEnable = fwdLimitEnable;
        this.hardwareLimitSwitchConfigs.ForwardLimitAutosetPositionEnable = fwdLimitSetPosEnable;
        this.hardwareLimitSwitchConfigs.ForwardLimitAutosetPositionValue = fwdLimitSetPos;
        return this;
    }

    /**
     * Enable rev limit with autoset position
     * 
     * @param revLimitEnable       enable reverse limit switch
     * @param revLimitSetPosEnable enable autoset position when fwd limit triggered
     * @param revLimitSetPos       position to autoset when rev limit triggered
     */
    public TalonFXIO withRevLimitSwitch(boolean revLimitEnable, boolean revLimitSetPosEnable, double revLimitSetPos) {
        if (this.hardwareLimitSwitchConfigs == null) {
            this.hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        }
        this.hardwareLimitSwitchConfigs.ReverseLimitEnable = revLimitEnable;
        this.hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionEnable = revLimitSetPosEnable;
        this.hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionValue = revLimitSetPos;
        return this;
    }

    /**
     * Setup remote fwd limit. Use this in conjunction with withFwdLimitSwitch to
     * enable and auto set position.
     * 
     * @param remoteFwdLimitSource   the remote source for the forward limit (remote
     *                               FX, CANifier, or CANcoder)
     * @param remoteFwdLimitDeviceID the remote device ID for the forward limit
     */
    public TalonFXIO withRemoteFwdLimit(ForwardLimitSourceValue remoteFwdLimitSource, int remoteFwdLimitDeviceID) {
        if (this.hardwareLimitSwitchConfigs == null) {
            this.hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        }
        this.hardwareLimitSwitchConfigs.ForwardLimitSource = remoteFwdLimitSource;
        this.hardwareLimitSwitchConfigs.ForwardLimitRemoteSensorID = remoteFwdLimitDeviceID;
        return this;
    }

    /**
     * Setup remote rev limit. Use this in conjunction with withRevLimitSwitch to
     * enable and auto set position.
     * 
     * @param remoteRevLimitSource   the remote source for the reverse limit (remote
     *                               FX, CANifier, or CANcoder)
     * @param remoteRevLimitDeviceID the remote device ID for the reverse limit
     */
    public TalonFXIO withRemoteRevLimit(ReverseLimitSourceValue remoteRevLimitSource, int remoteRevLimitDeviceID) {
        if (this.hardwareLimitSwitchConfigs == null) {
            this.hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        }
        this.hardwareLimitSwitchConfigs.ReverseLimitSource = remoteRevLimitSource;
        this.hardwareLimitSwitchConfigs.ReverseLimitRemoteSensorID = remoteRevLimitDeviceID;
        return this;
    }

    /**
     * Fwd limit switch polarity to specify noramlly open or normally closed
     * 
     * @param fwdLimitPolarity the polarity of the forward limit switch
     */
    public TalonFXIO withFwdLimitPolarity(ForwardLimitTypeValue fwdLimitPolarity) {
        if (this.hardwareLimitSwitchConfigs == null) {
            this.hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        }
        this.hardwareLimitSwitchConfigs.ForwardLimitType = fwdLimitPolarity;
        return this;
    }

    /**
     * Rev limit switch polarity to specify noramlly open or normally closed
     * 
     * @param revLimitPolarity the polarity of the reverse limit switch
     */
    public TalonFXIO withRevLimitPolarity(ReverseLimitTypeValue revLimitPolarity) {
        if (this.hardwareLimitSwitchConfigs == null) {
            this.hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        }
        this.hardwareLimitSwitchConfigs.ReverseLimitType = revLimitPolarity;
        return this;
    }

    /**
     * Set motor driection and neutral mode
     * 
     * @param clockwise true if clockwise, false if counter clockwise
     * @param brake     true if brake mode, false if coast mode
     */
    public TalonFXIO withMotorDirection(boolean clockwise, boolean brake) {
        if (this.motorOutputConfigs == null) {
            this.motorOutputConfigs = new MotorOutputConfigs();
        }
        this.motorOutputConfigs.Inverted = clockwise ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        this.motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        return this;
    }
}