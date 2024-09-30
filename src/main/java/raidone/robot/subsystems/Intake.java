package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import static raidone.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
    // private static IntakeState inSt = new enum thingy
    private CANSparkMax roller;
    private SparkLimitSwitch beam;
    private LaserCan laserCan;

    private static Intake intakeSys = new Intake();
    int distance = 9999;

    enum IntakeStateEnum {
        RUNNING_NO_NOTE,
        RUNNING_HAS_NOTE,
        IDLE_NO_NOTE,
        IDLE_HAS_NOTE,
    }
    private static IntakeStateEnum intakeState = IntakeStateEnum.IDLE_NO_NOTE;

    private Intake() {
        System.out.println("Intake Subsystem Init");

        roller = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        roller.restoreFactoryDefaults();
        roller.setIdleMode(IdleMode.kBrake);
        roller.setSmartCurrentLimit(CURRENT_LIMIT);

        // beam = roller.getForwardLimitSwitch(Type.kNormallyOpen);
        // beam.enableLimitSwitch(false);

        laserCan = new LaserCan(20);
        // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
        try {
          laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
          laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
          laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
          System.out.println("Configuration failed! " + e);
        }
    }

    public boolean getLimit() {
        boolean limitStatus = beam.isPressed();
        return limitStatus;
    }

    public boolean getLaserLimit(){
        if(distance < Constants.Intake.distanceThreshold){
            return true;
        } else {
            return false;
        }
    }

    public int getDistancePeriodic(){
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        int distance = 501;
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            distance = measurement.distance_mm;
        } 
        return distance;
    }

    public int getDistance(){
        return distance;
    }

    public void run(double s) {
        roller.set(s);
    }

    public void stop() {
        roller.stopMotor();
    }

    public void resetEncoder() {
        roller.getEncoder().setPosition(0);
    }

    public boolean isRetracted() {
        return Math.abs(roller.getEncoder().getPosition()) > 2;
    }

    public static Intake system() {
        return intakeSys;
    }

    public IntakeStateEnum getState(){
        return intakeState;
    }

    @Override
    public void periodic(){
        // state machine code here

        // if Wrist.system().wrSt.getValue == 8 and inSt.getV
        //Wrist.system().isHomed();
        //Arm.system().isHomed()
        // update IntakeState enum
        distance = getDistancePeriodic();
        SmartDashboard.putNumber("LaserCAN", distance);
        SmartDashboard.putBoolean("Beam Break", getLaserLimit());

        if(roller.getAppliedOutput() == 0.0 && distance < distanceThreshold){
            intakeState = IntakeStateEnum.IDLE_HAS_NOTE;
        } else if (roller.getAppliedOutput() == 0.0 && distance >= distanceThreshold){
            intakeState = IntakeStateEnum.IDLE_NO_NOTE;
        } else if(roller.getAppliedOutput() != 0 && distance >= distanceThreshold){
            intakeState = IntakeStateEnum.RUNNING_NO_NOTE;
        } else if(roller.getAppliedOutput() < 0 && distance < distanceThreshold){
            intakeState = IntakeStateEnum.RUNNING_HAS_NOTE;
        }
    }
}
