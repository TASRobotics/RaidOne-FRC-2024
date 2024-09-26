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
    private CANSparkMax roller;
    private SparkLimitSwitch beam;
    private LaserCan laserCan;

    private static Intake intakeSys = new Intake();
    int distance = 9999;

    private Intake() {
        System.out.println("Intake Subsystem Init");

        roller = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        roller.restoreFactoryDefaults();
        roller.setIdleMode(IdleMode.kBrake);
        roller.setSmartCurrentLimit(CURRENT_LIMIT);

        beam = roller.getForwardLimitSwitch(Type.kNormallyOpen);
        beam.enableLimitSwitch(false);

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
        int distance = 9999;
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

    @Override
    public void periodic(){
        distance = getDistancePeriodic();
        SmartDashboard.putNumber("LaserCAN", distance);
        SmartDashboard.putBoolean("Beam Break", getLaserLimit());
    }
}
