package raidone.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private static Arm arm;
    private boolean isHomed;

    public Arm(){
        isHomed = false;
    }

    public void stopMotors(){

    }

    public void setPos(double setpoint){

    }

    public boolean isHomed(){
        // isHomed = SparkMax.ReadLimit;
        return isHomed;
    }

    @Override
    public void periodic(){
        
    }
}