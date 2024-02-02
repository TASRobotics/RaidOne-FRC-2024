package raidone.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{
    private static Wrist wrist;
    private boolean isHomed;

    public Wrist() {
        isHomed = false;        
    }

    public void stopMotors() {
        
    }

    public void setPos(double setpoint) {
        
    }

    public boolean isHomed() {
        // isHomed = SparkMax.ReadLimit
        return isHomed;
    }

    @Override
    public void periodic() {
    
    }
}
