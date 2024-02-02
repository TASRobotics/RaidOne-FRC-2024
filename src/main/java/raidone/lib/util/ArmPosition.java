package raidone.lib.util;

public enum ArmPosition {
    HOME(0, 0),
    INTAKE(1, 1),
    AMP(2, 2);

    private double armPos;
    private double wristPos;

    private ArmPosition(double armPos, double wristPos){
        this.armPos = armPos;
        this.wristPos = wristPos;
    }

    public double getArmPos() {
        return armPos;
    }

    public void setArmPos(double armPos) {
        this.armPos = armPos;
    }

    public double getWristPos() {
        return wristPos;
    }
    
    public void setWristPos(double wristPos) {
        this.wristPos = wristPos;
    }
}