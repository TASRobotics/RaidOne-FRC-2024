package raidone.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Swerve;

public class OrdinalTurn extends Command {
    private double angle;
    private double currentAngle;
    private Swerve swerve;

    public OrdinalTurn(int angle, Swerve swerve) {
        this.swerve = swerve;
        this.angle = angle;
        currentAngle = Math.abs((swerve.getRotation().getDegrees())) % 360;
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(0, 0), 5, true, true);
        SmartDashboard.putNumber("Angle", currentAngle - angle);
        currentAngle = Math.abs((swerve.getRotation().getDegrees())) % 360;
    }

    @Override
    public boolean isFinished() {
        return (currentAngle < angle + 5.0 && currentAngle > angle - 5.0);
    }

    @Override
    public void end(boolean interupted) {
        swerve.stopAll();
    }
}
