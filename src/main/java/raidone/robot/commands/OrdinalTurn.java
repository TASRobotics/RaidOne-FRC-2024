package raidone.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import raidone.robot.subsystems.Swerve;

public class OrdinalTurn extends Command{
    private double angle;
    private double currentAngle;
    private Swerve swerve;

    public OrdinalTurn(int angle, Swerve swerve){
        this.swerve = swerve;
        this.angle = angle;
        currentAngle = swerve.getRotation().getDegrees()%360;
    }

    @Override
    public void execute(){
        swerve.drive(new Translation2d(0,0), currentAngle-angle, false, false);
    }
    
    @Override
    public boolean isFinished(){
        return (angle - currentAngle < 1.0 && angle - currentAngle > -1.0);
    }

    @Override
    public void end(boolean interupted){
        swerve.stopAll();
    }
}
