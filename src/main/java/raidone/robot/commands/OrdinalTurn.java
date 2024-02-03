package raidone.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import raidone.robot.subsystems.Swerve;

public class OrdinalTurn extends PIDCommand {

    public OrdinalTurn(double angle, Swerve swerve) {
        // this.swerve = swerve;
        // this.angle = angle;
        // currentAngle = Math.abs((swerve.getRotation().getDegrees())) % 360;
        super(
                new PIDController(0.08, 0, 0),
                swerve::getHeadingOtherOne,// new HeadingAsDoubleSupplier(swerve.getAngleDegrees() * (Math.PI / 180)),
                angle,
                output -> swerve.drive(new Translation2d(0, 0), output, false, true),
                swerve);

        getController().enableContinuousInput(-180, 180);

        getController().setTolerance(2, 2);

    }

    // @Override
    // public void execute() {
    //     swerve.drive(new Translation2d(0, 0), turnRight ? 5 : -5, true, true);
    //     SmartDashboard.putNumber("Angle", currentAngle - angle);
    //     currentAngle = Math.abs((swerve.getRotation().getDegrees())) % 360;
    // }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Is @ point", getController().atSetpoint());
        return getController().atSetpoint();
    }

    // @Override
    // public void end(boolean interupted) {
    //     swerve.stopAll();
    // }
}
